/**
 * @file narrowphase_test.cpp
 * @brief Contact generation: box-box SAT (face and edge-edge features, exact depths), BVH-narrowed
 *        mesh contacts against a brute-force oracle, and internal-edge correction on a tiled mesh floor.
 */
#include <coopa/testing/test.h>

#include <physxcoopa/world.h>
#include <physxcoopa/collision/sat.h>
#include <physxcoopa/collision/shape.h>
#include <random>
#include "support/physics_fixtures.h"

COOPA_TEST_SUITE("narrowphase");

using namespace coopa::physx;
using namespace physxtest;

COOPA_TEST(box_box_face_contact_has_four_points_at_known_depth) {
    geometry::OBB a;
    a.center = glm::vec3(0.0f);
    a.half_extents = glm::vec3(0.5f);
    geometry::OBB b;
    b.center = glm::vec3(0.8f, 0.0f, 0.0f);
    b.half_extents = glm::vec3(0.5f);

    collision::ContactManifold m;
    ASSERT_TRUE(collision::generate_box_box_contacts(a, b, m));
    ASSERT_NEAR(m.normal.x, 1.0f, 1e-3f);
    ASSERT_NEAR(std::abs(m.normal.y), 0.0f, 1e-3f);
    ASSERT_NEAR(std::abs(m.normal.z), 0.0f, 1e-3f);
    ASSERT_TRUE(m.count == 4); // full face-face overlap -> 4 clipped points
    for (uint8_t i = 0; i < m.count; ++i) {
        ASSERT_NEAR(m.points[i].penetration, 0.2f, 1e-3f);
    }
}

COOPA_TEST(box_box_edge_contact_is_a_single_edge_feature) {
    geometry::OBB a;
    a.center = glm::vec3(0.0f);
    a.half_extents = glm::vec3(0.5f);

    // An arbitrary tilt + diagonal offset, found by search, that lands the minimum-penetration
    // SAT axis on an edge-cross axis rather than either box's face normal -- unlike a
    // deliberately "corner-on" or single-axis-rotated setup, which (perhaps counter-
    // intuitively) still tends to resolve to a face axis for two equal-sized cubes.
    geometry::OBB b;
    b.orientation = glm::quat(-0.996603f, -0.0278176f, -0.0606705f, -0.0482471f);
    b.center = glm::vec3(0.864856f, 0.686217f, 0.79309f);
    b.half_extents = glm::vec3(0.5f);

    collision::ContactManifold m;
    ASSERT_TRUE(collision::generate_box_box_contacts(a, b, m));
    ASSERT_TRUE(m.count == 1);
    ASSERT_TRUE(m.points[0].feature_id >= 0x1000u); // edge-edge feature-id range
}

/**
 * BVH-narrowed mesh contact generation matches the brute-force every-triangle scan exactly --
 * same points, depths, feature ids and normal -- for spheres, capsules and boxes at random
 * poses around a bumpy 16x16-cell heightfield.
 */
COOPA_TEST(mesh_contacts_via_bvh_match_brute_force) {
    const int cells = 16;
    const float cell = 0.5f;
    auto height = [](float x, float y) { return 0.4f * std::sin(x * 1.3f) * std::cos(y * 0.9f); };
    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> indices;
    for (int j = 0; j <= cells; ++j)
        for (int i = 0; i <= cells; ++i) {
            float x = i * cell, y = j * cell;
            vertices.push_back(glm::vec3(x, y, height(x, y)));
        }
    for (int j = 0; j < cells; ++j)
        for (int i = 0; i < cells; ++i) {
            uint32_t a = j * (cells + 1) + i, b = a + 1, c = a + cells + 1, d = c + 1;
            indices.insert(indices.end(), {a, b, d, a, d, c});
        }
    geometry::TriangleMesh terrain = make_mesh_with_adjacency(std::move(vertices), std::move(indices));
    collision::Shape mesh_shape = collision::Shape::make_mesh(&terrain);
    const glm::vec3 mesh_pos(-1.0f, 0.5f, 0.2f);
    const glm::quat mesh_rot = glm::angleAxis(0.3f, glm::normalize(glm::vec3(0.2f, 0.1f, 1.0f)));

    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    int compared = 0, touching = 0;
    for (int trial = 0; trial < 600; ++trial) {
        collision::Shape probe;
        switch (trial % 3) {
            case 0: probe = collision::Shape::make_sphere(0.2f + 0.4f * unit(rng)); break;
            case 1: probe = collision::Shape::make_capsule(0.15f + 0.2f * unit(rng), 0.2f + 0.4f * unit(rng)); break;
            default: probe = collision::Shape::make_box(glm::vec3(0.1f + 0.4f * unit(rng), 0.1f + 0.4f * unit(rng), 0.1f + 0.3f * unit(rng))); break;
        }
        float lx = 0.5f + 7.0f * unit(rng), ly = 0.5f + 7.0f * unit(rng);
        glm::vec3 local(lx, ly, height(lx, ly) + 0.1f + 0.4f * (unit(rng) - 0.5f));
        glm::vec3 pos = mesh_pos + mesh_rot * local;
        glm::quat rot = glm::angleAxis(6.28f * unit(rng), glm::normalize(glm::vec3(unit(rng) - 0.5f, unit(rng) - 0.5f, unit(rng) + 0.1f)));

        collision::ContactManifold fast, brute;
        bool hit_fast = collision::generate_mesh_contacts(probe, pos, rot, mesh_shape, mesh_pos, mesh_rot, fast, true);
        bool hit_brute = collision::generate_mesh_contacts(probe, pos, rot, mesh_shape, mesh_pos, mesh_rot, brute, false);
        ASSERT_TRUE(hit_fast == hit_brute);
        ++compared;
        if (!hit_fast) continue;
        ++touching;
        ASSERT_TRUE(fast.count == brute.count);
        ASSERT_VEC_NEAR(fast.normal, brute.normal, 1e-6f);
        for (uint8_t k = 0; k < fast.count; ++k) {
            ASSERT_VEC_NEAR(fast.points[k].position, brute.points[k].position, 1e-6f);
            ASSERT_NEAR(fast.points[k].penetration, brute.points[k].penetration, 1e-6f);
            ASSERT_TRUE(fast.points[k].feature_id == brute.points[k].feature_id);
        }
    }
    ASSERT_TRUE(compared == 600);
    ASSERT_TRUE(touching > 150); // the pose range really exercises contacts, not just misses
}

COOPA_TEST(box_slides_across_internal_mesh_edge_without_a_bump) {
    geometry::TriangleMesh floor_mesh = make_two_triangle_floor();

    dynamics::PhysicsMaterial low_friction;
    low_friction.dynamic_friction = 0.05f;
    low_friction.static_friction = 0.05f;

    PhysicsWorld world;
    dynamics::Body floor_body;
    floor_body.type = dynamics::BodyType::Static;
    collision::Shape floor_shape = collision::Shape::make_mesh(&floor_mesh);
    floor_shape.material = &low_friction;
    world.add_body(floor_body, floor_shape);

    const float half_extent = 0.25f;
    dynamics::Body box;
    box.position = glm::vec3(-1.5f, 0.0f, half_extent + 0.02f);
    box.mass = 1.0f;
    box.inv_mass = 1.0f;
    box.inv_inertia_local = dynamics::box_inverse_inertia(glm::vec3(half_extent), box.mass);
    collision::Shape box_shape = collision::Shape::make_box(glm::vec3(half_extent));
    box_shape.material = &low_friction;
    dynamics::BodyId id = world.add_body(box, box_shape);

    // Settle onto the floor first, well clear of the shared diagonal edge (crossed at x == y, so
    // starting/staying at y=0 crosses it once, at x=0).
    for (int i = 0; i < 60; ++i) world.step_fixed(util::k_default_fixed_dt);

    dynamics::Body* b = world.get_body(id);
    b->wake();
    b->linear_velocity = glm::vec3(3.0f, 0.0f, 0.0f);

    // 40 steps at 3 m/s covers 2 units (x: -1.5 -> 0.5), crossing the shared seam at x=0 while
    // staying well inside the mesh's [-2,2] bounds on both sides.
    float max_vz_jump = 0.0f;
    float prev_vz = b->linear_velocity.z;
    for (int i = 0; i < 40; ++i) {
        world.step_fixed(util::k_default_fixed_dt);
        const dynamics::Body* cur = world.get_body(id);
        max_vz_jump = std::max(max_vz_jump, std::abs(cur->linear_velocity.z - prev_vz));
        prev_vz = cur->linear_velocity.z;
    }

    const dynamics::Body* final_b = world.get_body(id);
    ASSERT_TRUE(final_b->position.x > 0.0f); // slid across the shared edge, past x=0
    ASSERT_NEAR(final_b->position.z, half_extent, 0.05f); // stayed flush on the floor throughout
    ASSERT_TRUE(max_vz_jump < 1.0f); // no discontinuity crossing the internal edge
}
