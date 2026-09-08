#include <iostream>
#include <string>
#include <cassert>
#include <cmath>
#include <stdexcept>

#include <root_directory.h>

#include <physxcoopa/util/math.h>
#include <physxcoopa/geometry/aabb.h>
#include <physxcoopa/geometry/ray.h>
#include <physxcoopa/geometry/capsule.h>
#include <physxcoopa/dynamics/body.h>
#include <physxcoopa/dynamics/inertia.h>
#include <physxcoopa/world.h>
#include <physxcoopa/collision/shape.h>
#include <physxcoopa/collision/sat.h>
#include <physxcoopa/broadphase/aabb_tree.h>
#include <physxcoopa/broadphase/layer_matrix.h>
#include <physxcoopa/geometry/triangle_mesh.h>
#include <physxcoopa/query/queries.h>
#include <physxcoopa/debug/debug_draw.h>
#include <physxcoopa/system/physics_system.h>
#include <physxcoopa/components/box_collider.h>
#include <physxcoopa/components/sphere_collider.h>
#include <physxcoopa/components/rigidbody.h>

#include <coopa/scene/scene.h>
#include <coopa/scene/scene_object.h>
#include <coopa/scene/components/transform_component.h>

#include <glm/gtc/quaternion.hpp>

#include <random>
#include <set>
#include <utility>
#include <vector>

using namespace coopa::physx;

// ANSI Colors for nice UI
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_RESET   "\x1b[0m"

static int g_tests_run = 0;
static int g_tests_failed = 0;

#define RUN_TEST(test_func) \
    do { \
        std::cout << ANSI_COLOR_BLUE << "[ RUN      ] " << ANSI_COLOR_RESET << #test_func << std::endl; \
        g_tests_run++; \
        try { \
            test_func(); \
            std::cout << ANSI_COLOR_GREEN << "[       OK ] " << ANSI_COLOR_RESET << #test_func << std::endl; \
        } catch (const std::exception& e) { \
            std::cerr << ANSI_COLOR_RED << "[  FAILED  ] " << ANSI_COLOR_RESET << #test_func << " (Exception: " << e.what() << ")" << std::endl; \
            g_tests_failed++; \
        } catch (...) { \
            std::cerr << ANSI_COLOR_RED << "[  FAILED  ] " << ANSI_COLOR_RESET << #test_func << " (Unknown Exception)" << std::endl; \
            g_tests_failed++; \
        } \
    } while (0)

#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << ANSI_COLOR_RED << "  Assertion failed: " << #condition << " at " << __FILE__ << ":" << __LINE__ << ANSI_COLOR_RESET << std::endl; \
            throw std::runtime_error("Assertion failed: " #condition); \
        } \
    } while (0)

#define ASSERT_NEAR(val1, val2, eps) \
    do { \
        if (std::fabs((val1) - (val2)) > (eps)) { \
            std::cerr << ANSI_COLOR_RED << "  Assertion failed: " << #val1 << " ~= " << #val2 \
                      << " (Actual: " << (val1) << ", Expected: " << (val2) << ", eps: " << (eps) << ") at " \
                      << __FILE__ << ":" << __LINE__ << ANSI_COLOR_RESET << std::endl; \
            throw std::runtime_error("Assertion failed: " #val1 " ~= " #val2); \
        } \
    } while (0)

#define ASSERT_VEC3_NEAR(v1, v2, eps) \
    do { \
        ASSERT_NEAR((v1).x, (v2).x, (eps)); \
        ASSERT_NEAR((v1).y, (v2).y, (eps)); \
        ASSERT_NEAR((v1).z, (v2).z, (eps)); \
    } while (0)

// --- Phase 1: math, geometry ---

static void test_ray_aabb_hand_computed() {
    geometry::AABB box;
    box.min = glm::vec3(-1.0f, -1.0f, -1.0f);
    box.max = glm::vec3(1.0f, 1.0f, 1.0f);

    geometry::Ray ray;
    ray.origin = glm::vec3(0.0f, 0.0f, 5.0f);
    ray.direction = glm::vec3(0.0f, 0.0f, -1.0f);
    ray.max_distance = 100.0f;

    float t;
    ASSERT_TRUE(ray.intersect(box, t));
    ASSERT_NEAR(t, 4.0f, 1e-5f); // travels from z=5 to the box's near face at z=1

    geometry::Ray miss;
    miss.origin = glm::vec3(5.0f, 5.0f, 5.0f);
    miss.direction = glm::vec3(0.0f, 0.0f, -1.0f);
    miss.max_distance = 100.0f;
    ASSERT_TRUE(!miss.intersect(box, t));
}

static void test_closest_points_segment_segment_parallel() {
    // Two parallel segments, offset by 1 unit in x, both spanning z in [0,1].
    glm::vec3 c1, c2;
    geometry::closest_points_segment_segment(
        glm::vec3(0, 0, 0), glm::vec3(0, 0, 1),
        glm::vec3(1, 0, 0), glm::vec3(1, 0, 1),
        c1, c2);
    ASSERT_NEAR(glm::length(c2 - c1), 1.0f, 1e-4f);
    ASSERT_NEAR(c1.z, c2.z, 1e-4f); // parallel segments -> closest points at equal z

    // Perpendicular, crossing segments should meet exactly at the origin.
    geometry::closest_points_segment_segment(
        glm::vec3(-1, 0, 0), glm::vec3(1, 0, 0),
        glm::vec3(0, -1, 0), glm::vec3(0, 1, 0),
        c1, c2);
    ASSERT_VEC3_NEAR(c1, glm::vec3(0, 0, 0), 1e-4f);
    ASSERT_VEC3_NEAR(c2, glm::vec3(0, 0, 0), 1e-4f);
}

static void test_orthonormal_basis() {
    glm::vec3 n = util::safe_normalize(glm::vec3(0.3f, -0.7f, 0.4f));
    glm::vec3 t1, t2;
    util::orthonormal_basis(n, t1, t2);
    ASSERT_NEAR(glm::dot(n, t1), 0.0f, 1e-4f);
    ASSERT_NEAR(glm::dot(n, t2), 0.0f, 1e-4f);
    ASSERT_NEAR(glm::dot(t1, t2), 0.0f, 1e-4f);
    ASSERT_NEAR(glm::length(t1), 1.0f, 1e-4f);
    ASSERT_NEAR(glm::length(t2), 1.0f, 1e-4f);
}

// --- Phase 2: bodies, integrator, fixed-step world ---

static void test_free_fall_matches_analytic() {
    PhysicsWorld world;
    dynamics::Body b;
    b.type = dynamics::BodyType::Dynamic;
    b.position = glm::vec3(0.0f);
    b.inv_mass = 1.0f;
    dynamics::BodyId id = world.add_body(b);

    const float h = util::k_default_fixed_dt;
    for (int i = 0; i < 60; ++i) world.step_fixed(h); // 1 second

    const dynamics::Body* body = world.get_body(id);
    ASSERT_TRUE(body != nullptr);
    // Semi-implicit Euler's known result for constant acceleration g over N steps of h:
    // z = -g * h * (1+2+...+N) * h = -g*h^2*N*(N+1)/2 (velocity applied BEFORE position).
    float n = 60.0f;
    float expected_z = util::k_gravity_z * h * h * n * (n + 1.0f) * 0.5f;
    ASSERT_NEAR(body->position.z, expected_z, 0.01f);
    ASSERT_NEAR(body->position.z, -0.5f * 9.81f, 0.2f); // close to the continuous-time answer too
}

static void test_determinism_hash_stable_across_runs() {
    auto run = []() {
        PhysicsWorld world;
        for (int i = 0; i < 5; ++i) {
            dynamics::Body b;
            b.position = glm::vec3(float(i), 0.0f, float(i) * 2.0f);
            b.linear_velocity = glm::vec3(0.1f, -0.2f, 0.3f);
            world.add_body(b);
        }
        for (int i = 0; i < 600; ++i) world.step_fixed(util::k_default_fixed_dt);
        return world.world_state_hash();
    };
    uint64_t h1 = run();
    uint64_t h2 = run();
    ASSERT_TRUE(h1 == h2);
}

static void test_kinematic_and_static_bodies_do_not_fall() {
    PhysicsWorld world;
    dynamics::Body kin;
    kin.type = dynamics::BodyType::Kinematic;
    kin.position = glm::vec3(1.0f, 2.0f, 3.0f);
    dynamics::BodyId kin_id = world.add_body(kin);

    dynamics::Body stat;
    stat.type = dynamics::BodyType::Static;
    stat.position = glm::vec3(4.0f, 5.0f, 6.0f);
    dynamics::BodyId stat_id = world.add_body(stat);

    for (int i = 0; i < 60; ++i) world.step_fixed(util::k_default_fixed_dt);

    ASSERT_VEC3_NEAR(world.get_body(kin_id)->position, glm::vec3(1.0f, 2.0f, 3.0f), 1e-5f);
    ASSERT_VEC3_NEAR(world.get_body(stat_id)->position, glm::vec3(4.0f, 5.0f, 6.0f), 1e-5f);
}

static void test_body_id_generation_rejects_stale_handle() {
    PhysicsWorld world;
    dynamics::BodyId id_a = world.add_body(dynamics::Body{});
    world.remove_body(id_a);
    dynamics::BodyId id_b = world.add_body(dynamics::Body{}); // recycles id_a's slot
    ASSERT_TRUE(id_a.index == id_b.index);
    ASSERT_TRUE(id_a.generation != id_b.generation);
    ASSERT_TRUE(!world.is_valid(id_a));
    ASSERT_TRUE(world.is_valid(id_b));
}

static void test_box_and_sphere_inertia_are_positive() {
    glm::vec3 box_inv = dynamics::box_inverse_inertia(glm::vec3(0.5f), 2.0f);
    ASSERT_TRUE(box_inv.x > 0.0f && box_inv.y > 0.0f && box_inv.z > 0.0f);

    glm::vec3 sphere_inv = dynamics::sphere_inverse_inertia(1.0f, 2.0f);
    float expected = 1.0f / (0.4f * 2.0f * 1.0f * 1.0f);
    ASSERT_NEAR(sphere_inv.x, expected, 1e-5f);
    ASSERT_NEAR(sphere_inv.y, expected, 1e-5f);
    ASSERT_NEAR(sphere_inv.z, expected, 1e-5f);

    // A capsule's axial inertia (about its own long axis) must be less than or equal to its
    // perpendicular inertia -- i.e. its inverse-axial must be >= inverse-perpendicular --
    // since mass is concentrated closer to the long axis than to a perpendicular one.
    glm::vec3 cap_inv = dynamics::capsule_inverse_inertia(0.5f, 1.0f, 2.0f);
    ASSERT_TRUE(cap_inv.z >= cap_inv.x);
}

// --- Phase 3: sphere narrowphase + solver ---

static dynamics::BodyId add_static_ground(PhysicsWorld& world, dynamics::PhysicsMaterial* mat = nullptr) {
    dynamics::Body ground;
    ground.type = dynamics::BodyType::Static;
    ground.position = glm::vec3(0.0f, 0.0f, -0.1f); // top surface at world z=0
    collision::Shape shape = collision::Shape::make_box(glm::vec3(4.0f, 4.0f, 0.1f));
    shape.material = mat;
    return world.add_body(ground, shape);
}

static dynamics::BodyId add_dynamic_sphere(PhysicsWorld& world, const glm::vec3& pos, float radius,
                                            dynamics::PhysicsMaterial* mat = nullptr) {
    dynamics::Body body;
    body.type = dynamics::BodyType::Dynamic;
    body.position = pos;
    body.mass = 1.0f;
    body.inv_mass = 1.0f;
    body.inv_inertia_local = dynamics::sphere_inverse_inertia(radius, body.mass);
    collision::Shape shape = collision::Shape::make_sphere(radius);
    shape.material = mat;
    return world.add_body(body, shape);
}

static void test_sphere_rests_and_sleeps_on_ground() {
    PhysicsWorld world;
    add_static_ground(world);
    dynamics::BodyId sphere_id = add_dynamic_sphere(world, glm::vec3(0.0f, 0.0f, 2.0f), 0.5f);

    bool slept = false;
    for (int i = 0; i < 400 && !slept; ++i) {
        world.step_fixed(util::k_default_fixed_dt);
        if (!world.get_body(sphere_id)->awake) slept = true;
    }
    ASSERT_TRUE(slept);

    const dynamics::Body* b = world.get_body(sphere_id);
    ASSERT_NEAR(b->position.z, 0.5f, 0.05f);
    ASSERT_TRUE(glm::length(b->linear_velocity) < world.config().sleep_linear + 1e-4f);
}

static void test_restitution_one_bounces_high() {
    dynamics::PhysicsMaterial bouncy;
    bouncy.restitution = 1.0f;

    PhysicsWorld world;
    add_static_ground(world, &bouncy);
    float start_z = 2.0f;
    dynamics::BodyId id = add_dynamic_sphere(world, glm::vec3(0.0f, 0.0f, start_z), 0.5f, &bouncy);
    float drop_height = start_z - 0.5f;

    bool has_bounced = false;
    float max_z_after_bounce = 0.0f;
    for (int i = 0; i < 300; ++i) {
        world.step_fixed(util::k_default_fixed_dt);
        const dynamics::Body* b = world.get_body(id);
        if (b->linear_velocity.z > 0.0f) {
            has_bounced = true;
            max_z_after_bounce = std::max(max_z_after_bounce, b->position.z);
        }
    }
    ASSERT_TRUE(has_bounced);
    // A near-elastic bounce should return well above half the drop height -- loose bound to
    // stay robust to discretization/solver damping while still proving restitution works.
    ASSERT_TRUE(max_z_after_bounce > 0.5f + 0.6f * drop_height);
}

static void test_restitution_zero_never_bounces() {
    PhysicsWorld world;
    add_static_ground(world);
    dynamics::BodyId id = add_dynamic_sphere(world, glm::vec3(0.0f, 0.0f, 2.0f), 0.5f);

    bool contacted = false;
    float max_upward_after_contact = 0.0f;
    for (int i = 0; i < 300; ++i) {
        world.step_fixed(util::k_default_fixed_dt);
        const dynamics::Body* b = world.get_body(id);
        if (b->position.z < 0.6f) contacted = true;
        if (contacted) max_upward_after_contact = std::max(max_upward_after_contact, b->linear_velocity.z);
    }
    ASSERT_TRUE(contacted);
    ASSERT_TRUE(max_upward_after_contact < 1.0f);
}

static void test_energy_never_increases() {
    dynamics::PhysicsMaterial frictionless;
    frictionless.dynamic_friction = 0.0f;
    frictionless.static_friction = 0.0f;
    frictionless.restitution = 0.0f;

    PhysicsWorld world;
    add_static_ground(world, &frictionless);
    dynamics::BodyId id = add_dynamic_sphere(world, glm::vec3(0.0f, 0.0f, 2.0f), 0.5f, &frictionless);

    float max_ke_after_settle = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        world.step_fixed(util::k_default_fixed_dt);
        const dynamics::Body* b = world.get_body(id);
        float ke = 0.5f * b->mass * glm::dot(b->linear_velocity, b->linear_velocity);
        if (i > 150) { // past the initial fall/settle transient
            ASSERT_TRUE(ke <= max_ke_after_settle + 0.05f);
            max_ke_after_settle = std::max(max_ke_after_settle, ke);
        }
    }
}

static void test_static_and_kinematic_pairs_generate_no_manifolds() {
    PhysicsWorld world;
    dynamics::Body s1;
    s1.type = dynamics::BodyType::Static;
    s1.position = glm::vec3(0.0f);
    dynamics::Body s2 = s1;
    world.add_body(s1, collision::Shape::make_sphere(1.0f));
    world.add_body(s2, collision::Shape::make_sphere(1.0f));

    dynamics::Body k1;
    k1.type = dynamics::BodyType::Kinematic;
    k1.position = glm::vec3(5.0f, 0.0f, 0.0f);
    dynamics::Body k2 = k1;
    world.add_body(k1, collision::Shape::make_sphere(1.0f));
    world.add_body(k2, collision::Shape::make_sphere(1.0f));

    world.step_fixed(util::k_default_fixed_dt);
    ASSERT_TRUE(world.manifolds().empty());
}

static void test_kinematic_platform_pushes_resting_body() {
    PhysicsWorld world;
    world.set_gravity(glm::vec3(0.0f)); // isolate the push effect from gravity

    dynamics::Body platform;
    platform.type = dynamics::BodyType::Kinematic;
    platform.position = glm::vec3(0.0f);
    platform.linear_velocity = glm::vec3(0.0f, 0.0f, 2.0f);
    world.add_body(platform, collision::Shape::make_box(glm::vec3(2.0f, 2.0f, 0.1f)));

    dynamics::BodyId sphere_id = add_dynamic_sphere(world, glm::vec3(0.0f, 0.0f, 0.6f), 0.5f);

    for (int i = 0; i < 10; ++i) world.step_fixed(util::k_default_fixed_dt);

    const dynamics::Body* s = world.get_body(sphere_id);
    ASSERT_TRUE(s->position.z > 0.55f);
    ASSERT_TRUE(s->linear_velocity.z > 0.5f);
}

/**
 * The kinematic wake rule (dynamics/solver.h's wake_if_platform_moving) needs a body that is
 * genuinely ASLEEP before the platform starts moving -- test_kinematic_platform_pushes_resting_body
 * above starts the platform moving from step 0, so the sphere is still awake the whole time and
 * never exercises this rule at all. Without it, a body asleep on a platform that starts sliding
 * out from under it would hover in place while the platform moves away, since a sleeping body
 * skips integration entirely.
 */
static void test_kinematic_wake_rule_wakes_sleeping_body_on_moving_platform() {
    PhysicsWorld world;

    dynamics::Body platform;
    platform.type = dynamics::BodyType::Kinematic;
    platform.position = glm::vec3(0.0f, 0.0f, -0.5f);
    dynamics::BodyId platform_id = world.add_body(platform, collision::Shape::make_box(glm::vec3(2.0f, 2.0f, 0.5f)));

    dynamics::BodyId sphere_id = add_dynamic_sphere(world, glm::vec3(0.0f, 0.0f, 1.0f), 0.5f);

    // Let the sphere fall, rest on the stationary platform, and fall asleep.
    for (int i = 0; i < 120; ++i) world.step_fixed(util::k_default_fixed_dt);
    ASSERT_TRUE(!world.get_body(sphere_id)->awake);

    // Now slide the platform sideways -- the sphere should wake immediately, not hover in place.
    world.get_body(platform_id)->linear_velocity = glm::vec3(3.0f, 0.0f, 0.0f);
    world.step_fixed(util::k_default_fixed_dt);
    const dynamics::Body* s = world.get_body(sphere_id);
    ASSERT_TRUE(s->awake);
}

static void test_on_substep_signal_and_deferred_destroy() {
    PhysicsWorld world;
    dynamics::BodyId id = add_dynamic_sphere(world, glm::vec3(0.0f, 0.0f, 5.0f), 0.5f);

    int substep_calls = 0;
    bool destroyed_inside = false;
    world.on_substep.connect([&](PhysicsWorld& w, float) {
        ++substep_calls;
        if (substep_calls == 3) {
            w.destroy_body(id);
            destroyed_inside = true;
        }
    });

    for (int i = 0; i < 5; ++i) world.step_fixed(util::k_default_fixed_dt);

    ASSERT_TRUE(substep_calls == 5);
    ASSERT_TRUE(destroyed_inside);
    ASSERT_TRUE(!world.is_valid(id));
}

// --- Phase 4: boxes, SAT, feature ids ---

static dynamics::BodyId add_dynamic_box(PhysicsWorld& world, const glm::vec3& pos, const glm::vec3& half_extents,
                                         const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                         dynamics::PhysicsMaterial* mat = nullptr) {
    dynamics::Body body;
    body.position = pos;
    body.orientation = rot;
    body.mass = 1.0f;
    body.inv_mass = 1.0f;
    body.inv_inertia_local = dynamics::box_inverse_inertia(half_extents, body.mass);
    collision::Shape shape = collision::Shape::make_box(half_extents);
    shape.material = mat;
    return world.add_body(body, shape);
}

static dynamics::BodyId add_static_box(PhysicsWorld& world, const glm::vec3& pos, const glm::vec3& half_extents,
                                        const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                        dynamics::PhysicsMaterial* mat = nullptr) {
    dynamics::Body body;
    body.type = dynamics::BodyType::Static;
    body.position = pos;
    body.orientation = rot;
    collision::Shape shape = collision::Shape::make_box(half_extents);
    shape.material = mat;
    return world.add_body(body, shape);
}

static void test_sat_box_box_face_contact_known_penetration() {
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

static void test_sat_box_box_edge_contact_single_point() {
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

static void test_ten_box_stack_stable() {
    PhysicsWorld world;
    add_static_box(world, glm::vec3(0.0f, 0.0f, -0.5f), glm::vec3(5.0f, 5.0f, 0.5f));

    const float half = 0.5f;
    std::vector<dynamics::BodyId> ids;
    for (int i = 0; i < 10; ++i) {
        glm::vec3 pos(0.0f, 0.0f, half * (2 * i + 1));
        ids.push_back(add_dynamic_box(world, pos, glm::vec3(half)));
    }

    for (int i = 0; i < 1000; ++i) world.step_fixed(util::k_default_fixed_dt);

    for (auto id : ids) {
        const dynamics::Body* b = world.get_body(id);
        ASSERT_TRUE(std::abs(b->position.x) < 0.01f);
        ASSERT_TRUE(std::abs(b->position.y) < 0.01f);
        ASSERT_TRUE(!b->awake);
    }
}

static void test_friction_slope_static_vs_sliding() {
    const float angle_deg = 30.0f;
    glm::quat slope_rot = glm::angleAxis(glm::radians(angle_deg), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec3 slope_normal = slope_rot * glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 slope_tangent = glm::normalize(glm::vec3(slope_normal.z, 0.0f, -slope_normal.x)); // along the incline

    auto run = [&](float friction) {
        dynamics::PhysicsMaterial mat;
        mat.dynamic_friction = friction;
        mat.static_friction = friction;
        mat.friction_combine = dynamics::CombineMode::Minimum;

        PhysicsWorld world;
        add_static_box(world, glm::vec3(0.0f), glm::vec3(5.0f, 5.0f, 0.5f), slope_rot, &mat);
        glm::vec3 box_pos = slope_normal * 0.75f; // slope half-extent (0.5) + box half-extent (0.25)
        dynamics::BodyId id = add_dynamic_box(world, box_pos, glm::vec3(0.25f), slope_rot, &mat);

        glm::vec3 start = box_pos;
        for (int i = 0; i < 300; ++i) world.step_fixed(util::k_default_fixed_dt); // 5s
        const dynamics::Body* b = world.get_body(id);
        return glm::dot(b->position - start, slope_tangent);
    };

    float slid_high_friction = run(0.8f);   // > tan(30) ~= 0.577 -> should stay put
    float slid_low_friction = run(0.2f);    // < tan(30) -> should slide noticeably

    ASSERT_TRUE(std::abs(slid_high_friction) < 0.05f);
    ASSERT_TRUE(std::abs(slid_low_friction) > 0.3f); // slides a meaningful distance down the incline
}

static void test_mass_ratio_100_to_1_stable() {
    PhysicsWorld world;
    add_static_box(world, glm::vec3(0.0f, 0.0f, -0.5f), glm::vec3(5.0f, 5.0f, 0.5f));

    dynamics::Body light;
    light.position = glm::vec3(0.0f, 0.0f, 0.5f);
    light.mass = 1.0f;
    light.inv_mass = 1.0f;
    light.inv_inertia_local = dynamics::box_inverse_inertia(glm::vec3(0.5f), 1.0f);
    dynamics::BodyId light_id = world.add_body(light, collision::Shape::make_box(glm::vec3(0.5f)));

    dynamics::Body heavy;
    heavy.position = glm::vec3(0.0f, 0.0f, 1.5f);
    heavy.mass = 100.0f;
    heavy.inv_mass = 1.0f / 100.0f;
    heavy.inv_inertia_local = dynamics::box_inverse_inertia(glm::vec3(0.5f), 100.0f);
    dynamics::BodyId heavy_id = world.add_body(heavy, collision::Shape::make_box(glm::vec3(0.5f)));

    for (int i = 0; i < 500; ++i) {
        world.step_fixed(util::k_default_fixed_dt);
        const dynamics::Body* l = world.get_body(light_id);
        const dynamics::Body* h = world.get_body(heavy_id);
        ASSERT_TRUE(std::isfinite(l->position.x) && std::isfinite(l->position.z));
        ASSERT_TRUE(std::isfinite(h->position.z));
        ASSERT_TRUE(glm::length(l->linear_velocity) < 50.0f);
        ASSERT_TRUE(glm::length(h->linear_velocity) < 50.0f);
    }
    const dynamics::Body* l = world.get_body(light_id);
    const dynamics::Body* h = world.get_body(heavy_id);
    ASSERT_TRUE(l->position.z > 0.3f && l->position.z < 0.7f);
    ASSERT_TRUE(h->position.z > 1.3f && h->position.z < 1.7f);
}

// --- Phase 5: capsules ---

static dynamics::BodyId add_dynamic_capsule(PhysicsWorld& world, const glm::vec3& pos, float radius, float half_height,
                                             int axis, const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
    dynamics::Body body;
    body.position = pos;
    body.orientation = rot;
    body.mass = 1.0f;
    body.inv_mass = 1.0f;
    glm::vec3 raw = dynamics::capsule_inverse_inertia(radius, half_height, body.mass); // (perp, perp, axial)
    glm::vec3 local;
    if (axis == 0) local = glm::vec3(raw.z, raw.y, raw.x);      // axial -> X
    else if (axis == 1) local = glm::vec3(raw.x, raw.z, raw.y); // axial -> Y
    else local = raw;                                            // axial -> Z (already there)
    body.inv_inertia_local = local;
    collision::Shape shape = collision::Shape::make_capsule(radius, half_height, axis);
    return world.add_body(body, shape);
}

static void test_capsule_rests_on_box_floor() {
    PhysicsWorld world;
    add_static_box(world, glm::vec3(0.0f, 0.0f, -0.5f), glm::vec3(5.0f, 5.0f, 0.5f));

    const float radius = 0.3f;
    dynamics::BodyId id = add_dynamic_capsule(world, glm::vec3(0.0f, 0.0f, 1.5f), radius, 0.5f, 0 /*axis X*/);

    for (int i = 0; i < 400; ++i) world.step_fixed(util::k_default_fixed_dt);

    const dynamics::Body* b = world.get_body(id);
    ASSERT_NEAR(b->position.z, radius, 0.05f);
    ASSERT_TRUE(glm::length(b->linear_velocity) < 0.1f);
}

static void test_capsule_pile_settles_without_exploding() {
    PhysicsWorld world;
    add_static_box(world, glm::vec3(0.0f, 0.0f, -0.5f), glm::vec3(5.0f, 5.0f, 0.5f));

    std::vector<dynamics::BodyId> ids;
    for (int i = 0; i < 5; ++i) {
        glm::vec3 pos(0.1f * i, 0.0f, 1.0f + 0.5f * i);
        ids.push_back(add_dynamic_capsule(world, pos, 0.25f, 0.4f, 0));
    }

    for (int i = 0; i < 500; ++i) {
        world.step_fixed(util::k_default_fixed_dt);
        for (auto id : ids) {
            const dynamics::Body* b = world.get_body(id);
            ASSERT_TRUE(std::isfinite(b->position.x) && std::isfinite(b->position.z));
            ASSERT_TRUE(glm::length(b->linear_velocity) < 50.0f);
            ASSERT_TRUE(b->position.z > -1.0f); // never tunnels through the floor
        }
    }
}

// --- Phase 8: mesh colliders ---

/**
 * Two coplanar triangles sharing one diagonal edge, forming a flat 4x4 floor:
 *   v3(-2,2,0) ---- v2(2,2,0)
 *      |  tri1  /     |
 *      |      /       |
 *      |    /   tri0  |
 *   v0(-2,-2,0) ---- v1(2,-2,0)
 * tri0 = (v0,v1,v2), tri1 = (v0,v2,v3); both wind to face normal +Z. The shared edge is
 * tri0's edge2 (v2->v0) / tri1's edge0 (v0->v2) -- adjacency below wires exactly that pair,
 * every other edge is a mesh boundary (default-constructed k_no_neighbor).
 */
static geometry::TriangleMesh make_two_triangle_floor() {
    std::vector<glm::vec3> vertices = {
        glm::vec3(-2.0f, -2.0f, 0.0f), glm::vec3(2.0f, -2.0f, 0.0f),
        glm::vec3(2.0f, 2.0f, 0.0f), glm::vec3(-2.0f, 2.0f, 0.0f),
    };
    std::vector<uint32_t> indices = {0, 1, 2, 0, 2, 3};
    std::vector<glm::vec3> normals = {glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f)};
    std::vector<geometry::TriangleAdjacency> adjacency(2);
    adjacency[0].neighbor[2] = 1;
    adjacency[1].neighbor[0] = 0;
    return geometry::TriangleMesh(std::move(vertices), std::move(indices), std::move(normals), std::move(adjacency));
}

static void test_box_slides_across_mesh_floor_no_edge_discontinuity() {
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

// --- Phase 9: queries ---

static void test_raycast_hits_box_with_exact_t_point_and_normal() {
    PhysicsWorld world;
    dynamics::BodyId id = add_static_box(world, glm::vec3(5.0f, 0.0f, 0.0f), glm::vec3(0.5f));

    geometry::Ray ray;
    ray.origin = glm::vec3(0.0f, 0.0f, 0.0f);
    ray.direction = glm::vec3(1.0f, 0.0f, 0.0f);
    ray.max_distance = 100.0f;

    query::RaycastHit hit;
    ASSERT_TRUE(world.raycast(ray, hit));
    ASSERT_NEAR(hit.distance, 4.5f, 1e-3f); // box's near face at x=4.5
    ASSERT_VEC3_NEAR(hit.point, glm::vec3(4.5f, 0.0f, 0.0f), 1e-3f);
    ASSERT_VEC3_NEAR(hit.normal, glm::vec3(-1.0f, 0.0f, 0.0f), 1e-3f);
    ASSERT_TRUE(hit.body.index == id.index);
}

static void test_raycast_all_returns_hits_sorted_by_distance() {
    PhysicsWorld world;
    add_static_box(world, glm::vec3(9.0f, 0.0f, 0.0f), glm::vec3(0.5f));
    add_static_box(world, glm::vec3(3.0f, 0.0f, 0.0f), glm::vec3(0.5f));
    add_static_box(world, glm::vec3(6.0f, 0.0f, 0.0f), glm::vec3(0.5f));

    geometry::Ray ray;
    ray.origin = glm::vec3(0.0f, 0.0f, 0.0f);
    ray.direction = glm::vec3(1.0f, 0.0f, 0.0f);
    ray.max_distance = 100.0f;

    std::vector<query::RaycastHit> hits = world.raycast_all(ray);
    ASSERT_TRUE(hits.size() == 3);
    ASSERT_TRUE(hits[0].distance < hits[1].distance);
    ASSERT_TRUE(hits[1].distance < hits[2].distance);
    ASSERT_NEAR(hits[0].distance, 2.5f, 1e-3f);
    ASSERT_NEAR(hits[1].distance, 5.5f, 1e-3f);
    ASSERT_NEAR(hits[2].distance, 8.5f, 1e-3f);
}

static void test_overlap_sphere_and_overlap_box_find_expected_bodies() {
    PhysicsWorld world;
    dynamics::BodyId near_id = add_static_box(world, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.5f));
    add_static_box(world, glm::vec3(20.0f, 0.0f, 0.0f), glm::vec3(0.5f));

    std::vector<dynamics::BodyId> sphere_hits = world.overlap_sphere(glm::vec3(0.2f, 0.0f, 0.0f), 0.5f);
    ASSERT_TRUE(sphere_hits.size() == 1);
    ASSERT_TRUE(sphere_hits[0].index == near_id.index);

    geometry::OBB query_box;
    query_box.center = glm::vec3(0.0f);
    query_box.half_extents = glm::vec3(1.0f);
    std::vector<dynamics::BodyId> box_hits = world.overlap_box(query_box);
    ASSERT_TRUE(box_hits.size() == 1);
    ASSERT_TRUE(box_hits[0].index == near_id.index);
}

// --- Phase 9: triggers ---

static void test_trigger_enter_stay_exit_fires_correct_sequence() {
    PhysicsWorld world;
    world.set_gravity(glm::vec3(0.0f)); // isolate the trigger pass from falling

    dynamics::Body trigger_body;
    trigger_body.type = dynamics::BodyType::Static;
    collision::Shape trigger_shape = collision::Shape::make_sphere(1.0f);
    trigger_shape.is_trigger = true;
    dynamics::BodyId trigger_id = world.add_body(trigger_body, trigger_shape);

    // A dynamic probe (triggers only ever pair with a Dynamic body -- see
    // generate_manifolds_()'s either_dynamic check) flies straight through the trigger volume.
    dynamics::Body probe;
    probe.type = dynamics::BodyType::Dynamic;
    probe.position = glm::vec3(-3.0f, 0.0f, 0.0f);
    probe.linear_velocity = glm::vec3(2.0f, 0.0f, 0.0f);
    probe.mass = 1.0f;
    probe.inv_mass = 1.0f;
    probe.inv_inertia_local = dynamics::sphere_inverse_inertia(0.1f, probe.mass);
    dynamics::BodyId probe_id = world.add_body(probe, collision::Shape::make_sphere(0.1f));

    int enter_count = 0, stay_count = 0, exit_count = 0;
    for (int i = 0; i < 200; ++i) { // 2 m/s over 6 units (x: -3 -> 3) takes 3s = 180 steps
        world.clear_events();
        world.step_fixed(util::k_default_fixed_dt);
        for (const auto& ev : world.events()) {
            if (!ev.is_trigger) continue;
            bool is_this_pair = (ev.a.index == trigger_id.index && ev.b.index == probe_id.index) ||
                                 (ev.a.index == probe_id.index && ev.b.index == trigger_id.index);
            if (!is_this_pair) continue;
            if (ev.type == collision::ContactEvent::Type::Enter) ++enter_count;
            else if (ev.type == collision::ContactEvent::Type::Stay) ++stay_count;
            else ++exit_count;
        }
    }

    ASSERT_TRUE(enter_count == 1);
    ASSERT_TRUE(exit_count == 1);
    ASSERT_TRUE(stay_count > 0); // fires every step while inside
}

static void test_debug_draw_emits_collider_bvh_and_contact_lines() {
    PhysicsWorld world;
    add_static_box(world, glm::vec3(0.0f, 0.0f, -0.5f), glm::vec3(5.0f, 5.0f, 0.5f));
    add_dynamic_sphere(world, glm::vec3(0.0f, 0.0f, 0.5f), 0.5f);

    for (int i = 0; i < 30; ++i) world.step_fixed(util::k_default_fixed_dt); // settle into contact

    debug::DebugDraw draw;
    world.debug_draw(draw);
    ASSERT_TRUE(!draw.lines.empty()); // colliders + BVH nodes always produce something

    debug::DebugDraw colliders_only;
    world.debug_draw(colliders_only, debug::DebugDrawFlags::Colliders);
    ASSERT_TRUE(!colliders_only.lines.empty());
    ASSERT_TRUE(colliders_only.lines.size() < draw.lines.size()); // strictly fewer without BVH/contacts
}

// --- Phase 6: broadphase ---

static void test_aabb_tree_matches_brute_force() {
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> pos_dist(-20.0f, 20.0f);
    std::uniform_real_distribution<float> size_dist(0.1f, 2.0f);

    const int n = 1000;
    std::vector<geometry::AABB> boxes(n);
    for (int i = 0; i < n; ++i) {
        glm::vec3 center(pos_dist(rng), pos_dist(rng), pos_dist(rng));
        glm::vec3 half(size_dist(rng), size_dist(rng), size_dist(rng));
        boxes[i].min = center - half;
        boxes[i].max = center + half;
    }

    broadphase::AABBTree tree;
    std::vector<int32_t> proxies(n);
    for (int i = 0; i < n; ++i) {
        proxies[i] = tree.create_proxy(boxes[i], reinterpret_cast<void*>(static_cast<uintptr_t>(i)));
    }

    // Brute-force oracle.
    std::set<std::pair<int, int>> brute_force_pairs;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (boxes[i].overlaps(boxes[j])) brute_force_pairs.insert({i, j});
        }
    }

    std::set<std::pair<int, int>> tree_pairs;
    for (int i = 0; i < n; ++i) {
        tree.query(boxes[i], [&](void* user_data) {
            int j = static_cast<int>(reinterpret_cast<uintptr_t>(user_data));
            if (j != i) tree_pairs.insert({std::min(i, j), std::max(i, j)});
        });
    }

    // The tree's fat AABBs make it a conservative OVER-approximation -- every true overlap
    // must be found, but a fat-margin near-miss may also appear.
    for (const auto& p : brute_force_pairs) {
        ASSERT_TRUE(tree_pairs.count(p) == 1);
    }

    // Move every proxy slightly and re-verify: still a superset of a fresh brute-force pass
    // over the NEW boxes, and every fat AABB still contains its own tight bounds.
    for (int i = 0; i < n; ++i) {
        glm::vec3 shift(0.05f, -0.03f, 0.02f);
        boxes[i].min += shift;
        boxes[i].max += shift;
        tree.move_proxy(proxies[i], boxes[i], shift);
        ASSERT_TRUE(tree.fat_bounds(proxies[i]).contains(boxes[i]));
    }
    std::set<std::pair<int, int>> brute_force_pairs2;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (boxes[i].overlaps(boxes[j])) brute_force_pairs2.insert({i, j});
        }
    }
    std::set<std::pair<int, int>> tree_pairs2;
    for (int i = 0; i < n; ++i) {
        tree.query(boxes[i], [&](void* user_data) {
            int j = static_cast<int>(reinterpret_cast<uintptr_t>(user_data));
            if (j != i) tree_pairs2.insert({std::min(i, j), std::max(i, j)});
        });
    }
    for (const auto& p : brute_force_pairs2) {
        ASSERT_TRUE(tree_pairs2.count(p) == 1);
    }
}

static void test_layer_matrix_blocks_pair_before_narrowphase() {
    PhysicsWorld world;
    world.layers().set_layer_collision(0, 1, false);

    dynamics::Body a;
    a.type = dynamics::BodyType::Static;
    a.position = glm::vec3(0.0f);
    collision::Shape shape_a = collision::Shape::make_sphere(1.0f);
    shape_a.layer = 0;
    world.add_body(a, shape_a);

    dynamics::Body b;
    b.position = glm::vec3(0.5f, 0.0f, 0.0f); // deeply overlapping shape_a
    b.mass = 1.0f;
    b.inv_mass = 1.0f;
    collision::Shape shape_b = collision::Shape::make_sphere(1.0f);
    shape_b.layer = 1;
    dynamics::BodyId id_b = world.add_body(b, shape_b);

    world.step_fixed(util::k_default_fixed_dt);
    ASSERT_TRUE(world.manifolds().empty());

    // Re-enable and confirm the same overlap now DOES produce a manifold (proves the
    // previous empty result was the layer filter, not a broadphase/narrowphase miss).
    world.layers().set_layer_collision(0, 1, true);
    world.step_fixed(util::k_default_fixed_dt);
    ASSERT_TRUE(!world.manifolds().empty());
    (void)id_b;
}

// --- Phase 7: scene binding ---

static void test_headless_scene_binding_creates_bodies_and_settles() {
    using namespace coopa::scene;

    Scene scene("PhysicsBindTest");

    auto ground = std::make_unique<SceneObject>("ground");
    ground->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, -0.5f));
    auto* ground_box = ground->add_component<components::BoxCollider>();
    ground_box->set_size(glm::vec3(8.0f, 8.0f, 1.0f));
    scene.add_root_object(std::move(ground));

    auto box = std::make_unique<SceneObject>("box");
    box->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 2.0f));
    box->add_component<components::BoxCollider>()->set_size(glm::vec3(1.0f));
    box->add_component<components::RigidbodyComponent>();
    scene.add_root_object(std::move(box));

    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);
    ASSERT_TRUE(sys->bound_count() == 0); // not gathered until the first execute()

    for (int i = 0; i < 300; ++i) {
        scene.update(util::k_default_fixed_dt);
        scene.late_update(util::k_default_fixed_dt);
    }

    ASSERT_TRUE(sys->bound_count() == 2); // ground + box each bind to exactly one body

    SceneObject* box_obj = scene.find_object("box");
    ASSERT_TRUE(box_obj != nullptr);
    float z = box_obj->get_transform()->transform().position().z;
    ASSERT_NEAR(z, 0.5f, 0.05f); // ground top at z=0, box half-extent 0.5
}

static void test_scene_binding_runtime_parameter_change_rides_revision() {
    using namespace coopa::scene;

    Scene scene("PhysicsRevisionTest");
    auto obj = std::make_unique<SceneObject>("sphere");
    obj->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 5.0f));
    auto* sphere = obj->add_component<components::SphereCollider>();
    sphere->set_radius(0.5f);
    obj->add_component<components::RigidbodyComponent>();
    scene.add_root_object(std::move(obj));

    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);
    scene.update(util::k_default_fixed_dt); // first execute() binds the body

    SceneObject* sphere_obj = scene.find_object("sphere");
    auto* collider = sphere_obj->get_component<components::SphereCollider>();
    dynamics::BodyId id = collider->body_id();
    ASSERT_TRUE(id.is_valid());
    ASSERT_NEAR(sys->world().get_shape(id)->radius, 0.5f, 1e-5f);

    collider->set_radius(1.5f); // no refresh() call -- must ride the per-frame revision check
    scene.update(util::k_default_fixed_dt);

    ASSERT_NEAR(sys->world().get_shape(id)->radius, 1.5f, 1e-5f);
    ASSERT_TRUE(collider->body_id() == id); // same body, shape updated in place -- not rebuilt
}

int main() {
    RUN_TEST(test_ray_aabb_hand_computed);
    RUN_TEST(test_closest_points_segment_segment_parallel);
    RUN_TEST(test_orthonormal_basis);

    RUN_TEST(test_free_fall_matches_analytic);
    RUN_TEST(test_determinism_hash_stable_across_runs);
    RUN_TEST(test_kinematic_and_static_bodies_do_not_fall);
    RUN_TEST(test_body_id_generation_rejects_stale_handle);
    RUN_TEST(test_box_and_sphere_inertia_are_positive);

    RUN_TEST(test_sphere_rests_and_sleeps_on_ground);
    RUN_TEST(test_restitution_one_bounces_high);
    RUN_TEST(test_restitution_zero_never_bounces);
    RUN_TEST(test_energy_never_increases);
    RUN_TEST(test_static_and_kinematic_pairs_generate_no_manifolds);
    RUN_TEST(test_kinematic_platform_pushes_resting_body);
    RUN_TEST(test_kinematic_wake_rule_wakes_sleeping_body_on_moving_platform);
    RUN_TEST(test_on_substep_signal_and_deferred_destroy);

    RUN_TEST(test_sat_box_box_face_contact_known_penetration);
    RUN_TEST(test_sat_box_box_edge_contact_single_point);
    RUN_TEST(test_ten_box_stack_stable);
    RUN_TEST(test_friction_slope_static_vs_sliding);
    RUN_TEST(test_mass_ratio_100_to_1_stable);

    RUN_TEST(test_capsule_rests_on_box_floor);
    RUN_TEST(test_capsule_pile_settles_without_exploding);

    RUN_TEST(test_box_slides_across_mesh_floor_no_edge_discontinuity);

    RUN_TEST(test_raycast_hits_box_with_exact_t_point_and_normal);
    RUN_TEST(test_raycast_all_returns_hits_sorted_by_distance);
    RUN_TEST(test_overlap_sphere_and_overlap_box_find_expected_bodies);
    RUN_TEST(test_trigger_enter_stay_exit_fires_correct_sequence);
    RUN_TEST(test_debug_draw_emits_collider_bvh_and_contact_lines);

    RUN_TEST(test_aabb_tree_matches_brute_force);
    RUN_TEST(test_layer_matrix_blocks_pair_before_narrowphase);

    RUN_TEST(test_headless_scene_binding_creates_bodies_and_settles);
    RUN_TEST(test_scene_binding_runtime_parameter_change_rides_revision);

    std::cout << "===========================================" << std::endl;
    std::cout << "Tests run: " << g_tests_run << ", Failed: " << g_tests_failed << std::endl;
    std::cout << "===========================================" << std::endl;

    return g_tests_failed == 0 ? 0 : 1;
}
