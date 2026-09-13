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
#include <physxcoopa/cloth/cloth_builder.h>
#include <physxcoopa/cloth/cloth_solver.h>
#include <physxcoopa/components/cloth.h>
#include <physxcoopa/components/box_collider.h>
#include <physxcoopa/components/sphere_collider.h>
#include <physxcoopa/components/rigidbody.h>
#include <physxcoopa/loaders/physics_material_loader.h>
#include <physxcoopa/util/physics_settings.h>

#include <coopa/scene/scene.h>
#include <coopa/scene/scene_object.h>
#include <coopa/scene/components/transform_component.h>
#include <coopa/asset/asset_manager.h>
#include <coopa/job/engine.h>

#include <glm/gtc/quaternion.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <set>
#include <sstream>
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

/**
 * The kinematic wake rule's linear check (exercised above) misses a kinematic body that only
 * ROTATES in place -- e.g. physics_test's spinner, hinged at its own Transform origin, whose
 * position never changes. Before dynamics/solver.h's wake_if_kinematic_moving also checked
 * angular_velocity, a sleeping body such a paddle sweeps into would never wake, and
 * solve_velocity_pass()'s "skip unless at least one side is an awake dynamic body" gate would
 * then skip that contact forever -- the paddle visually passing straight through it.
 */
static void test_kinematic_spin_wakes_sleeping_body_it_sweeps_into() {
    PhysicsWorld world;
    add_static_ground(world);
    dynamics::BodyId sphere_id = add_dynamic_sphere(world, glm::vec3(3.0f, 0.0f, 1.0f), 0.5f);

    // Let it settle onto the ground and fall asleep before the paddle appears.
    for (int i = 0; i < 120; ++i) world.step_fixed(util::k_default_fixed_dt);
    ASSERT_TRUE(!world.get_body(sphere_id)->awake);
    glm::vec3 rest_pos = world.get_body(sphere_id)->position;

    // A kinematic "paddle" already overlapping the sleeping sphere, spinning in place -- zero
    // linear_velocity, nonzero angular_velocity only.
    dynamics::Body paddle;
    paddle.type = dynamics::BodyType::Kinematic;
    paddle.position = rest_pos;
    paddle.angular_velocity = glm::vec3(0.0f, 0.0f, 2.0f);
    world.add_body(paddle, collision::Shape::make_box(glm::vec3(1.0f, 1.0f, 1.0f)));

    world.step_fixed(util::k_default_fixed_dt);
    ASSERT_TRUE(world.get_body(sphere_id)->awake);
}

/**
 * dynamics/solver.h's apply_impulse_pair() used to apply a computed impulse to any body with
 * type == Dynamic, regardless of `awake` -- inconsistent with warm_start()/solve_velocity_pass()
 * themselves, which already zero a sleeping body's inv_mass/inv_inertia (correctly treating it
 * as immovable) before deriving that impulse's magnitude. The bug: a sleeping body touched by an
 * awake neighbor ended up with a nonzero velocity baked in while still marked asleep --
 * integrate_velocities() skips a sleeping body, so nothing visibly moved yet, but the very next
 * time the body woke (by any means) it would pop with this stale, unaccounted-for velocity
 * already in it. Three flush-stacked boxes, where the bottom one sleeps before its neighbors
 * finish settling, is exactly this shape.
 */
static void test_sleeping_body_unaffected_by_awake_neighbors_impulse() {
    PhysicsWorld world;
    world.set_gravity(glm::vec3(0.0f)); // isolate the impulse effect from gravity/settling

    dynamics::BodyId sleeper_id = add_dynamic_sphere(world, glm::vec3(0.0f, 0.0f, 0.0f), 0.5f);
    dynamics::Body* sleeper = world.get_body(sleeper_id);
    sleeper->awake = false;
    sleeper->linear_velocity = glm::vec3(0.0f);
    sleeper->angular_velocity = glm::vec3(0.0f);

    // Overlapping at spawn (radii sum to 1.0, centers 0.9 apart) so a manifold exists from the
    // very first step, approaching further still.
    dynamics::BodyId mover_id = add_dynamic_sphere(world, glm::vec3(0.9f, 0.0f, 0.0f), 0.5f);
    world.get_body(mover_id)->linear_velocity = glm::vec3(-2.0f, 0.0f, 0.0f);

    world.step_fixed(util::k_default_fixed_dt);

    const dynamics::Body* s = world.get_body(sleeper_id);
    ASSERT_TRUE(!s->awake);
    ASSERT_VEC3_NEAR(s->linear_velocity, glm::vec3(0.0f), 1e-6f);
    ASSERT_VEC3_NEAR(s->angular_velocity, glm::vec3(0.0f), 1e-6f);
}

/**
 * Body::apply_impulse()/apply_angular_impulse() change velocity IMMEDIATELY (unlike
 * add_force()/add_torque(), which only take effect on the next integrate_forces() call) --
 * confirmed here with no step_fixed() call at all, so the change can only have come from the
 * impulse methods themselves, not the solver/integrator.
 */
static void test_body_impulse_and_angular_impulse_change_velocity_immediately() {
    PhysicsWorld world;
    world.set_gravity(glm::vec3(0.0f));
    dynamics::BodyId id = add_dynamic_sphere(world, glm::vec3(0.0f), 0.5f); // mass 1, inv_mass 1
    dynamics::Body* b = world.get_body(id);

    b->apply_impulse(glm::vec3(4.0f, 0.0f, 0.0f));
    ASSERT_VEC3_NEAR(b->linear_velocity, glm::vec3(4.0f, 0.0f, 0.0f), 1e-6f); // impulse * inv_mass

    glm::vec3 inv_i = b->inv_inertia_local; // sphere: isotropic, so world == local here (identity orientation)
    b->apply_angular_impulse(glm::vec3(0.0f, 0.0f, 2.0f));
    ASSERT_VEC3_NEAR(b->angular_velocity, glm::vec3(0.0f, 0.0f, 2.0f * inv_i.z), 1e-6f);
}

/**
 * add_force()/add_torque()/add_force_at_position()/apply_impulse()/apply_angular_impulse() all
 * wake a sleeping Dynamic body (matching Unity's AddForce) -- previously they silently
 * accumulated into a sleeping body's force_accum/torque_accum or wrote velocity directly, with
 * no effect until something ELSE woke the body (integrate_forces()/integrate_velocities() both
 * early-out on `!awake`). Confirmed two ways: the flag itself, and that a subsequent step
 * actually integrates the change (not just that `awake` reads true).
 */
static void test_body_force_and_impulse_apis_wake_a_sleeping_body() {
    PhysicsWorld world;
    world.set_gravity(glm::vec3(0.0f));

    auto sleeping_sphere = [&]() {
        dynamics::BodyId id = add_dynamic_sphere(world, glm::vec3(0.0f), 0.5f);
        dynamics::Body* b = world.get_body(id);
        b->awake = false;
        b->sleep_timer = 0.0f;
        return id;
    };

    dynamics::BodyId force_id = sleeping_sphere();
    world.get_body(force_id)->add_force(glm::vec3(1.0f, 0.0f, 0.0f));
    ASSERT_TRUE(world.get_body(force_id)->awake);

    dynamics::BodyId impulse_id = sleeping_sphere();
    world.get_body(impulse_id)->apply_impulse(glm::vec3(1.0f, 0.0f, 0.0f));
    ASSERT_TRUE(world.get_body(impulse_id)->awake);
    ASSERT_VEC3_NEAR(world.get_body(impulse_id)->linear_velocity, glm::vec3(1.0f, 0.0f, 0.0f), 1e-6f);

    // Functional check, not just the flag: a woken body actually integrates on the next step.
    world.step_fixed(util::k_default_fixed_dt);
    ASSERT_TRUE(world.get_body(impulse_id)->position.x > 0.0f);
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

/**
 * A box dropped at an oblique seed orientation must tumble on landing, then come fully to rest
 * flat on a face -- not freeze mid-topple, balanced on an edge/corner. Written while chasing a
 * report of exactly that in physics_test's stack; this single-box-on-flat-ground case passes
 * even without any fix (see test_three_box_concrete_stack_settles_flat's doc for how that
 * investigation actually concluded), but it's still a legitimate general regression to keep.
 */
static void test_tumbled_box_settles_flat_not_balanced_on_edge() {
    PhysicsWorld world;
    add_static_ground(world);

    // Tilted ~25 degrees about a horizontal axis so it lands corner/edge-first and must tumble,
    // rather than settling straight down already aligned with a face.
    glm::quat tilt = glm::angleAxis(glm::radians(25.0f), glm::normalize(glm::vec3(1.0f, 0.3f, 0.0f)));
    dynamics::BodyId id = add_dynamic_box(world, glm::vec3(0.0f, 0.0f, 2.0f), glm::vec3(0.5f), tilt);

    for (int i = 0; i < 600; ++i) world.step_fixed(util::k_default_fixed_dt); // 10s -- generous settle budget

    const dynamics::Body* b = world.get_body(id);
    bool settled = !b->awake || (glm::length(b->linear_velocity) < 0.02f && glm::length(b->angular_velocity) < 0.02f);
    ASSERT_TRUE(settled);

    // "Flat" means one of the box's local axes ends up within ~2 degrees of world +Z.
    glm::vec3 local_axes[3] = {
        b->orientation * glm::vec3(1.0f, 0.0f, 0.0f),
        b->orientation * glm::vec3(0.0f, 1.0f, 0.0f),
        b->orientation * glm::vec3(0.0f, 0.0f, 1.0f),
    };
    float best_alignment = 0.0f;
    for (const auto& axis : local_axes) best_alignment = std::max(best_alignment, std::abs(axis.z));
    ASSERT_TRUE(best_alignment > 0.999f); // cos(2 degrees) ~= 0.99939
}

/**
 * CCD (Phase 4 -- speculative contacts): a genuine tunneling reproduction. A small box (half-
 * extent 0.1) starts entirely clear of a thin static wall (half-extent 0.05, so 0.1 thick) and
 * moves at 50 m/s toward it -- one substep's displacement (50/60 ~= 0.833) is roughly 2.5x the
 * combined "skip zone" (box's own 0.2 full width + wall's 0.1 thickness = 0.3), so BEFORE this
 * substep integrates, the box's PRE-step pose ([4.4,4.6] on the approach axis) doesn't overlap
 * the wall ([4.95,5.05]), and its naive POST-step pose ([5.233,5.433]) doesn't either -- the box
 * would cross from entirely-before to entirely-after the wall within one substep with no
 * ordinary (non-speculative) contact ever generated to stop it. This is confirmed by hand
 * computation, not assumed: the ordinary narrowphase test at the pre-step pose provably finds no
 * overlap (4.6 < 4.95), so without the speculative path this substep produces zero manifolds and
 * the box would simply integrate straight through. The fast-pair gate must fire here (box's own
 * displacement 0.833 >> its own extent 0.1) and produce a speculative contact that clamps the
 * closing velocity before integration, stopping the box on the near side.
 */
static void test_fast_box_does_not_tunnel_through_thin_wall() {
    PhysicsWorld world;
    world.set_gravity(glm::vec3(0.0f)); // isolate the CCD effect from gravity

    dynamics::Body wall;
    wall.type = dynamics::BodyType::Static;
    wall.position = glm::vec3(5.0f, 0.0f, 0.0f);
    dynamics::BodyId wall_id = world.add_body(wall, collision::Shape::make_box(glm::vec3(0.05f, 2.0f, 2.0f)));

    dynamics::BodyId box_id = add_dynamic_box(world, glm::vec3(4.5f, 0.0f, 0.0f), glm::vec3(0.1f));
    world.get_body(box_id)->linear_velocity = glm::vec3(50.0f, 0.0f, 0.0f);

    world.step_fixed(util::k_default_fixed_dt);

    const dynamics::Body* box = world.get_body(box_id);
    // Naive (no-collision-response) integration would have put it at x ~= 5.333, past the
    // wall's far face (5.05) -- must not have crossed even the wall's CENTER, let alone its far
    // side, and must be markedly slower than the un-constrained 50 m/s it started with.
    ASSERT_TRUE(box->position.x < 5.0f);
    ASSERT_TRUE(box->linear_velocity.x < 50.0f);

    // Let it settle fully and confirm a physically sane final state: resting against the wall's
    // near face (4.95), not embedded past it, and eventually at rest.
    for (int i = 0; i < 300; ++i) world.step_fixed(util::k_default_fixed_dt);
    const dynamics::Body* settled = world.get_body(box_id);
    ASSERT_TRUE(settled->position.x < 4.951f); // near face at 4.95; small slop tolerance
    ASSERT_TRUE(glm::length(settled->linear_velocity) < 0.05f);
    (void)wall_id;
}

/**
 * The exact 3-box, "concrete"-material configuration physics_test's stack uses (scale 0.8 ->
 * half_extents 0.4, flush-stacked, dynamic/static friction 0.6/0.7, restitution 0.05), driven
 * directly at the headless PhysicsWorld level. Added while diagnosing a report that the stack
 * tumbles and then never fully settles (floats/balances on an edge) in the actual toyengine
 * scene: this isolated case settles perfectly flat and stays asleep, with full 4-point face
 * manifolds throughout -- which rules out physxcoopa's core solver/narrowphase as the cause and
 * points at the Scene/PhysicsSystem integration layer above it instead (see the plan this test
 * landed with for the investigation that led here).
 */
static void test_three_box_concrete_stack_settles_flat() {
    PhysicsWorld world;
    dynamics::PhysicsMaterial concrete;
    concrete.dynamic_friction = 0.6f;
    concrete.static_friction = 0.7f;
    concrete.restitution = 0.05f;
    add_static_box(world, glm::vec3(0.0f, 0.0f, -0.5f), glm::vec3(10.0f, 10.0f, 0.5f), glm::quat(1, 0, 0, 0), &concrete);

    const float half = 0.4f;
    std::vector<dynamics::BodyId> ids;
    for (int i = 0; i < 3; ++i) {
        glm::vec3 pos(0.0f, 0.0f, half * (2 * i + 1));
        ids.push_back(add_dynamic_box(world, pos, glm::vec3(half), glm::quat(1, 0, 0, 0), &concrete));
    }

    for (int i = 0; i < 600; ++i) world.step_fixed(util::k_default_fixed_dt); // 10s

    for (auto id : ids) {
        const dynamics::Body* b = world.get_body(id);
        ASSERT_TRUE(!b->awake);
        glm::vec3 up = b->orientation * glm::vec3(0.0f, 0.0f, 1.0f);
        ASSERT_TRUE(up.z > 0.999f); // flat, not balanced on an edge/corner
        ASSERT_TRUE(std::abs(b->position.x) < 0.01f);
        ASSERT_TRUE(std::abs(b->position.y) < 0.01f);
    }
}

static void test_diag_transform_roundtrip_noise() {
    coopa::util::Transform t;
    t.set_position(glm::vec3(9.4f, 6.4f, 0.79f));
    // An arbitrary oblique quaternion, representative of a tumbled box's resting orientation --
    // not axis-aligned, not a "nice" angle.
    glm::quat original = glm::normalize(glm::quat(0.8123f, 0.31f, -0.44f, 0.192f));
    t.set_rotation_quat(original);

    util::Trs trs = util::world_trs(t);

    glm::quat rot_diff = trs.rotation * glm::inverse(original);
    float rot_delta = 1.0f - std::abs(std::clamp(rot_diff.w, -1.0f, 1.0f));
    float pos_delta = glm::length(trs.position - glm::vec3(9.4f, 6.4f, 0.79f));

    std::cerr << "[DIAG] roundtrip pos_delta=" << pos_delta << " rot_delta=" << rot_delta << "\n";
    std::cerr << "[DIAG] original=(" << original.w << "," << original.x << "," << original.y << "," << original.z << ")\n";
    std::cerr << "[DIAG] roundtrip=(" << trs.rotation.w << "," << trs.rotation.x << "," << trs.rotation.y << "," << trs.rotation.z << ")\n";

    // Now simulate repeated per-frame round-trips through set_world_trs -> world_trs, as
    // sync_transforms_in_/write_transforms_back_ would do every frame for a body at rest,
    // to see whether the noise compounds or stays bounded.
    glm::vec3 pos = trs.position;
    glm::quat rot = trs.rotation;
    for (int i = 0; i < 300; ++i) {
        util::set_world_trs(t, pos, rot);
        util::Trs next = util::world_trs(t);
        glm::quat diff = next.rotation * glm::inverse(rot);
        float step_rot_delta = 1.0f - std::abs(std::clamp(diff.w, -1.0f, 1.0f));
        float step_pos_delta = glm::length(next.position - pos);
        if (i < 5 || step_rot_delta > 1e-5f || step_pos_delta > 1e-4f) {
            std::cerr << "[DIAG] step=" << i << " pos_delta=" << step_pos_delta << " rot_delta=" << step_rot_delta << "\n";
        }
        pos = next.position;
        rot = next.rotation;
    }
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

static void test_overlap_capsule_finds_expected_bodies() {
    PhysicsWorld world;
    dynamics::BodyId near_id = add_static_box(world, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.5f));
    add_static_box(world, glm::vec3(20.0f, 0.0f, 0.0f), glm::vec3(0.5f));

    geometry::Capsule query_capsule;
    query_capsule.a = glm::vec3(0.0f, 0.0f, -0.4f);
    query_capsule.b = glm::vec3(0.0f, 0.0f, 0.4f);
    query_capsule.radius = 0.3f;
    std::vector<dynamics::BodyId> hits = world.overlap_capsule(query_capsule);
    ASSERT_TRUE(hits.size() == 1);
    ASSERT_TRUE(hits[0].index == near_id.index);
}

static void test_raycast_any_finds_a_hit_without_necessarily_the_closest() {
    PhysicsWorld world;
    add_static_box(world, glm::vec3(3.0f, 0.0f, 0.0f), glm::vec3(0.5f));
    add_static_box(world, glm::vec3(6.0f, 0.0f, 0.0f), glm::vec3(0.5f));

    geometry::Ray hit_ray;
    hit_ray.origin = glm::vec3(0.0f, 0.0f, 0.0f);
    hit_ray.direction = glm::vec3(1.0f, 0.0f, 0.0f);
    hit_ray.max_distance = 100.0f;
    ASSERT_TRUE(world.raycast_any(hit_ray)); // correctness of the boolean, not traversal order

    geometry::Ray miss_ray;
    miss_ray.origin = glm::vec3(0.0f, 0.0f, 0.0f);
    miss_ray.direction = glm::vec3(-1.0f, 0.0f, 0.0f); // nothing behind the origin
    miss_ray.max_distance = 100.0f;
    ASSERT_TRUE(!world.raycast_any(miss_ray));
}

/**
 * `include_triggers` (default true, matching every query's behavior before this parameter
 * existed) lets a caller exclude trigger colliders -- previously impossible, every query always
 * saw every enabled shape regardless of is_trigger.
 */
static void test_query_include_triggers_flag_excludes_trigger_colliders() {
    PhysicsWorld world;
    dynamics::BodyId trigger_id = add_static_box(world, glm::vec3(0.0f), glm::vec3(0.5f));
    world.get_shape(trigger_id)->is_trigger = true;

    std::vector<dynamics::BodyId> with_triggers = world.overlap_sphere(glm::vec3(0.0f), 1.0f, ~0u, true);
    ASSERT_TRUE(with_triggers.size() == 1);

    std::vector<dynamics::BodyId> without_triggers = world.overlap_sphere(glm::vec3(0.0f), 1.0f, ~0u, false);
    ASSERT_TRUE(without_triggers.empty());
}

/**
 * box_cast()/capsule_cast()'s discretized-stepped-sweep approximation against a simple,
 * axis-aligned known target -- exact geometry, so the expected hit distance is hand-computed
 * (see each assertion's comment), not just "found something."
 */
static void test_box_cast_and_capsule_cast_hit_known_target() {
    PhysicsWorld world;
    dynamics::BodyId target_id = add_static_box(world, glm::vec3(5.0f, 0.0f, 0.0f), glm::vec3(0.5f));

    query::RaycastHit box_hit;
    bool box_found = world.box_cast(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.4f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                     glm::vec3(1.0f, 0.0f, 0.0f), 100.0f, box_hit);
    ASSERT_TRUE(box_found);
    // Casting box (half-extent 0.4, centered at origin+d) touches the target's near face (x=4.5)
    // when its own leading face reaches it: d + 0.4 == 4.5 -> d == 4.1.
    ASSERT_NEAR(box_hit.distance, 4.1f, 0.05f);
    ASSERT_TRUE(box_hit.body.index == target_id.index);

    query::RaycastHit capsule_hit;
    bool capsule_found = world.capsule_cast(glm::vec3(0.0f, 0.0f, 0.0f), 0.3f, 0.3f, /*direction_axis=*/2,
                                             glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f),
                                             100.0f, capsule_hit);
    ASSERT_TRUE(capsule_found);
    // Capsule's long axis is Z, so its extent along the +X cast direction is exactly `radius`:
    // touches the target's near face when d + 0.3 == 4.5 -> d == 4.2.
    ASSERT_NEAR(capsule_hit.distance, 4.2f, 0.05f);
    ASSERT_TRUE(capsule_hit.body.index == target_id.index);
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

/**
 * Shape::local_rotation -- a single (non-compound) collider's shape posed at an angle relative
 * to its own body, previously impossible (every world_*() helper applied the body's rotation
 * only). A capsule authored along local Z, rotated 90 degrees about X, actually points along
 * world -Y once instanced -- confirmed three independent ways (raycast, overlap, debug draw) so
 * a bug that only fixed one of the four world_*()/raycast_shape() call sites this touched would
 * still be caught.
 */
static void test_shape_local_rotation_reorients_capsule_consistently() {
    PhysicsWorld world;
    dynamics::Body body;
    body.type = dynamics::BodyType::Static;
    body.position = glm::vec3(0.0f);
    collision::Shape shape = collision::Shape::make_capsule(0.3f, 1.0f); // default axis Z, local_center 0
    shape.local_rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    dynamics::BodyId id = world.add_body(body, shape);

    // Rotating local Z (0,0,1) by +90 degrees about X maps it to world (0,-1,0) -- so the
    // capsule's actual endpoints are (0,1,0) and (0,-1,0), radius 0.3, entirely off the Z axis
    // it would occupy without local_rotation.
    const glm::vec3 expected_a(0.0f, 1.0f, 0.0f);
    const glm::vec3 expected_b(0.0f, -1.0f, 0.0f);

    // 1. Raycast: a ray straight down world -Y should hit the capsule's near end cap.
    geometry::Ray ray;
    ray.origin = glm::vec3(0.0f, 3.0f, 0.0f);
    ray.direction = glm::vec3(0.0f, -1.0f, 0.0f);
    ray.max_distance = 100.0f;
    query::RaycastHit hit;
    ASSERT_TRUE(world.raycast(ray, hit));
    ASSERT_NEAR(hit.distance, 3.0f - 1.3f, 1e-3f); // end cap surface at y = 1.0 + radius 0.3

    // A ray held at z=1.3 throughout (sweeping along Y, well clear of the origin) must miss --
    // the ACTUAL segment lies entirely in the z=0 plane (radius 0.3 << 1.3), so this only hits
    // if the shape were still sitting at its un-rotated Z-axis position (whose end cap sits
    // right around z=1.3). Deliberately not a ray straight down world -Z through the origin:
    // the rotated segment ALSO passes through the origin (it's centered there), so that ray
    // would hit either orientation and wouldn't distinguish them.
    geometry::Ray miss_ray;
    miss_ray.origin = glm::vec3(0.0f, 3.0f, 1.3f);
    miss_ray.direction = glm::vec3(0.0f, -1.0f, 0.0f);
    miss_ray.max_distance = 100.0f;
    query::RaycastHit miss_hit;
    ASSERT_TRUE(!world.raycast(miss_ray, miss_hit));

    // 2. Overlap: a small sphere at the actual (rotated) capsule center overlaps; the same
    // sphere at the UN-rotated capsule's would-be center (0,0,0.9) does not.
    ASSERT_TRUE(!world.overlap_sphere(glm::vec3(0.0f), 0.1f).empty());
    ASSERT_TRUE(world.overlap_sphere(glm::vec3(0.0f, 0.0f, 0.9f), 0.1f).empty());

    // 3. Debug draw: every emitted line endpoint for this capsule must lie within `radius` (plus
    // a small tolerance for the ring/cap approximation) of the ACTUAL rotated segment, not the
    // un-rotated one.
    debug::DebugDraw draw;
    world.debug_draw(draw, debug::DebugDrawFlags::Colliders);
    ASSERT_TRUE(!draw.lines.empty());
    for (const auto& line : draw.lines) {
        float dist_a = glm::length(line.a - geometry::closest_point_on_segment(line.a, expected_a, expected_b));
        float dist_b = glm::length(line.b - geometry::closest_point_on_segment(line.b, expected_a, expected_b));
        ASSERT_TRUE(dist_a < 0.35f);
        ASSERT_TRUE(dist_b < 0.35f);
    }
    (void)id;
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

/**
 * RigidbodyComponent's fields used to be read only once, at bind time (create_body_for_()) --
 * a runtime `rb->mass = 5.0f` or `rb->is_kinematic = true` after that had no effect, unlike
 * Collider's own parameters (the previous test), which already rode the per-frame revision
 * check. update_changed_rigidbodies_() closes that gap; this drives mass, use_gravity, and an
 * is_kinematic round-trip through the actual Scene/PhysicsSystem binding path.
 */
static void test_scene_binding_rigidbody_runtime_property_changes_take_effect() {
    using namespace coopa::scene;

    Scene scene("PhysicsRigidbodyReconcileTest");
    auto obj = std::make_unique<SceneObject>("sphere");
    obj->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 5.0f));
    obj->add_component<components::SphereCollider>()->set_radius(0.5f);
    auto* rb = obj->add_component<components::RigidbodyComponent>();
    scene.add_root_object(std::move(obj));

    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);
    scene.update(util::k_default_fixed_dt); // first execute() binds the body

    SceneObject* sphere_obj = scene.find_object("sphere");
    dynamics::BodyId id = sphere_obj->get_component<components::SphereCollider>()->body_id();
    ASSERT_TRUE(id.is_valid());
    const dynamics::Body* body = sys->world().get_body(id);
    ASSERT_NEAR(body->mass, 1.0f, 1e-5f);
    glm::vec3 inertia_at_mass_1 = body->inv_inertia_local;
    ASSERT_TRUE(body->use_gravity);
    ASSERT_TRUE(body->type == dynamics::BodyType::Dynamic);

    // Mass: inv_mass and inv_inertia_local must both be recomputed from the new value, not just
    // the mass field itself.
    rb->mass = 4.0f;
    rb->use_gravity = false;
    scene.update(util::k_default_fixed_dt);
    body = sys->world().get_body(id);
    ASSERT_NEAR(body->mass, 4.0f, 1e-5f);
    ASSERT_NEAR(body->inv_mass, 0.25f, 1e-5f);
    ASSERT_TRUE(body->inv_inertia_local.x < inertia_at_mass_1.x); // heavier -> smaller inverse inertia
    ASSERT_TRUE(!body->use_gravity);

    // is_kinematic: a BodyType flip, not just a scalar -- round-trip it and confirm mass/inertia
    // come back correctly on the way back to Dynamic (set_body_type()'s own responsibility).
    rb->is_kinematic = true;
    scene.update(util::k_default_fixed_dt);
    ASSERT_TRUE(sys->world().get_body(id)->type == dynamics::BodyType::Kinematic);

    rb->is_kinematic = false;
    scene.update(util::k_default_fixed_dt);
    body = sys->world().get_body(id);
    ASSERT_TRUE(body->type == dynamics::BodyType::Dynamic);
    ASSERT_NEAR(body->mass, 4.0f, 1e-5f);
    ASSERT_NEAR(body->inv_mass, 0.25f, 1e-5f);
}

/**
 * RigidbodyComponent::center_of_mass_override/inertia_tensor_override -- Unity's
 * Rigidbody.centerOfMass/Rigidbody.inertiaTensor. Checked directly against the bound Body right
 * after bind (no simulation needed): the override composes with any collider `center` (here
 * zero) via center_offset_for_(), and body.position/inv_inertia_local should reflect it exactly.
 */
static void test_rigidbody_center_of_mass_and_inertia_override_fold_into_body() {
    using namespace coopa::scene;

    Scene scene("PhysicsComOverrideTest");
    auto obj = std::make_unique<SceneObject>("box");
    obj->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 5.0f));
    obj->add_component<components::BoxCollider>()->set_size(glm::vec3(1.0f)); // centered box, no `center` offset
    auto* rb = obj->add_component<components::RigidbodyComponent>();
    rb->center_of_mass_override = glm::vec3(0.3f, 0.0f, 0.0f);
    rb->inertia_tensor_override = glm::vec3(9.0f, 9.0f, 9.0f); // deliberately far from the box's natural value
    scene.add_root_object(std::move(obj));

    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);
    sys->world().set_gravity(glm::vec3(0.0f)); // isolate the fold-in check from one substep of fall
    scene.update(util::k_default_fixed_dt);

    SceneObject* box_obj = scene.find_object("box");
    dynamics::BodyId id = box_obj->get_component<components::BoxCollider>()->body_id();
    ASSERT_TRUE(id.is_valid());
    const dynamics::Body* body = sys->world().get_body(id);

    // Pivot was (0,0,5), identity rotation -- true center should be pivot + override exactly.
    ASSERT_VEC3_NEAR(body->position, glm::vec3(0.3f, 0.0f, 5.0f), 1e-5f);
    ASSERT_VEC3_NEAR(body->inv_inertia_local, glm::vec3(9.0f, 9.0f, 9.0f), 1e-5f);
}

/**
 * RigidbodyComponent::sleep()/wake()/is_sleeping() and set_velocity()'s wake-on-write, exercised
 * through the component API (not Body directly, unlike the two headless tests earlier) --
 * confirms the delegation actually reaches the bound Body.
 */
static void test_rigidbody_component_sleep_wake_and_set_velocity_wakes() {
    using namespace coopa::scene;

    Scene scene("PhysicsRigidbodySleepWakeTest");
    auto obj = std::make_unique<SceneObject>("sphere");
    obj->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 5.0f));
    obj->add_component<components::SphereCollider>()->set_radius(0.5f);
    auto* rb = obj->add_component<components::RigidbodyComponent>();
    scene.add_root_object(std::move(obj));

    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);
    scene.update(util::k_default_fixed_dt);
    (void)sys;

    ASSERT_TRUE(!rb->is_sleeping()); // freshly bound bodies start awake

    rb->sleep();
    ASSERT_TRUE(rb->is_sleeping());
    ASSERT_VEC3_NEAR(rb->velocity(), glm::vec3(0.0f), 1e-6f);

    rb->wake();
    ASSERT_TRUE(!rb->is_sleeping());

    rb->sleep();
    ASSERT_TRUE(rb->is_sleeping());
    rb->set_velocity(glm::vec3(1.0f, 0.0f, 0.0f)); // must wake the body, not just set velocity
    ASSERT_TRUE(!rb->is_sleeping());
    ASSERT_VEC3_NEAR(rb->velocity(), glm::vec3(1.0f, 0.0f, 0.0f), 1e-6f);
}

/**
 * The exact scenario that caused physics_test's stack to tumble and never settle: a
 * corner-origin mesh (cube.000's [0,1]^3 local vertices) compensated for via
 * `BoxCollider center: {0.5,0.5,0.5}` on a dynamic Rigidbody. PhysicsSystem::create_body_for_()
 * used to leave body.position pinned to the Transform's raw pivot while the shape (and mass/
 * inertia) actually sat 0.5 units away in each axis -- every dynamics formula silently assumed
 * body.position WAS the center of mass, so gravity produced a persistent spurious torque about
 * the wrong point. Fixed by folding the collider's center offset into body.position as the true
 * center of mass (see create_body_for_()'s doc) and converting back to the pivot every frame in
 * write_transforms_back_(). Unlike test_tumbled_box_settles_flat_not_balanced_on_edge (which
 * passes even without any fix -- it has no center offset) and
 * test_three_box_concrete_stack_settles_flat (headless PhysicsWorld, no Scene/Collider::center()
 * involved), this test drives the bug through the actual Scene/PhysicsSystem binding path with a
 * nonzero collider center, which is the only place the bug ever lived.
 */
static void test_scene_binding_box_collider_center_offset_settles_flat() {
    using namespace coopa::scene;

    Scene scene("PhysicsCenterOffsetTest");

    auto ground = std::make_unique<SceneObject>("ground");
    ground->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, -0.5f));
    ground->add_component<components::BoxCollider>()->set_size(glm::vec3(8.0f, 8.0f, 1.0f));
    scene.add_root_object(std::move(ground));

    // Pivot ("Transform position") is the box's corner, matching cube.000's convention; the
    // collider's center offset re-centers the 1x1x1 shape onto that pivot exactly like
    // physics_test's stack_1/2/3 objects (BoxCollider size:{1,1,1} center:{0.5,0.5,0.5}).
    auto box = std::make_unique<SceneObject>("box");
    auto* transform = box->add_component<TransformComponent>();
    transform->transform().set_position(glm::vec3(0.0f, 0.0f, 2.5f));
    glm::quat tilt = glm::angleAxis(glm::radians(25.0f), glm::normalize(glm::vec3(1.0f, 0.3f, 0.0f)));
    transform->transform().set_rotation_quat(tilt);
    auto* box_collider = box->add_component<components::BoxCollider>();
    box_collider->set_size(glm::vec3(1.0f));
    box_collider->set_center(glm::vec3(0.5f));
    box->add_component<components::RigidbodyComponent>();
    scene.add_root_object(std::move(box));

    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);

    for (int i = 0; i < 600; ++i) { // 10s -- generous settle budget, matching the headless tests
        scene.update(util::k_default_fixed_dt);
        scene.late_update(util::k_default_fixed_dt);
    }

    SceneObject* box_obj = scene.find_object("box");
    ASSERT_TRUE(box_obj != nullptr);
    dynamics::BodyId id = box_obj->get_component<components::BoxCollider>()->body_id();
    const dynamics::Body* b = sys->world().get_body(id);
    ASSERT_TRUE(b != nullptr);

    bool settled = !b->awake || (glm::length(b->linear_velocity) < 0.02f && glm::length(b->angular_velocity) < 0.02f);
    ASSERT_TRUE(settled);

    // "Flat" means one of the box's local axes ends up within ~2 degrees of world +Z.
    glm::vec3 local_axes[3] = {
        b->orientation * glm::vec3(1.0f, 0.0f, 0.0f),
        b->orientation * glm::vec3(0.0f, 1.0f, 0.0f),
        b->orientation * glm::vec3(0.0f, 0.0f, 1.0f),
    };
    float best_alignment = 0.0f;
    for (const auto& axis : local_axes) best_alignment = std::max(best_alignment, std::abs(axis.z));
    ASSERT_TRUE(best_alignment > 0.999f); // cos(2 degrees) ~= 0.99939

    // Ground top at z=0; resting flat, the pivot (Transform position, NOT the center of mass)
    // must land back at z~=0 -- exactly cube.000's corner-origin convention -- confirming
    // write_transforms_back_() correctly converts true-center space back to the authored pivot.
    float pivot_z = box_obj->get_transform()->transform().position().z;
    ASSERT_NEAR(pivot_z, 0.0f, 0.05f);
}

/**
 * Phase 5 -- compound colliders: one Rigidbody (on the ROOT object, which deliberately has NO
 * Collider of its own -- exercises Binding::sync_owner's edge case) owning two CHILD
 * SceneObjects' BoxColliders, one bigger than the other, offset so they genuinely overlap in
 * one region. Verifies every piece create_compound_body_() is responsible for: composite mass
 * (volume-weighted split of the authored total), composite center of mass (hand-computed, not
 * just "some value"), individual raycast attribution to the correct CHILD collider (not just
 * the body), and that the two children's genuine geometric overlap never produces a
 * self-collision manifold.
 *
 * Geometry (root at world (0,0,5), no rotation):
 * - child_a: BoxCollider size {2,1,1} (half-extents 1,0.5,0.5), local offset (0,0,0) -- volume
 *   2.0, world center (0,0,5), spans x[-1,1] y[-0.5,0.5] z[4.5,5.5].
 * - child_b: BoxCollider size {1,1,1} (half-extents 0.5,0.5,0.5), local offset (0,0.7,0) --
 *   volume 1.0, world center (0,0.7,5), spans x[-0.5,0.5] y[0.2,1.2] z[4.5,5.5].
 * - Overlap region: x[-0.5,0.5] y[0.2,0.5] z[4.5,5.5] -- genuinely overlapping.
 * - Rigidbody mass = 3.0 -> volume-split masses 2.0/1.0 -> composite center of mass (local) =
 *   (2*(0,0,0) + 1*(0,0.7,0)) / 3 = (0, 0.23333, 0) -> world true center (0, 0.23333, 5).
 */
static void test_compound_collider_two_children_one_rigidbody() {
    using namespace coopa::scene;

    Scene scene("PhysicsCompoundColliderTest");

    auto root = std::make_unique<SceneObject>("compound_root");
    root->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 5.0f));
    auto* rb = root->add_component<components::RigidbodyComponent>();
    rb->mass = 3.0f;
    SceneObject* root_ptr = root.get();

    auto child_a = std::make_unique<SceneObject>("child_a");
    auto* child_a_tc = child_a->add_component<TransformComponent>();
    child_a_tc->set_parent_transform(&root_ptr->get_transform()->transform());
    child_a_tc->transform().set_position(glm::vec3(0.0f, 0.0f, 0.0f));
    auto* collider_a = child_a->add_component<components::BoxCollider>();
    collider_a->set_size(glm::vec3(2.0f, 1.0f, 1.0f));
    SceneObject* child_a_ptr = root_ptr->add_child(std::move(child_a));
    (void)child_a_ptr;

    auto child_b = std::make_unique<SceneObject>("child_b");
    auto* child_b_tc = child_b->add_component<TransformComponent>();
    child_b_tc->set_parent_transform(&root_ptr->get_transform()->transform());
    child_b_tc->transform().set_position(glm::vec3(0.0f, 0.7f, 0.0f));
    auto* collider_b = child_b->add_component<components::BoxCollider>();
    collider_b->set_size(glm::vec3(1.0f, 1.0f, 1.0f));
    root_ptr->add_child(std::move(child_b));

    scene.add_root_object(std::move(root));
    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);
    sys->world().set_gravity(glm::vec3(0.0f)); // isolate bind-time correctness from any fall
    scene.update(util::k_default_fixed_dt);

    // Both children share the SAME body, resolved via the primary collider (whichever one
    // regather_() picked -- root has no Collider of its own, so it's whichever child was
    // gathered first; either is fine, this test only needs them to AGREE).
    dynamics::BodyId id = collider_a->body_id();
    ASSERT_TRUE(id.is_valid());
    ASSERT_TRUE(collider_b->body_id() == id); // same compound body

    const dynamics::Body* body = sys->world().get_body(id);
    ASSERT_TRUE(body != nullptr);
    ASSERT_TRUE(body->type == dynamics::BodyType::Dynamic);

    // Composite mass: total, not just one child's.
    ASSERT_NEAR(body->mass, 3.0f, 1e-4f);
    ASSERT_NEAR(body->inv_mass, 1.0f / 3.0f, 1e-4f);

    // Composite center of mass -- hand-computed above, not a placeholder check.
    ASSERT_VEC3_NEAR(body->position, glm::vec3(0.0f, 0.23333f, 5.0f), 1e-3f);

    // Inertia composition produced something sane (positive, finite) on all three axes --
    // exact values aren't hand-verified here (see compose_mass_properties()'s own doc for the
    // diagonal-only approximation), just that composition didn't degenerate to zero/garbage.
    ASSERT_TRUE(body->inv_inertia_local.x > 0.0f && body->inv_inertia_local.y > 0.0f &&
                body->inv_inertia_local.z > 0.0f);

    // Raycast attribution: a point unambiguously inside ONLY child_a, and one unambiguously
    // inside ONLY child_b (see the geometry comment above) -- each must resolve to the
    // correct, DISTINCT Collider*, not just "the compound body somehow got hit."
    system::PhysicsSystem::RaycastHit hit_a;
    geometry::Ray ray_a;
    ray_a.origin = glm::vec3(0.8f, 0.0f, 10.0f);
    ray_a.direction = glm::vec3(0.0f, 0.0f, -1.0f);
    ray_a.max_distance = 100.0f;
    ASSERT_TRUE(sys->raycast(ray_a, hit_a));
    ASSERT_TRUE(hit_a.collider == collider_a);

    system::PhysicsSystem::RaycastHit hit_b;
    geometry::Ray ray_b;
    ray_b.origin = glm::vec3(0.0f, 1.0f, 10.0f);
    ray_b.direction = glm::vec3(0.0f, 0.0f, -1.0f);
    ray_b.max_distance = 100.0f;
    ASSERT_TRUE(sys->raycast(ray_b, hit_b));
    ASSERT_TRUE(hit_b.collider == collider_b);

    // No self-collision: child_a and child_b genuinely overlap geometrically, but they're
    // siblings of the SAME body -- narrowphase must never produce a manifold with both sides
    // resolving to this body (discover_pairs_()'s owner-based self-rejection, see its doc).
    for (int i = 0; i < 5; ++i) scene.update(util::k_default_fixed_dt);
    for (const auto& m : sys->world().manifolds()) {
        ASSERT_TRUE(!(m.a == id && m.b == id));
    }
}

/**
 * The full YAML-authoring-equivalent path: a HingeJointComponent on a "door" object, naming a
 * static "frame" object via `connected_object`, with only ONE anchor/axis authored (in the
 * door's own local frame) -- regather_joints_()/create_hinge_joint_for_() must resolve the
 * frame side automatically (Unity's autoConfigureConnectedAnchor default, see
 * HingeJointComponent's own doc) and produce the same physically correct swing-to-limit
 * behavior the headless test_hinge_joint_door_swings_and_stops_at_limit() already verified
 * directly against PhysicsWorld -- this test instead exercises the Scene/PhysicsSystem binding
 * layer on top of it (component gather, connected_object name resolution, Collider->BodyId
 * lookup), the part a hand-built PhysicsWorld test can't reach at all.
 */
static void test_scene_binding_hinge_joint_door_resolves_and_swings_to_limit() {
    using namespace coopa::scene;

    Scene scene("PhysicsHingeJointTest");

    auto frame = std::make_unique<SceneObject>("frame");
    frame->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 0.0f));
    frame->add_component<components::BoxCollider>()->set_size(glm::vec3(0.2f, 2.0f, 2.0f));
    scene.add_root_object(std::move(frame));

    auto door = std::make_unique<SceneObject>("door");
    door->add_component<TransformComponent>()->transform().set_position(glm::vec3(1.0f, 0.0f, 0.0f));
    door->add_component<components::BoxCollider>()->set_size(glm::vec3(2.0f, 1.0f, 0.2f));
    auto* rb = door->add_component<components::RigidbodyComponent>();
    rb->initial_angular_velocity = glm::vec3(0.0f, 0.0f, 5.0f); // fast push, well past the limit if unconstrained
    auto* hinge = door->add_component<components::HingeJointComponent>();
    hinge->connected_object = "frame";
    hinge->anchor = glm::vec3(-1.0f, 0.0f, 0.0f); // door's own hinge edge, in the door's local frame
    hinge->axis = glm::vec3(0.0f, 0.0f, 1.0f);
    hinge->use_limits = true;
    hinge->min_angle_deg = 0.0f;
    hinge->max_angle_deg = 90.0f;
    scene.add_root_object(std::move(door));

    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);
    sys->world().set_gravity(glm::vec3(0.0f));

    for (int i = 0; i < 300; ++i) { // 5s -- generous settle budget, matching the headless version
        scene.update(util::k_default_fixed_dt);
        scene.late_update(util::k_default_fixed_dt);
    }

    SceneObject* door_obj = scene.find_object("door");
    ASSERT_TRUE(door_obj != nullptr);
    auto* hinge_comp = door_obj->get_component<components::HingeJointComponent>();
    ASSERT_TRUE(hinge_comp != nullptr);
    dynamics::JointId joint_id = hinge_comp->joint_id();
    ASSERT_TRUE(joint_id.is_valid());

    const dynamics::HingeJoint* j = sys->world().get_joint(joint_id);
    ASSERT_TRUE(j != nullptr && j->valid);
    const dynamics::Body* frame_body = sys->world().get_body(j->a);
    const dynamics::Body* door_body = sys->world().get_body(j->b);
    ASSERT_TRUE(frame_body != nullptr && door_body != nullptr);

    float angle = dynamics::hinge_current_angle(frame_body->orientation, door_body->orientation, j->local_axis_a,
                                                 j->rest_relative_rotation);
    ASSERT_NEAR(angle, glm::radians(90.0f), glm::radians(3.0f));
    ASSERT_TRUE(glm::length(door_body->angular_velocity) < 0.05f);

    glm::vec3 world_anchor_a = frame_body->position + frame_body->orientation * j->local_anchor_a;
    glm::vec3 world_anchor_b = door_body->position + door_body->orientation * j->local_anchor_b;
    ASSERT_NEAR(glm::length(world_anchor_b - world_anchor_a), 0.0f, 0.05f);
}

static void test_collision_event_carries_contact_payload_and_clears_on_exit() {
    using namespace coopa::scene;

    Scene scene("PhysicsCollisionPayloadTest");

    auto ground = std::make_unique<SceneObject>("ground");
    ground->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, -0.5f));
    ground->add_component<components::BoxCollider>()->set_size(glm::vec3(8.0f, 8.0f, 1.0f));
    scene.add_root_object(std::move(ground));

    auto sphere_obj = std::make_unique<SceneObject>("sphere");
    sphere_obj->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 3.0f));
    auto* sphere_collider = sphere_obj->add_component<components::SphereCollider>();
    sphere_collider->set_radius(0.5f);
    auto bouncy = std::make_shared<dynamics::PhysicsMaterial>();
    bouncy->restitution = 0.9f;
    sphere_collider->set_material(bouncy);
    sphere_obj->add_component<components::RigidbodyComponent>();
    scene.add_root_object(std::move(sphere_obj));

    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);

    int enter_count = 0, exit_count = 0;
    bool enter_had_contacts = false, enter_had_impulse = false;
    float enter_normal_z = 0.0f;
    bool exit_had_zero_contacts = true;

    sphere_collider->on_collision_enter.connect([&](components::Collider&, const components::Collision& c) {
        ++enter_count;
        enter_had_contacts = c.contact_count > 0;
        enter_had_impulse = glm::length(c.impulse) > 0.0f;
        enter_normal_z = c.normal.z;
        ASSERT_TRUE(c.collider != nullptr);
        ASSERT_TRUE(c.object != nullptr && c.object->name() == "ground");
    });
    sphere_collider->on_collision_exit.connect([&](components::Collider&, const components::Collision& c) {
        ++exit_count;
        if (c.contact_count != 0) exit_had_zero_contacts = false;
    });

    for (int i = 0; i < 300; ++i) {
        scene.update(util::k_default_fixed_dt);
        scene.late_update(util::k_default_fixed_dt);
    }

    ASSERT_TRUE(enter_count >= 1);
    ASSERT_TRUE(exit_count >= 1); // combined restitution ~0.45 must separate at least once
    ASSERT_TRUE(enter_had_contacts);
    ASSERT_TRUE(enter_had_impulse);
    ASSERT_TRUE(enter_normal_z < 0.0f); // points from sphere (self) down toward the ground (other)
    ASSERT_TRUE(exit_had_zero_contacts);
    (void)sys;
}

static void test_scene_query_wrappers_resolve_collider_and_object() {
    using namespace coopa::scene;

    Scene scene("PhysicsQueryTest");
    auto obj = std::make_unique<SceneObject>("target");
    obj->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 0.0f));
    obj->add_component<components::SphereCollider>()->set_radius(1.0f);
    scene.add_root_object(std::move(obj));

    scene.start();
    system::PhysicsSystem* sys = system::install_physics_system(scene);
    scene.update(util::k_default_fixed_dt); // first execute() binds the body

    geometry::Ray ray;
    ray.origin = glm::vec3(0.0f, 0.0f, 5.0f);
    ray.direction = glm::vec3(0.0f, 0.0f, -1.0f);
    ray.max_distance = 100.0f;

    system::PhysicsSystem::RaycastHit hit;
    ASSERT_TRUE(sys->raycast(ray, hit));
    ASSERT_TRUE(hit.collider != nullptr);
    ASSERT_TRUE(hit.object != nullptr && hit.object->name() == "target");
    ASSERT_NEAR(hit.point.z, 1.0f, 1e-4f);

    auto overlapping = sys->overlap_sphere(glm::vec3(0.0f), 1.5f);
    ASSERT_TRUE(overlapping.size() == 1);
    ASSERT_TRUE(overlapping[0] == hit.collider);
}

// --- Materials, settings, job-parallel determinism ---

static void test_physics_material_combine_modes() {
    dynamics::PhysicsMaterial a;
    a.dynamic_friction = 0.2f;
    a.restitution = 0.1f;
    dynamics::PhysicsMaterial b;
    b.dynamic_friction = 0.8f;
    b.restitution = 0.9f;

    // Priority order is Multiply > Maximum > Minimum > Average -- the higher-priority mode of
    // the two wins, regardless of which side (a or b) specified it.
    ASSERT_NEAR(dynamics::combine(a.dynamic_friction, b.dynamic_friction,
                                   dynamics::CombineMode::Average, dynamics::CombineMode::Average), 0.5f, 1e-5f);
    ASSERT_NEAR(dynamics::combine(a.dynamic_friction, b.dynamic_friction,
                                   dynamics::CombineMode::Minimum, dynamics::CombineMode::Average), 0.2f, 1e-5f);
    ASSERT_NEAR(dynamics::combine(a.restitution, b.restitution,
                                   dynamics::CombineMode::Maximum, dynamics::CombineMode::Average), 0.9f, 1e-5f);
    ASSERT_NEAR(dynamics::combine(a.restitution, b.restitution,
                                   dynamics::CombineMode::Multiply, dynamics::CombineMode::Maximum), 0.1f * 0.9f, 1e-5f);
}

static void test_collider_material_resolution_prefers_asset_over_inline() {
    components::BoxCollider collider;
    ASSERT_TRUE(collider.material() == nullptr); // unset -> caller falls back to default_material()

    auto inline_mat = std::make_shared<dynamics::PhysicsMaterial>();
    inline_mat->restitution = 0.42f;
    collider.set_material(inline_mat);
    ASSERT_TRUE(collider.material() == inline_mat.get());

    // An asset handle, once loaded, takes priority over the inline material -- set up a real
    // AssetManager + PhysicsMaterialLoader + on-disk material so this exercises the actual
    // scene-authoring path (`material: <name>`), not just the accessor logic.
    namespace fs = std::filesystem;
    fs::path dir = fs::temp_directory_path() / "physxcoopa_test_materials";
    fs::create_directories(dir);
    fs::path file = dir / "ice.yaml";
    {
        std::ofstream out(file);
        out << "dynamic_friction: 0.02\n"
               "static_friction: 0.03\n"
               "restitution: 0.0\n"
               "friction_combine: Minimum\n"
               "restitution_combine: Average\n";
    }

    coopa::asset::AssetManager assets;
    assets.register_loader<dynamics::PhysicsMaterial>(std::make_unique<loaders::PhysicsMaterialLoader>());
    assets.add_search_root(dir.string());

    auto handle = assets.load<dynamics::PhysicsMaterial>("ice.yaml");
    ASSERT_TRUE(handle.is_loaded());
    ASSERT_NEAR(handle->dynamic_friction, 0.02f, 1e-6f);
    ASSERT_NEAR(handle->static_friction, 0.03f, 1e-6f);
    ASSERT_TRUE(handle->friction_combine == dynamics::CombineMode::Minimum);
    ASSERT_TRUE(handle->restitution_combine == dynamics::CombineMode::Average);

    collider.set_material_asset(handle);
    ASSERT_TRUE(collider.material() == handle.get());        // asset now wins over the inline material
    ASSERT_NEAR(collider.material()->dynamic_friction, 0.02f, 1e-6f);

    fs::remove_all(dir);
}

static void test_parse_physics_settings_and_apply_to_world() {
    std::istringstream iss(
        "gravity: { x: 0.0, y: 0.0, z: -3.0 }\n"
        "fixed_timestep: 0.02\n"
        "solver:\n"
        "  velocity_iterations: 4\n"
        "layers: [\"Default\", \"Ground\", \"Triggers\"]\n"
        "ignore_layer_collisions:\n"
        "  - [Triggers, Triggers]\n"
        "parallel_threshold: 8\n");
    fkyaml::node node = fkyaml::node::deserialize(iss);
    util::PhysicsSettings settings = util::parse_physics_settings(node);

    ASSERT_NEAR(settings.gravity.z, -3.0f, 1e-6f);
    ASSERT_NEAR(settings.solver.fixed_dt, 0.02f, 1e-6f);
    ASSERT_TRUE(settings.solver.velocity_iterations == 4);
    ASSERT_TRUE(settings.layer_names.size() == 3);
    ASSERT_TRUE(settings.parallel_threshold == 8);
    ASSERT_TRUE(settings.ignore_pairs.size() == 1);
    ASSERT_TRUE(settings.ignore_pairs[0].first == 2 && settings.ignore_pairs[0].second == 2); // "Triggers" == index 2

    PhysicsWorld world;
    apply_physics_settings(world, settings);
    ASSERT_NEAR(world.gravity().z, -3.0f, 1e-6f);
    ASSERT_TRUE(world.config().velocity_iterations == 4);
    ASSERT_TRUE(!world.layers().should_collide(2, 2));  // Triggers vs Triggers turned off
    ASSERT_TRUE(world.layers().should_collide(1, 2));   // untouched pair still collides
}

// --- Joints ---

/**
 * A door: a static frame (world.add_body() with no shape -- Shape::enabled's default, "no
 * collision, just a fixed anchor") hinged to a dynamic panel via a vertical (Z) axis, with a
 * 90-degree swing limit. Given a fast initial spin (not gravity -- a vertical-axis door has no
 * gravity torque about its own hinge, matching real doors), it must swing and come to rest AT
 * the limit, not past it, while the point and axis constraints hold throughout the fast swing.
 */
static void test_hinge_joint_door_swings_and_stops_at_limit() {
    PhysicsWorld world;
    world.set_gravity(glm::vec3(0.0f));

    dynamics::Body frame_body;
    frame_body.type = dynamics::BodyType::Static;
    frame_body.position = glm::vec3(0.0f);
    dynamics::BodyId frame_id = world.add_body(frame_body);

    dynamics::Body door_body;
    door_body.position = glm::vec3(1.0f, 0.0f, 0.0f); // door's own center, 1 unit from the hinge
    door_body.angular_velocity = glm::vec3(0.0f, 0.0f, 5.0f); // fast push, well past the limit if unconstrained
    dynamics::BodyId door_id = world.add_body(door_body);

    // local_anchor_b = (-1,0,0): offset from the door's own center back to the hinge point, so
    // world_anchor_b starts exactly at world_anchor_a (0,0,0), matching the frame's anchor.
    dynamics::JointId joint = world.add_hinge_joint(
        frame_id, door_id, glm::vec3(0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, 1.0f), /*use_limits=*/true, /*min_angle=*/0.0f, /*max_angle=*/glm::radians(90.0f));
    ASSERT_TRUE(joint.is_valid());

    for (int i = 0; i < 300; ++i) world.step_fixed(util::k_default_fixed_dt); // 5s -- generous settle budget

    const dynamics::Body* frame = world.get_body(frame_id);
    const dynamics::Body* door = world.get_body(door_id);
    const dynamics::HingeJoint* j = world.get_joint(joint);
    ASSERT_TRUE(j != nullptr);

    // Point constraint held: the door's hinge edge is still at the frame's anchor.
    glm::vec3 world_anchor_a = frame->position + frame->orientation * j->local_anchor_a;
    glm::vec3 world_anchor_b = door->position + door->orientation * j->local_anchor_b;
    ASSERT_NEAR(glm::length(world_anchor_b - world_anchor_a), 0.0f, 0.02f);

    // Axis constraint held: the door didn't tip out of its horizontal swing plane.
    glm::vec3 world_axis_b = glm::normalize(door->orientation * j->local_axis_b);
    ASSERT_NEAR(world_axis_b.z, 1.0f, 0.02f);

    // Stopped AT the limit, not past it, and settled (not still spinning against the stop).
    float angle = dynamics::hinge_current_angle(frame->orientation, door->orientation, j->local_axis_a, j->rest_relative_rotation);
    ASSERT_NEAR(angle, glm::radians(90.0f), glm::radians(3.0f));
    ASSERT_TRUE(glm::length(door->angular_velocity) < 0.05f);
}

/**
 * A free (no limits) hinge -- a pendulum hanging from a static ceiling anchor, swinging under
 * gravity about a horizontal (Y) axis. Confirms the point stays anchored and the axis stays
 * aligned throughout real, sustained swinging motion (not just at rest), and that it actually
 * DOES swing (gravity produces real torque through the joint, not a rigid lock).
 */
static void test_hinge_joint_free_pendulum_swings_without_drift() {
    PhysicsWorld world; // default gravity -Z

    dynamics::Body ceiling_body;
    ceiling_body.type = dynamics::BodyType::Static;
    ceiling_body.position = glm::vec3(0.0f);
    dynamics::BodyId ceiling_id = world.add_body(ceiling_body);

    dynamics::Body pendulum_body;
    pendulum_body.position = glm::vec3(1.0f, 0.0f, 0.0f); // hangs out horizontally at first
    dynamics::BodyId pendulum_id = world.add_body(pendulum_body);

    dynamics::JointId joint = world.add_hinge_joint(ceiling_id, pendulum_id, glm::vec3(0.0f),
                                                      glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                                                      glm::vec3(0.0f, 1.0f, 0.0f), /*use_limits=*/false);
    ASSERT_TRUE(joint.is_valid());

    float start_z = world.get_body(pendulum_id)->position.z;
    float max_drift = 0.0f;
    float min_axis_alignment = 1.0f;
    for (int i = 0; i < 120; ++i) { // 2s -- enough to swing well away from the horizontal start
        world.step_fixed(util::k_default_fixed_dt);
        const dynamics::Body* ceiling = world.get_body(ceiling_id);
        const dynamics::Body* pendulum = world.get_body(pendulum_id);
        const dynamics::HingeJoint* j = world.get_joint(joint);
        glm::vec3 world_anchor_a = ceiling->position + ceiling->orientation * j->local_anchor_a;
        glm::vec3 world_anchor_b = pendulum->position + pendulum->orientation * j->local_anchor_b;
        max_drift = std::max(max_drift, glm::length(world_anchor_b - world_anchor_a));

        glm::vec3 world_axis_a = glm::normalize(ceiling->orientation * j->local_axis_a);
        glm::vec3 world_axis_b = glm::normalize(pendulum->orientation * j->local_axis_b);
        min_axis_alignment = std::min(min_axis_alignment, glm::dot(world_axis_a, world_axis_b));
    }

    ASSERT_TRUE(max_drift < 0.05f); // point constraint never drifted meaningfully, even mid-swing
    ASSERT_TRUE(min_axis_alignment > 0.98f); // axes stayed close to parallel throughout (cos ~11 degrees)

    // Actually swung: gravity pulled it down and away from its purely-horizontal start.
    float end_z = world.get_body(pendulum_id)->position.z;
    ASSERT_TRUE(end_z < start_z - 0.3f);
}

/**
 * `min_angle == max_angle == 0` locks the hinge's one remaining DOF entirely -- a fully rigid
 * attachment with no separate "fixed joint" implementation (see joint.h's file doc). A hard
 * spin plus gravity torque (both trying to rotate the panel) must produce ~zero net rotation.
 */
static void test_hinge_joint_zero_range_limit_acts_rigid() {
    PhysicsWorld world;
    world.set_gravity(glm::vec3(0.0f, 0.0f, -9.81f));

    dynamics::Body frame_body;
    frame_body.type = dynamics::BodyType::Static;
    frame_body.position = glm::vec3(0.0f);
    dynamics::BodyId frame_id = world.add_body(frame_body);

    dynamics::Body panel_body;
    panel_body.position = glm::vec3(1.0f, 0.0f, 0.0f);
    panel_body.angular_velocity = glm::vec3(0.0f, 0.0f, 5.0f); // hard spin about the (rigidly locked) hinge axis
    dynamics::BodyId panel_id = world.add_body(panel_body);

    // Hinge axis Z (spin is about the locked axis); gravity (-Z) has no torque about a
    // vertical axis here either, so this specifically isolates whether the ANGULAR SPIN gets
    // absorbed -- gravity is still on to confirm the point constraint holds a hanging weight too.
    dynamics::JointId joint = world.add_hinge_joint(
        frame_id, panel_id, glm::vec3(0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 0.0f, 1.0f), /*use_limits=*/true, /*min_angle=*/0.0f, /*max_angle=*/0.0f);
    ASSERT_TRUE(joint.is_valid());

    for (int i = 0; i < 120; ++i) world.step_fixed(util::k_default_fixed_dt);

    const dynamics::Body* frame = world.get_body(frame_id);
    const dynamics::Body* panel = world.get_body(panel_id);
    const dynamics::HingeJoint* j = world.get_joint(joint);
    float angle = dynamics::hinge_current_angle(frame->orientation, panel->orientation, j->local_axis_a, j->rest_relative_rotation);
    ASSERT_NEAR(angle, 0.0f, glm::radians(3.0f));

    // Point constraint still holds the panel up against gravity.
    glm::vec3 world_anchor_a = frame->position + frame->orientation * j->local_anchor_a;
    glm::vec3 world_anchor_b = panel->position + panel->orientation * j->local_anchor_b;
    ASSERT_NEAR(glm::length(world_anchor_b - world_anchor_a), 0.0f, 0.05f);
}

/**
 * Two DYNAMIC bodies joined only by a hinge (no contact between them) must island-unite for
 * sleep purposes, same as two bodies touching via a contact manifold already do -- see
 * update_islands_and_sleep()'s doc. Body A starts completely at rest (would satisfy its OWN
 * sleep_timer within sleep_time on its own if NOT unioned with B); body B starts with a fast
 * spin that easily outlasts sleep_time. If the union-find correctly includes joint pairs, A
 * must NOT have gone to sleep by the time B is still clearly moving.
 */
static void test_hinge_joint_islands_dynamic_pair_for_sleep() {
    PhysicsWorld world;
    world.set_gravity(glm::vec3(0.0f));

    dynamics::Body a_body;
    a_body.position = glm::vec3(0.0f);
    dynamics::BodyId a_id = world.add_body(a_body); // completely at rest from frame 0

    dynamics::Body b_body;
    b_body.position = glm::vec3(1.0f, 0.0f, 0.0f);
    b_body.angular_velocity = glm::vec3(0.0f, 0.0f, 2.0f); // spins about the hinge axis, only
                                                             // damped by angular_drag -- stays
                                                             // well above sleep_angular for a
                                                             // long time (slow exponential decay)
    dynamics::BodyId b_id = world.add_body(b_body);

    dynamics::JointId joint = world.add_hinge_joint(a_id, b_id, glm::vec3(0.0f), glm::vec3(-1.0f, 0.0f, 0.0f),
                                                      glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f),
                                                      /*use_limits=*/false);
    ASSERT_TRUE(joint.is_valid());

    // sleep_time defaults to 0.5s (30 steps) -- run well past that. B is still clearly moving
    // (angular_drag's exponential decay from 2.0 rad/s takes far longer than this to reach
    // sleep_angular=0.02), so if A were sleeping independently it would have slept by now.
    for (int i = 0; i < 90; ++i) world.step_fixed(util::k_default_fixed_dt); // 1.5s
    ASSERT_TRUE(glm::length(world.get_body(b_id)->angular_velocity) > 0.5f); // confirm B really is still moving
    ASSERT_TRUE(world.get_body(a_id)->awake); // A must still be awake -- gated by B via the shared island
}

/**
 * @brief The determinism argument PhysicsWorld::discover_pairs_()/narrowphase_()'s docs make:
 *        a job-parallel run must produce a bit-identical world_state_hash() to a fully serial
 *        run of the same scene. Forces the parallel path on every stage (parallel_threshold(1))
 *        so even this modest body count exercises it, rather than silently falling back serial.
 */
static void test_job_parallel_stepping_matches_serial_determinism() {
    auto build_and_run = [](coopa::job::JobEngine* jobs) {
        PhysicsWorld world;
        world.set_parallel_threshold(1);
        world.set_job_engine(jobs);

        add_static_ground(world);

        std::mt19937 rng(1234);
        std::uniform_real_distribution<float> xy(-3.0f, 3.0f);
        std::uniform_real_distribution<float> zh(2.0f, 8.0f);
        for (int i = 0; i < 80; ++i) {
            dynamics::Body b;
            b.type = dynamics::BodyType::Dynamic;
            b.position = glm::vec3(xy(rng), xy(rng), zh(rng));
            b.mass = 1.0f;
            b.inv_mass = 1.0f;
            b.inv_inertia_local = dynamics::sphere_inverse_inertia(0.4f, 1.0f);
            world.add_body(b, collision::Shape::make_sphere(0.4f));
        }

        for (int i = 0; i < 240; ++i) world.step_fixed(util::k_default_fixed_dt);
        return world.world_state_hash();
    };

    uint64_t serial_hash = build_and_run(nullptr);

    coopa::job::JobEngine jobs(4);
    uint64_t parallel_hash = build_and_run(&jobs);

    ASSERT_TRUE(serial_hash == parallel_hash);
}

// --- Phase 11: cloth ---

/** @brief Builds a 4x4 m sheet centred at `center`, with the given resolution and params. */
static cloth::Cloth make_test_sheet(uint32_t res, const glm::vec3& center,
                                    const cloth::ClothParams& params = {}) {
    cloth::GridClothDesc desc;
    desc.columns = res;
    desc.rows = res;
    desc.width = 4.0f;
    desc.height = 4.0f;
    desc.center = center;
    desc.total_mass = 1.0f;
    desc.params = params;
    return cloth::make_grid_cloth(desc);
}

/**
 * @brief Grid topology counts, plus the property the whole parallel story rests on: every
 *        ConstraintBatch's particle set is pairwise disjoint. If that ever regresses, a
 *        job-parallel dispatcher would silently produce nondeterministic results rather than
 *        crashing, so it is asserted structurally here rather than being left to a race.
 */
static void test_cloth_grid_builder_topology_and_disjoint_batches() {
    const uint32_t res = 9;
    cloth::Cloth c = make_test_sheet(res, glm::vec3(0.0f, 0.0f, 2.0f));

    ASSERT_TRUE(c.particles.size() == res * res);
    ASSERT_TRUE(c.columns == res && c.rows == res);
    // (res-1)*res horizontal + the same vertical + two diagonals per cell.
    const std::size_t structural = 2u * (res - 1) * res;
    const std::size_t shear = 2u * (res - 1) * (res - 1);
    ASSERT_TRUE(c.stretch.size() == structural + shear);
    ASSERT_TRUE(c.bend.size() == 2u * (res - 2) * res);
    ASSERT_TRUE(c.triangles.size() == (res - 1) * (res - 1) * 6);

    // Rest lengths must match the authored spacing exactly (4 m over res-1 gaps).
    const float spacing = 4.0f / static_cast<float>(res - 1);
    ASSERT_NEAR(c.stretch[0].rest_length, spacing, 1e-5f);

    auto batches_disjoint = [](const std::vector<cloth::ClothConstraint>& list,
                               const std::vector<cloth::ConstraintBatch>& batches,
                               std::size_t particle_count) {
        for (const cloth::ConstraintBatch& b : batches) {
            std::vector<bool> seen(particle_count, false);
            for (uint32_t i = b.begin; i < b.end; ++i) {
                if (seen[list[i].a] || seen[list[i].b]) return false;
                seen[list[i].a] = true;
                seen[list[i].b] = true;
            }
        }
        return true;
    };
    ASSERT_TRUE(batches_disjoint(c.stretch, c.stretch_batches, c.particles.size()));
    ASSERT_TRUE(batches_disjoint(c.bend, c.bend_batches, c.particles.size()));

    // Every constraint must land in exactly one batch -- a gap would silently stop solving part
    // of the sheet, which no visual check would obviously catch.
    std::size_t covered = 0;
    for (const cloth::ConstraintBatch& b : c.stretch_batches) covered += b.end - b.begin;
    ASSERT_TRUE(covered == c.stretch.size());
    covered = 0;
    for (const cloth::ConstraintBatch& b : c.bend_batches) covered += b.end - b.begin;
    ASSERT_TRUE(covered == c.bend.size());
}

/** @brief An unpinned sheet in free space falls exactly 0.5*g*t^2 -- the integrator sanity check,
 *         mirroring test_free_fall_matches_analytic() for rigid bodies. Constraints are all
 *         satisfied at rest, so they contribute nothing and the analytic result holds exactly. */
static void test_cloth_free_fall_matches_analytic() {
    PhysicsWorld world;
    // Zero the two things that would otherwise (correctly) break the analytic result: velocity
    // damping, and the aerodynamic drag/lift that are on by default -- a horizontal sheet falling
    // flat presents maximum area to the flow, so default air_drag alone slows this fall by ~27%.
    cloth::ClothParams params;
    params.damping = 0.0f;
    params.air_drag = 0.0f;
    params.air_lift = 0.0f;
    cloth::Cloth c = make_test_sheet(5, glm::vec3(0.0f, 0.0f, 10.0f), params);
    const glm::vec3 start = c.particles[0].position;
    cloth::ClothId id = world.add_cloth(std::move(c));

    const float h = util::k_default_fixed_dt;
    const int steps = 60;
    for (int i = 0; i < steps; ++i) world.step_fixed(h);

    const cloth::Cloth* sim = world.get_cloth(id);
    const float t = static_cast<float>(steps) * h;
    // Semi-implicit Euler over N substeps of hs accumulates 0.5*g*t^2 + 0.5*g*t*hs.
    const float hs = h / static_cast<float>(world.config().cloth_substeps);
    const float expected_drop = 0.5f * 9.81f * t * t + 0.5f * 9.81f * t * hs;
    ASSERT_NEAR(start.z - sim->particles[0].position.z, expected_drop, 0.05f);
    // Damping is the only thing acting laterally, and there is none: X/Y must not drift at all.
    ASSERT_NEAR(sim->particles[0].position.x, start.x, 1e-4f);
    ASSERT_NEAR(sim->particles[0].position.y, start.y, 1e-4f);
}

/** @brief Statically pinned particles never move, and the rest of the sheet hangs below them. */
static void test_cloth_static_anchors_hold_and_sheet_hangs() {
    PhysicsWorld world;
    cloth::Cloth c = make_test_sheet(11, glm::vec3(0.0f, 0.0f, 5.0f));
    const uint32_t pinned = cloth::pin_static(c, glm::vec3(0.0f, 0.0f, 5.0f), 0.5f);
    ASSERT_TRUE(pinned > 0);
    cloth::build_tethers(c);
    cloth::ClothId id = world.add_cloth(std::move(c));

    for (int i = 0; i < 300; ++i) world.step_fixed(util::k_default_fixed_dt);

    const cloth::Cloth* sim = world.get_cloth(id);
    for (const cloth::ClothAnchor& a : sim->anchors) {
        ASSERT_VEC3_NEAR(sim->particles[a.particle].position, a.world_position, 1e-5f);
        ASSERT_TRUE(sim->particles[a.particle].inv_mass == 0.0f);
    }
    // The free corners must have fallen well below the pinned centre.
    ASSERT_TRUE(sim->bounds.min.z < 4.0f);
}

/**
 * @brief A sheet dropped onto a static sphere ends up entirely OUTSIDE it, and stays taut.
 *
 * The headline correctness test: no particle inside radius + thickness after settling is exactly
 * what "the cloth does not sink through the ball" means, and the stretch bound is what separates
 * a draped sheet from one that has exploded.
 */
static void test_cloth_drapes_over_static_sphere_without_penetrating() {
    PhysicsWorld world;

    const float radius = 1.0f;
    const glm::vec3 sphere_center(0.0f, 0.0f, 3.0f);
    dynamics::Body ball;
    ball.type = dynamics::BodyType::Static;
    ball.position = sphere_center;
    ball.inv_mass = 0.0f;
    world.add_body(ball, collision::Shape::make_sphere(radius));

    cloth::ClothParams params;
    params.thickness = 0.03f;
    cloth::Cloth c = make_test_sheet(21, sphere_center + glm::vec3(0.0f, 0.0f, radius + 0.05f), params);
    cloth::pin_static(c, sphere_center + glm::vec3(0.0f, 0.0f, radius + 0.05f), 0.4f);
    cloth::build_tethers(c);
    cloth::ClothId id = world.add_cloth(std::move(c));

    for (int i = 0; i < 300; ++i) world.step_fixed(util::k_default_fixed_dt);

    const cloth::Cloth* sim = world.get_cloth(id);
    for (const cloth::ClothParticle& p : sim->particles) {
        // A small tolerance below `thickness`: a particle may sit slightly inside the standoff
        // between the collision projection and the next substep's constraint pass. Inside the
        // SPHERE itself is what must never happen.
        ASSERT_TRUE(glm::length(p.position - sphere_center) >= radius);
    }

    float max_stretch = 0.0f;
    for (const cloth::ClothConstraint& k : sim->stretch) {
        const float len = glm::length(sim->particles[k.a].position - sim->particles[k.b].position);
        max_stretch = std::max(max_stretch, len / k.rest_length);
    }
    ASSERT_TRUE(max_stretch < 1.25f);

    // And the sheet must actually have wrapped the ball, not just hovered above it.
    ASSERT_TRUE(sim->bounds.min.z < sphere_center.z);
}

/** @brief Tethers cap every particle's distance from its anchor at the geodesic rest length times
 *         tether_scale, even under a hard sideways yank that distance constraints alone would need
 *         far more iterations to resist. */
static void test_cloth_tethers_cap_distance_to_anchor() {
    PhysicsWorld world;

    cloth::ClothParams params;
    params.tether_scale = 1.02f;
    params.external_acceleration = glm::vec3(200.0f, 0.0f, 0.0f); // a violent lateral yank
    cloth::Cloth c = make_test_sheet(11, glm::vec3(0.0f, 0.0f, 5.0f), params);
    cloth::pin_static(c, glm::vec3(0.0f, 0.0f, 5.0f), 0.5f);
    cloth::build_tethers(c);
    ASSERT_TRUE(!c.tethers.empty());
    cloth::ClothId id = world.add_cloth(std::move(c));

    for (int i = 0; i < 180; ++i) world.step_fixed(util::k_default_fixed_dt);

    const cloth::Cloth* sim = world.get_cloth(id);
    for (const cloth::ClothTether& t : sim->tethers) {
        const float d = glm::length(sim->particles[t.particle].position -
                                    sim->particles[t.anchor].position);
        ASSERT_TRUE(d <= t.max_length + 1e-3f);
    }
}

/** @brief Particles anchored to a kinematic body track that body exactly as it moves, and the rest
 *         of the sheet comes with it -- the "cape on a moving character" case. */
static void test_cloth_anchors_follow_moving_kinematic_body() {
    PhysicsWorld world;

    dynamics::Body ball;
    ball.type = dynamics::BodyType::Kinematic;
    ball.position = glm::vec3(0.0f, 0.0f, 3.0f);
    ball.inv_mass = 0.0f;
    ball.use_gravity = false;
    dynamics::BodyId ball_id = world.add_body(ball, collision::Shape::make_sphere(1.0f));

    cloth::Cloth c = make_test_sheet(15, glm::vec3(0.0f, 0.0f, 4.05f));
    const uint32_t pinned = cloth::pin_to_body(c, ball_id, *world.get_body(ball_id),
                                               glm::vec3(0.0f, 0.0f, 4.0f), 0.4f);
    ASSERT_TRUE(pinned > 0);
    cloth::build_tethers(c);
    cloth::ClothId id = world.add_cloth(std::move(c));

    for (int i = 0; i < 120; ++i) world.step_fixed(util::k_default_fixed_dt);

    const float h = util::k_default_fixed_dt;
    const float speed = 2.0f;
    for (int i = 0; i < 120; ++i) {
        dynamics::Body* b = world.get_body(ball_id);
        b->position.x += speed * h;
        b->linear_velocity = glm::vec3(speed, 0.0f, 0.0f);
        world.step_fixed(h);
    }

    const dynamics::Body* b = world.get_body(ball_id);
    const cloth::Cloth* sim = world.get_cloth(id);
    for (const cloth::ClothAnchor& a : sim->anchors) {
        const glm::vec3 expected = b->position + b->orientation * a.local_position;
        ASSERT_VEC3_NEAR(sim->particles[a.particle].position, expected, 1e-4f);
    }
    // The sheet as a whole has travelled with the ball (it trails, so it lags -- but not by much).
    float mean_x = 0.0f;
    for (const cloth::ClothParticle& p : sim->particles) mean_x += p.position.x;
    mean_x /= static_cast<float>(sim->particles.size());
    ASSERT_TRUE(mean_x > b->position.x - 0.5f && mean_x <= b->position.x + 0.5f);
}

/**
 * @brief Self-collision separates particles that are not topological neighbours, and does nothing
 *        when disabled.
 *
 * Deliberately isolated from the constraint solve: the cloth's stretch/bend constraints are
 * cleared, so the only thing that can move a particle is the self-collision stage itself. Testing
 * it through a naturally-folding drape instead would make the assertion depend on whether that
 * particular sheet happened to fold at all, which is exactly the sort of test that passes for the
 * wrong reason. The real-drape path is still exercised (with self_collision on) by
 * test_cloth_stepping_is_deterministic_serial_and_parallel().
 */
static void test_cloth_self_collision_separates_non_neighbour_particles() {
    auto run = [](bool self_collision) {
        PhysicsWorld world;

        cloth::ClothParams params;
        params.self_collision = self_collision;
        params.gravity_scale = 0.0f;
        params.air_drag = 0.0f;
        params.air_lift = 0.0f;
        cloth::Cloth c = make_test_sheet(9, glm::vec3(0.0f, 0.0f, 5.0f), params);
        const float d_min = c.params.self_distance;
        ASSERT_TRUE(d_min > 0.0f); // derived from the grid spacing at build time

        c.stretch.clear();
        c.bend.clear();
        c.stretch_batches.clear();
        c.bend_batches.clear();

        // Collapse every particle into a ~1 cm blob, spread deterministically so no two start
        // exactly coincident (a zero separation vector has no defined push direction).
        for (std::size_t i = 0; i < c.particles.size(); ++i) {
            const float f = static_cast<float>(i) * 0.013f;
            c.particles[i].position = glm::vec3(std::sin(f) * 0.01f, std::cos(f) * 0.01f,
                                                5.0f + std::sin(f * 2.0f) * 0.01f);
            c.particles[i].prev_position = c.particles[i].position;
            c.particles[i].velocity = glm::vec3(0.0f);
        }

        const uint32_t cols = c.columns;
        cloth::ClothId id = world.add_cloth(std::move(c));
        for (int i = 0; i < 60; ++i) world.step_fixed(util::k_default_fixed_dt);

        const cloth::Cloth* sim = world.get_cloth(id);
        std::size_t violations = 0;
        for (std::size_t i = 0; i < sim->particles.size(); ++i) {
            for (std::size_t j = i + 1; j < sim->particles.size(); ++j) {
                const int32_t xi = static_cast<int32_t>(i % cols), yi = static_cast<int32_t>(i / cols);
                const int32_t xj = static_cast<int32_t>(j % cols), yj = static_cast<int32_t>(j / cols);
                if (std::abs(xi - xj) <= 1 && std::abs(yi - yj) <= 1) continue; // 1-ring is exempt
                if (glm::length(sim->particles[i].position - sim->particles[j].position) < d_min * 0.8f) {
                    ++violations;
                }
            }
        }
        return violations;
    };

    ASSERT_TRUE(run(/*self_collision=*/true) == 0);
    ASSERT_TRUE(run(/*self_collision=*/false) > 0); // the control: without it, they stay collapsed
}

/** @brief A settled cloth sleeps, and a moving anchor body wakes it again -- the cloth analogue of
 *         the kinematic wake rule dynamics/solver.h applies to rigid bodies. */
static void test_cloth_sleeps_when_settled_and_wakes_on_anchor_motion() {
    PhysicsWorld world;

    dynamics::Body post;
    post.type = dynamics::BodyType::Kinematic;
    post.position = glm::vec3(0.0f, 0.0f, 5.0f);
    post.inv_mass = 0.0f;
    post.use_gravity = false;
    dynamics::BodyId post_id = world.add_body(post, collision::Shape{}); // no collider needed

    cloth::ClothParams params;
    params.damping = 4.0f;          // settle fast, so the test stays short
    params.sleep_threshold = 0.25f;
    params.sleep_time = 0.25f;
    cloth::Cloth c = make_test_sheet(9, glm::vec3(0.0f, 0.0f, 5.0f), params);
    cloth::pin_to_body(c, post_id, *world.get_body(post_id), glm::vec3(0.0f, 0.0f, 5.0f), 3.0f);
    cloth::ClothId id = world.add_cloth(std::move(c));

    for (int i = 0; i < 600; ++i) world.step_fixed(util::k_default_fixed_dt);
    ASSERT_TRUE(!world.get_cloth(id)->awake);

    // A sleeping cloth must be perfectly frozen, not merely slow.
    const glm::vec3 before = world.get_cloth(id)->particles.back().position;
    for (int i = 0; i < 60; ++i) world.step_fixed(util::k_default_fixed_dt);
    ASSERT_VEC3_NEAR(world.get_cloth(id)->particles.back().position, before, 1e-6f);

    world.get_body(post_id)->position.x += 0.5f;
    world.step_fixed(util::k_default_fixed_dt);
    ASSERT_TRUE(world.get_cloth(id)->awake);
}

/** @brief Cloth particle state is part of world_state_hash(), and the whole step is invariant to
 *         whether a JobEngine is installed -- the same oracle
 *         test_job_parallel_stepping_matches_serial_determinism() uses for rigid bodies, extended
 *         to cover the cloth pass. Also asserts the hash actually MOVES when cloth exists, so a
 *         hash that silently ignored particles could not pass. */
static void test_cloth_stepping_is_deterministic_serial_and_parallel() {
    auto build_and_run = [](coopa::job::JobEngine* jobs) {
        PhysicsWorld world;
        world.set_parallel_threshold(1);
        world.set_job_engine(jobs);

        dynamics::Body ball;
        ball.type = dynamics::BodyType::Static;
        ball.position = glm::vec3(0.0f, 0.0f, 3.0f);
        ball.inv_mass = 0.0f;
        world.add_body(ball, collision::Shape::make_sphere(1.0f));

        cloth::ClothParams params;
        params.self_collision = true;
        params.wind = glm::vec3(0.4f, 0.0f, 0.0f);
        params.wind_turbulence = 0.6f;
        cloth::Cloth c = make_test_sheet(13, glm::vec3(0.0f, 0.0f, 4.05f), params);
        cloth::pin_static(c, glm::vec3(0.0f, 0.0f, 4.05f), 0.4f);
        cloth::build_tethers(c);
        world.add_cloth(std::move(c));

        for (int i = 0; i < 180; ++i) world.step_fixed(util::k_default_fixed_dt);
        return world.world_state_hash();
    };

    const uint64_t serial_hash = build_and_run(nullptr);
    coopa::job::JobEngine jobs(4);
    const uint64_t parallel_hash = build_and_run(&jobs);
    ASSERT_TRUE(serial_hash == parallel_hash);

    // Rebuilding the same scene twice must reproduce the hash exactly (turbulence is derived from
    // the cloth's own accumulated time, never rand()).
    ASSERT_TRUE(build_and_run(nullptr) == serial_hash);

    PhysicsWorld empty;
    ASSERT_TRUE(empty.world_state_hash() != serial_hash);
}

/** @brief ClothId generational handles reject a stale handle after the slot is recycled, matching
 *         test_body_id_generation_rejects_stale_handle(). */
static void test_cloth_id_generation_rejects_stale_handle() {
    PhysicsWorld world;
    cloth::ClothId a = world.add_cloth(make_test_sheet(5, glm::vec3(0.0f)));
    ASSERT_TRUE(world.is_valid(a));
    ASSERT_TRUE(world.cloth_count() == 1);

    world.remove_cloth(a);
    ASSERT_TRUE(!world.is_valid(a));
    ASSERT_TRUE(world.get_cloth(a) == nullptr);
    ASSERT_TRUE(world.cloth_count() == 0);

    cloth::ClothId b = world.add_cloth(make_test_sheet(5, glm::vec3(0.0f)));
    ASSERT_TRUE(b.index == a.index);   // slot recycled
    ASSERT_TRUE(b.generation != a.generation);
    ASSERT_TRUE(world.is_valid(b) && !world.is_valid(a));

    // An empty cloth is refused rather than occupying a slot.
    ASSERT_TRUE(!world.add_cloth(cloth::Cloth{}).is_valid());
}

/** @brief ClothComponent binds through PhysicsSystem, resolves an anchor named by object, follows
 *         that object's kinematic Rigidbody, and unbinds when the component goes away. */
static void test_scene_binding_cloth_resolves_anchor_and_follows_body() {
    using namespace coopa::scene;

    Scene scene("ClothBindTest");

    auto ball = std::make_unique<SceneObject>("ball");
    ball->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 3.0f));
    ball->add_component<components::SphereCollider>()->set_radius(1.0f);
    auto* rb = ball->add_component<components::RigidbodyComponent>();
    rb->is_kinematic = true;
    rb->use_gravity = false;
    SceneObject* ball_ptr = ball.get();
    scene.add_root_object(std::move(ball));

    auto sheet = std::make_unique<SceneObject>("cloth");
    sheet->add_component<TransformComponent>()->transform().set_position(glm::vec3(0.0f, 0.0f, 4.05f));
    auto* cc = sheet->add_component<components::ClothComponent>();
    cc->columns = 15;
    cc->rows = 15;
    cc->width = 4.0f;
    cc->height = 4.0f;
    components::ClothAnchorSpec spec;
    spec.object = "ball";
    spec.point = glm::vec3(0.0f, 0.0f, 1.0f); // the ball's own local frame: its crown
    spec.radius = 0.4f;
    cc->anchors.push_back(spec);
    scene.add_root_object(std::move(sheet));

    scene.start();
    system::install_physics_system(scene);

    const float h = util::k_default_fixed_dt;
    for (int i = 0; i < 180; ++i) {
        scene.update(h);
        scene.late_update(h);
    }

    const cloth::Cloth* sim = cc->cloth();
    ASSERT_TRUE(sim != nullptr);
    ASSERT_TRUE(sim->particles.size() == 15u * 15u);
    ASSERT_TRUE(!sim->anchors.empty());
    for (const cloth::ClothParticle& p : sim->particles) {
        ASSERT_TRUE(glm::length(p.position - glm::vec3(0.0f, 0.0f, 3.0f)) >= 1.0f);
    }

    for (int i = 0; i < 120; ++i) {
        coopa::util::Transform& t = ball_ptr->get_transform()->transform();
        t.set_position(t.position() + glm::vec3(2.0f * h, 0.0f, 0.0f));
        scene.update(h);
        scene.late_update(h);
    }

    sim = cc->cloth();
    const float ball_x = ball_ptr->get_transform()->transform().position().x;
    ASSERT_TRUE(ball_x > 3.0f);
    float mean_x = 0.0f;
    for (const cloth::ClothParticle& p : sim->particles) mean_x += p.position.x;
    mean_x /= static_cast<float>(sim->particles.size());
    ASSERT_TRUE(std::abs(mean_x - ball_x) < 0.6f);

    // Removing the component releases the cloth on the next refresh.
    cc->owner->remove_component(cc);
    dynamic_cast<system::PhysicsSystem*>(scene.find_system("Physics"))->refresh();
    scene.update(h);
    scene.late_update(h);
    ASSERT_TRUE(dynamic_cast<system::PhysicsSystem*>(scene.find_system("Physics"))->world().cloth_count() == 0);
}

/**
 * @brief Self-collision must never be the last word on a particle's position: whatever it pushes,
 *        the rigid-collision stage runs afterwards and puts it back outside the body.
 *
 * Built to fail loudly on the stage order, not to hope for it. Two non-neighbouring particles are
 * placed 2 cm apart just outside a sphere with self_distance forced to 20 cm, so self-collision
 * must shove them 9 cm apart -- driving the inner one ~5 cm INSIDE the sphere. Every other force is
 * removed (no constraints, no gravity, every other particle parked far away) so the assertion can
 * only be about ordering.
 */
static void test_cloth_self_collision_cannot_push_a_particle_into_a_collider() {
    PhysicsWorld world;

    const float radius = 1.0f;
    dynamics::Body ball;
    ball.type = dynamics::BodyType::Static;
    ball.position = glm::vec3(0.0f);
    ball.inv_mass = 0.0f;
    world.add_body(ball, collision::Shape::make_sphere(radius));

    cloth::ClothParams params;
    params.self_collision = true;
    params.self_distance = 0.2f;   // >> the 2 cm gap below, so the push is large and predictable
    params.gravity_scale = 0.0f;
    params.air_drag = 0.0f;
    params.air_lift = 0.0f;
    params.thickness = 0.03f;
    // ONE substep, so exactly one pass of each stage runs and the ordering is directly observable.
    // At the default 4 substeps the defect hides itself: a particle self-collision shoved inside the
    // sphere gets projected back out by the NEXT substep's rigid stage, and after four rounds the
    // residual is under a centimetre -- small enough to pass a naive assertion while still being
    // exactly the intermittent surface-popping this ordering causes in a real drape.
    params.substeps = 1;
    // Parked far from the sphere; only the two probe particles are moved onto it.
    cloth::Cloth c = make_test_sheet(9, glm::vec3(0.0f, 0.0f, 50.0f), params);
    c.stretch.clear();
    c.bend.clear();
    c.stretch_batches.clear();
    c.bend_batches.clear();

    const uint32_t probe_a = 0;                 // (0,0)
    const uint32_t probe_b = 5 * c.columns + 5; // (5,5) -- well outside the exempt 1-ring
    c.particles[probe_a].position = glm::vec3(radius + 0.06f, 0.0f, 0.0f);
    c.particles[probe_b].position = glm::vec3(radius + 0.04f, 0.0f, 0.0f);
    for (uint32_t i : {probe_a, probe_b}) {
        c.particles[i].prev_position = c.particles[i].position;
        c.particles[i].velocity = glm::vec3(0.0f);
    }

    cloth::ClothId id = world.add_cloth(std::move(c));
    world.step_fixed(util::k_default_fixed_dt);

    const cloth::Cloth* sim = world.get_cloth(id);
    // They must have been separated -- otherwise the test proves nothing about ordering.
    const float gap = glm::length(sim->particles[probe_a].position - sim->particles[probe_b].position);
    ASSERT_TRUE(gap > 0.05f);
    // ...and neither may have been left inside the sphere by that separation. The rigid stage runs
    // last, so the surviving clearance is the FULL standoff, not merely "not interpenetrating".
    ASSERT_TRUE(glm::length(sim->particles[probe_a].position) >= radius + params.thickness - 1e-4f);
    ASSERT_TRUE(glm::length(sim->particles[probe_b].position) >= radius + params.thickness - 1e-4f);
}

/**
 * @brief A still collider's projection is unchanged by the swept code path, to the bit.
 *
 * The swept branch has to be inert when nothing is moving, or every settled drape in the engine
 * would silently shift. Asserted analytically rather than by hashing: a particle inside a stationary
 * unit sphere lands exactly on radius + thickness along its own radial direction.
 */
static void test_cloth_still_collider_sweep_is_inert() {
    collision::Shape sphere = collision::Shape::make_sphere(1.0f);
    sphere.enabled = true;

    cloth::ClothCollider collider;
    collider.shape = &sphere;
    collider.position = glm::vec3(0.0f);
    collider.bounds = collision::world_bounds(sphere, collider.position, collider.orientation);
    ASSERT_TRUE(!collider.is_swept());

    glm::vec3 p(0.5f, 0.0f, 0.0f);
    glm::vec3 normal(0.0f);
    float depth = 0.0f;
    ASSERT_TRUE(cloth::project_particle(collider, 0.03f, p, normal, depth));
    ASSERT_VEC3_NEAR(p, glm::vec3(1.03f, 0.0f, 0.0f), 1e-5f);
    ASSERT_VEC3_NEAR(normal, glm::vec3(1.0f, 0.0f, 0.0f), 1e-5f);
    ASSERT_NEAR(depth, 0.53f, 1e-5f);

    // A particle already clear of the surface is untouched.
    glm::vec3 q(2.0f, 0.0f, 0.0f);
    ASSERT_TRUE(!cloth::project_particle(collider, 0.03f, q, normal, depth));
    ASSERT_VEC3_NEAR(q, glm::vec3(2.0f, 0.0f, 0.0f), 1e-6f);
}

/**
 * @brief A moving collider is projected out of the volume it SWEEPS, not just the pose it starts at.
 *
 * This is what keeps the sheet clear of a collider that the renderer will draw at several
 * intermediate poses before the cloth is solved again (above 60 Hz). Both shape paths are covered:
 * a sphere, whose swept volume is a capsule and therefore exact; and a box, which falls back to
 * probing the start and end poses and keeping the deeper correction.
 */
static void test_cloth_swept_collider_projects_out_of_the_motion_path() {
    glm::vec3 normal(0.0f);
    float depth = 0.0f;

    // --- Sphere: exact, via the capsule the motion segment describes ---
    collision::Shape sphere = collision::Shape::make_sphere(1.0f);
    sphere.enabled = true;
    cloth::ClothCollider moving;
    moving.shape = &sphere;
    moving.position = glm::vec3(0.0f);
    moving.sweep = glm::vec3(2.0f, 0.0f, 0.0f);
    moving.bounds = geometry::AABB::merge(
        collision::world_bounds(sphere, moving.position, moving.orientation),
        collision::world_bounds(sphere, moving.position + moving.sweep, moving.orientation));
    ASSERT_TRUE(moving.is_swept());

    // 1.58 m from the start centre -- comfortably clear of the sphere where it is now, but only
    // 0.5 m off the line it travels along.
    const glm::vec3 start(1.5f, 0.5f, 0.0f);
    glm::vec3 p = start;
    cloth::ClothCollider still = moving;
    still.sweep = glm::vec3(0.0f);
    ASSERT_TRUE(!cloth::project_particle(still, 0.03f, p, normal, depth)); // unswept: no contact
    ASSERT_VEC3_NEAR(p, start, 1e-6f);

    ASSERT_TRUE(cloth::project_particle(moving, 0.03f, p, normal, depth)); // swept: pushed clear
    ASSERT_VEC3_NEAR(p, glm::vec3(1.5f, 1.03f, 0.0f), 1e-5f);
    ASSERT_VEC3_NEAR(normal, glm::vec3(0.0f, 1.0f, 0.0f), 1e-5f);

    // --- Box: start/end probe, deeper correction wins ---
    collision::Shape box = collision::Shape::make_box(glm::vec3(0.5f));
    box.enabled = true;
    cloth::ClothCollider moving_box;
    moving_box.shape = &box;
    moving_box.position = glm::vec3(0.0f);
    moving_box.sweep = glm::vec3(2.0f, 0.0f, 0.0f);
    moving_box.end_orientation = moving_box.orientation;
    moving_box.bounds = geometry::AABB::merge(
        collision::world_bounds(box, moving_box.position, moving_box.orientation),
        collision::world_bounds(box, moving_box.position + moving_box.sweep, moving_box.end_orientation));

    // x = 1.52 is 1.02 m clear of the box at the origin, but 2 cm INSIDE it once it has travelled.
    glm::vec3 b(1.52f, 0.0f, 0.0f);
    cloth::ClothCollider still_box = moving_box;
    still_box.sweep = glm::vec3(0.0f);
    ASSERT_TRUE(!cloth::project_particle(still_box, 0.03f, b, normal, depth));

    ASSERT_TRUE(cloth::project_particle(moving_box, 0.03f, b, normal, depth));
    ASSERT_NEAR(b.x, 2.0f - (0.5f + 0.03f), 1e-5f); // exits the near -X face of the END pose
    ASSERT_VEC3_NEAR(normal, glm::vec3(-1.0f, 0.0f, 0.0f), 1e-5f);
}

/**
 * @brief A particle that crosses a mesh surface in one substep must be put back on the side it
 *        came from, not left on the far side.
 *
 * This is the failure the mesh branch's proximity-only test cannot see: the particle lands beyond
 * `thickness` of every triangle, so the BVH query returns nothing and it stays through the floor
 * permanently. Sphere/box/capsule have no equivalent hole -- they all handle "already inside"
 * explicitly -- but a triangle soup has no cheap inside to test against, so the crossing itself has
 * to be caught.
 */
static void test_cloth_mesh_collider_stops_a_tunnelling_particle() {
    geometry::TriangleMesh floor = make_two_triangle_floor(); // a 4x4 quad in the z = 0 plane
    collision::Shape shape = collision::Shape::make_mesh(&floor);
    shape.enabled = true;

    cloth::ClothCollider collider;
    collider.shape = &shape;
    collider.position = glm::vec3(0.0f);
    collider.bounds = collision::world_bounds(shape, collider.position, collider.orientation);

    const float thickness = 0.03f;
    const glm::vec3 start(0.0f, 0.0f, 0.5f);
    glm::vec3 p(0.0f, 0.0f, -0.5f); // one substep of a ~120 m/s particle: straight through
    glm::vec3 normal(0.0f);
    float depth = 0.0f;

    ASSERT_TRUE(cloth::project_particle(collider, thickness, p, normal, depth, &start));
    ASSERT_NEAR(p.z, thickness, 1e-4f);                       // back on the entry side
    ASSERT_VEC3_NEAR(normal, glm::vec3(0.0f, 0.0f, 1.0f), 1e-4f);

    // ...and symmetrically from below: a mesh is a surface, not a one-way gate.
    const glm::vec3 start_below(0.0f, 0.0f, -0.5f);
    glm::vec3 q(0.0f, 0.0f, 0.5f);
    ASSERT_TRUE(cloth::project_particle(collider, thickness, q, normal, depth, &start_below));
    ASSERT_NEAR(q.z, -thickness, 1e-4f);
    ASSERT_VEC3_NEAR(normal, glm::vec3(0.0f, 0.0f, -1.0f), 1e-4f);
}

/** @brief Driven through the real solver, not just project_particle(): a cloth fired hard at a mesh
 *         floor ends up above it. Catches a stage-7 wiring mistake that the unit test above cannot
 *         (e.g. failing to pass the particle's start-of-substep position). */
static void test_cloth_mesh_collider_stops_tunnelling_through_the_solver() {
    geometry::TriangleMesh floor = make_two_triangle_floor();

    PhysicsWorld world;
    dynamics::Body floor_body;
    floor_body.type = dynamics::BodyType::Static;
    floor_body.inv_mass = 0.0f;
    world.add_body(floor_body, collision::Shape::make_mesh(&floor));

    cloth::ClothParams params;
    params.thickness = 0.03f;
    params.air_drag = 0.0f;
    params.air_lift = 0.0f;
    // A small sheet dropped from just above the floor at high speed.
    cloth::GridClothDesc desc;
    desc.columns = 5;
    desc.rows = 5;
    desc.width = 1.0f;
    desc.height = 1.0f;
    desc.center = glm::vec3(0.0f, 0.0f, 1.0f);
    desc.total_mass = 1.0f;
    desc.params = params;
    cloth::Cloth c = cloth::make_grid_cloth(desc);
    for (cloth::ClothParticle& particle : c.particles) particle.velocity = glm::vec3(0.0f, 0.0f, -60.0f);
    cloth::ClothId id = world.add_cloth(std::move(c));

    for (int i = 0; i < 120; ++i) world.step_fixed(util::k_default_fixed_dt);

    const cloth::Cloth* sim = world.get_cloth(id);
    for (const cloth::ClothParticle& particle : sim->particles) {
        ASSERT_TRUE(particle.position.z > 0.0f);
    }
}

/** @brief A particle already sitting shallowly BEHIND the surface is recovered through the front
 *         face, not pushed further behind. Today's code derives the push direction from
 *         (particle - closest point), which for a penetrating particle points the wrong way and
 *         cements the penetration at exactly `thickness` on the wrong side. */
static void test_cloth_mesh_collider_recovers_a_particle_behind_the_surface() {
    geometry::TriangleMesh floor = make_two_triangle_floor();
    collision::Shape shape = collision::Shape::make_mesh(&floor);
    shape.enabled = true;

    cloth::ClothCollider collider;
    collider.shape = &shape;
    collider.position = glm::vec3(0.0f);
    collider.bounds = collision::world_bounds(shape, collider.position, collider.orientation);

    const float thickness = 0.03f;
    glm::vec3 p(0.0f, 0.0f, -0.01f); // 1 cm under the floor, motionless
    glm::vec3 normal(0.0f);
    float depth = 0.0f;

    ASSERT_TRUE(cloth::project_particle(collider, thickness, p, normal, depth));
    ASSERT_NEAR(p.z, thickness, 1e-4f);
    ASSERT_VEC3_NEAR(normal, glm::vec3(0.0f, 0.0f, 1.0f), 1e-4f);
}

/** @brief A particle a hair ABOVE the surface keeps taking exactly the path it takes today: pushed
 *         straight up to `thickness` along the separation direction. The regression guard for every
 *         settled drape in the engine. */
static void test_cloth_mesh_collider_front_side_projection_is_unchanged() {
    geometry::TriangleMesh floor = make_two_triangle_floor();
    collision::Shape shape = collision::Shape::make_mesh(&floor);
    shape.enabled = true;

    cloth::ClothCollider collider;
    collider.shape = &shape;
    collider.position = glm::vec3(0.0f);
    collider.bounds = collision::world_bounds(shape, collider.position, collider.orientation);

    const float thickness = 0.03f;
    glm::vec3 normal(0.0f);
    float depth = 0.0f;

    glm::vec3 p(0.0f, 0.0f, 0.01f);
    ASSERT_TRUE(cloth::project_particle(collider, thickness, p, normal, depth));
    ASSERT_NEAR(p.z, thickness, 1e-4f);
    ASSERT_VEC3_NEAR(normal, glm::vec3(0.0f, 0.0f, 1.0f), 1e-4f);

    // Beyond the standoff: untouched. Also pins the gate -- a particle whose motion is shorter than
    // `thickness` cannot have crossed, so the swept raycast must not fire and must not perturb it.
    const glm::vec3 start(0.0f, 0.0f, 0.11f);
    glm::vec3 q(0.0f, 0.0f, 0.10f);
    ASSERT_TRUE(!cloth::project_particle(collider, thickness, q, normal, depth, &start));
    ASSERT_VEC3_NEAR(q, glm::vec3(0.0f, 0.0f, 0.10f), 1e-6f);
}

/**
 * @brief A particle in a concave pocket -- in front of one triangle, behind its angled neighbour --
 *        must not be flipped across the mesh.
 *
 * This is the guard on the "behind EVERY nearby triangle" rule. Deciding inside-ness from the single
 * nearest triangle's normal would teleport this particle through the wall, which is the internal-
 * edge failure mesh_contact.h needs TriangleAdjacency to avoid for rigid contacts.
 */
static void test_cloth_mesh_collider_concave_pocket_is_not_flipped() {
    // A right-angled inside corner: floor in the z = 0 plane (normal +Z) and wall in the x = 0
    // plane (normal +X). A particle just inside the corner is in front of both.
    std::vector<glm::vec3> vertices = {
        glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(1.0f, -1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        glm::vec3(0.0f, -1.0f, 1.0f), glm::vec3(0.0f, 1.0f, 1.0f),
    };
    std::vector<uint32_t> indices = {0, 1, 2, 0, 2, 3, 0, 3, 5, 0, 5, 4};
    std::vector<glm::vec3> normals = {
        glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f),
    };
    std::vector<geometry::TriangleAdjacency> adjacency(4);
    geometry::TriangleMesh corner(std::move(vertices), std::move(indices),
                                   std::move(normals), std::move(adjacency));

    collision::Shape shape = collision::Shape::make_mesh(&corner);
    shape.enabled = true;
    cloth::ClothCollider collider;
    collider.shape = &shape;
    collider.position = glm::vec3(0.0f);
    collider.bounds = collision::world_bounds(shape, collider.position, collider.orientation);

    const float thickness = 0.03f;
    // 2 mm behind the WALL but 2 cm above the FLOOR -- so the nearest triangle (the wall, 2 mm away)
    // reports "behind" while the floor reports "in front". Deciding inside-ness from the nearest
    // triangle alone would flip this particle 3.2 cm across the wall plane and into the room; the
    // "behind EVERY nearby triangle" rule leaves it on the side it was already on.
    glm::vec3 p(-0.002f, 0.0f, 0.02f);
    glm::vec3 normal(0.0f);
    float depth = 0.0f;
    ASSERT_TRUE(cloth::project_particle(collider, thickness, p, normal, depth));

    ASSERT_TRUE(p.x <= 0.0f);   // never teleported through the wall
    ASSERT_TRUE(p.z >= 0.0f);   // and still above the floor

    // The genuinely-penetrating case still recovers: put it behind BOTH faces and it must come back
    // out, which is what stops the rule above from simply disabling recovery near a corner.
    glm::vec3 q(-0.002f, 0.0f, -0.002f);
    ASSERT_TRUE(cloth::project_particle(collider, thickness, q, normal, depth));
    ASSERT_TRUE(q.x > 0.0f || q.z > 0.0f);
}

/** @brief A sheet settling on a mesh floor rests ON it, at the standoff, with nothing below --
 *         the end-to-end regression guard for the common mesh-collider case. */
static void test_cloth_settles_on_mesh_floor_at_thickness() {
    geometry::TriangleMesh floor = make_two_triangle_floor();

    PhysicsWorld world;
    dynamics::Body floor_body;
    floor_body.type = dynamics::BodyType::Static;
    floor_body.inv_mass = 0.0f;
    world.add_body(floor_body, collision::Shape::make_mesh(&floor));

    cloth::ClothParams params;
    params.thickness = 0.03f;
    cloth::GridClothDesc desc;
    desc.columns = 9;
    desc.rows = 9;
    desc.width = 2.0f;
    desc.height = 2.0f;
    desc.center = glm::vec3(0.0f, 0.0f, 0.6f);
    desc.total_mass = 1.0f;
    desc.params = params;
    cloth::ClothId id = world.add_cloth(cloth::make_grid_cloth(desc));

    for (int i = 0; i < 300; ++i) world.step_fixed(util::k_default_fixed_dt);

    const cloth::Cloth* sim = world.get_cloth(id);
    float lowest = 1e9f;
    for (const cloth::ClothParticle& p : sim->particles) lowest = std::min(lowest, p.position.z);
    ASSERT_TRUE(lowest >= 0.0f);                       // nothing sank through
    ASSERT_NEAR(lowest, params.thickness, 5e-3f);      // and it is resting at the standoff
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
    RUN_TEST(test_kinematic_spin_wakes_sleeping_body_it_sweeps_into);
    RUN_TEST(test_sleeping_body_unaffected_by_awake_neighbors_impulse);
    RUN_TEST(test_body_impulse_and_angular_impulse_change_velocity_immediately);
    RUN_TEST(test_body_force_and_impulse_apis_wake_a_sleeping_body);
    RUN_TEST(test_on_substep_signal_and_deferred_destroy);

    RUN_TEST(test_sat_box_box_face_contact_known_penetration);
    RUN_TEST(test_sat_box_box_edge_contact_single_point);
    RUN_TEST(test_tumbled_box_settles_flat_not_balanced_on_edge);
    RUN_TEST(test_fast_box_does_not_tunnel_through_thin_wall);
    RUN_TEST(test_diag_transform_roundtrip_noise);
    RUN_TEST(test_three_box_concrete_stack_settles_flat);
    RUN_TEST(test_ten_box_stack_stable);
    RUN_TEST(test_friction_slope_static_vs_sliding);
    RUN_TEST(test_mass_ratio_100_to_1_stable);

    RUN_TEST(test_capsule_rests_on_box_floor);
    RUN_TEST(test_capsule_pile_settles_without_exploding);

    RUN_TEST(test_box_slides_across_mesh_floor_no_edge_discontinuity);

    RUN_TEST(test_raycast_hits_box_with_exact_t_point_and_normal);
    RUN_TEST(test_raycast_all_returns_hits_sorted_by_distance);
    RUN_TEST(test_overlap_sphere_and_overlap_box_find_expected_bodies);
    RUN_TEST(test_overlap_capsule_finds_expected_bodies);
    RUN_TEST(test_raycast_any_finds_a_hit_without_necessarily_the_closest);
    RUN_TEST(test_query_include_triggers_flag_excludes_trigger_colliders);
    RUN_TEST(test_box_cast_and_capsule_cast_hit_known_target);
    RUN_TEST(test_trigger_enter_stay_exit_fires_correct_sequence);
    RUN_TEST(test_debug_draw_emits_collider_bvh_and_contact_lines);
    RUN_TEST(test_shape_local_rotation_reorients_capsule_consistently);

    RUN_TEST(test_aabb_tree_matches_brute_force);
    RUN_TEST(test_layer_matrix_blocks_pair_before_narrowphase);

    RUN_TEST(test_headless_scene_binding_creates_bodies_and_settles);
    RUN_TEST(test_scene_binding_runtime_parameter_change_rides_revision);
    RUN_TEST(test_scene_binding_rigidbody_runtime_property_changes_take_effect);
    RUN_TEST(test_rigidbody_center_of_mass_and_inertia_override_fold_into_body);
    RUN_TEST(test_rigidbody_component_sleep_wake_and_set_velocity_wakes);
    RUN_TEST(test_scene_binding_box_collider_center_offset_settles_flat);
    RUN_TEST(test_compound_collider_two_children_one_rigidbody);
    RUN_TEST(test_scene_binding_hinge_joint_door_resolves_and_swings_to_limit);
    RUN_TEST(test_collision_event_carries_contact_payload_and_clears_on_exit);
    RUN_TEST(test_scene_query_wrappers_resolve_collider_and_object);

    RUN_TEST(test_physics_material_combine_modes);
    RUN_TEST(test_collider_material_resolution_prefers_asset_over_inline);
    RUN_TEST(test_parse_physics_settings_and_apply_to_world);
    RUN_TEST(test_hinge_joint_door_swings_and_stops_at_limit);
    RUN_TEST(test_hinge_joint_free_pendulum_swings_without_drift);
    RUN_TEST(test_hinge_joint_zero_range_limit_acts_rigid);
    RUN_TEST(test_hinge_joint_islands_dynamic_pair_for_sleep);
    RUN_TEST(test_job_parallel_stepping_matches_serial_determinism);

    RUN_TEST(test_cloth_grid_builder_topology_and_disjoint_batches);
    RUN_TEST(test_cloth_free_fall_matches_analytic);
    RUN_TEST(test_cloth_static_anchors_hold_and_sheet_hangs);
    RUN_TEST(test_cloth_drapes_over_static_sphere_without_penetrating);
    RUN_TEST(test_cloth_tethers_cap_distance_to_anchor);
    RUN_TEST(test_cloth_anchors_follow_moving_kinematic_body);
    RUN_TEST(test_cloth_self_collision_separates_non_neighbour_particles);
    RUN_TEST(test_cloth_sleeps_when_settled_and_wakes_on_anchor_motion);
    RUN_TEST(test_cloth_stepping_is_deterministic_serial_and_parallel);
    RUN_TEST(test_cloth_id_generation_rejects_stale_handle);
    RUN_TEST(test_scene_binding_cloth_resolves_anchor_and_follows_body);
    RUN_TEST(test_cloth_self_collision_cannot_push_a_particle_into_a_collider);
    RUN_TEST(test_cloth_still_collider_sweep_is_inert);
    RUN_TEST(test_cloth_swept_collider_projects_out_of_the_motion_path);
    RUN_TEST(test_cloth_mesh_collider_stops_a_tunnelling_particle);
    RUN_TEST(test_cloth_mesh_collider_stops_tunnelling_through_the_solver);
    RUN_TEST(test_cloth_mesh_collider_recovers_a_particle_behind_the_surface);
    RUN_TEST(test_cloth_mesh_collider_front_side_projection_is_unchanged);
    RUN_TEST(test_cloth_mesh_collider_concave_pocket_is_not_flipped);
    RUN_TEST(test_cloth_settles_on_mesh_floor_at_thickness);

    std::cout << "===========================================" << std::endl;
    std::cout << "Tests run: " << g_tests_run << ", Failed: " << g_tests_failed << std::endl;
    std::cout << "===========================================" << std::endl;

    return g_tests_failed == 0 ? 0 : 1;
}
