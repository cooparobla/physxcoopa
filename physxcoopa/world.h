/**
 * @file world.h
 * @brief PhysicsWorld -- the Scene-agnostic simulation core. No dependency on coopa::scene;
 *        the ISceneSystem adapter (system/physics_system.h) is the only thing that knows
 *        about Scene/SceneObject/Transform.
 */

#ifndef PHYSXCOOPA_WORLD_H
#define PHYSXCOOPA_WORLD_H

#include <physxcoopa/util/config.h>
#include <physxcoopa/util/math.h>
#include <physxcoopa/dynamics/body.h>
#include <physxcoopa/dynamics/integrator.h>
#include <physxcoopa/dynamics/solver.h>
#include <physxcoopa/collision/shape.h>
#include <physxcoopa/collision/manifold.h>
#include <physxcoopa/collision/contact_event.h>
#include <physxcoopa/collision/narrowphase.h>
#include <physxcoopa/broadphase/aabb_tree.h>
#include <physxcoopa/broadphase/layer_matrix.h>
#include <physxcoopa/broadphase/pair_cache.h>
#include <physxcoopa/query/queries.h>
#include <physxcoopa/debug/debug_draw.h>

#include <coopa/event/signal.h>
#include <coopa/debug/logger.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace coopa {
namespace physx {

/**
 * @class PhysicsWorld
 * @brief Owns every Body and Shape and drives the fixed-substep simulation loop.
 *
 * Depends only on glm, coopa::event::Signal and coopa::debug::Logger -- never on
 * coopa::scene. This is what lets test.cpp exercise the whole solver/narrowphase/broadphase
 * stack with no Scene at all, and lets gameplay reach the world (via
 * `scene->find_system("Physics")`, see PhysicsSystem) without ever seeing a Scene reference
 * baked into PhysicsWorld itself.
 */
class PhysicsWorld {
public:
    /**
     * @brief Fired once per step_fixed(), after kinematic sync-in/force integration but
     *        before broadphase -- see the file-level step_fixed() doc for exactly where.
     *        Gameplay may call add_force/add_torque/set_velocity/wake from a connected slot
     *        (applied immediately); create_body/destroy_body issued from here are deferred
     *        to the top of the NEXT step_fixed instead -- see those methods' docs.
     */
    coopa::event::Signal<PhysicsWorld&, float> on_substep;

    explicit PhysicsWorld(const util::PhysicsConfig& config = {})
        : config_(config), logger_("Physics") {}

    // --- Body lifecycle (immediate -- for setup-time use, outside step_fixed) ---

    /**
     * @brief Creates a new body with an optional collision shape, returning a generational
     *        handle to it. A default-constructed Shape (`enabled == false`) means the body
     *        participates in dynamics but never in collision, matching Unity's "Rigidbody
     *        with no Collider" case.
     *
     * @param initial Initial dynamic state.
     * @param shape   Collision geometry, or a disabled Shape for none.
     * @return Handle to the new body.
     */
    dynamics::BodyId add_body(const dynamics::Body& initial, const collision::Shape& shape = collision::Shape{}) {
        uint32_t index;
        if (!free_indices_.empty()) {
            index = free_indices_.back();
            free_indices_.pop_back();
            bodies_[index] = initial;
            shapes_[index] = shape;
        } else {
            index = static_cast<uint32_t>(bodies_.size());
            bodies_.push_back(initial);
            shapes_.push_back(shape);
            generations_.push_back(0);
            alive_.push_back(false);
            tree_proxy_.push_back(broadphase::k_null_node);
            in_static_tree_.push_back(false);
        }
        alive_[index] = true;
        dynamics::Body& b = bodies_[index];
        b.prev_position = b.position;
        b.prev_orientation = b.orientation;
        b.last_written_position = b.position;
        b.last_written_orientation = b.orientation;
        dynamics::update_world_inertia(b);
        create_proxy_(index);
        return dynamics::BodyId{index, generations_[index]};
    }

    /** @brief Destroys a body immediately. A no-op if `id` is already invalid/stale. */
    void remove_body(dynamics::BodyId id) {
        if (!is_valid(id)) return;
        destroy_proxy_(id.index);
        alive_[id.index] = false;
        ++generations_[id.index];
        free_indices_.push_back(id.index);
    }

    /** @brief Replaces a live body's collision shape immediately. No-op if `id` is invalid. */
    void set_shape(dynamics::BodyId id, const collision::Shape& shape) {
        if (!is_valid(id)) return;
        destroy_proxy_(id.index);
        shapes_[id.index] = shape;
        create_proxy_(id.index);
    }

    /** @brief The layer matrix consulted before any narrowphase work. */
    broadphase::LayerMatrix& layers() { return layer_matrix_; }
    const broadphase::LayerMatrix& layers() const { return layer_matrix_; }

    // --- Body lifecycle (deferrable -- safe to call from an on_substep slot) ---

    /**
     * @brief Creates a body, deferring the actual insertion to the top of the next
     *        step_fixed() if called while a substep is in progress (i.e. from an on_substep
     *        slot). When deferred, returns an invalid BodyId -- a substep callback cannot
     *        learn the new body's id synchronously; give it a stable external key of your
     *        own (e.g. store it in a member the callback owns) if you need to find it later.
     *        Outside step_fixed(), behaves exactly like add_body().
     */
    dynamics::BodyId create_body(const dynamics::Body& initial, const collision::Shape& shape = collision::Shape{}) {
        if (!in_step_) return add_body(initial, shape);
        DeferredCommand cmd;
        cmd.type = DeferredCommand::Type::Create;
        cmd.body = initial;
        cmd.shape = shape;
        deferred_commands_.push_back(cmd);
        return dynamics::BodyId{};
    }

    /**
     * @brief Destroys a body, deferring the actual slot free to the top of the next
     *        step_fixed() if called from an on_substep slot -- but invalidates any of THIS
     *        step's already-generated manifolds referencing `id` immediately, so the solver
     *        never solves a constraint against a body destroyed mid-step. Outside
     *        step_fixed(), behaves exactly like remove_body().
     */
    void destroy_body(dynamics::BodyId id) {
        if (!in_step_) {
            remove_body(id);
            return;
        }
        if (!is_valid(id)) return;
        for (auto& m : manifolds_) {
            if (m.valid && (m.a == id || m.b == id)) m.valid = false;
        }
        DeferredCommand cmd;
        cmd.type = DeferredCommand::Type::Destroy;
        cmd.target = id;
        deferred_commands_.push_back(cmd);
    }

    /** @brief True if `id` addresses a currently-alive body (index in range, generation matches). */
    bool is_valid(dynamics::BodyId id) const {
        return id.is_valid() && id.index < bodies_.size() && alive_[id.index] &&
               generations_[id.index] == id.generation;
    }

    /** @brief Returns the body `id` addresses, or nullptr if `id` is invalid/stale. */
    dynamics::Body* get_body(dynamics::BodyId id) {
        return is_valid(id) ? &bodies_[id.index] : nullptr;
    }
    const dynamics::Body* get_body(dynamics::BodyId id) const {
        return is_valid(id) ? &bodies_[id.index] : nullptr;
    }

    /** @brief Returns the shape `id` addresses, or nullptr if `id` is invalid/stale. */
    collision::Shape* get_shape(dynamics::BodyId id) {
        return is_valid(id) ? &shapes_[id.index] : nullptr;
    }
    const collision::Shape* get_shape(dynamics::BodyId id) const {
        return is_valid(id) ? &shapes_[id.index] : nullptr;
    }

    /**
     * @brief Invokes `fn(BodyId, Body&)` for every currently-alive body, in index order.
     *
     * Index order (not creation order once slots are recycled) -- fine for anything that
     * doesn't depend on iteration order for determinism, since the body array itself is
     * never reordered by removal (see "Determinism and parallelism" in the plan).
     */
    template <typename Fn>
    void for_each_body(Fn&& fn) {
        for (uint32_t i = 0; i < bodies_.size(); ++i) {
            if (alive_[i]) fn(dynamics::BodyId{i, generations_[i]}, bodies_[i]);
        }
    }
    template <typename Fn>
    void for_each_body(Fn&& fn) const {
        for (uint32_t i = 0; i < bodies_.size(); ++i) {
            if (alive_[i]) fn(dynamics::BodyId{i, generations_[i]}, bodies_[i]);
        }
    }

    /** @brief Number of currently-alive bodies. */
    size_t alive_body_count() const {
        size_t n = 0;
        for (bool a : alive_) n += a ? 1 : 0;
        return n;
    }

    /** @brief This step's contact manifolds (valid between step_fixed() calls, for debug draw
     *         and diagnostics -- do not mutate). */
    const std::vector<collision::ContactManifold>& manifolds() const { return manifolds_; }

    /**
     * @brief This frame's collision/trigger enter/stay/exit events -- accumulated across every
     *        substep step() ran, cleared at the top of the next step() call. Drain (read, then
     *        act on) once per frame, after step() returns, same as the plan's "Events are
     *        queued during the substep loop and emitted once at the end of execute()" -- the
     *        emitting (firing coopa::event::Signal callbacks) belongs to the caller (e.g.
     *        PhysicsSystem), since PhysicsWorld itself has no Collider/SceneObject to fire on.
     *        A caller driving step_fixed() directly (test.cpp, the determinism harness) is
     *        responsible for its own clear_events() between checks if it cares about
     *        per-substep rather than accumulated deltas.
     */
    const std::vector<collision::ContactEvent>& events() const { return events_; }

    /** @brief Clears the accumulated event queue -- step() already does this each call; only
     *         needed by a caller driving step_fixed() directly across multiple checks. */
    void clear_events() { events_.clear(); }

    // --- Queries ---
    //
    // Only valid between phases, not from inside on_substep (mid-solve, not-yet-integrated
    // state). `layer_mask` follows Unity's convention: bit `shape.layer` of the mask, not the
    // LayerMatrix used for solver pair rejection -- a query has no "other side" to look up a
    // collision-matrix entry against.

    /** @brief Closest hit along `ray`, or false if nothing was hit. */
    bool raycast(const geometry::Ray& ray, query::RaycastHit& hit, uint32_t layer_mask = ~0u) const {
        bool found = false;
        float best_t = ray.max_distance;
        auto visit = [&](void* user_data) {
            uint32_t i = user_data_to_index_(user_data);
            if (!((layer_mask >> shapes_[i].layer) & 1u)) return;
            geometry::Ray clipped = ray;
            clipped.max_distance = best_t;
            float t;
            glm::vec3 n;
            if (!query::raycast_shape(clipped, shapes_[i], bodies_[i].position, bodies_[i].orientation, t, n)) return;
            best_t = t;
            hit.point = ray.origin + ray.direction * t;
            hit.normal = n;
            hit.distance = t;
            hit.body = dynamics::BodyId{i, generations_[i]};
            found = true;
        };
        dynamic_tree_.raycast(ray, visit);
        static_tree_.raycast(ray, visit);
        return found;
    }

    /** @brief Every hit along `ray`, sorted nearest-first (unlike raycast(), not clipped to the
     *         closest hit as it goes, since every overlapping body along the ray is wanted). */
    std::vector<query::RaycastHit> raycast_all(const geometry::Ray& ray, uint32_t layer_mask = ~0u) const {
        std::vector<query::RaycastHit> hits;
        auto visit = [&](void* user_data) {
            uint32_t i = user_data_to_index_(user_data);
            if (!((layer_mask >> shapes_[i].layer) & 1u)) return;
            float t;
            glm::vec3 n;
            if (!query::raycast_shape(ray, shapes_[i], bodies_[i].position, bodies_[i].orientation, t, n)) return;
            query::RaycastHit hit;
            hit.point = ray.origin + ray.direction * t;
            hit.normal = n;
            hit.distance = t;
            hit.body = dynamics::BodyId{i, generations_[i]};
            hits.push_back(hit);
        };
        dynamic_tree_.raycast(ray, visit);
        static_tree_.raycast(ray, visit);
        std::sort(hits.begin(), hits.end(), [](const query::RaycastHit& a, const query::RaycastHit& b) {
            return a.distance < b.distance;
        });
        return hits;
    }

    /**
     * @brief Casts a sphere of `radius` from `origin` along `dir`, up to `max_distance`.
     *
     * v1 simplification: approximates each candidate shape as inflated by `radius` (a rounded
     * box becomes a sharp box, a rounded capsule keeps its exact swept-sphere shape since a
     * capsule inflated by a sphere is just a larger capsule) and raycasts the inflated shape --
     * exact for Sphere and Capsule targets, a conservative-at-the-corners approximation for Box
     * and TriangleMesh. Good enough for v1's use cases (character-controller-style probes);
     * revisit with a true swept-volume test if a caller needs exact rounded-corner behavior.
     */
    bool sphere_cast(const glm::vec3& origin, float radius, const glm::vec3& dir, float max_distance,
                      query::RaycastHit& hit, uint32_t layer_mask = ~0u) const {
        geometry::Ray ray;
        ray.origin = origin;
        ray.direction = dir;
        ray.max_distance = max_distance;

        bool found = false;
        float best_t = max_distance;
        auto visit = [&](void* user_data) {
            uint32_t i = user_data_to_index_(user_data);
            if (!((layer_mask >> shapes_[i].layer) & 1u)) return;
            collision::Shape inflated = shapes_[i];
            switch (inflated.type) {
                case collision::ShapeType::Sphere: inflated.radius += radius; break;
                case collision::ShapeType::Box: inflated.half_extents += glm::vec3(radius); break;
                case collision::ShapeType::Capsule: inflated.capsule_radius += radius; break;
                case collision::ShapeType::TriangleMesh: break; // no inflation available; exact-surface cast only
            }
            geometry::Ray clipped = ray;
            clipped.max_distance = best_t;
            float t;
            glm::vec3 n;
            if (!query::raycast_shape(clipped, inflated, bodies_[i].position, bodies_[i].orientation, t, n)) return;
            best_t = t;
            hit.point = ray.origin + ray.direction * t;
            hit.normal = n;
            hit.distance = t;
            hit.body = dynamics::BodyId{i, generations_[i]};
            found = true;
        };
        dynamic_tree_.raycast(ray, visit);
        static_tree_.raycast(ray, visit);
        return found;
    }

    /** @brief Every body whose shape overlaps a world-space sphere. */
    std::vector<dynamics::BodyId> overlap_sphere(const glm::vec3& center, float radius, uint32_t layer_mask = ~0u) const {
        geometry::Sphere query_sphere{center, radius};
        geometry::AABB bounds;
        bounds.min = center - glm::vec3(radius);
        bounds.max = center + glm::vec3(radius);

        std::vector<dynamics::BodyId> results;
        auto visit = [&](void* user_data) {
            uint32_t i = user_data_to_index_(user_data);
            if (!((layer_mask >> shapes_[i].layer) & 1u)) return;
            if (query::shape_overlaps_sphere(shapes_[i], bodies_[i].position, bodies_[i].orientation, query_sphere)) {
                results.push_back(dynamics::BodyId{i, generations_[i]});
            }
        };
        dynamic_tree_.query(bounds, visit);
        static_tree_.query(bounds, visit);
        return results;
    }

    /** @brief Every body whose shape overlaps a world-space OBB. */
    std::vector<dynamics::BodyId> overlap_box(const geometry::OBB& box, uint32_t layer_mask = ~0u) const {
        geometry::AABB bounds = box.bounds();

        std::vector<dynamics::BodyId> results;
        auto visit = [&](void* user_data) {
            uint32_t i = user_data_to_index_(user_data);
            if (!((layer_mask >> shapes_[i].layer) & 1u)) return;
            if (query::shape_overlaps_obb(shapes_[i], bodies_[i].position, bodies_[i].orientation, box)) {
                results.push_back(dynamics::BodyId{i, generations_[i]});
            }
        };
        dynamic_tree_.query(bounds, visit);
        static_tree_.query(bounds, visit);
        return results;
    }

    /**
     * @brief Fills `out` with this world's current debug geometry: collider wireframes (colored
     *        white/awake, green/sleeping, yellow/trigger -- trigger takes priority over
     *        sleep state), last step's contact points/normals (red), and both broadphase
     *        trees' node bounds. Pure data -- no Vulkan, no gfxcoopa; the consuming engine
     *        renders `out.lines` however it likes (see the plan's toyengine-integration
     *        section for where that render pass belongs).
     */
    void debug_draw(debug::DebugDraw& out, debug::DebugDrawFlags flags = debug::DebugDrawFlags::All) const {
        if (debug::has_flag(flags, debug::DebugDrawFlags::Colliders)) {
            for (uint32_t i = 0; i < bodies_.size(); ++i) {
                if (!alive_[i] || !shapes_[i].enabled) continue;
                const collision::Shape& shape = shapes_[i];
                const dynamics::Body& body = bodies_[i];
                uint32_t color = shape.is_trigger ? debug::colors::k_trigger
                                                   : (body.awake ? debug::colors::k_awake : debug::colors::k_sleeping);
                switch (shape.type) {
                    case collision::ShapeType::Sphere:
                        out.add_sphere(collision::world_sphere(shape, body.position, body.orientation), color);
                        break;
                    case collision::ShapeType::Box:
                        out.add_obb(collision::world_obb(shape, body.position, body.orientation), color);
                        break;
                    case collision::ShapeType::Capsule:
                        out.add_capsule(collision::world_capsule(shape, body.position, body.orientation), color);
                        break;
                    case collision::ShapeType::TriangleMesh: {
                        if (!shape.mesh) break;
                        glm::mat4 transform = glm::mat4_cast(body.orientation);
                        transform[3] = glm::vec4(body.position + body.orientation * shape.local_center, 1.0f);
                        out.add_mesh(*shape.mesh, transform, color);
                        break;
                    }
                }
            }
        }

        if (debug::has_flag(flags, debug::DebugDrawFlags::Contacts)) {
            constexpr float k_normal_length = 0.3f;
            for (const auto& m : manifolds_) {
                if (m.is_trigger) continue;
                for (uint8_t i = 0; i < m.count; ++i) {
                    const glm::vec3& p = m.points[i].position;
                    out.add_line(p, p + m.normal * k_normal_length, debug::colors::k_contact_normal);
                }
            }
        }

        if (debug::has_flag(flags, debug::DebugDrawFlags::BVH)) {
            auto draw_tree = [&](const broadphase::AABBTree& tree) {
                tree.for_each_node([&](const geometry::AABB& box, int32_t) { out.add_aabb(box, debug::colors::k_bvh_node); });
            };
            draw_tree(static_tree_);
            draw_tree(dynamic_tree_);
        }
    }

    // --- Configuration ---

    void set_gravity(const glm::vec3& g) { gravity_ = g; }
    const glm::vec3& gravity() const { return gravity_; }

    util::PhysicsConfig& config() { return config_; }
    const util::PhysicsConfig& config() const { return config_; }

    // --- Stepping ---

    /**
     * @brief Accumulates `dt` and runs zero or more fixed substeps.
     *
     * `dt` is clamped to k_max_frame_time before accumulating -- mandatory, not defensive:
     * a consumer's very first frame can include all of window/device/scene initialization.
     *
     * @param dt Frame delta time in seconds (unclamped, raw).
     */
    void step(float dt) {
        events_.clear();
        accumulator_ += std::min(dt, util::k_max_frame_time);
        const float h = util::k_default_fixed_dt;
        uint32_t n = 0;
        while (accumulator_ >= h && n < config_.max_substeps) {
            step_fixed(h);
            accumulator_ -= h;
            ++n;
        }
        interpolation_alpha_ = accumulator_ / h;
    }

    /**
     * @brief Runs exactly one fixed substep, with no accumulator involved.
     *
     * This is what test.cpp and the determinism harness drive directly. Order (matching the
     * plan's architecture diagram):
     *   1. drain deferred structural commands queued by last step's on_substep
     *   2. integrate forces (gravity, drag) -> velocities
     *   3-4. brute-force broadphase (all alive-shaped pairs) + layer reject (Phase 6 adds a tree)
     *   5. narrowphase -> this step's manifolds
     *   6. (islands are built as part of the solve, alongside sleep -- see solver.h)
     *   7. fire on_substep
     *   8. solve: warm start -> velocity+relax iterations -> position correction
     *   9. integrate velocities -> positions/orientations
     *   10. sleep/wake (part of solve())
     *
     * @param h Substep length in seconds.
     */
    void step_fixed(float h) {
        drain_deferred_commands_();

        in_step_ = true;

        for_each_body([&](dynamics::BodyId, dynamics::Body& b) {
            dynamics::apply_gravity(b, gravity_);
        });
        for_each_body([&](dynamics::BodyId, dynamics::Body& b) {
            dynamics::integrate_forces(b, h);
        });

        generate_manifolds_();

        on_substep.emit(*this, h);

        dynamics::solve(bodies_, alive_, manifolds_, solver_state_, config_, h);

        for_each_body([&](dynamics::BodyId, dynamics::Body& b) {
            b.prev_position = b.position;
            b.prev_orientation = b.orientation;
            dynamics::integrate_velocities(b, h, config_.use_exact_quaternion_integration,
                                            config_.max_linear_velocity);
            dynamics::update_world_inertia(b);
        });

        in_step_ = false;
    }

    /** @brief Fractional progress (in [0,1)) through the next substep, for render interpolation. */
    float interpolation_alpha() const { return interpolation_alpha_; }

    /**
     * @brief FNV-1a hash over every alive body's position/orientation/velocity bit patterns.
     *
     * The determinism harness: two independent runs of the same scene must produce the same
     * hash after the same number of step_fixed() calls, and the hash must be invariant to
     * spawning and destroying a distant, non-interacting body mid-run (the actual test for
     * pair-ordering nondeterminism -- a same-process double-run alone would pass even with a
     * pointer-keyed hash map in the solve loop, since allocator state repeats too).
     *
     * @return FNV-1a hash of every alive body's kinematic state.
     */
    uint64_t world_state_hash() const {
        uint64_t h = 1469598103934665603ull; // FNV-1a 64-bit offset basis
        auto mix = [&h](const void* data, size_t len) {
            const uint8_t* bytes = static_cast<const uint8_t*>(data);
            for (size_t i = 0; i < len; ++i) {
                h ^= bytes[i];
                h *= 1099511628211ull; // FNV-1a 64-bit prime
            }
        };
        for_each_body([&](dynamics::BodyId, const dynamics::Body& b) {
            mix(&b.position, sizeof(b.position));
            mix(&b.orientation, sizeof(b.orientation));
            mix(&b.linear_velocity, sizeof(b.linear_velocity));
            mix(&b.angular_velocity, sizeof(b.angular_velocity));
        });
        return h;
    }

private:
    /** @brief Creates this body's broadphase proxy if its shape is enabled. Static bodies go
     *         into static_tree_ (built once, never refit here); Dynamic and Kinematic bodies
     *         go into dynamic_tree_ (refit every step in sync_broadphase_()). A body with no
     *         shape gets no proxy at all -- it never participates in broadphase or narrowphase. */
    void create_proxy_(uint32_t index) {
        const collision::Shape& shape = shapes_[index];
        if (!shape.enabled) return;
        const dynamics::Body& body = bodies_[index];
        geometry::AABB bounds = collision::world_bounds(shape, body.position, body.orientation);
        void* user_data = index_to_user_data_(index);
        if (body.type == dynamics::BodyType::Static) {
            tree_proxy_[index] = static_tree_.create_proxy(bounds, user_data);
            in_static_tree_[index] = true;
        } else {
            tree_proxy_[index] = dynamic_tree_.create_proxy(bounds, user_data);
            in_static_tree_[index] = false;
        }
    }

    /** @brief Destroys this body's broadphase proxy, if it has one. */
    void destroy_proxy_(uint32_t index) {
        if (tree_proxy_[index] == broadphase::k_null_node) return;
        if (in_static_tree_[index]) static_tree_.destroy_proxy(tree_proxy_[index]);
        else dynamic_tree_.destroy_proxy(tree_proxy_[index]);
        tree_proxy_[index] = broadphase::k_null_node;
    }

    static void* index_to_user_data_(uint32_t index) {
        return reinterpret_cast<void*>(static_cast<uintptr_t>(index));
    }
    static uint32_t user_data_to_index_(void* user_data) {
        return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(user_data));
    }

    /** @brief Refits every dynamic-tree proxy (Dynamic and Kinematic bodies) to this step's
     *         current world bounds. Static proxies never move, so static_tree_ needs no
     *         per-step work. */
    void sync_broadphase_() {
        for (uint32_t i = 0; i < bodies_.size(); ++i) {
            if (!alive_[i] || tree_proxy_[i] == broadphase::k_null_node || in_static_tree_[i]) continue;
            geometry::AABB bounds = collision::world_bounds(shapes_[i], bodies_[i].position, bodies_[i].orientation);
            dynamic_tree_.move_proxy(tree_proxy_[i], bounds, bodies_[i].linear_velocity);
        }
    }

    /**
     * @brief Broadphase (two-tree query) + narrowphase over this step's overlapping pairs.
     *
     * Static-static, static-kinematic and kinematic-kinematic pairs are never generated:
     * dynamic_tree_ holds both Dynamic and Kinematic bodies (so a self-query against it can
     * surface a kinematic-kinematic pair), filtered out by the same `either_dynamic` check
     * brute-force used; static_tree_ holds only Static bodies, so a dynamic-tree-vs-static-
     * tree cross query can only ever surface dynamic-static or kinematic-static pairs, and
     * the same filter drops the latter.
     *
     * Determinism: PairCache::add() is called while iterating bodies_ in ascending index
     * order, and finalize() sorts the result -- so manifolds_' order depends only on body
     * indices, never on tree-internal layout or query traversal order.
     */
    void generate_manifolds_() {
        sync_broadphase_();
        pair_cache_.advance(); // roll last step's pairs into "previous"; start this step fresh
        collision_pair_cache_.advance();
        trigger_pair_cache_.advance();

        for (uint32_t i = 0; i < bodies_.size(); ++i) {
            if (!alive_[i] || tree_proxy_[i] == broadphase::k_null_node) continue;
            geometry::AABB bounds = collision::world_bounds(shapes_[i], bodies_[i].position, bodies_[i].orientation);

            dynamic_tree_.query(bounds, [&](void* user_data) {
                uint32_t j = user_data_to_index_(user_data);
                pair_cache_.add(i, j);
            });
            if (in_static_tree_[i]) {
                // A static body's own tree never needs querying against itself; only a
                // dynamic/kinematic body queries the static tree, giving each cross pair
                // exactly one discovery path.
            } else {
                static_tree_.query(bounds, [&](void* user_data) {
                    uint32_t j = user_data_to_index_(user_data);
                    pair_cache_.add(i, j);
                });
            }
        }
        pair_cache_.finalize();

        manifolds_.clear();
        for (const auto& pair : pair_cache_.current()) {
            uint32_t i = pair.a;
            uint32_t j = pair.b;
            if (i >= bodies_.size() || j >= bodies_.size() || !alive_[i] || !alive_[j]) continue;

            const dynamics::Body& bi = bodies_[i];
            const dynamics::Body& bj = bodies_[j];
            bool either_dynamic = bi.type == dynamics::BodyType::Dynamic || bj.type == dynamics::BodyType::Dynamic;
            if (!either_dynamic) continue;

            if (!layer_matrix_.should_collide(shapes_[i].layer, shapes_[j].layer)) continue;

            dynamics::BodyId id_i{i, generations_[i]};
            dynamics::BodyId id_j{j, generations_[j]};

            collision::ContactManifold m;
            if (collision::generate_contacts(id_i, shapes_[i], bi.position, bi.orientation,
                                              id_j, shapes_[j], bj.position, bj.orientation, m)) {
                manifolds_.push_back(m);
                (m.is_trigger ? trigger_pair_cache_ : collision_pair_cache_).add(i, j);
            }
        }

        collision_pair_cache_.finalize();
        trigger_pair_cache_.finalize();
        emit_contact_events_(collision_pair_cache_, /*is_trigger=*/false);
        emit_contact_events_(trigger_pair_cache_, /*is_trigger=*/true);
    }

    /**
     * @brief Turns one pair cache's added/stayed/removed classification into ContactEvents,
     *        appended to `events_` for the caller to drain after step()/step_fixed() returns.
     *
     * Body indices are translated to BodyId here (not when queued into the pair cache, which
     * only ever stores raw indices) using each index's CURRENT generation -- correct because a
     * removed pair's bodies are, by definition, either still alive (an ordinary exit as they
     * separate) or were destroyed via destroy_body(), which the plan's own deferred-command
     * queue already resolves before generate_manifolds_() runs, so any index still present in
     * `removed` at this point addresses whatever now legitimately occupies that slot -- an exit
     * event for a body that no longer exists is meaningless and safe to drop, and for a slot
     * that's been reused, the generation bump makes it a different BodyId, not a stale one.
     */
    void emit_contact_events_(const broadphase::PairCache& cache, bool is_trigger) {
        std::vector<broadphase::ProxyPair> added, stayed, removed;
        cache.classify(added, stayed, removed);

        auto push = [&](const broadphase::ProxyPair& pair, collision::ContactEvent::Type type) {
            if (pair.a >= bodies_.size() || pair.b >= bodies_.size()) return;
            if ((type != collision::ContactEvent::Type::Exit) && (!alive_[pair.a] || !alive_[pair.b])) return;
            collision::ContactEvent ev;
            ev.a = dynamics::BodyId{pair.a, generations_[pair.a]};
            ev.b = dynamics::BodyId{pair.b, generations_[pair.b]};
            ev.type = type;
            ev.is_trigger = is_trigger;
            events_.push_back(ev);
        };
        for (const auto& pair : added) push(pair, collision::ContactEvent::Type::Enter);
        for (const auto& pair : stayed) push(pair, collision::ContactEvent::Type::Stay);
        for (const auto& pair : removed) push(pair, collision::ContactEvent::Type::Exit);
    }

    /** @brief Applies deferred structural commands queued by the previous step's on_substep,
     *         in the order they were pushed (deterministic, single-threaded). */
    void drain_deferred_commands_() {
        for (auto& cmd : deferred_commands_) {
            if (cmd.type == DeferredCommand::Type::Create) {
                add_body(cmd.body, cmd.shape);
            } else {
                remove_body(cmd.target);
            }
        }
        deferred_commands_.clear();
    }

    struct DeferredCommand {
        enum class Type { Create, Destroy } type = Type::Create;
        dynamics::Body body;
        collision::Shape shape;
        dynamics::BodyId target;
    };

    util::PhysicsConfig config_;
    coopa::debug::Logger logger_;

    glm::vec3 gravity_{0.0f, 0.0f, util::k_gravity_z};

    std::vector<dynamics::Body> bodies_;
    std::vector<collision::Shape> shapes_;
    std::vector<uint32_t> generations_;
    std::vector<bool> alive_;
    std::vector<uint32_t> free_indices_;

    broadphase::AABBTree static_tree_{config_.aabb_margin};
    broadphase::AABBTree dynamic_tree_{config_.aabb_margin};
    std::vector<int32_t> tree_proxy_;   /**< Parallel to bodies_; k_null_node if no proxy. */
    std::vector<bool> in_static_tree_;  /**< Parallel to bodies_; which tree tree_proxy_ indexes into. */
    broadphase::PairCache pair_cache_;
    broadphase::PairCache collision_pair_cache_; /**< Confirmed (narrowphase-hit) non-trigger pairs, for enter/stay/exit. */
    broadphase::PairCache trigger_pair_cache_;   /**< Confirmed (narrowphase-hit) trigger pairs, for enter/stay/exit. */
    broadphase::LayerMatrix layer_matrix_;

    std::vector<collision::ContactManifold> manifolds_;
    std::vector<collision::ContactEvent> events_; /**< This frame's enter/stay/exit events; cleared at the top of step(). */
    dynamics::SolverState solver_state_;

    std::vector<DeferredCommand> deferred_commands_;
    bool in_step_ = false;

    float accumulator_ = 0.0f;
    float interpolation_alpha_ = 0.0f;
};

} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_WORLD_H
