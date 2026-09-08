/**
 * @file solver.h
 * @brief Sequential-impulse split-impulse solver: warm start, base velocity iterations,
 *        a final restitution correction, position correction, then island-based sleep/wake.
 *
 * Split-impulse, not Baumgarte-in-the-velocity-solver: the velocity solve below carries zero
 * positional bias -- all positional correction happens in solve_position(), separately, after
 * velocity has already converged. Pairing a velocity-solver bias with a separate position pass
 * (the more common "textbook" scheme) double-corrects, since the bias injects energy the
 * position pass then has to absorb again -- visible as stacks that gently "breathe."
 *
 * Restitution ordering: the main velocity iterations always target 0 (non-penetration +
 * friction), converging a base resting/sliding solution across every contact; restitution is
 * then applied as a small number of FINAL iterations on top of that base, not mixed into it
 * and not run before it -- see solve()'s comment for why either of those orderings quietly
 * cancels the bounce it was supposed to produce.
 */

#ifndef PHYSXCOOPA_DYNAMICS_SOLVER_H
#define PHYSXCOOPA_DYNAMICS_SOLVER_H

#include <physxcoopa/dynamics/body.h>
#include <physxcoopa/dynamics/island.h>
#include <physxcoopa/collision/manifold.h>
#include <physxcoopa/util/config.h>
#include <physxcoopa/util/math.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace coopa {
namespace physx {
namespace dynamics {

/**
 * @brief Packs an ordered body-index pair and a feature id into one warm-start cache key.
 *
 * Ordering by (min,max) index means the same physical contact hashes identically regardless
 * of which body happens to be manifold.a vs manifold.b this step.
 */
inline uint64_t pack_warm_start_key(uint32_t index_a, uint32_t index_b, uint32_t feature_id) {
    uint32_t lo = std::min(index_a, index_b);
    uint32_t hi = std::max(index_a, index_b);
    uint64_t key = (static_cast<uint64_t>(lo) << 40) ^ (static_cast<uint64_t>(hi) << 12) ^ feature_id;
    return key;
}

/**
 * @struct SolverState
 * @brief Solver state that persists across steps -- currently just the warm-start impulse
 *        cache. Owned by PhysicsWorld, passed into solve() by reference each substep.
 *
 * The cache is a hash map keyed by an integer (not a pointer), and is only ever touched via
 * point lookups/inserts by a specific computed key -- never iterated for output -- so it does
 * not violate the "no pointer-keyed hash map may be iterated inside step_fixed" determinism
 * rule (see the plan's "Determinism and parallelism").
 */
struct SolverState {
    struct CachedImpulse {
        float normal = 0.0f;
        float tangent[2] = {0.0f, 0.0f};
    };
    std::unordered_map<uint64_t, CachedImpulse> impulses;
};

/** @brief Applies a world-space impulse at `point` to the pair (A,B): -impulse to A, +impulse
 *         to B (impulse is defined along the manifold's A-to-B normal convention). No-op on
 *         the side(s) that aren't dynamic. */
inline void apply_impulse_pair(Body& a, Body& b, const glm::vec3& point, const glm::vec3& impulse) {
    if (a.type == BodyType::Dynamic) {
        a.linear_velocity -= impulse * a.inv_mass;
        a.angular_velocity -= a.inv_inertia_world * glm::cross(point - a.position, impulse);
    }
    if (b.type == BodyType::Dynamic) {
        b.linear_velocity += impulse * b.inv_mass;
        b.angular_velocity += b.inv_inertia_world * glm::cross(point - b.position, impulse);
    }
}

/**
 * @brief Captures each contact point's pre-solve approach velocity and derives its
 *        restitution target, before any impulse (warm-start or otherwise) is applied this step.
 *
 * `restitution_bias` is left at 0 (a resting contact) unless the approach speed exceeds
 * config.restitution_threshold -- below that, restitution is deliberately zero regardless of
 * the material's configured value, which is what stops resting contacts from micro-bouncing.
 */
inline void capture_restitution_bias(std::vector<Body>& bodies, std::vector<collision::ContactManifold>& manifolds,
                                      const util::PhysicsConfig& config) {
    for (auto& m : manifolds) {
        if (!m.valid || m.is_trigger) continue;
        Body& a = bodies[m.a.index];
        Body& b = bodies[m.b.index];
        for (uint8_t i = 0; i < m.count; ++i) {
            auto& p = m.points[i];
            glm::vec3 ra = p.position - a.position;
            glm::vec3 rb = p.position - b.position;
            glm::vec3 va = a.linear_velocity + glm::cross(a.angular_velocity, ra);
            glm::vec3 vb = b.linear_velocity + glm::cross(b.angular_velocity, rb);
            float v_approach = glm::dot(vb - va, m.normal);
            p.restitution_bias = (-v_approach > config.restitution_threshold) ? -m.restitution * v_approach : 0.0f;
        }
    }
}

/** @brief Seeds each point's impulse accumulators from the previous step's cache (if any
 *         point with the same key existed) and immediately re-applies that impulse, so the
 *         first velocity iteration this step starts from last step's converged solution
 *         instead of zero -- the single biggest quality difference between a settling stack
 *         and one that jitters forever. */
inline void warm_start(std::vector<Body>& bodies, std::vector<collision::ContactManifold>& manifolds,
                        SolverState& state) {
    for (auto& m : manifolds) {
        if (!m.valid || m.is_trigger) continue;
        Body& a = bodies[m.a.index];
        Body& b = bodies[m.b.index];
        bool a_active = a.type == BodyType::Dynamic && a.awake;
        bool b_active = b.type == BodyType::Dynamic && b.awake;
        if (!a_active && !b_active) continue; // a sleeping body has no gravity to counteract
                                               // this step -- re-applying its last impulse
                                               // every step would accelerate it forever
        glm::vec3 t1, t2;
        util::orthonormal_basis(m.normal, t1, t2);

        for (uint8_t i = 0; i < m.count; ++i) {
            auto& p = m.points[i];
            uint64_t key = pack_warm_start_key(m.a.index, m.b.index, p.feature_id);
            auto it = state.impulses.find(key);
            if (it != state.impulses.end()) {
                p.normal_impulse = it->second.normal;
                p.tangent_impulse[0] = it->second.tangent[0];
                p.tangent_impulse[1] = it->second.tangent[1];
            } else {
                p.normal_impulse = 0.0f;
                p.tangent_impulse[0] = 0.0f;
                p.tangent_impulse[1] = 0.0f;
            }
            glm::vec3 impulse = m.normal * p.normal_impulse + t1 * p.tangent_impulse[0] + t2 * p.tangent_impulse[1];
            if (glm::dot(impulse, impulse) > 0.0f) apply_impulse_pair(a, b, p.position, impulse);
        }
    }
}

/**
 * @brief One Gauss-Seidel velocity-iteration pass over every manifold point: solves the
 *        clamped normal constraint (with restitution when `use_restitution`), then Coulomb
 *        friction against the just-updated accumulated normal impulse, applying each point's
 *        impulse immediately (so later points/manifolds see its effect this same pass).
 *
 * Sequential (not Jacobi) application is load-bearing here, not just the simpler option: a
 * box resting on a slope needs friction (acting at the contact face, offset from the center
 * of mass along the normal) to redistribute normal force asymmetrically across the contact
 * points -- more on the downhill edge, less on the uphill edge -- which is what actually
 * keeps a physically-flush box from rotating/sliding under a friction coefficient above
 * tan(slope angle). That redistribution is a real physical effect (shifting the torque
 * reference point from the contact patch to the center of mass, which sits off the patch
 * along the normal, turns a symmetric tangential force into a net torque -- see the
 * accompanying plan notes), and it converges far faster when each point reacts to the
 * immediately-preceding points' impulses than when all points solve from one shared
 * pre-pass snapshot.
 *
 * `order_offset` rotates which point in each manifold is solved first across calls (see
 * solve()) -- for a manifold with no genuine asymmetric-load requirement (e.g. a box resting
 * flat on another box), sequential order alone gives the first-solved point a consistent
 * advantage that shows up as a small torque bias, which does not average out over repeated
 * iterations in the SAME order; cycling the starting point through all of a manifold's points
 * (rather than just reversing, which only swaps a 2-way bias rather than cancelling it) does,
 * while still preserving the sequential coupling the slope case above depends on.
 *
 * Never applies positional bias -- see the file doc for why that's the whole point.
 */
inline void solve_velocity_pass(std::vector<Body>& bodies, std::vector<collision::ContactManifold>& manifolds,
                                 bool use_restitution, uint32_t order_offset) {
    for (auto& m : manifolds) {
        if (!m.valid || m.is_trigger) continue;
        Body& a = bodies[m.a.index];
        Body& b = bodies[m.b.index];
        bool a_dyn = a.type == BodyType::Dynamic && a.awake;
        bool b_dyn = b.type == BodyType::Dynamic && b.awake;
        if (!a_dyn && !b_dyn) continue;

        float inv_mass_a = a_dyn ? a.inv_mass : 0.0f;
        float inv_mass_b = b_dyn ? b.inv_mass : 0.0f;
        glm::mat3 inv_ia = a_dyn ? a.inv_inertia_world : glm::mat3(0.0f);
        glm::mat3 inv_ib = b_dyn ? b.inv_inertia_world : glm::mat3(0.0f);

        glm::vec3 t1, t2;
        util::orthonormal_basis(m.normal, t1, t2);

        for (uint8_t k = 0; k < m.count; ++k) {
            uint8_t i = static_cast<uint8_t>((k + order_offset) % m.count);
            auto& p = m.points[i];
            glm::vec3 ra = p.position - a.position;
            glm::vec3 rb = p.position - b.position;

            // --- Normal constraint ---
            {
                glm::vec3 ra_x_n = glm::cross(ra, m.normal);
                glm::vec3 rb_x_n = glm::cross(rb, m.normal);
                float k_normal = inv_mass_a + inv_mass_b +
                                  glm::dot(ra_x_n, inv_ia * ra_x_n) +
                                  glm::dot(rb_x_n, inv_ib * rb_x_n);
                if (k_normal > util::k_epsilon) {
                    glm::vec3 va = a.linear_velocity + glm::cross(a.angular_velocity, ra);
                    glm::vec3 vb = b.linear_velocity + glm::cross(b.angular_velocity, rb);
                    float vn = glm::dot(vb - va, m.normal);
                    float target = use_restitution ? p.restitution_bias : 0.0f;
                    float lambda = -(vn - target) / k_normal;
                    float new_impulse = std::max(p.normal_impulse + lambda, 0.0f);
                    float delta = new_impulse - p.normal_impulse;
                    p.normal_impulse = new_impulse;
                    apply_impulse_pair(a, b, p.position, m.normal * delta);
                }
            }

            // --- Friction (Coulomb, bounded by the just-updated accumulated normal impulse) ---
            const glm::vec3* tangents[2] = {&t1, &t2};
            for (int t = 0; t < 2; ++t) {
                const glm::vec3& dir = *tangents[t];
                glm::vec3 ra_x_t = glm::cross(ra, dir);
                glm::vec3 rb_x_t = glm::cross(rb, dir);
                float k_t = inv_mass_a + inv_mass_b +
                            glm::dot(ra_x_t, inv_ia * ra_x_t) +
                            glm::dot(rb_x_t, inv_ib * rb_x_t);
                if (k_t <= util::k_epsilon) continue;

                glm::vec3 va = a.linear_velocity + glm::cross(a.angular_velocity, ra);
                glm::vec3 vb = b.linear_velocity + glm::cross(b.angular_velocity, rb);
                float vt = glm::dot(vb - va, dir);
                float lambda_t = -vt / k_t;
                float max_friction = m.friction * p.normal_impulse;
                float old_impulse = p.tangent_impulse[t];
                float new_impulse = std::clamp(old_impulse + lambda_t, -max_friction, max_friction);
                float delta = new_impulse - old_impulse;
                p.tangent_impulse[t] = new_impulse;
                apply_impulse_pair(a, b, p.position, dir * delta);
            }
        }
    }
}

/**
 * @brief Linear-only non-linear-Gauss-Seidel position correction: nudges body positions
 *        directly along each manifold's normal, tracking how much correction has already
 *        been consumed via each body's cumulative displacement since `ref_position` so
 *        successive iterations converge rather than repeatedly over-correcting the same
 *        original penetration value.
 *
 * v1 simplification: rotational correction is not applied, only translation -- acceptable
 * for the shapes/scenarios v1 targets, and does not inject velocity (only position moves).
 */
inline void solve_position(std::vector<Body>& bodies, std::vector<collision::ContactManifold>& manifolds,
                            const util::PhysicsConfig& config) {
    std::vector<glm::vec3> ref_position(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i) ref_position[i] = bodies[i].position;

    for (uint32_t iter = 0; iter < config.position_iterations; ++iter) {
        for (auto& m : manifolds) {
            if (!m.valid || m.is_trigger) continue;
            Body& a = bodies[m.a.index];
            Body& b = bodies[m.b.index];
            bool a_dyn = a.type == BodyType::Dynamic && a.awake;
            bool b_dyn = b.type == BodyType::Dynamic && b.awake;
            if (!a_dyn && !b_dyn) continue;

            float inv_mass_a = a_dyn ? a.inv_mass : 0.0f;
            float inv_mass_b = b_dyn ? b.inv_mass : 0.0f;
            float k = inv_mass_a + inv_mass_b;
            if (k <= util::k_epsilon) continue;

            glm::vec3 da = a.position - ref_position[m.a.index];
            glm::vec3 db = b.position - ref_position[m.b.index];

            for (uint8_t i = 0; i < m.count; ++i) {
                auto& p = m.points[i];
                float current_penetration = p.penetration - glm::dot(db - da, m.normal);
                float correction_mag = std::max(0.0f, current_penetration - config.linear_slop) * config.position_correction;
                correction_mag = std::min(correction_mag, config.max_linear_correction);
                if (correction_mag <= 0.0f) continue;

                glm::vec3 correction = m.normal * (correction_mag / k);
                if (a_dyn) { a.position -= correction * inv_mass_a; da = a.position - ref_position[m.a.index]; }
                if (b_dyn) { b.position += correction * inv_mass_b; db = b.position - ref_position[m.b.index]; }
            }
        }
    }
}

/**
 * @brief Rebuilds the warm-start cache from this step's fully-solved manifolds, so next
 *        step's warm_start() has something to seed from.
 */
inline void rebuild_warm_start_cache(std::vector<collision::ContactManifold>& manifolds, SolverState& state) {
    state.impulses.clear();
    for (auto& m : manifolds) {
        if (!m.valid || m.is_trigger) continue;
        for (uint8_t i = 0; i < m.count; ++i) {
            const auto& p = m.points[i];
            uint64_t key = pack_warm_start_key(m.a.index, m.b.index, p.feature_id);
            SolverState::CachedImpulse cached;
            cached.normal = p.normal_impulse;
            cached.tangent[0] = p.tangent_impulse[0];
            cached.tangent[1] = p.tangent_impulse[1];
            state.impulses[key] = cached;
        }
    }
}

/**
 * @brief Builds this step's islands, updates per-body sleep timers, applies the kinematic
 *        wake rule, and puts fully-settled islands to sleep.
 *
 * Wake rule (beyond "new contact / applied force / teleport"): a sleeping dynamic body
 * touching a kinematic body whose derived velocity is non-zero wakes immediately, regardless
 * of contact freshness -- otherwise a box asleep on a stationary platform that starts moving
 * would hover in place while the platform slides out from under it.
 */
inline void update_islands_and_sleep(std::vector<Body>& bodies, const std::vector<bool>& alive,
                                      std::vector<collision::ContactManifold>& manifolds,
                                      const util::PhysicsConfig& config, float h) {
    IslandUnionFind uf(bodies.size());

    for (auto& m : manifolds) {
        if (!m.valid) continue;
        bool a_dyn = bodies[m.a.index].type == BodyType::Dynamic;
        bool b_dyn = bodies[m.b.index].type == BodyType::Dynamic;
        if (a_dyn && b_dyn) uf.unite(m.a.index, m.b.index);
    }

    for (uint32_t i = 0; i < bodies.size(); ++i) {
        if (!alive[i]) continue;
        Body& b = bodies[i];
        if (b.type != BodyType::Dynamic || !b.awake) continue;
        float lin2 = glm::dot(b.linear_velocity, b.linear_velocity);
        float ang2 = glm::dot(b.angular_velocity, b.angular_velocity);
        if (lin2 < config.sleep_linear * config.sleep_linear && ang2 < config.sleep_angular * config.sleep_angular) {
            b.sleep_timer += h;
        } else {
            b.sleep_timer = 0.0f;
        }
    }

    // Kinematic wake rule.
    for (auto& m : manifolds) {
        if (!m.valid) continue;
        Body& a = bodies[m.a.index];
        Body& b = bodies[m.b.index];
        auto wake_if_platform_moving = [&](Body& kin, Body& dyn) {
            if (kin.type != BodyType::Kinematic || dyn.type != BodyType::Dynamic || dyn.awake) return;
            if (glm::dot(kin.linear_velocity, kin.linear_velocity) > config.sleep_linear * config.sleep_linear) {
                dyn.awake = true;
                dyn.sleep_timer = 0.0f;
            }
        };
        wake_if_platform_moving(a, b);
        wake_if_platform_moving(b, a);
    }

    // Island-level sleep decision: an island sleeps only once EVERY member's own timer has
    // reached sleep_time (the least-still body in the island gates the whole group) -- lets a
    // multi-body stack sleep as a unit instead of the bottom sleeping while the top settles.
    std::unordered_map<uint32_t, float> island_min_timer;
    for (uint32_t i = 0; i < bodies.size(); ++i) {
        if (!alive[i]) continue;
        Body& b = bodies[i];
        if (b.type != BodyType::Dynamic || !b.awake) continue;
        uint32_t root = uf.find(i);
        auto it = island_min_timer.find(root);
        if (it == island_min_timer.end() || b.sleep_timer < it->second) island_min_timer[root] = b.sleep_timer;
    }
    for (uint32_t i = 0; i < bodies.size(); ++i) {
        if (!alive[i]) continue;
        Body& b = bodies[i];
        if (b.type != BodyType::Dynamic || !b.awake) continue;
        uint32_t root = uf.find(i);
        if (island_min_timer[root] >= config.sleep_time) {
            b.awake = false;
            b.linear_velocity = glm::vec3(0.0f);
            b.angular_velocity = glm::vec3(0.0f);
        }
    }
}

/**
 * @brief Runs the full solve: warm start, restitution capture, velocity iterations, relax
 *        iterations, position correction, cache rebuild, then islands/sleep.
 *
 * @param bodies    Body array (mutated in place).
 * @param alive     Parallel liveness flags.
 * @param manifolds This step's narrowphase output (mutated: impulse accumulators, positions).
 * @param state     Cross-step warm-start cache.
 * @param config    Solver tunables.
 * @param h         Substep length in seconds.
 */
inline void solve(std::vector<Body>& bodies, const std::vector<bool>& alive,
                   std::vector<collision::ContactManifold>& manifolds,
                   SolverState& state, const util::PhysicsConfig& config, float h) {
    capture_restitution_bias(bodies, manifolds, config);
    warm_start(bodies, manifolds, state);

    // Main pass targets 0 (non-penetration + friction), converging the base resting/sliding
    // solution across every contact. Restitution is applied AFTERWARD, as a final correction
    // on top of that converged base -- if it ran first (or were re-targeted to 0 afterward),
    // a later zero-target pass would cancel the very bounce it just added, since nothing
    // distinguishes "this point is genuinely separating on purpose" from "this point should
    // be resting" once both have been driven toward the same target. Matches how Box2D
    // applies restitution: a single dedicated correction after the main solve, never undone.
    for (uint32_t i = 0; i < config.velocity_iterations; ++i) {
        solve_velocity_pass(bodies, manifolds, /*use_restitution=*/false, /*order_offset=*/i);
    }
    for (uint32_t i = 0; i < config.relax_iterations; ++i) {
        solve_velocity_pass(bodies, manifolds, /*use_restitution=*/true, /*order_offset=*/i);
    }

    solve_position(bodies, manifolds, config);
    rebuild_warm_start_cache(manifolds, state);
    update_islands_and_sleep(bodies, alive, manifolds, config, h);
}

} // namespace dynamics
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_DYNAMICS_SOLVER_H
