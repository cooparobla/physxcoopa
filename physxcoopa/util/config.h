/**
 * @file config.h
 * @brief Solver and simulation tunables, collected in one struct so they are discoverable
 *        and directly testable rather than scattered as magic numbers through the solver.
 */

#ifndef PHYSXCOOPA_UTIL_CONFIG_H
#define PHYSXCOOPA_UTIL_CONFIG_H

#include <physxcoopa/util/math.h>

#include <cstdint>

namespace coopa {
namespace physx {
namespace util {

/**
 * @struct PhysicsConfig
 * @brief All solver/broadphase tunables for one PhysicsWorld.
 */
struct PhysicsConfig {
    /** @brief Sequential-impulse iterations for the main velocity solve. Deliberately higher
     *         than the "textbook" 8: a resting multi-point contact under an off-axis load
     *         (e.g. a box on a slope, where friction needs to redistribute normal force
     *         asymmetrically across the contact face -- see solve_velocity_pass()'s doc)
     *         converges markedly better at 16 than at 8 in practice. */
    uint32_t velocity_iterations = 16;

    /** @brief Final restitution-correction iterations, run AFTER the main (always-target-0)
     *         velocity pass and before position correction -- applying restitution as a
     *         last correction on top of the converged resting/sliding solution, rather than
     *         mixed into the main pass, is what stops it from being immediately cancelled by
     *         later target-0 iterations (which cannot distinguish "genuinely bouncing" from
     *         "should be resting" once both already sit at the same target). */
    uint32_t relax_iterations = 2;

    /** @brief Non-linear Gauss-Seidel position-correction iterations. */
    uint32_t position_iterations = 4;

    /** @brief Fraction of penetration (beyond linear_slop) corrected per position iteration.
     *         Used ONLY in the position pass -- the velocity solve carries zero positional
     *         bias (split-impulse design; see the plan's "Design rationale"). */
    float position_correction = 0.2f;

    /** @brief Penetration allowed to remain uncorrected (avoids jitter from correcting to
     *         exactly zero). */
    float linear_slop = 0.005f;

    /** @brief Maximum per-iteration positional correction, to avoid a single deep-penetration
     *         contact injecting a huge correction in one step. */
    float max_linear_correction = 0.2f;

    /** @brief Minimum pre-solve approach speed for restitution to apply at all; below this,
     *         a contact is treated as resting (restitution = 0) regardless of the material's
     *         configured value. 0.5 m/s, not the more common 1.0 -- at 1.0 m/s a 5cm drop in
     *         this Z-up scene at 60 Hz impacts right at the threshold and would flip
     *         unpredictably between bouncy and dead across runs. */
    float restitution_threshold = 0.5f;

    /** @brief Below this linear speed (m/s) a body accumulates sleep time. */
    float sleep_linear = 0.01f;

    /** @brief Below this angular speed (rad/s) a body accumulates sleep time. */
    float sleep_angular = 0.02f;

    /** @brief Seconds below both sleep thresholds before an island is put to sleep. */
    float sleep_time = 0.5f;

    /** @brief Fat-AABB margin added around a broadphase proxy so it only needs
     *         re-insertion once it escapes its enlarged bounds. */
    float aabb_margin = 0.1f;

    /** @brief Maximum fixed substeps run per PhysicsWorld::step() call. */
    uint32_t max_substeps = 8;

    /** @brief Substep length in seconds -- Unity's Time.fixedDeltaTime equivalent. Defaults to
     *         the historical 1/60 constant; PhysicsWorld::step() reads this field rather than
     *         the constant directly, so a scene's `physics:` settings block can override it. */
    float fixed_dt = k_default_fixed_dt;

    /** @brief Hard clamp on a dynamic body's linear speed. Originally v1's entire stand-in for
     *         CCD (see the plan's "Design rationale" for why the FIRST attempt at margin-
     *         inflated speculative contacts didn't work against that era's velocity-solver
     *         bias); real speculative contacts now exist (world.h's narrowphase_() fast-pair
     *         gate, collision::generate_contacts()'s `allow_speculative`, this file's own
     *         solve_velocity_pass()) and are the PRIMARY tunneling defense for the shape pairs
     *         they cover (Sphere/Box, not yet Capsule or TriangleMesh -- see
     *         generate_contacts()'s doc). This clamp remains as a cheap backstop for whatever
     *         isn't covered (a fast body against a static mesh, or two bodies both moving fast
     *         enough simultaneously that neither's per-body gate alone tells the full story),
     *         not the primary mechanism anymore. */
    float max_linear_velocity = 200.0f;

    /** @brief Use the exact exponential-map quaternion integration step instead of the
     *         default first-order form, which visibly gains energy on fast spinners. */
    bool use_exact_quaternion_integration = false;

    /** @brief XPBD substeps every cloth runs per 1/60 s, unless a cloth overrides it via
     *         ClothParams::substeps.
     *
     *  Cloth is stepped once per FRAME, not per fixed substep (see PhysicsWorld::step_cloths_),
     *  so what this field really pins down is the INTERNAL substep length, `fixed_dt /
     *  cloth_substeps` -- 1/240 s by default. step_cloths_() sizes each frame's count to hold that
     *  length as the frame rate varies, which keeps both the sheet's behaviour and its cost per
     *  second frame-rate independent; at 60 Hz the count is exactly this value.
     *
     *  4 (i.e. an effective 240 Hz for cloth) is where a mid-weight sheet stops showing visible
     *  stretch as its anchor body moves, and is still under 0.15 ms for a 25x25 sheet. This is
     *  the dominant quality knob for cloth: Macklin et al.'s central result is that for a fixed
     *  budget, spending it on more SUBSTEPS converges strictly better than spending it on more
     *  iterations within a substep -- which is why this defaults to 4 while cloth_iterations
     *  defaults to 1, the reverse of the ratio the velocity/relax iterations use for rigid
     *  contacts. */
    uint32_t cloth_substeps = 4;

    /** @brief Constraint iterations per cloth substep, unless a cloth overrides it via
     *         ClothParams::iterations. See cloth_substeps for why 1 is the right default. Raise
     *         only for a sheet that must be perfectly inextensible at very low substep counts. */
    uint32_t cloth_iterations = 1;
};

} // namespace util
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_UTIL_CONFIG_H
