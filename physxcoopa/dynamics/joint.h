/**
 * @file joint.h
 * @brief HingeJoint -- a revolute (1 rotational DOF) constraint between two bodies, and its
 *        generational handle.
 *
 * Only the hinge type ships in v1 -- covers a door (limited swing) and a passive cantilever
 * (unlimited swing) equally well. A rigid "fixed joint" needs no separate implementation:
 * `use_limits` with `min_angle == max_angle` (typically both 0) already locks the one DOF a
 * hinge leaves free, on top of the point + axis-alignment constraints that are always active --
 * see solve_joint_velocity_pass()'s doc in dynamics/solver.h. No motor, no spring/damper (not
 * needed for a door or a passive cantilever; a natural, separable future addition if ever
 * needed -- see this file's design notes in the project's own planning history).
 */

#ifndef PHYSXCOOPA_DYNAMICS_JOINT_H
#define PHYSXCOOPA_DYNAMICS_JOINT_H

#include <physxcoopa/dynamics/body.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>

namespace coopa {
namespace physx {
namespace dynamics {

/**
 * @struct JointId
 * @brief Generational handle into PhysicsWorld's joint array -- same shape as BodyId (see its
 *        own doc for why: a plain, non-atomic generation bump is enough since v1's joint array
 *        is touched only from physxcoopa's single owning thread).
 */
struct JointId {
    static constexpr uint32_t k_invalid_index = 0xFFFFFFFFu;

    uint32_t index = k_invalid_index;
    uint32_t generation = 0;

    bool is_valid() const { return index != k_invalid_index; }

    bool operator==(const JointId& other) const {
        return index == other.index && generation == other.generation;
    }
    bool operator!=(const JointId& other) const { return !(*this == other); }
};

/**
 * @struct HingeJoint
 * @brief One hinge constraint between bodies `a` and `b`.
 *
 * Anchors/axes are in each body's OWN local frame (rotated/translated by that body's own
 * position/orientation to get the world-space constraint target each solve -- exactly how
 * collision::Shape's local_center/local_rotation already work, see collision/shape.h).
 *
 * Unlike ContactManifold (rediscovered fresh by broadphase every substep, so its warm-start
 * impulses need a stable cross-step key -- see dynamics/solver.h's pack_warm_start_key()),
 * a HingeJoint is an explicit, persistent object: the same struct instance is reused substep to
 * substep (PhysicsWorld::joints_[id.index]), so its impulse accumulators just live as plain
 * fields here and are automatically "warm" every step with no key/hash-map machinery needed.
 */
struct HingeJoint {
    BodyId a;
    BodyId b;

    glm::vec3 local_anchor_a{0.0f};
    glm::vec3 local_anchor_b{0.0f};
    /** @brief Hinge axis in each body's own local frame -- need not be unit length as authored,
     *         normalized wherever it's read (see solve_joint_velocity_pass()'s doc). */
    glm::vec3 local_axis_a{0.0f, 0.0f, 1.0f};
    glm::vec3 local_axis_b{0.0f, 0.0f, 1.0f};

    bool use_limits = false;
    float min_angle = 0.0f; /**< Radians, relative to `rest_relative_rotation`. */
    float max_angle = 0.0f; /**< Radians, relative to `rest_relative_rotation`. */

    /** @brief `inverse(a.orientation) * b.orientation` at the moment this joint was created --
     *         the reference the current swing angle is measured against for limits (Unity's
     *         HingeJoint.limits are relative to the joint's initial relative orientation the
     *         same way). Set once by PhysicsWorld::add_hinge_joint(), never touched again. */
    glm::quat rest_relative_rotation{1.0f, 0.0f, 0.0f, 0.0f};

    /** @brief Per-world-axis (X,Y,Z) point-constraint impulse accumulator. */
    glm::vec3 point_impulse{0.0f};
    /** @brief Per-perpendicular-axis (see solve_joint_velocity_pass()'s `p`/`q` basis) axis-
     *         alignment impulse accumulator. */
    glm::vec2 axis_impulse{0.0f};
    /** @brief Angle-limit impulse accumulator -- meaningful only when `use_limits`. */
    float limit_impulse = 0.0f;

    bool enabled = true;
    /** @brief False once either body is destroyed -- mirrors ContactManifold::valid; a joint
     *         referencing a dead body is skipped by every solver pass, never dereferenced. */
    bool valid = true;

    /** @brief When false (the default -- matching Unity's `HingeJoint.enableCollision`), `a` and
     *         `b` never generate contacts against EACH OTHER, regardless of the layer matrix: a
     *         hinge's whole reason to exist is that the two bodies touch right at the anchor (a
     *         door's edge sits flush against its frame), so without this the contact solver would
     *         fight the joint solver over that same overlap every step -- the joint constraint
     *         wants them to interpenetrate exactly at the hinge line, contact resolution wants to
     *         push them apart, and the two fight to a standstill. See PhysicsWorld::narrowphase_()
     *         for where this is consulted, right alongside the layer-matrix check it sits next to. */
    bool collide_connected = false;
};

} // namespace dynamics
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_DYNAMICS_JOINT_H
