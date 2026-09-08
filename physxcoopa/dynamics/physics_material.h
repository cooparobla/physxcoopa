/**
 * @file physics_material.h
 * @brief Surface friction/restitution properties, shared by pointer across colliders --
 *        Unity's PhysicMaterial equivalent.
 */

#ifndef PHYSXCOOPA_DYNAMICS_PHYSICS_MATERIAL_H
#define PHYSXCOOPA_DYNAMICS_PHYSICS_MATERIAL_H

#include <algorithm>

namespace coopa {
namespace physx {
namespace dynamics {

/**
 * @brief How two materials' friction or restitution values combine at a contact.
 *
 * When the two materials at a contact disagree on which mode to use, Unity's asymmetric
 * tie-break applies: Multiply > Maximum > Minimum > Average (the higher-priority mode wins).
 */
enum class CombineMode {
    Average,
    Minimum,
    Maximum,
    Multiply,
};

/**
 * @struct PhysicsMaterial
 * @brief Friction and restitution for one surface. Instances are shared by pointer; a
 *        Collider with no material assigned uses PhysicsMaterial::default_material().
 */
struct PhysicsMaterial {
    float dynamic_friction = 0.6f;
    float static_friction = 0.6f;
    float restitution = 0.0f;
    CombineMode friction_combine = CombineMode::Average;
    CombineMode restitution_combine = CombineMode::Average;

    /** @brief The material used by a Collider with no PhysicsMaterial assigned. */
    static const PhysicsMaterial& default_material() {
        static const PhysicsMaterial instance{};
        return instance;
    }
};

/**
 * @brief Combines two scalar values (friction or restitution) per the higher-priority of the
 *        two materials' combine modes.
 *
 * @param a    First material's value.
 * @param b    Second material's value.
 * @param mode_a First material's combine mode.
 * @param mode_b Second material's combine mode.
 * @return Combined value.
 */
inline float combine(float a, float b, CombineMode mode_a, CombineMode mode_b) {
    CombineMode mode = std::max(mode_a, mode_b); // enum order IS the priority order below
    switch (mode) {
        case CombineMode::Multiply: return a * b;
        case CombineMode::Maximum:  return std::max(a, b);
        case CombineMode::Minimum:  return std::min(a, b);
        case CombineMode::Average:
        default:                    return 0.5f * (a + b);
    }
}

} // namespace dynamics
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_DYNAMICS_PHYSICS_MATERIAL_H
