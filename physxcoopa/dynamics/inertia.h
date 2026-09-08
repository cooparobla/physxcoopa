/**
 * @file inertia.h
 * @brief Analytic inverse-inertia tensors for the primitive shapes physxcoopa supports.
 *
 * Every shape here has principal axes aligned with its own local frame, so the local-space
 * inertia tensor is diagonal and is stored/returned as just its three diagonal entries
 * (Body::inv_inertia_local). The world-space tensor (generally non-diagonal once rotated)
 * is reconstructed each substep as `R * diag(inv_inertia_local) * R^T` -- see body.h.
 */

#ifndef PHYSXCOOPA_DYNAMICS_INERTIA_H
#define PHYSXCOOPA_DYNAMICS_INERTIA_H

#include <physxcoopa/util/math.h>

#include <glm/glm.hpp>

namespace coopa {
namespace physx {
namespace dynamics {

/**
 * @brief Inverse of the diagonal local-space inertia tensor for a solid box.
 *
 * @param half_extents Box half-extents.
 * @param mass         Body mass.
 * @return Diagonal (Ixx, Iyy, Izz)^-1. A component is 0 (infinite inertia about that axis)
 *         when `mass` is 0 (used for static/kinematic bodies, whose inv_mass is also 0).
 */
inline glm::vec3 box_inverse_inertia(const glm::vec3& half_extents, float mass) {
    if (mass <= 0.0f) return glm::vec3(0.0f);
    glm::vec3 e = 2.0f * half_extents; // full extents
    glm::vec3 e2 = e * e;
    glm::vec3 i;
    i.x = mass * (e2.y + e2.z) / 12.0f;
    i.y = mass * (e2.x + e2.z) / 12.0f;
    i.z = mass * (e2.x + e2.y) / 12.0f;
    return glm::vec3(1.0f / std::max(i.x, util::k_epsilon),
                      1.0f / std::max(i.y, util::k_epsilon),
                      1.0f / std::max(i.z, util::k_epsilon));
}

/**
 * @brief Inverse of the local-space inertia tensor for a solid sphere (isotropic).
 *
 * @param radius Sphere radius.
 * @param mass   Body mass.
 * @return Diagonal (I,I,I)^-1, I = (2/5) m r^2.
 */
inline glm::vec3 sphere_inverse_inertia(float radius, float mass) {
    if (mass <= 0.0f) return glm::vec3(0.0f);
    float i = 0.4f * mass * radius * radius;
    float inv = 1.0f / std::max(i, util::k_epsilon);
    return glm::vec3(inv);
}

/**
 * @brief Inverse of the local-space inertia tensor for a solid capsule (cylinder + two
 *        hemispherical caps), with the capsule's long axis along local Z.
 *
 * Mass is distributed between the cylindrical and hemispherical parts by volume ratio.
 * Derivation: a solid hemisphere has moment of inertia (2/5) m_h r^2 about an axis through
 * the ORIGINAL sphere's center (i.e. the hemisphere's flat face), for both the polar axis
 * and any axis perpendicular to it through that same point -- the two are equal there by
 * the same symmetry that gives a full sphere (2/5) M R^2 about any diameter. Each
 * hemisphere's flat face sits `half_height` from the capsule's center of mass, so the
 * perpendicular (X/Y) inertia additionally picks up a parallel-axis term m_h * half_height^2;
 * the axial (Z) inertia does not, since translation along the axis itself doesn't change an
 * axial moment.
 *
 * @param radius      Capsule radius.
 * @param half_height Distance from the capsule's center to a hemisphere's flat face (i.e.
 *                     the cylindrical section's half-length -- NOT including the caps).
 * @param mass        Body mass.
 * @return Diagonal (Ixx, Iyy, Izz)^-1 in the capsule's own local frame, Z = long axis.
 *         Permute this to match a CapsuleCollider's `direction` (0=X, 1=Y, 2=Z) before
 *         assigning to Body::inv_inertia_local.
 */
inline glm::vec3 capsule_inverse_inertia(float radius, float half_height, float mass) {
    if (mass <= 0.0f) return glm::vec3(0.0f);

    float r = radius;
    float hh = half_height;

    float vol_cyl = 3.14159265358979323846f * r * r * (2.0f * hh);
    float vol_hemi = (4.0f / 3.0f) * 3.14159265358979323846f * r * r * r; // both hemispheres combined
    float vol_total = vol_cyl + vol_hemi;
    if (vol_total < util::k_epsilon) return sphere_inverse_inertia(r, mass);

    float m_cyl = mass * (vol_cyl / vol_total);
    float m_hemi = mass * (vol_hemi / vol_total); // both hemispheres combined

    float i_axial = 0.5f * m_cyl * r * r + 0.4f * m_hemi * r * r;
    float i_perp = m_cyl * (3.0f * r * r + 4.0f * hh * hh) / 12.0f
                 + 0.4f * m_hemi * r * r
                 + m_hemi * hh * hh;

    return glm::vec3(1.0f / std::max(i_perp, util::k_epsilon),
                      1.0f / std::max(i_perp, util::k_epsilon),
                      1.0f / std::max(i_axial, util::k_epsilon));
}

} // namespace dynamics
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_DYNAMICS_INERTIA_H
