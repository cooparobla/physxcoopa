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
#include <physxcoopa/collision/shape.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>

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

/**
 * @brief Dispatches to the analytic inverse-inertia function matching `shape.type`, permuting
 *        the capsule result to match `shape.capsule_axis` (0=X, 1=Y, 2=Z; see
 *        capsule_inverse_inertia()'s doc -- its own result is Z-axis-local).
 *
 * Lives here (not in PhysicsSystem) because it depends on nothing but a Shape and a mass -- no
 * Scene/Collider involvement -- so PhysicsWorld::set_body_type() can share it for a runtime Kinematic->Dynamic transition without
 * PhysicsWorld (which has no Scene dependency, see world.h's file doc) reaching into
 * system/physics_system.h.
 *
 * @param shape Shape to derive inertia from.
 * @param mass  Body mass.
 * @return Diagonal inverse inertia tensor, or (0,0,0) for TriangleMesh (matches
 *         PhysicsSystem::create_compound_body_()'s dynamic-mesh-collider rejection -- this dispatch
 *         is never actually reached with a Dynamic TriangleMesh body in practice).
 */
inline glm::vec3 inertia_for_shape(const collision::Shape& shape, float mass) {
    switch (shape.type) {
        case collision::ShapeType::Sphere:
            return sphere_inverse_inertia(shape.radius, mass);
        case collision::ShapeType::Box:
            return box_inverse_inertia(shape.half_extents, mass);
        case collision::ShapeType::Capsule: {
            glm::vec3 raw = capsule_inverse_inertia(shape.capsule_radius, shape.capsule_half_height, mass);
            if (shape.capsule_axis == 0) return glm::vec3(raw.z, raw.y, raw.x);
            if (shape.capsule_axis == 1) return glm::vec3(raw.x, raw.z, raw.y);
            return raw;
        }
        case collision::ShapeType::TriangleMesh:
        default:
            return glm::vec3(0.0f);
    }
}

/**
 * @struct ChildMassInput
 * @brief One compound-collider child's contribution, relative to the owning body's own local
 *        frame -- the input to compose_mass_properties() below.
 */
struct ChildMassInput {
    glm::vec3 local_offset{0.0f};                   /**< Child's center, in the body's local frame. */
    glm::quat local_rotation{1.0f, 0.0f, 0.0f, 0.0f}; /**< Child's orientation, relative to the body. */
    float mass = 0.0f;
    /** @brief This child's OWN diagonal inverse inertia about ITS OWN center, in ITS OWN local
     *         frame -- exactly what inertia_for_shape() already returns for a single shape. */
    glm::vec3 inv_inertia_local{0.0f};
};

/**
 * @struct MassProperties
 * @brief Composite mass/center-of-mass/inertia produced by compose_mass_properties().
 */
struct MassProperties {
    float mass = 0.0f;
    glm::vec3 center_of_mass{0.0f};    /**< In the body's own local frame. */
    glm::vec3 inv_inertia_local{0.0f}; /**< About center_of_mass, body-local axes -- see the
                                             diagonal-only approximation note below. */
};

/**
 * @brief Composes N compound-collider children into one body's mass/center-of-mass/inertia via
 *        the parallel-axis theorem.
 *
 * DIAGONAL-ONLY APPROXIMATION: computes each child's full 3x3 forward inertia tensor (rotated
 * into the body's frame via `R * I * R^T`, then parallel-axis-shifted from the child's own
 * center to the COMPOSITE center of mass), sums them, then keeps only the diagonal of that sum
 * -- discarding whatever off-diagonal product-of-inertia terms it produced. This is exact when
 * every child is symmetric about the body's own local axes (the common case: children arranged
 * along cardinal directions/offsets), and an approximation otherwise (a modest rotational-
 * response error under torque) -- deliberately traded against widening
 * Body::inv_inertia_local from glm::vec3 to a full matrix, which would ripple into every
 * existing single-shape body/test/API in this codebase (RigidbodyComponent::
 * inertia_tensor_override included) for a case only compound bodies ever exercise. Mass and
 * center of mass are always exact regardless -- only the tensor's off-diagonal terms are
 * dropped. A single-child call degenerates to exactly that child's own values (offset/rotation
 * fold in via the same math, at zero approximation error since there's nothing to sum against).
 *
 * @param children Every child shape's contribution, relative to the body's own local frame.
 * @return Composite properties; `mass`/`inv_inertia_local` are both 0 for an empty `children`.
 */
inline MassProperties compose_mass_properties(const std::vector<ChildMassInput>& children) {
    MassProperties out;
    if (children.empty()) return out;

    float total_mass = 0.0f;
    glm::vec3 weighted_offset(0.0f);
    for (const auto& c : children) {
        total_mass += c.mass;
        weighted_offset += c.mass * c.local_offset;
    }
    if (total_mass <= 0.0f) return out;
    glm::vec3 com = weighted_offset / total_mass;

    glm::mat3 tensor(0.0f); // forward (non-inverse) composite tensor about `com`, body-local axes
    for (const auto& c : children) {
        if (c.mass <= 0.0f) continue;

        // This child's own forward diagonal tensor, in ITS OWN local frame -- invert back from
        // inv_inertia_local (every inertia_*() function above already clamps this away from
        // zero via util::k_epsilon, so the forward value is always well-defined here).
        glm::vec3 local_diag(c.inv_inertia_local.x > 0.0f ? 1.0f / c.inv_inertia_local.x : 0.0f,
                              c.inv_inertia_local.y > 0.0f ? 1.0f / c.inv_inertia_local.y : 0.0f,
                              c.inv_inertia_local.z > 0.0f ? 1.0f / c.inv_inertia_local.z : 0.0f);
        glm::mat3 child_local(0.0f);
        child_local[0][0] = local_diag.x;
        child_local[1][1] = local_diag.y;
        child_local[2][2] = local_diag.z;

        // Rotate into the body's own frame.
        glm::mat3 r = glm::mat3_cast(c.local_rotation);
        glm::mat3 child_in_body_frame = r * child_local * glm::transpose(r);

        // Parallel axis theorem, from the child's own center to the COMPOSITE center of mass:
        // I += m * (dot(d,d) * Identity3 - outer(d,d)).
        glm::vec3 d = c.local_offset - com;
        glm::mat3 outer(d.x * d.x, d.x * d.y, d.x * d.z,
                         d.y * d.x, d.y * d.y, d.y * d.z,
                         d.z * d.x, d.z * d.y, d.z * d.z);
        glm::mat3 parallel_axis = c.mass * (glm::dot(d, d) * glm::mat3(1.0f) - outer);

        tensor += child_in_body_frame + parallel_axis;
    }

    out.mass = total_mass;
    out.center_of_mass = com;
    out.inv_inertia_local = glm::vec3(tensor[0][0] > util::k_epsilon ? 1.0f / tensor[0][0] : 0.0f,
                                       tensor[1][1] > util::k_epsilon ? 1.0f / tensor[1][1] : 0.0f,
                                       tensor[2][2] > util::k_epsilon ? 1.0f / tensor[2][2] : 0.0f);
    return out;
}

} // namespace dynamics
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_DYNAMICS_INERTIA_H
