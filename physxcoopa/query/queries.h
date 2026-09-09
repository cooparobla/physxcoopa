/**
 * @file queries.h
 * @brief Scene queries: raycasts, casts, and overlap tests against everything currently in a
 *        PhysicsWorld. PhysicsWorld exposes the actual public API (raycast/raycast_all/
 *        sphere_cast/overlap_sphere/overlap_box); this file holds the shape-dispatch helpers
 *        those methods are built from, kept separate so world.h's own body isn't a wall of
 *        per-shape-type raycast/overlap code.
 *
 * PhysicsWorld has no dependency on coopa::scene (see world.h's file doc), so `RaycastHit`
 * carries a `BodyId`, not a `SceneObject*` -- gameplay code resolves a hit's owning
 * SceneObject/Collider itself (e.g. via a small BodyId->Collider map the way PhysicsSystem
 * already tracks Collider->BodyId) if it needs one; PhysicsWorld itself never sees that type.
 *
 * These queries are only valid between phases, not from inside `on_substep` -- a raycast
 * issued from a substep callback would observe mid-solve, not-yet-integrated state.
 */

#ifndef PHYSXCOOPA_QUERY_QUERIES_H
#define PHYSXCOOPA_QUERY_QUERIES_H

#include <physxcoopa/geometry/ray.h>
#include <physxcoopa/geometry/sphere.h>
#include <physxcoopa/geometry/obb.h>
#include <physxcoopa/collision/shape.h>
#include <physxcoopa/collision/narrowphase.h>
#include <physxcoopa/dynamics/body.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace coopa {
namespace physx {
namespace query {

/**
 * @struct RaycastHit
 * @brief One raycast/sphere-cast result.
 */
struct RaycastHit {
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 0.0f, 1.0f};
    float distance = 0.0f;
    dynamics::BodyId body;
    /** @brief Which of `body`'s shape slots was actually hit (PhysicsWorld::shape_at()'s
     *         return type -- a raw slot index, k_invalid_shape by default). For a compound
     *         body (more than one shape), this is what lets a caller resolve the exact CHILD
     *         collider touched, not just the body -- see PhysicsSystem::collider_for_shape_(). */
    uint32_t shape_index = 0xFFFFFFFFu;
};

/**
 * @brief Exact ray-vs-shape dispatch, analytic for Sphere/Box/Capsule and BVH descent for
 *        TriangleMesh (the ray is transformed into the mesh's local space once, rather than
 *        transforming every candidate triangle into world space).
 *
 * @return True if the ray hits `shape` (placed at `pos`/`rot`) within [0, ray.max_distance].
 */
inline bool raycast_shape(const geometry::Ray& ray, const collision::Shape& shape,
                           const glm::vec3& pos, const glm::quat& rot, float& t, glm::vec3& normal) {
    switch (shape.type) {
        case collision::ShapeType::Sphere:
            return geometry::ray_vs_sphere(ray, collision::world_sphere(shape, pos, rot), t, normal);
        case collision::ShapeType::Box:
            return geometry::ray_vs_obb(ray, collision::world_obb(shape, pos, rot), t, normal);
        case collision::ShapeType::Capsule:
            return geometry::ray_vs_capsule(ray, collision::world_capsule(shape, pos, rot), t, normal);
        case collision::ShapeType::TriangleMesh: {
            if (!shape.mesh) return false;
            glm::vec3 mesh_center = pos + rot * shape.local_center;
            glm::quat mesh_rot = rot * shape.local_rotation;
            glm::quat inv_rot = glm::inverse(mesh_rot);
            // Dividing both origin and direction by mesh_scale keeps the local-space `t`
            // parameter numerically equal to the world-space one -- see shape.h's mesh_scale
            // doc -- so best_t below needs no further rescaling.
            float inv_scale = shape.mesh_scale > 1e-8f ? 1.0f / shape.mesh_scale : 1.0f;

            geometry::Ray local_ray;
            local_ray.origin = inv_rot * (ray.origin - mesh_center) * inv_scale;
            local_ray.direction = inv_rot * ray.direction * inv_scale;
            local_ray.max_distance = ray.max_distance;

            bool found = false;
            float best_t = local_ray.max_distance;
            uint32_t best_tri = 0;
            shape.mesh->bvh().raycast(local_ray, [&](uint32_t tri) {
                glm::vec3 v0, v1, v2;
                shape.mesh->triangle_vertices(tri, v0, v1, v2);
                float candidate_t;
                if (geometry::ray_vs_triangle(local_ray, v0, v1, v2, candidate_t) && candidate_t < best_t) {
                    best_t = candidate_t;
                    best_tri = tri;
                    found = true;
                }
            });
            if (!found) return false;
            t = best_t;
            normal = glm::normalize(mesh_rot * shape.mesh->normals()[best_tri]);
            return true;
        }
    }
    return false;
}

/** @brief True if `shape` (at `pos`/`rot`) overlaps `query`, a world-space sphere. */
inline bool shape_overlaps_sphere(const collision::Shape& shape, const glm::vec3& pos, const glm::quat& rot,
                                   const geometry::Sphere& query) {
    glm::vec3 n, p;
    float pen;
    switch (shape.type) {
        case collision::ShapeType::Sphere:
            return collision::sphere_vs_sphere(collision::world_sphere(shape, pos, rot), query, n, pen, p);
        case collision::ShapeType::Box:
            return collision::sphere_vs_box(query, collision::world_obb(shape, pos, rot), n, pen, p);
        case collision::ShapeType::Capsule:
            return collision::capsule_vs_sphere(collision::world_capsule(shape, pos, rot), query, n, pen, p);
        case collision::ShapeType::TriangleMesh: {
            collision::Shape sphere_shape = collision::Shape::make_sphere(query.radius);
            collision::ContactManifold m;
            return collision::generate_mesh_contacts(sphere_shape, query.center, glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                                       shape, pos, rot, m);
        }
    }
    return false;
}

/** @brief True if `shape` (at `pos`/`rot`) overlaps `query`, a world-space OBB. */
inline bool shape_overlaps_obb(const collision::Shape& shape, const glm::vec3& pos, const glm::quat& rot,
                                const geometry::OBB& query) {
    glm::vec3 n, p;
    float pen;
    switch (shape.type) {
        case collision::ShapeType::Sphere:
            return collision::sphere_vs_box(collision::world_sphere(shape, pos, rot), query, n, pen, p);
        case collision::ShapeType::Box: {
            collision::ContactManifold m;
            return collision::generate_box_box_contacts(collision::world_obb(shape, pos, rot), query, m);
        }
        case collision::ShapeType::Capsule: {
            collision::ContactManifold m;
            return collision::capsule_vs_box(collision::world_capsule(shape, pos, rot), query, m);
        }
        case collision::ShapeType::TriangleMesh: {
            collision::Shape box_shape = collision::Shape::make_box(query.half_extents);
            collision::ContactManifold m;
            return collision::generate_mesh_contacts(box_shape, query.center, query.orientation, shape, pos, rot, m);
        }
    }
    return false;
}

/**
 * @brief Rotation mapping world +Z onto unit vector `dir` -- the shortest arc between them.
 *        Used only to pose a capsule::make_capsule() Shape (always local-Z-axis, see
 *        shape.h's capsule_axis doc) along an arbitrary world-space direction via the `rot`
 *        parameter every world_*() helper and generate_mesh_contacts() already take, rather
 *        than needing a per-shape local rotation field (see shape_overlaps_capsule()'s
 *        TriangleMesh case, the only caller).
 */
inline glm::quat quat_from_z_to(const glm::vec3& dir) {
    const glm::vec3 z(0.0f, 0.0f, 1.0f);
    float d = glm::dot(z, dir);
    if (d > 1.0f - 1e-6f) return glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // already aligned
    if (d < -1.0f + 1e-6f) { // exactly opposite -- any perpendicular axis gives a valid 180 flip
        glm::vec3 axis = std::abs(z.x) < 0.9f ? glm::cross(z, glm::vec3(1.0f, 0.0f, 0.0f))
                                               : glm::cross(z, glm::vec3(0.0f, 1.0f, 0.0f));
        return glm::angleAxis(glm::pi<float>(), glm::normalize(axis));
    }
    glm::vec3 axis = glm::normalize(glm::cross(z, dir));
    return glm::angleAxis(std::acos(glm::clamp(d, -1.0f, 1.0f)), axis);
}

/** @brief True if `shape` (at `pos`/`rot`) overlaps `query`, a world-space capsule. */
inline bool shape_overlaps_capsule(const collision::Shape& shape, const glm::vec3& pos, const glm::quat& rot,
                                    const geometry::Capsule& query) {
    glm::vec3 n, p;
    float pen;
    switch (shape.type) {
        case collision::ShapeType::Sphere:
            return collision::capsule_vs_sphere(query, collision::world_sphere(shape, pos, rot), n, pen, p);
        case collision::ShapeType::Box: {
            collision::ContactManifold m;
            return collision::capsule_vs_box(query, collision::world_obb(shape, pos, rot), m);
        }
        case collision::ShapeType::Capsule:
            return collision::capsule_vs_capsule(query, collision::world_capsule(shape, pos, rot), n, pen, p);
        case collision::ShapeType::TriangleMesh: {
            glm::vec3 seg = query.b - query.a;
            float len = glm::length(seg);
            glm::quat cap_rot = len > 1e-8f ? quat_from_z_to(seg / len) : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            glm::vec3 cap_pos = (query.a + query.b) * 0.5f;
            collision::Shape capsule_shape = collision::Shape::make_capsule(query.radius, len * 0.5f);
            collision::ContactManifold m;
            return collision::generate_mesh_contacts(capsule_shape, cap_pos, cap_rot, shape, pos, rot, m);
        }
    }
    return false;
}

} // namespace query
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_QUERY_QUERIES_H
