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

#include <algorithm>
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
            glm::quat inv_rot = glm::inverse(rot);

            geometry::Ray local_ray;
            local_ray.origin = inv_rot * (ray.origin - mesh_center);
            local_ray.direction = inv_rot * ray.direction;
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
            normal = glm::normalize(rot * shape.mesh->normals()[best_tri]);
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

} // namespace query
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_QUERY_QUERIES_H
