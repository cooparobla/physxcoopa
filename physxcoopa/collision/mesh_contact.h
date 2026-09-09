/**
 * @file mesh_contact.h
 * @brief Convex-shape-vs-static-triangle-mesh contact generation, with internal-edge
 *        correction: the classic ghost-collision bug where a box sliding across a flat mesh
 *        floor catches on the shared edge between two coplanar triangles, because a naive
 *        per-triangle contact normal points along the edge rather than the surface.
 *
 * v1 approach: brute-force iterate every triangle (the BVH exists for Phase 9's raycasts;
 * a mesh collider's triangle count in the scenes this targets is small enough that querying
 * it for narrowphase isn't worth the added complexity yet). Two different per-triangle contact
 * tests feed the same correction step, because Sphere/Capsule and Box need genuinely different
 * penetration measurements (see box_vs_triangle_contacts_'s doc for why a box has no
 * meaningful "closest point on surface" once it's overlapping at all):
 * - Sphere/Capsule: closest point on their core (a point or segment) to the triangle, offset
 *   by the shape's radius -- sphere_or_capsule_vs_triangle_contact_.
 * - Box: every corner tested directly against the triangle's plane, one contact per embedded
 *   corner -- box_vs_triangle_contacts_.
 *
 * Either way, the resulting normal is then corrected: if the shape approaches from genuinely
 * outside the triangle's face (the common case -- resting/sliding on a surface), use the
 * triangle's OWN face normal rather than the raw closest-point direction, which is what stops
 * the direction from ever pointing along a shared internal edge. Only when the shape is
 * meaningfully on the "wrong" side of this triangle's face (a genuine silhouette edge/corner,
 * e.g. the mesh's boundary) does the raw direction survive.
 */

#ifndef PHYSXCOOPA_COLLISION_MESH_CONTACT_H
#define PHYSXCOOPA_COLLISION_MESH_CONTACT_H

#include <physxcoopa/geometry/triangle_mesh.h>
#include <physxcoopa/geometry/obb.h>
#include <physxcoopa/geometry/sphere.h>
#include <physxcoopa/geometry/capsule.h>
#include <physxcoopa/collision/shape.h>
#include <physxcoopa/collision/manifold.h>
#include <physxcoopa/util/math.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace coopa {
namespace physx {
namespace collision {

/**
 * @brief Closest point on triangle (a,b,c) to `p` (Ericson, "Real-Time Collision Detection",
 *        5.1.5 -- the standard barycentric-region case analysis).
 */
inline glm::vec3 closest_point_on_triangle(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = glm::dot(ab, ap), d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;

    glm::vec3 bp = p - b;
    float d3 = glm::dot(ab, bp), d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + v * ab;
    }

    glm::vec3 cp = p - c;
    float d5 = glm::dot(ab, cp), d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + w * ac;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}

/**
 * @brief Per-triangle contact for a Sphere or Capsule: both have a "core" (a point or a
 *        segment) offset from the actual surface by a constant radius, so penetration is
 *        simply `radius - distance(core, triangle)` -- exactly how sphere_vs_box and
 *        capsule_vs_box already work elsewhere in narrowphase. `on_shape` is the closest point
 *        on the core (not the shape's swept surface); the capsule case alternates between the
 *        segment and the triangle a few times, since both are convex sets needing mutual
 *        projection to converge on their true closest points.
 *
 * @return True if `radius - distance(core, triangle) > 0`.
 */
inline bool sphere_or_capsule_vs_triangle_contact_(const Shape& shape, const glm::vec3& pos, const glm::quat& rot,
                                                    const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                                    glm::vec3& on_shape, glm::vec3& on_tri, float& penetration) {
    if (shape.type == ShapeType::Sphere) {
        on_shape = world_sphere(shape, pos, rot).center;
        on_tri = closest_point_on_triangle(on_shape, v0, v1, v2);
    } else {
        geometry::Capsule cap = world_capsule(shape, pos, rot);
        on_tri = (v0 + v1 + v2) * (1.0f / 3.0f);
        for (int i = 0; i < 6; ++i) {
            on_shape = geometry::closest_point_on_segment(on_tri, cap.a, cap.b);
            on_tri = closest_point_on_triangle(on_shape, v0, v1, v2);
        }
    }
    float radius = shape.type == ShapeType::Sphere ? shape.radius : shape.capsule_radius;
    float dist = glm::length(on_shape - on_tri);
    penetration = radius - dist;
    return penetration > 0.0f;
}

/**
 * @brief Per-triangle contact for a Box: a box has no core-radius offset, so the
 *        closest-point-on-surface trick above doesn't work -- once a box's face has
 *        overlapped a triangle's plane at all, the "closest point on the box" to any point on
 *        that triangle is already inside the box (distance 0), destroying the penetration
 *        signal immediately rather than growing it as the box sinks in further.
 *
 * Instead: test all 8 corners against the triangle's own plane directly (a convex-vs-halfspace
 * test per corner, the same one SAT reduces to along a single fixed axis) and emit one contact
 * per embedded corner -- a box resting flat generates all 4 corners on its underside as one
 * consistent 4-point manifold, matching what box-vs-box's face clipping produces, rather than
 * an arbitrarily-chosen single corner that would inject spurious torque as the box rocks between
 * which corner is "deepest" from one step to the next. This only tests the triangle's own
 * face-normal axis (not a full per-triangle SAT with the box's face/edge axes too), which is
 * sufficient for the resting/sliding-on-a-surface case this loop exists for; a box approaching
 * a mesh edge/corner from the side falls to the closest-point-based `on_tri` (already clamped
 * into the finite triangle) and the internal-edge correction below, same as the sphere/capsule
 * path.
 *
 * @param emit Called as `emit(on_shape, on_tri, penetration)` for each embedded corner.
 */
template <typename Fn>
inline void box_vs_triangle_contacts_(const Shape& shape, const glm::vec3& pos, const glm::quat& rot,
                                       const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                       const glm::vec3& face_normal, Fn&& emit) {
    geometry::OBB box = world_obb(shape, pos, rot);
    glm::vec3 ax, ay, az;
    box.axes(ax, ay, az);
    const glm::vec3& he = box.half_extents;
    float footprint_limit = glm::length(he);

    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                glm::vec3 corner = box.center + (float(sx) * he.x) * ax + (float(sy) * he.y) * ay + (float(sz) * he.z) * az;
                float signed_dist = glm::dot(corner - v0, face_normal);
                if (signed_dist >= 0.0f) continue;

                glm::vec3 on_tri = closest_point_on_triangle(corner, v0, v1, v2);

                // Reject triangles this corner isn't actually over: the plane test above doesn't
                // know about the triangle's finite extent, so an unrelated triangle sharing the
                // same plane would otherwise report the same penetration. Compare against the
                // corner's own plane projection, not the corner itself, since the corner is
                // offset from the plane by the penetration depth along `face_normal` -- that
                // offset must not count against the in-plane footprint test.
                glm::vec3 plane_point = corner - signed_dist * face_normal;
                if (glm::length(plane_point - on_tri) > footprint_limit) continue;

                emit(corner, on_tri, -signed_dist);
            }
        }
    }
}

/** @brief Barycentric weights of `p` w.r.t. triangle (a,b,c): {weight of a, weight of b, weight of c}. */
inline glm::vec3 barycentric_(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = glm::dot(v0, v0), d01 = glm::dot(v0, v1), d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0), d21 = glm::dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < util::k_epsilon) return glm::vec3(1.0f, 0.0f, 0.0f);
    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    return glm::vec3(1.0f - v - w, v, w);
}

/**
 * @brief Which of triangle `tri`'s edges (0: v0-v1, 1: v1-v2, 2: v2-v0) barycentric point `bary`
 *        lies on, or -1 for a face-interior point or a vertex (two weights near zero at once --
 *        deliberately left uncorrected by edge adjacency, see generate_mesh_contacts).
 */
inline int classify_triangle_edge_(const glm::vec3& bary) {
    constexpr float k_edge_eps = 1e-3f;
    bool near_a = bary.x < k_edge_eps, near_b = bary.y < k_edge_eps, near_c = bary.z < k_edge_eps;
    if (near_c && !near_a && !near_b) return 0;
    if (near_a && !near_b && !near_c) return 1;
    if (near_b && !near_a && !near_c) return 2;
    return -1;
}

/**
 * @brief Internal-edge correction (Phase 8): clamps a raw closest-point normal into a normal
 *        consistent with the mesh's actual surface, using the edge adjacency built by
 *        TriangleMeshLoader.
 *
 * - Face-interior (or vertex) contacts: unambiguous, but still prefer the face normal whenever
 *   the shape is genuinely on its outward side -- only a true silhouette keeps the raw direction.
 * - Boundary edges (no neighbour, `k_no_neighbor`): no constraint -- these are the mesh's
 *   perimeter, where the raw closest-feature direction is the only correct answer.
 * - Nearly-coplanar neighbours (the classic ghost-collision case -- a flat floor tessellated
 *   into many triangles): always trust this triangle's own face normal, which is what stops
 *   the shared edge from ever producing a normal that points along the surface instead of
 *   away from it.
 * - Genuine creases (a real corner): clamp to whichever of the two adjacent face normals the
 *   raw direction is closer to, rather than either an edge-tangent direction or an
 *   unconstrained raw normal.
 */
inline glm::vec3 correct_mesh_normal_(int edge, const glm::vec3& face_normal, const glm::vec3& neighbor_normal,
                                       bool has_neighbor, const glm::vec3& raw_normal) {
    if (edge < 0) {
        return glm::dot(raw_normal, face_normal) > 0.0f ? face_normal : raw_normal;
    }
    if (!has_neighbor) return raw_normal;

    float coplanarity = glm::dot(face_normal, neighbor_normal);
    if (coplanarity > 0.999f) return face_normal;

    return glm::dot(raw_normal, face_normal) >= glm::dot(raw_normal, neighbor_normal) ? face_normal : neighbor_normal;
}

/**
 * @brief Generates a manifold between a convex Shape and a static TriangleMesh shape.
 *
 * @param shape       The convex (Sphere/Box/Capsule) shape.
 * @param pos, rot    That shape's world pose.
 * @param mesh_shape  The TriangleMesh shape (its own pose is `mesh_pos`/`mesh_rot`).
 * @param out         Output manifold, normal pointing from the mesh's surface toward the
 *                     convex shape -- caller (generate_contacts) applies the final A->B sign.
 * @return True if any triangle produced a contact point.
 */
inline bool generate_mesh_contacts(const Shape& shape, const glm::vec3& pos, const glm::quat& rot,
                                    const Shape& mesh_shape, const glm::vec3& mesh_pos, const glm::quat& mesh_rot,
                                    ContactManifold& out) {
    if (!mesh_shape.mesh) return false;
    const geometry::TriangleMesh& mesh = *mesh_shape.mesh;

    glm::mat4 mesh_transform(glm::mat3_cast(mesh_rot * mesh_shape.local_rotation) * mesh_shape.mesh_scale);
    mesh_transform[3] = glm::vec4(mesh_pos + mesh_rot * mesh_shape.local_center, 1.0f);

    out = ContactManifold{};
    bool any_hit = false;
    glm::vec3 best_normal(0.0f, 0.0f, 1.0f);

    for (size_t tri = 0; tri < mesh.triangle_count(); ++tri) {
        if (out.count >= 4) break;
        glm::vec3 lv0, lv1, lv2;
        mesh.triangle_vertices(static_cast<uint32_t>(tri), lv0, lv1, lv2);
        glm::vec3 v0 = glm::vec3(mesh_transform * glm::vec4(lv0, 1.0f));
        glm::vec3 v1 = glm::vec3(mesh_transform * glm::vec4(lv1, 1.0f));
        glm::vec3 v2 = glm::vec3(mesh_transform * glm::vec4(lv2, 1.0f));
        glm::vec3 face_normal = glm::normalize(mesh_rot * mesh.normals()[tri]);
        const geometry::TriangleAdjacency& adjacency = mesh.adjacency()[tri];

        // Shared per-point path: classify where on this triangle the point landed, correct the
        // normal via adjacency, and append to the manifold. `raw_normal` is the fallback used
        // only for a genuine silhouette/crease (edge/vertex region with no coplanar neighbour);
        // a box's corner-based test always passes its own face_normal here (see
        // box_vs_triangle_contacts_'s doc for why the box case has no meaningful raw direction).
        auto emit_point = [&](const glm::vec3& on_shape, const glm::vec3& on_tri, float penetration,
                               const glm::vec3& raw_normal, uint32_t feature_id) {
            if (out.count >= 4) return;
            (void)on_shape;
            glm::vec3 bary = barycentric_(on_tri, v0, v1, v2);
            int edge = classify_triangle_edge_(bary);

            glm::vec3 neighbor_normal(0.0f);
            bool has_neighbor = false;
            if (edge >= 0) {
                uint32_t neighbor_tri = adjacency.neighbor[edge];
                if (neighbor_tri != geometry::k_no_neighbor) {
                    has_neighbor = true;
                    neighbor_normal = glm::normalize(mesh_rot * mesh.normals()[neighbor_tri]);
                }
            }

            glm::vec3 normal = correct_mesh_normal_(edge, face_normal, neighbor_normal, has_neighbor, raw_normal);
            glm::vec3 point = on_tri + normal * (penetration * 0.5f);
            out.add_point(point, penetration, feature_id);
            best_normal = normal;
            any_hit = true;
        };

        if (shape.type == ShapeType::Box) {
            uint32_t corner_index = 0;
            box_vs_triangle_contacts_(shape, pos, rot, v0, v1, v2, face_normal,
                                       [&](const glm::vec3& on_shape, const glm::vec3& on_tri, float penetration) {
                                           uint32_t feature_id = (static_cast<uint32_t>(tri) << 3) | (corner_index++ & 0x7u);
                                           emit_point(on_shape, on_tri, penetration, face_normal, feature_id);
                                       });
        } else {
            glm::vec3 on_shape, on_tri;
            float penetration = 0.0f;
            if (!sphere_or_capsule_vs_triangle_contact_(shape, pos, rot, v0, v1, v2, on_shape, on_tri, penetration)) continue;
            glm::vec3 diff = on_shape - on_tri; // points from triangle toward shape
            float dist = glm::length(diff);
            glm::vec3 raw_normal = dist > util::k_epsilon ? diff / dist : face_normal;
            emit_point(on_shape, on_tri, penetration, raw_normal, static_cast<uint32_t>(tri));
        }
    }

    if (!any_hit) return false;
    out.normal = best_normal;
    return true;
}

} // namespace collision
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_COLLISION_MESH_CONTACT_H
