/**
 * @file triangle_mesh.h
 * @brief Format-agnostic static triangle mesh: welded vertices, per-triangle normals and
 *        edge adjacency. Construction is from raw arrays only -- asset loading (weld,
 *        fan-triangulation, BVH construction) is physxcoopa::loaders::TriangleMeshLoader.
 */

#ifndef PHYSXCOOPA_GEOMETRY_TRIANGLE_MESH_H
#define PHYSXCOOPA_GEOMETRY_TRIANGLE_MESH_H

#include <physxcoopa/geometry/aabb.h>
#include <physxcoopa/geometry/mesh_bvh.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <limits>
#include <vector>

namespace coopa {
namespace physx {
namespace geometry {

/** @brief Sentinel neighbour-triangle index meaning "no neighbour" (boundary edge). */
inline constexpr uint32_t k_no_neighbor = 0xFFFFFFFFu;

/**
 * @struct TriangleAdjacency
 * @brief Per-triangle neighbour info, indexed by triangle. Edge `i` of a triangle runs from
 *        vertex `i` to vertex `(i+1)%3`; `neighbor[i]` is the triangle sharing that edge, or
 *        k_no_neighbor for a mesh-boundary edge.
 */
struct TriangleAdjacency {
    uint32_t neighbor[3] = {k_no_neighbor, k_no_neighbor, k_no_neighbor};
};

/**
 * @class TriangleMesh
 * @brief A static, welded triangle mesh with precomputed per-triangle normals and adjacency.
 *
 * Used as the collision geometry for MeshCollider. Non-convex mesh colliders are static-only
 * (see RigidbodyComponent's bind-time check) -- this class itself has no notion of that
 * restriction, it is purely geometric storage.
 */
class TriangleMesh {
public:
    TriangleMesh() = default;

    /**
     * @brief Builds a TriangleMesh from already-welded vertex/index data plus precomputed
     *        per-triangle normals and adjacency.
     *
     * @param vertices  Welded vertex positions.
     * @param indices   Triangle list, 3 indices per triangle.
     * @param normals   One normal per triangle (indices.size()/3 entries).
     * @param adjacency One TriangleAdjacency per triangle.
     */
    TriangleMesh(std::vector<glm::vec3> vertices,
                 std::vector<uint32_t> indices,
                 std::vector<glm::vec3> normals,
                 std::vector<TriangleAdjacency> adjacency)
        : vertices_(std::move(vertices)),
          indices_(std::move(indices)),
          normals_(std::move(normals)),
          adjacency_(std::move(adjacency)) {
        bounds_ = compute_bounds_();
        bvh_.build(vertices_, indices_);
    }

    /** @brief Welded vertex positions. */
    const std::vector<glm::vec3>& vertices() const { return vertices_; }

    /** @brief Triangle list, 3 indices per triangle. */
    const std::vector<uint32_t>& indices() const { return indices_; }

    /** @brief One face normal per triangle. */
    const std::vector<glm::vec3>& normals() const { return normals_; }

    /** @brief One TriangleAdjacency per triangle. */
    const std::vector<TriangleAdjacency>& adjacency() const { return adjacency_; }

    /** @brief Number of triangles in the mesh. */
    size_t triangle_count() const { return indices_.size() / 3; }

    /** @brief The three vertex positions of triangle `tri`. */
    void triangle_vertices(uint32_t tri, glm::vec3& v0, glm::vec3& v1, glm::vec3& v2) const {
        v0 = vertices_[indices_[tri * 3 + 0]];
        v1 = vertices_[indices_[tri * 3 + 1]];
        v2 = vertices_[indices_[tri * 3 + 2]];
    }

    /** @brief Object-space bounds of the whole mesh. */
    const AABB& bounds() const { return bounds_; }

    /** @brief Static BVH over this mesh's triangles, built once at construction. */
    const MeshBVH& bvh() const { return bvh_; }

private:
    AABB compute_bounds_() const {
        AABB out;
        for (const auto& v : vertices_) {
            out.min = glm::min(out.min, v);
            out.max = glm::max(out.max, v);
        }
        return out;
    }

    std::vector<glm::vec3> vertices_;
    std::vector<uint32_t> indices_;
    std::vector<glm::vec3> normals_;
    std::vector<TriangleAdjacency> adjacency_;
    AABB bounds_;
    MeshBVH bvh_;
};

} // namespace geometry
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_GEOMETRY_TRIANGLE_MESH_H
