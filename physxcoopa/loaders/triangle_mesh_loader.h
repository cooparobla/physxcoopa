/**
 * @file triangle_mesh_loader.h
 * @brief coopa::asset loader for geometry::TriangleMesh — decodes a Blender-exported mesh YAML
 *        (the same `vertices:`/`faces:` format gfxcoopa::MeshLoader reads) into a welded,
 *        adjacency-built collision mesh.
 *
 * Unlike gfxcoopa's MeshLoader, physics has no GPU step at all: decode_typed() does the entire
 * pipeline (parse, fan-triangulate, weld, adjacency, BVH) off-thread, and finalize_typed()
 * returns its argument unchanged. See the plan's Phase 8 for why each step below is mandatory,
 * not an optimization -- in particular, welding is required for the edge-adjacency table to
 * find shared edges at all, since exported meshes are already split one vertex per face-corner.
 */

#ifndef PHYSXCOOPA_LOADERS_TRIANGLE_MESH_LOADER_H
#define PHYSXCOOPA_LOADERS_TRIANGLE_MESH_LOADER_H

#include <physxcoopa/geometry/triangle_mesh.h>

#include <coopa/asset/asset_loader.h>
#include <coopa/asset/asset_id.h>
#include <coopa/debug/logger.h>

#include <fkYAML/node.hpp>
#include <glm/glm.hpp>

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace coopa {
namespace physx {
namespace loaders {

namespace detail {

/** @brief Packs two triangle-vertex indices into an order-independent 64-bit edge key. */
inline uint64_t edge_key_(uint32_t a, uint32_t b) {
    uint32_t lo = a < b ? a : b;
    uint32_t hi = a < b ? b : a;
    return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

/** @brief Packs a quantized 3D grid cell into a 64-bit key for the welding spatial hash. */
inline uint64_t cell_key_(int32_t x, int32_t y, int32_t z) {
    auto to_u = [](int32_t v) { return static_cast<uint64_t>(static_cast<uint32_t>(v)) & 0x1FFFFFu; };
    return (to_u(x) << 42) | (to_u(y) << 21) | to_u(z);
}

} // namespace detail

/**
 * @class TriangleMeshLoader
 * @brief Registers as the coopa::asset loader for geometry::TriangleMesh.
 *
 * @code
 * assets.register_loader<geometry::TriangleMesh>(std::make_unique<loaders::TriangleMeshLoader>());
 * auto mesh = assets.load<geometry::TriangleMesh>("meshes/cube.000.yaml", ctx.scene_dir);
 * @endcode
 */
class TriangleMeshLoader : public coopa::asset::TypedAssetLoader<geometry::TriangleMesh, geometry::TriangleMesh> {
public:
    std::shared_ptr<geometry::TriangleMesh> decode_typed(const coopa::asset::AssetId& id,
                                                          const coopa::asset::LoadContext& ctx) override {
        std::ifstream ifs(ctx.resolved_path);
        if (!ifs) {
            throw std::runtime_error("[physxcoopa] TriangleMeshLoader: failed to open '" + id.path() + "'");
        }
        fkyaml::node node = fkyaml::node::deserialize(ifs);
        coopa::debug::Logger logger("Physics");

        // --- 1. Parse raw (unwelded) vertices ---
        std::vector<glm::vec3> raw_vertices;
        if (node.contains("vertices")) {
            for (const auto& v : node.at("vertices")) {
                raw_vertices.push_back({v.at(0).get_value<float>(), v.at(1).get_value<float>(), v.at(2).get_value<float>()});
            }
        }
        if (raw_vertices.empty()) {
            throw std::runtime_error("[physxcoopa] TriangleMeshLoader: '" + id.path() + "' has no vertices");
        }

        // --- 2. Fan-triangulate each face polygon into raw-index triangles ---
        std::vector<uint32_t> raw_indices;
        if (node.contains("faces")) {
            for (const auto& face : node.at("faces")) {
                std::vector<uint32_t> poly;
                for (const auto& idx : face) poly.push_back(idx.get_value<uint32_t>());
                if (poly.size() < 3) {
                    logger.warn("TriangleMeshLoader: skipping face with < 3 indices in '" + id.path() + "'");
                    continue;
                }
                for (size_t i = 1; i + 1 < poly.size(); ++i) {
                    raw_indices.push_back(poly[0]);
                    raw_indices.push_back(poly[i]);
                    raw_indices.push_back(poly[i + 1]);
                }
            }
        }

        // --- 3. Weld raw vertices via a spatial hash, epsilon relative to the mesh's own bounds ---
        glm::vec3 bounds_min(raw_vertices[0]), bounds_max(raw_vertices[0]);
        for (const auto& v : raw_vertices) {
            bounds_min = glm::min(bounds_min, v);
            bounds_max = glm::max(bounds_max, v);
        }
        float diagonal = glm::length(bounds_max - bounds_min);
        float epsilon = diagonal > 0.0f ? diagonal * 1e-4f : 1e-5f;
        float cell_size = epsilon > 0.0f ? epsilon : 1e-5f;

        std::vector<glm::vec3> welded_vertices;
        std::vector<uint32_t> remap(raw_vertices.size());
        std::unordered_multimap<uint64_t, uint32_t> spatial_hash; // cell key -> welded index

        auto cell_of = [&](const glm::vec3& p) {
            return glm::ivec3(static_cast<int32_t>(std::floor(p.x / cell_size)),
                               static_cast<int32_t>(std::floor(p.y / cell_size)),
                               static_cast<int32_t>(std::floor(p.z / cell_size)));
        };

        for (size_t i = 0; i < raw_vertices.size(); ++i) {
            const glm::vec3& p = raw_vertices[i];
            glm::ivec3 cell = cell_of(p);
            uint32_t found = UINT32_MAX;
            for (int dx = -1; dx <= 1 && found == UINT32_MAX; ++dx) {
                for (int dy = -1; dy <= 1 && found == UINT32_MAX; ++dy) {
                    for (int dz = -1; dz <= 1 && found == UINT32_MAX; ++dz) {
                        uint64_t key = detail::cell_key_(cell.x + dx, cell.y + dy, cell.z + dz);
                        auto range = spatial_hash.equal_range(key);
                        for (auto it = range.first; it != range.second; ++it) {
                            if (glm::length(welded_vertices[it->second] - p) <= epsilon) {
                                found = it->second;
                                break;
                            }
                        }
                    }
                }
            }
            if (found != UINT32_MAX) {
                remap[i] = found;
            } else {
                uint32_t new_index = static_cast<uint32_t>(welded_vertices.size());
                welded_vertices.push_back(p);
                spatial_hash.emplace(detail::cell_key_(cell.x, cell.y, cell.z), new_index);
                remap[i] = new_index;
            }
        }

        // --- 4. Remap triangle indices, dropping any made degenerate by welding ---
        std::vector<uint32_t> indices;
        indices.reserve(raw_indices.size());
        for (size_t t = 0; t + 2 < raw_indices.size(); t += 3) {
            uint32_t a = remap[raw_indices[t + 0]];
            uint32_t b = remap[raw_indices[t + 1]];
            uint32_t c = remap[raw_indices[t + 2]];
            if (a == b || b == c || a == c) continue;
            glm::vec3 normal = glm::cross(welded_vertices[b] - welded_vertices[a], welded_vertices[c] - welded_vertices[a]);
            if (glm::length(normal) <= util_epsilon_()) continue;
            indices.push_back(a);
            indices.push_back(b);
            indices.push_back(c);
        }

        size_t triangle_count = indices.size() / 3;

        // --- 5. Per-triangle face normals ---
        std::vector<glm::vec3> normals(triangle_count);
        for (size_t t = 0; t < triangle_count; ++t) {
            const glm::vec3& a = welded_vertices[indices[t * 3 + 0]];
            const glm::vec3& b = welded_vertices[indices[t * 3 + 1]];
            const glm::vec3& c = welded_vertices[indices[t * 3 + 2]];
            normals[t] = glm::normalize(glm::cross(b - a, c - a));
        }

        // --- 6. Edge adjacency: (min_v, max_v) -> owning (triangle, edge slot) occurrences ---
        std::vector<geometry::TriangleAdjacency> adjacency(triangle_count);
        std::unordered_map<uint64_t, std::vector<std::pair<uint32_t, uint32_t>>> edge_owners; // edge key -> [(tri, edge_slot)]
        for (uint32_t t = 0; t < triangle_count; ++t) {
            for (uint32_t e = 0; e < 3; ++e) {
                uint32_t v0 = indices[t * 3 + e];
                uint32_t v1 = indices[t * 3 + (e + 1) % 3];
                edge_owners[detail::edge_key_(v0, v1)].push_back({t, e});
            }
        }
        for (const auto& entry : edge_owners) {
            const auto& owners = entry.second;
            if (owners.size() == 2) {
                adjacency[owners[0].first].neighbor[owners[0].second] = owners[1].first;
                adjacency[owners[1].first].neighbor[owners[1].second] = owners[0].first;
            } else if (owners.size() > 2) {
                logger.warn("TriangleMeshLoader: non-manifold edge (" + std::to_string(owners.size()) +
                            " owning triangles) in '" + id.path() + "' -- left unconnected");
            }
            // owners.size() == 1: boundary edge, k_no_neighbor is already the default.
        }

        return std::make_shared<geometry::TriangleMesh>(std::move(welded_vertices), std::move(indices),
                                                          std::move(normals), std::move(adjacency));
    }

    std::shared_ptr<geometry::TriangleMesh> finalize_typed(std::shared_ptr<geometry::TriangleMesh> decoded,
                                                            const coopa::asset::AssetId&,
                                                            const coopa::asset::LoadContext&) override {
        return decoded;
    }

    const char* type_name() const override { return "TriangleMesh"; }

private:
    static float util_epsilon_() { return 1e-8f; }
};

} // namespace loaders
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_LOADERS_TRIANGLE_MESH_LOADER_H
