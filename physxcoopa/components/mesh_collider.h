/**
 * @file mesh_collider.h
 * @brief Static triangle-mesh collider component.
 *
 * The AssetHandle is populated by physx_yaml.h's parser via `AssetManager::load<TriangleMesh>()`,
 * backed by `loaders::TriangleMeshLoader`. If the handle hasn't finished loading yet,
 * make_shape() returns a disabled Shape, so a scene referencing a mesh still being resolved
 * loads and renders inertly rather than crashing.
 */

#ifndef PHYSXCOOPA_COMPONENTS_MESH_COLLIDER_H
#define PHYSXCOOPA_COMPONENTS_MESH_COLLIDER_H

#include <physxcoopa/components/collider.h>
#include <physxcoopa/geometry/triangle_mesh.h>

#include <coopa/asset/asset_handle.h>

#include <algorithm>
#include <string>

namespace coopa {
namespace physx {
namespace components {

/**
 * @class MeshCollider
 * @brief A static (or, with `convex = true` in a future revision, dynamic) triangle-mesh
 *        collider. Non-convex mesh colliders are static-only, matching Unity -- PhysicsSystem
 *        must refuse to bind one to a non-kinematic Rigidbody.
 */
class MeshCollider : public Collider {
public:
    std::string type_name() const override { return "MeshCollider"; }

    const coopa::asset::AssetHandle<geometry::TriangleMesh>& mesh() const { return mesh_; }
    void set_mesh(coopa::asset::AssetHandle<geometry::TriangleMesh> handle) {
        mesh_ = std::move(handle);
        bump_revision_();
    }

    bool convex() const { return convex_; }
    void set_convex(bool v) { convex_ = v; bump_revision_(); }

    collision::Shape make_shape(const glm::vec3& world_scale) const override {
        if (!mesh_.is_loaded()) return collision::Shape{};
        // Max component of world scale, same rule as Sphere/Capsule -- see Shape::mesh_scale's
        // doc for why v1 supports uniform mesh scale only.
        float scale = std::max({world_scale.x, world_scale.y, world_scale.z});
        collision::Shape s = collision::Shape::make_mesh(mesh_.get(), center() * world_scale);
        s.mesh_scale = scale;
        return s;
    }

private:
    coopa::asset::AssetHandle<geometry::TriangleMesh> mesh_;
    bool convex_ = false;
};

} // namespace components
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_COMPONENTS_MESH_COLLIDER_H
