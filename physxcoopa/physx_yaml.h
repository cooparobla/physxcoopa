/**
 * @file physx_yaml.h
 * @brief register_physics_components() -- wires physxcoopa's components into SceneLoader.
 *        install_physics_system() (system/physics_system.h) is the other half of setup.
 */

#ifndef PHYSXCOOPA_PHYSX_YAML_H
#define PHYSXCOOPA_PHYSX_YAML_H

#include <physxcoopa/system/physics_system.h>
#include <physxcoopa/components/box_collider.h>
#include <physxcoopa/components/sphere_collider.h>
#include <physxcoopa/components/capsule_collider.h>
#include <physxcoopa/components/mesh_collider.h>
#include <physxcoopa/components/rigidbody.h>
#include <physxcoopa/geometry/triangle_mesh.h>
#include <physxcoopa/loaders/triangle_mesh_loader.h>

#include <coopa/scene/scene_loader.h>
#include <coopa/scene/scene_object.h>
#include <coopa/asset/asset_manager.h>

#include <glm/glm.hpp>

#include <string>

namespace coopa {
namespace physx {

namespace detail {

inline glm::vec3 read_vec3_(const fkyaml::node& node, const std::string& key, const glm::vec3& fallback) {
    if (!node.contains(key)) return fallback;
    const auto& sub = node.at(key);
    glm::vec3 v = fallback;
    if (sub.contains("x")) v.x = sub.at("x").get_value<float>();
    if (sub.contains("y")) v.y = sub.at("y").get_value<float>();
    if (sub.contains("z")) v.z = sub.at("z").get_value<float>();
    return v;
}

} // namespace detail

/**
 * @brief Registers every physxcoopa component's YAML parser with SceneLoader.
 *
 * Each name is registered once -- SceneLoader::normalize_tag_() strips a leading '!' before
 * every lookup, so a single "BoxCollider" registration matches both a `!BoxCollider` YAML
 * tag and a `type: BoxCollider` key (see the plan's constraint 5).
 *
 * @param assets AssetManager used to resolve MeshCollider's mesh reference. Must outlive
 *               every subsequent SceneLoader::load() call, matching
 *               gfxcoopa::register_render_components()'s convention.
 */
inline void register_physics_components(coopa::asset::AssetManager& assets) {
    using coopa::scene::SceneLoader;
    using coopa::scene::SceneObject;

    assets.register_loader<geometry::TriangleMesh>(std::make_unique<loaders::TriangleMeshLoader>());

    SceneLoader::register_component_parser("BoxCollider",
        [](const fkyaml::node& node, SceneObject& obj, const SceneLoader::ParseContext&) {
            auto* c = obj.add_component<components::BoxCollider>();
            c->set_size(detail::read_vec3_(node, "size", glm::vec3(1.0f)));
            c->set_center(detail::read_vec3_(node, "center", glm::vec3(0.0f)));
            if (node.contains("is_trigger")) c->set_is_trigger(node.at("is_trigger").get_value<bool>());
            if (node.contains("layer")) c->set_layer(node.at("layer").get_value<uint32_t>());
        });

    SceneLoader::register_component_parser("SphereCollider",
        [](const fkyaml::node& node, SceneObject& obj, const SceneLoader::ParseContext&) {
            auto* c = obj.add_component<components::SphereCollider>();
            if (node.contains("radius")) c->set_radius(node.at("radius").get_value<float>());
            c->set_center(detail::read_vec3_(node, "center", glm::vec3(0.0f)));
            if (node.contains("is_trigger")) c->set_is_trigger(node.at("is_trigger").get_value<bool>());
            if (node.contains("layer")) c->set_layer(node.at("layer").get_value<uint32_t>());
        });

    SceneLoader::register_component_parser("CapsuleCollider",
        [](const fkyaml::node& node, SceneObject& obj, const SceneLoader::ParseContext&) {
            auto* c = obj.add_component<components::CapsuleCollider>();
            if (node.contains("radius")) c->set_radius(node.at("radius").get_value<float>());
            if (node.contains("height")) c->set_height(node.at("height").get_value<float>());
            if (node.contains("direction")) c->set_direction(node.at("direction").get_value<int>());
            c->set_center(detail::read_vec3_(node, "center", glm::vec3(0.0f)));
            if (node.contains("is_trigger")) c->set_is_trigger(node.at("is_trigger").get_value<bool>());
            if (node.contains("layer")) c->set_layer(node.at("layer").get_value<uint32_t>());
        });

    SceneLoader::register_component_parser("MeshCollider",
        [&assets](const fkyaml::node& node, SceneObject& obj, const SceneLoader::ParseContext& ctx) {
            auto* c = obj.add_component<components::MeshCollider>();
            if (node.contains("convex")) c->set_convex(node.at("convex").get_value<bool>());
            if (node.contains("mesh_path")) {
                std::string key = node.at("mesh_path").get_value<std::string>();
                std::string virtual_path = "meshes/" + key + ".yaml";
                c->set_mesh(assets.load<geometry::TriangleMesh>(virtual_path, ctx.scene_dir));
            }
        });

    SceneLoader::register_component_parser("Rigidbody",
        [](const fkyaml::node& node, SceneObject& obj, const SceneLoader::ParseContext&) {
            auto* rb = obj.add_component<components::RigidbodyComponent>();
            if (node.contains("mass")) rb->mass = node.at("mass").get_value<float>();
            if (node.contains("drag")) rb->drag = node.at("drag").get_value<float>();
            if (node.contains("angular_drag")) rb->angular_drag = node.at("angular_drag").get_value<float>();
            if (node.contains("use_gravity")) rb->use_gravity = node.at("use_gravity").get_value<bool>();
            if (node.contains("is_kinematic")) rb->is_kinematic = node.at("is_kinematic").get_value<bool>();
        });
}

} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_PHYSX_YAML_H
