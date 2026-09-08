/**
 * @file collider.h
 * @brief Base class for every collider component -- passive descriptor data only, no
 *        body-creation logic. PhysicsSystem (system/physics_system.h) is the only thing
 *        that ever creates or destroys a physics body from a Collider.
 */

#ifndef PHYSXCOOPA_COMPONENTS_COLLIDER_H
#define PHYSXCOOPA_COMPONENTS_COLLIDER_H

#include <physxcoopa/collision/shape.h>
#include <physxcoopa/dynamics/body.h>
#include <physxcoopa/dynamics/physics_material.h>

#include <coopa/scene/component.h>
#include <coopa/event/signal.h>

#include <glm/glm.hpp>

#include <cstdint>

namespace coopa {
namespace physx {
namespace components {

/**
 * @class Collider
 * @brief Base for BoxCollider/SphereCollider/CapsuleCollider/MeshCollider.
 *
 * A Collider is pure data until PhysicsSystem's reconcile pass binds it to a body -- it
 * overrides Component::start() with nothing, because SceneLoader::load() calls Scene::start()
 * itself, before the app has a chance to call install_physics_system() (see the plan's
 * "Physics components override start() with nothing").
 *
 * `revision_` is bumped by every setter; PhysicsSystem compares it against the value at last
 * reconcile to decide whether to rebuild this collider's shape (see PhysicsSystem's doc for
 * why a full rebuild-on-any-change, copying AnimationSystem's pattern, would be wrong here --
 * a collider owns a live broadphase proxy, warm-start impulses and a sleep timer that must
 * survive an unrelated collider's parameter change elsewhere in the scene).
 */
class Collider : public coopa::scene::Component {
public:
    /** @brief Fired when this trigger-collider-free collider starts touching another. */
    coopa::event::Signal<Collider&, Collider&> on_collision_enter;
    /** @brief Fired every step this collider remains touching another. */
    coopa::event::Signal<Collider&, Collider&> on_collision_stay;
    /** @brief Fired when this collider stops touching another. */
    coopa::event::Signal<Collider&, Collider&> on_collision_exit;
    /** @brief Fired when a trigger collider starts overlapping another. */
    coopa::event::Signal<Collider&, Collider&> on_trigger_enter;
    /** @brief Fired every step a trigger collider remains overlapping another. */
    coopa::event::Signal<Collider&, Collider&> on_trigger_stay;
    /** @brief Fired when a trigger collider stops overlapping another. */
    coopa::event::Signal<Collider&, Collider&> on_trigger_exit;

    void start() override { /* intentionally empty -- see class doc */ }

    bool is_trigger() const { return is_trigger_; }
    void set_is_trigger(bool v) { is_trigger_ = v; ++revision_; }

    uint32_t layer() const { return layer_; }
    void set_layer(uint32_t v) { layer_ = v; ++revision_; }

    const glm::vec3& center() const { return center_; }
    void set_center(const glm::vec3& v) { center_ = v; ++revision_; }

    dynamics::PhysicsMaterial* material() const { return material_; }
    void set_material(dynamics::PhysicsMaterial* m) { material_ = m; ++revision_; }

    /** @brief Bumped by every setter above (and by a subclass's own shape-parameter setters). */
    uint32_t revision() const { return revision_; }

    /** @brief The physics body this collider is currently bound to, or an invalid id before
     *         PhysicsSystem's first reconcile pass. Set only by PhysicsSystem. */
    dynamics::BodyId body_id() const { return body_id_; }

    /** @brief Called only by PhysicsSystem's reconcile pass. */
    void set_body_id(dynamics::BodyId id) { body_id_ = id; }

    /**
     * @brief Builds this collider's world-scale Shape.
     *
     * @param world_scale The owning SceneObject's world-space scale (per-axis for boxes,
     *                     max-component for spheres/capsules -- Unity's rule, since a
     *                     non-uniformly-scaled sphere/capsule is no longer that shape).
     * @return A Shape ready to hand to PhysicsWorld::add_body()/set_shape().
     */
    virtual collision::Shape make_shape(const glm::vec3& world_scale) const = 0;

protected:
    /** @brief Increments revision_ -- exposed so a subclass's own setters can bump it too. */
    void bump_revision_() { ++revision_; }

private:
    bool is_trigger_ = false;
    uint32_t layer_ = 0;
    glm::vec3 center_{0.0f};
    dynamics::PhysicsMaterial* material_ = nullptr;

    uint32_t revision_ = 0;
    dynamics::BodyId body_id_;
};

} // namespace components
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_COMPONENTS_COLLIDER_H
