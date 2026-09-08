/**
 * @file rigidbody.h
 * @brief Dynamics-configuration component -- mass, drag, kinematic flag, constraints -- plus
 *        the Unity-style force/velocity API gameplay code calls directly.
 *
 * A RigidbodyComponent only takes effect when a Collider is also present on the same
 * SceneObject: PhysicsSystem's reconcile pass is keyed on Collider (a body needs a shape to
 * exist in v1's binding layer, even though PhysicsWorld itself supports a shapeless body --
 * see Shape::enabled's doc). A bare Rigidbody with no Collider is a known v1 limitation, not
 * a bug: Unity's "Rigidbody with no Collider falls under gravity but hits nothing" case isn't
 * reachable from scene YAML today, only via PhysicsWorld::add_body() called directly.
 */

#ifndef PHYSXCOOPA_COMPONENTS_RIGIDBODY_H
#define PHYSXCOOPA_COMPONENTS_RIGIDBODY_H

#include <physxcoopa/world.h>
#include <physxcoopa/dynamics/body.h>
#include <physxcoopa/util/transform_bridge.h>

#include <coopa/scene/component.h>
#include <coopa/scene/scene_object.h>
#include <coopa/scene/components/transform_component.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <string>

namespace coopa {
namespace physx {
namespace components {

/**
 * @class RigidbodyComponent
 * @brief Configures a body's dynamics; also the gameplay-facing force/velocity API.
 *
 * `world_`/`body_id_` are set only by PhysicsSystem's reconcile pass (see set_body_binding()),
 * mirroring Collider::set_body_id() -- this is what lets add_force() etc. work without this
 * header depending on system/physics_system.h (which itself must depend on this header to
 * read mass/drag/kinematic during reconcile; a dependency the other way would cycle).
 */
class RigidbodyComponent : public coopa::scene::Component {
public:
    std::string type_name() const override { return "Rigidbody"; }

    float mass = 1.0f;
    float drag = 0.0f;
    float angular_drag = 0.05f;
    bool use_gravity = true;
    bool is_kinematic = false;
    dynamics::Interpolation interpolation = dynamics::Interpolation::Interpolate;
    uint32_t constraints = dynamics::kConstraintNone;

    /** @brief Called only by PhysicsSystem's reconcile pass. */
    void set_body_binding(PhysicsWorld* world, dynamics::BodyId id) {
        world_ = world;
        body_id_ = id;
    }

    dynamics::BodyId body_id() const { return body_id_; }

    void add_force(const glm::vec3& f) {
        if (dynamics::Body* b = body_()) b->add_force(f);
    }
    void add_torque(const glm::vec3& t) {
        if (dynamics::Body* b = body_()) b->add_torque(t);
    }
    void add_force_at_position(const glm::vec3& f, const glm::vec3& world_point) {
        if (dynamics::Body* b = body_()) b->add_force_at_position(f, world_point);
    }

    glm::vec3 velocity() const {
        const dynamics::Body* b = body_();
        return b ? b->linear_velocity : glm::vec3(0.0f);
    }
    void set_velocity(const glm::vec3& v) {
        if (dynamics::Body* b = body_()) b->linear_velocity = v;
    }

    glm::vec3 angular_velocity() const {
        const dynamics::Body* b = body_();
        return b ? b->angular_velocity : glm::vec3(0.0f);
    }
    void set_angular_velocity(const glm::vec3& w) {
        if (dynamics::Body* b = body_()) b->angular_velocity = w;
    }

    /**
     * @brief Moves a kinematic body's transform smoothly -- for v1, this is a direct alias
     *        for writing the owning object's Transform position; PhysicsSystem's sync-in
     *        pass derives the correct kinematic velocity from the resulting frame-to-frame
     *        position delta, exactly as if a script had called Transform::set_position()
     *        directly. Provided under this name for Unity-API familiarity.
     */
    void move_position(const glm::vec3& world_position) {
        if (!owner) return;
        coopa::util::Transform& t = owner->get_transform()->transform();
        util::set_world_trs(t, world_position, t.rotation_quat());
    }

    /** @brief See move_position()'s doc -- the rotational equivalent. */
    void move_rotation(const glm::quat& world_rotation) {
        if (!owner) return;
        coopa::util::Transform& t = owner->get_transform()->transform();
        util::set_world_trs(t, util::world_trs(t).position, world_rotation);
    }

private:
    dynamics::Body* body_() {
        return world_ ? world_->get_body(body_id_) : nullptr;
    }
    const dynamics::Body* body_() const {
        return world_ ? world_->get_body(body_id_) : nullptr;
    }

    PhysicsWorld* world_ = nullptr;
    dynamics::BodyId body_id_;
};

} // namespace components
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_COMPONENTS_RIGIDBODY_H
