/**
 * @file physics_system.h
 * @brief PhysicsSystem -- the ISceneSystem adapter binding PhysicsWorld to a coopa::scene
 *        Scene. This is the only file in physxcoopa that depends on coopa::scene.
 */

#ifndef PHYSXCOOPA_SYSTEM_PHYSICS_SYSTEM_H
#define PHYSXCOOPA_SYSTEM_PHYSICS_SYSTEM_H

#include <physxcoopa/world.h>
#include <physxcoopa/util/config.h>
#include <physxcoopa/util/error.h>
#include <physxcoopa/util/transform_bridge.h>
#include <physxcoopa/dynamics/inertia.h>
#include <physxcoopa/components/collider.h>
#include <physxcoopa/components/rigidbody.h>

#include <coopa/scene/scene.h>
#include <coopa/scene/scene_system.h>
#include <coopa/scene/scene_object.h>
#include <coopa/scene/components/transform_component.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace coopa {
namespace physx {
namespace system {

/**
 * @class PhysicsSystem
 * @brief Registered at coopa::scene::UpdatePhase::Physics (100) by default; owns a
 *        PhysicsWorld and reconciles Collider components to bodies every frame.
 *
 * Modelled on coopa::anim::AnimationSystem for the lazy-gather shape, but NOT for its
 * full-rebuild-on-dirty behavior: a Collider owns a live broadphase proxy, warm-start
 * impulses and a sleep timer, so a full rebuild on every refresh() would cold-start every
 * stack in the scene. Instead, refresh() only triggers a diff against the last-known
 * collider set (create/destroy), and every frame -- dirty or not -- checks each already-
 * bound collider's revision() for in-place parameter changes (see update_changed_shapes_()).
 */
class PhysicsSystem : public coopa::scene::ISceneSystem {
public:
    explicit PhysicsSystem(const util::PhysicsConfig& config = {}) : world_(config) {}

    const char* system_name() const override { return "Physics"; }

    void on_attach(coopa::scene::Scene&) override { dirty_ = true; }

    void execute(coopa::scene::Scene& scene, const coopa::scene::FrameContext& ctx) override {
        if (dirty_) {
            regather_(scene);
            dirty_ = false;
        }
        update_changed_shapes_();
        sync_transforms_in_(ctx.delta_time);
        world_.step(ctx.delta_time);
        write_transforms_back_();
        dispatch_events_();
    }

    /**
     * @brief Re-gathers the Collider list on the next execute() call. Call after
     *        adding/removing a Collider (or its owning SceneObject) at runtime -- exactly
     *        the same contract AnimationSystem::refresh() documents.
     *
     * Safe under a missed call: the next real refresh only ever *erases* map entries absent
     * from the fresh gather; a forgotten refresh after destroying an object leaks a body
     * until the next refresh, it never dereferences a stale pointer.
     */
    void refresh() { dirty_ = true; }

    PhysicsWorld& world() { return world_; }
    const PhysicsWorld& world() const { return world_; }

    /** @brief Number of colliders currently bound to a body -- for tests/diagnostics. */
    size_t bound_count() const { return bound_.size(); }

private:
    void regather_(coopa::scene::Scene& scene) {
        std::vector<components::Collider*> current = scene.get_components<components::Collider>();
        std::unordered_set<components::Collider*> current_set(current.begin(), current.end());

        for (auto it = bound_.begin(); it != bound_.end();) {
            if (current_set.find(it->first) == current_set.end()) {
                collider_by_index_.erase(it->second.index);
                world_.remove_body(it->second);
                last_revision_.erase(it->first);
                it = bound_.erase(it);
            } else {
                ++it;
            }
        }

        for (components::Collider* col : current) {
            if (bound_.find(col) != bound_.end()) continue; // already bound, revision handled elsewhere
            glm::vec3 world_scale = current_world_scale_(col->owner);
            dynamics::BodyId id = create_body_for_(col, world_scale);
            bound_[col] = id;
            collider_by_index_[id.index] = col;
            col->set_body_id(id);
            bind_rigidbody_(col, id);
            last_revision_[col] = col->revision();
        }
    }

    /** @brief Checked every frame regardless of dirty_ -- see the plan's rationale: runtime
     *         parameter changes ride this pass, no separate dirty flag needed for them. */
    void update_changed_shapes_() {
        for (auto& entry : bound_) {
            components::Collider* col = entry.first;
            if (last_revision_[col] == col->revision()) continue;
            glm::vec3 world_scale = current_world_scale_(col->owner);
            collision::Shape shape = col->make_shape(world_scale);
            shape.is_trigger = col->is_trigger();
            shape.layer = col->layer();
            shape.material = col->material();
            world_.set_shape(entry.second, shape);
            last_revision_[col] = col->revision();
        }
    }

    dynamics::BodyId create_body_for_(components::Collider* col, const glm::vec3& world_scale) {
        auto* rb = col->owner->get_component<components::RigidbodyComponent>();
        util::Trs trs = util::world_trs(col->owner->get_transform()->transform());

        dynamics::Body body;
        body.position = trs.position;
        body.orientation = trs.rotation;

        if (!rb) {
            body.type = dynamics::BodyType::Static;
        } else if (rb->is_kinematic) {
            body.type = dynamics::BodyType::Kinematic;
        } else {
            body.type = dynamics::BodyType::Dynamic;
            body.mass = rb->mass;
            body.inv_mass = rb->mass > 0.0f ? 1.0f / rb->mass : 0.0f;
            body.linear_drag = rb->drag;
            body.angular_drag = rb->angular_drag;
            body.use_gravity = rb->use_gravity;
            body.constraints = rb->constraints;
            body.interpolation = rb->interpolation;
        }

        collision::Shape shape = col->make_shape(world_scale);
        shape.is_trigger = col->is_trigger();
        shape.layer = col->layer();
        shape.material = col->material();

        if (shape.type == collision::ShapeType::TriangleMesh && body.type == dynamics::BodyType::Dynamic) {
            util::throw_error("MeshCollider on '" + col->owner->name() +
                               "' is attached to a non-kinematic Rigidbody -- non-convex mesh "
                               "colliders are static-only (matching Unity); mark the Rigidbody "
                               "kinematic or remove the MeshCollider.");
        }

        if (body.type == dynamics::BodyType::Dynamic) {
            body.inv_inertia_local = inertia_for_(shape, body.mass);
        }

        dynamics::BodyId id = world_.add_body(body, shape);
        dynamics::Body* stored = world_.get_body(id);
        stored->last_written_position = trs.position;
        stored->last_written_orientation = trs.rotation;
        return id;
    }

    static glm::vec3 inertia_for_(const collision::Shape& shape, float mass) {
        switch (shape.type) {
            case collision::ShapeType::Sphere:
                return dynamics::sphere_inverse_inertia(shape.radius, mass);
            case collision::ShapeType::Box:
                return dynamics::box_inverse_inertia(shape.half_extents, mass);
            case collision::ShapeType::Capsule: {
                glm::vec3 raw = dynamics::capsule_inverse_inertia(shape.capsule_radius, shape.capsule_half_height, mass);
                if (shape.capsule_axis == 0) return glm::vec3(raw.z, raw.y, raw.x);
                if (shape.capsule_axis == 1) return glm::vec3(raw.x, raw.z, raw.y);
                return raw;
            }
            case collision::ShapeType::TriangleMesh:
            default:
                return glm::vec3(0.0f);
        }
    }

    void bind_rigidbody_(components::Collider* col, dynamics::BodyId id) {
        if (auto* rb = col->owner->get_component<components::RigidbodyComponent>()) {
            rb->set_body_binding(&world_, id);
        }
    }

    /** @brief Reads world position/rotation for every bound body, deriving kinematic
     *         velocity or detecting a script-driven teleport on a dynamic body. */
    void sync_transforms_in_(float dt) {
        for (auto& entry : bound_) {
            dynamics::Body* body = world_.get_body(entry.second);
            if (!body) continue;
            coopa::util::Transform& transform = entry.first->owner->get_transform()->transform();
            util::Trs trs = util::world_trs(transform);

            if (body->type == dynamics::BodyType::Kinematic) {
                if (dt > util::k_epsilon) {
                    body->linear_velocity = (trs.position - body->last_written_position) / dt;
                    body->angular_velocity = derive_angular_velocity_(
                        body->last_written_orientation, trs.rotation, dt);
                }
                body->position = trs.position;
                body->orientation = trs.rotation;
                body->last_written_position = trs.position;
                body->last_written_orientation = trs.rotation;
            } else if (body->type == dynamics::BodyType::Dynamic) {
                float pos_delta = glm::length(trs.position - body->last_written_position);
                glm::quat rot_diff = trs.rotation * glm::inverse(body->last_written_orientation);
                float rot_delta = 1.0f - std::abs(std::clamp(rot_diff.w, -1.0f, 1.0f));
                if (pos_delta > 1e-4f || rot_delta > 1e-5f) {
                    body->position = trs.position;
                    body->orientation = trs.rotation;
                    body->wake();
                }
            }
        }
    }

    static glm::vec3 derive_angular_velocity_(const glm::quat& from, const glm::quat& to, float dt) {
        glm::quat delta = to * glm::inverse(from);
        float w = std::clamp(delta.w, -1.0f, 1.0f);
        float angle = 2.0f * std::acos(w);
        glm::vec3 axis(delta.x, delta.y, delta.z);
        float axis_len = glm::length(axis);
        if (axis_len < util::k_epsilon || angle < util::k_epsilon) return glm::vec3(0.0f);
        return (axis / axis_len) * (angle / dt);
    }

    /** @brief Writes each dynamic body's interpolated (or raw, per Body::interpolation)
     *         transform back, and records exactly what was written for next frame's
     *         teleport-detection comparison. */
    void write_transforms_back_() {
        float alpha = world_.interpolation_alpha();
        for (auto& entry : bound_) {
            dynamics::Body* body = world_.get_body(entry.second);
            if (!body || body->type != dynamics::BodyType::Dynamic) continue;

            glm::vec3 pos;
            glm::quat rot;
            if (body->interpolation == dynamics::Interpolation::Interpolate) {
                pos = glm::mix(body->prev_position, body->position, alpha);
                rot = glm::slerp(body->prev_orientation, body->orientation, alpha);
            } else {
                pos = body->position;
                rot = body->orientation;
            }

            coopa::util::Transform& transform = entry.first->owner->get_transform()->transform();
            util::set_world_trs(transform, pos, rot);
            body->last_written_position = pos;
            body->last_written_orientation = rot;
        }
    }

    static glm::vec3 current_world_scale_(coopa::scene::SceneObject* owner) {
        return util::world_trs(owner->get_transform()->transform()).scale;
    }

    /**
     * @brief Fires each of this frame's ContactEvents on both Colliders involved -- e.g. an
     *        Enter between A and B fires A's on_collision_enter(A, B) AND B's own
     *        on_collision_enter(B, A), matching Unity's two-sided callback convention.
     *
     * A BodyId whose index no longer maps to a bound Collider (the body was destroyed and its
     * Collider unbound this same frame, e.g. via a substep callback's deferred destroy) is
     * silently skipped -- there is no Collider left to fire on, and PhysicsWorld's own
     * generation check already keeps a reused slot's events from being misattributed (see
     * PhysicsWorld::emit_contact_events_'s doc).
     */
    void dispatch_events_() {
        for (const auto& ev : world_.events()) {
            auto it_a = collider_by_index_.find(ev.a.index);
            auto it_b = collider_by_index_.find(ev.b.index);
            if (it_a == collider_by_index_.end() || it_b == collider_by_index_.end()) continue;
            fire_(ev, it_a->second, it_b->second);
            fire_(ev, it_b->second, it_a->second);
        }
    }

    static void fire_(const collision::ContactEvent& ev, components::Collider* self, components::Collider* other) {
        coopa::event::Signal<components::Collider&, components::Collider&>* signal = nullptr;
        if (ev.is_trigger) {
            switch (ev.type) {
                case collision::ContactEvent::Type::Enter: signal = &self->on_trigger_enter; break;
                case collision::ContactEvent::Type::Stay: signal = &self->on_trigger_stay; break;
                case collision::ContactEvent::Type::Exit: signal = &self->on_trigger_exit; break;
            }
        } else {
            switch (ev.type) {
                case collision::ContactEvent::Type::Enter: signal = &self->on_collision_enter; break;
                case collision::ContactEvent::Type::Stay: signal = &self->on_collision_stay; break;
                case collision::ContactEvent::Type::Exit: signal = &self->on_collision_exit; break;
            }
        }
        signal->emit(*self, *other);
    }

    PhysicsWorld world_;
    std::unordered_map<components::Collider*, dynamics::BodyId> bound_;
    std::unordered_map<uint32_t, components::Collider*> collider_by_index_;
    std::unordered_map<components::Collider*, uint32_t> last_revision_;
    bool dirty_ = true;
};

/**
 * @brief Constructs a PhysicsSystem and registers it, mirroring install_animation_system().
 *
 * @param scene Scene to install into.
 * @param order Registration order; defaults to UpdatePhase::Physics (100), the slot
 *              scene_system.h reserves for exactly this. Pass e.g. 325 for an app whose
 *              gameplay is entirely animation-driven platforms with dynamic riders, so
 *              physics runs after Animation(300) but before TransformResolve(350) --
 *              see the plan's "PhysicsSystem — reconcile, don't rebuild" for why 100 stays
 *              the default and why the value must stay below LateBehaviour(400) regardless.
 * @return Non-owning pointer to the installed system.
 */
inline PhysicsSystem* install_physics_system(coopa::scene::Scene& scene,
                                              int order = static_cast<int>(coopa::scene::UpdatePhase::Physics),
                                              const util::PhysicsConfig& config = {}) {
    auto sys = std::make_unique<PhysicsSystem>(config);
    PhysicsSystem* raw = sys.get();
    scene.add_system(std::move(sys), order);
    return raw;
}

} // namespace system
} // namespace physx
} // namespace coopa

#endif // PHYSXCOOPA_SYSTEM_PHYSICS_SYSTEM_H
