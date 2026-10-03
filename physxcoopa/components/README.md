# physxcoopa/components (`coopa::physx::components`)

Unity-equivalent `coopa::scene::Component`s. All passive descriptor data — none of them creates
or destroys a physics body itself; `system::PhysicsSystem`'s reconcile pass is the only thing that
does, diffing the scene's current `Collider` set against last frame's rather than rebuilding on
every change (a collider owns a live broadphase proxy, warm-start impulses and a sleep timer that
must survive an unrelated collider's parameter change elsewhere in the scene).

| File | Purpose |
|---|---|
| [`collider.h`](collider.h) | `Collider` (base) — `is_trigger`/`layer`/`center`/`material`, the six `on_collision_*`/`on_trigger_*` signals, a `revision_` counter bumped by every setter (what `PhysicsSystem` diffs against to know a shape needs rebuilding), and the pure-virtual `make_shape(world_scale)`. |
| [`box_collider.h`](box_collider.h) | `BoxCollider` — `size` (full extents, Unity semantics; half-extents internally). |
| [`sphere_collider.h`](sphere_collider.h) | `SphereCollider` — `radius`. |
| [`capsule_collider.h`](capsule_collider.h) | `CapsuleCollider` — `radius`/`height`/`direction` (default `2`/Z, not Unity's Y, given this world is Z-up). |
| [`mesh_collider.h`](mesh_collider.h) | `MeshCollider` — `mesh` (an `AssetHandle<TriangleMesh>`) + `convex` (unused in v1: non-convex mesh colliders are static-only, matching Unity; a dynamic mesh collider needs a convex-hull builder, deliberately out of scope alongside GJK/EPA). |
| [`rigidbody.h`](rigidbody.h) | `RigidbodyComponent` — mass/drag/`is_kinematic`/`interpolation`/`constraints`, plus the gameplay-facing `add_force()`/`add_torque()`/`add_impulse_at_position()`/`velocity()`/`velocity_at_point()`/`move_position()`/`move_rotation()` API, and `local_center_of_mass()`/`world_center_of_mass()`/`rotation()` -- the live body pose a substep callback needs to place points authored in the owner's frame (e.g. buoyancy pontoons). Only takes effect alongside a `Collider` on the same object — PhysicsSystem's reconcile is keyed on `Collider`, so a bare `Rigidbody` with no `Collider` is a known v1 limitation (reachable via `PhysicsWorld::add_body()` directly, just not from scene YAML). |
| [`cloth.h`](cloth.h) | `ClothComponent` — a rectangular XPBD sheet built in the owner's local XY plane at bind time; every `cloth::ClothParams` field is authorable flat, plus `ClothAnchorSpec` pinned regions naming a sibling object. Resolved by `PhysicsSystem::regather_cloths_()`, never by the YAML parser (the named object may not exist yet while parsing). Binding is creation-time only; the runtime tunables (wind, friction, damping) are live through `cloth_mut()`. |
| [`fixed_update_behaviour.h`](fixed_update_behaviour.h) | `FixedUpdateBehaviour` — subclass and override `fixed_update(world, h)` for a per-substep callback, without adding a `Component::fixed_update()` hook to libcoopa itself; connects to `PhysicsWorld::on_substep` lazily on first `update()`. |

## Usage Example

```cpp
auto* col = object.add_component<coopa::physx::components::BoxCollider>();
col->set_size(glm::vec3(1.0f));
col->set_is_trigger(true);
col->on_trigger_enter.connect([](auto& self, auto& other) {
    // ...
});

auto* rb = object.add_component<coopa::physx::components::RigidbodyComponent>();
rb->mass = 2.0f;
```
