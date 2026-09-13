# physxcoopa/system (`coopa::physx::system`)

The `coopa::scene` adapter — the only file in physxcoopa that depends on `coopa::scene` at all.
`PhysicsWorld` itself (`world.h`) never sees a `Scene` reference.

```text
PhysicsSystem::execute(scene, ctx)
  1. regather_()            -- diff the scene's Collider set against last frame's (create/destroy bodies)
  2. update_changed_shapes_() -- every bound collider's revision() checked for in-place param changes
  3. sync_transforms_in_()  -- kinematic velocity derivation, dynamic-body teleport detection
  4. world_.step(ctx.delta_time)
  5. write_transforms_back_() -- interpolated (or raw) transform write, teleport-detection bookkeeping
  6. dispatch_events_()     -- fires each of this frame's ContactEvents on both Colliders involved
```

| File | Purpose |
|---|---|
| [`physics_system.h`](physics_system.h) | `PhysicsSystem : ISceneSystem` (registered at `UpdatePhase::Physics = 100` by default) and `install_physics_system(scene)`. Owns the `PhysicsWorld`; every method above is a private step of `execute()`. The reconcile pass runs `regather_()` → `regather_joints_()` → `regather_cloths_()` in that order: a joint needs both connected objects already bound to a body, and a `ClothComponent`'s anchors name objects whose `Rigidbody` must likewise already be bound. |

**PhysicsSystem's transform reads/writes always use `Transform::get_world_matrix()`** (the
lazy-recomputing accessor), never the newer non-recomputing `world_matrix()` — physics runs
before `Animation`/`TransformResolve`/`LateBehaviour` all run this frame, so it cannot assume any
resolve pass has completed yet for objects it wasn't itself responsible for dirtying.

## Usage Example

```cpp
#include <physxcoopa/physx_yaml.h>

coopa::physx::register_physics_components(assets);      // once, alongside other component registrations
coopa::physx::install_physics_system(scene);             // once, after the scene's first load

// Per frame: scene.update(dt) already drives every installed ISceneSystem, physics included.
```
