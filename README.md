# physxcoopa

A header-only C++20 real-time rigid-body physics engine, in the same family as libcoopa,
gfxcoopa, and sfxcoopa: `add_subdirectory()` it as a sibling repo and link `coopa::physx`.

**Z-up, right-handed, gravity defaults to `{0, 0, -9.81}`** — every other coopa repo's world is
Z-up (`toyengine`'s camera, `gfxcoopa`'s fog height falloff, etc.), and physxcoopa follows that
convention rather than the more common Y-up one a physics engine ported from elsewhere might
assume. `CapsuleCollider`'s default axis is Z (`direction = 2`), not Unity's Y, for the same
reason.

## Architecture

physxcoopa is a two-layer split: a Scene-agnostic simulation core, and a thin adapter plugging it
into libcoopa's scene-system pipeline.

```text
PhysicsWorld            -- the simulation. No dependency on coopa::scene at all.
  step(dt)                accumulates and runs 0..max_substeps fixed calls to step_fixed(h)
  step_fixed(h)            ONE substep, no accumulator -- what test.cpp drives directly
  add_body / remove_body, raycast/overlap, on_substep signal, events(), debug_draw(), world_state_hash()

PhysicsSystem : ISceneSystem   -- the scene adapter (system/). Owns a PhysicsWorld.
  execute(Scene&, FrameContext)
    1. reconcile Collider/Rigidbody components -> bodies (diff, not rebuild)
    2. sync world transforms -> bodies (kinematic velocity derivation, teleport detection)
    3. world_.step(ctx.delta_time)
    4. write interpolated transforms back; dispatch queued collision/trigger events
```

`PhysicsWorld` depends only on glm, `coopa::event::Signal`, and `coopa::debug::Logger` — never on
`coopa::scene`. That split is what lets `test.cpp` exercise the entire solver/narrowphase/
broadphase stack with no `Scene` at all, and what lets gameplay code reach the world via
`scene->find_system("Physics")` without ever seeing `PhysicsWorld`'s internals.

**Everything runs on the main thread.** `coopa::job`'s `parallel_for_blocking()` is the proven
idiom for a future job-parallel narrowphase (see `AnimationSystem`/`TransformSystem` for existing
call sites to model against), but no consumer of physxcoopa currently installs a job engine on its
`Scene`, so v1 ships zero job-dispatch code.

## Modules

| Module | Contents |
|---|---|
| [`util/`](physxcoopa/util/README.md) | Math constants, `PhysicsConfig`, error helper, the `Transform` world-space bridge. |
| [`geometry/`](physxcoopa/geometry/README.md) | `AABB`/`Ray`/`Sphere`/`OBB`/`Capsule`/`TriangleMesh`/`MeshBVH` — pure geometric primitives. |
| [`collision/`](physxcoopa/collision/README.md) | `Shape`, `ContactManifold`/`ContactEvent`, and every shape-pair's contact generation (SAT, clipping, segment math, mesh internal-edge correction). |
| [`broadphase/`](physxcoopa/broadphase/README.md) | Dynamic AABB tree, layer matrix, persistent pair cache. |
| [`dynamics/`](physxcoopa/dynamics/README.md) | `Body`, inertia tensors, the integrator, islands, and the sequential-impulse solver. |
| [`cloth/`](physxcoopa/cloth/README.md) | Small-substep XPBD cloth — sheets that drape over rigid colliders and follow bodies they are pinned to. One-way coupled, like Unity's `Cloth`. |
| [`query/`](physxcoopa/query/README.md) | Raycast/sphere-cast/overlap shape-dispatch helpers behind `PhysicsWorld`'s query API. |
| [`debug/`](physxcoopa/debug/README.md) | Pure-data debug line visualization. |
| [`loaders/`](physxcoopa/loaders/README.md) | `TriangleMeshLoader` — `coopa::asset::AssetManager` integration for mesh colliders. |
| [`components/`](physxcoopa/components/README.md) | `Collider`/`BoxCollider`/`SphereCollider`/`CapsuleCollider`/`MeshCollider`/`RigidbodyComponent`/`HingeJointComponent`/`ClothComponent`/`FixedUpdateBehaviour`. |
| [`system/`](physxcoopa/system/README.md) | `PhysicsSystem` — the only file depending on `coopa::scene`. |
| [`world.h`](physxcoopa/world.h) | `PhysicsWorld` — the object `test.cpp` constructs directly. |
| [`physx_yaml.h`](physxcoopa/physx_yaml.h) | `register_physics_components(AssetManager&)` + re-exports `install_physics_system()`. |

## Usage Example

Headless, no Scene at all (exactly how `test.cpp` exercises the engine):

```cpp
#include <physxcoopa/world.h>

coopa::physx::PhysicsWorld world;

coopa::physx::dynamics::Body ground;
ground.type = coopa::physx::dynamics::BodyType::Static;
world.add_body(ground, coopa::physx::collision::Shape::make_box({5.0f, 5.0f, 0.5f}));

coopa::physx::dynamics::Body ball;
ball.position = {0.0f, 0.0f, 5.0f};
ball.mass = 1.0f;
ball.inv_mass = 1.0f;
world.add_body(ball, coopa::physx::collision::Shape::make_sphere(0.5f));

for (int i = 0; i < 300; ++i) world.step_fixed(coopa::physx::util::k_default_fixed_dt);
```

Scene-bound, via `toyengine`/`blendy`:

```cpp
#include <physxcoopa/physx_yaml.h>

coopa::physx::register_physics_components(assets);   // once, alongside other component registrations
coopa::physx::install_physics_system(scene);          // once, right after the scene's first load
// scene.update(dt) already drives every installed ISceneSystem, physics included.
```

## Caveats

- **Kinematic bodies interacting with `Animator`-driven transforms**: physics runs at
  `UpdatePhase::Physics = 100`, before `Animation = 300`. An `Animator` writing to the same
  `Transform` a kinematic `Rigidbody` is bound to will have its write overwritten every frame by
  `PhysicsSystem`'s sync-in pass unless the app installs physics at a later order (e.g. `325`,
  after `Animation` but before the optional `TransformResolve = 350`) — see
  `install_physics_system()`'s doc.
- **Queries (`raycast`/`raycast_all`/`sphere_cast`/`overlap_sphere`/`overlap_box`) are only valid
  between phases, never from inside `on_substep`** — a query issued from a substep callback
  observes mid-solve, not-yet-integrated state.
- **Non-convex `MeshCollider`s are static-only**, matching Unity. Attaching one to a non-kinematic
  `Rigidbody` throws at bind time.
- **`world_state_hash()`** (FNV-1a over every body's position/orientation/velocity bits) is
  checked from Phase 2 onward specifically so every later phase (broadphase tree vs. brute force,
  mesh colliders, ...) has a bit-exact oracle to diff against, not just a visual "looks right."

## Documentation

```bash
coopadocs build   # generates ./.docs/index.html from every file's Doxygen-style comments
coopadocs show    # opens it
```
