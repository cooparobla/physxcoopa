# physxcoopa

**A header-only C++20 library for real-time rigid-body and cloth physics.**

physxcoopa simulates rigid bodies with box, sphere, capsule and triangle-mesh colliders,
hinge joints, triggers and XPBD cloth. It has two layers. `PhysicsWorld` is a plain
simulation you can drive from any program. `PhysicsSystem` is an optional adapter that binds
the world to a [libcoopa](https://github.com/cooparobla/libcoopa) scene, so colliders and
rigid bodies can be written as YAML components. The library has no renderer. It gives you
debug geometry as plain line segments to draw however you like.

<table>
  <tr>
    <td><img src="docs/images/physics_test.jpg" alt="Rigid-body test arena with spheres, boxes, a hinged door and stacked planks"></td>
    <td><img src="docs/images/cloth_test.jpg" alt="A cloth sheet draped over a ball"></td>
  </tr>
  <tr>
    <td align="center"><sub>Restitution, friction, stacking, hinges and triggers</sub></td>
    <td align="center"><sub>XPBD cloth pinned to a moving ball</sub></td>
  </tr>
</table>

<sub>Both scenes are simulated by physxcoopa and rendered by
[toyengine](https://github.com/cooparobla/toyengine), one of its users.</sub>

## Features

### Rigid bodies
- **Z-up, in metres.** Gravity defaults to `{0, 0, -9.81}`. A capsule's default axis is Z.
- **Static, kinematic and dynamic bodies.** Kinematic bodies take their velocity from how far
  they moved, so they carry and push dynamic bodies.
- **Colliders.** Box, sphere, capsule and triangle mesh. Several colliders under one rigid body
  form a compound body. Colliders can have a local offset (`center`).
- **Sequential-impulse solver.** Warm starting, a separate restitution pass and split-impulse
  position correction. Stacks settle and stay put.
- **Physics materials.** Static and dynamic friction, restitution, and Unity-style combine
  modes (`Average`, `Minimum`, `Maximum`, `Multiply`).
- **Sleeping.** Bodies are grouped into islands, and an island sleeps once it has been still for
  a short time.
- **Fast bodies.** Speculative contacts stop fast spheres and boxes from passing through thin
  walls. A speed cap covers the remaining cases.
- **Hinge joints.** Optional angle limits. A zero range gives a rigid weld. No motors or springs.
- **Per-axis freezing** of position and rotation, center-of-mass and inertia overrides, linear
  and angular drag.

### Collision and queries
- **Broadphase.** Two dynamic AABB trees (static and moving) and a persistent pair cache.
- **Layers.** 32 collision layers with a layer matrix. Layers can have names.
- **Events.** Collision and trigger enter, stay and exit. Collision events carry the contact
  points, normal, relative velocity and impulse.
- **Queries.** `raycast`, `raycast_any`, `raycast_all`, `sphere_cast`, `box_cast`,
  `capsule_cast`, `overlap_sphere`, `overlap_box` and `overlap_capsule`, each with a layer mask
  and an option to skip triggers.
- **Mesh colliders.** Meshes are welded, get a BVH, and correct internal edges, so a box slides
  across a tiled floor without catching on seams.

### Cloth
- **XPBD cloth** with many small substeps. Stretch and bend constraints, tethers, Coulomb
  friction, and optional self-collision.
- **Collides with every rigid shape**, including meshes and moving bodies.
- **Pinning.** Pin regions of a sheet to the world or to a rigid body, and the sheet follows it.
  Coupling is one-way: the cloth never pushes bodies.
- **Wind and air.** Wind, turbulence, drag and lift.
- **Sleeping.** A settled sheet stops simulating until an anchor or a nearby collider moves.

### Engine integration
- **Fixed timestep.** `step(dt)` runs fixed substeps (1/60 s by default) and exposes an
  interpolation factor for smooth rendering.
- **Deterministic.** `world_state_hash()` hashes every body's state, and the tests check that
  repeated runs, and serial versus multithreaded runs, give identical results.
- **Optional multithreading** on libcoopa's job engine. Without one, everything runs on the
  calling thread.
- **Debug drawing.** Collider wireframes, contacts, joints and broadphase nodes as colored line
  segments.
- **Scene components.** `BoxCollider`, `SphereCollider`, `CapsuleCollider`, `MeshCollider`,
  `Rigidbody`, `HingeJoint` and `Cloth`, with YAML parsers and Unity-style collision callbacks.

## Getting started

### 1. Clone

physxcoopa builds on [libcoopa](https://github.com/cooparobla/libcoopa) (scene graph, assets,
events, jobs; it also provides glm and fkYAML). Its `CMakeLists.txt` looks for libcoopa in a
folder named `libcoopa` next to the top-level project, unless a `coopa::lib` target already
exists. Clone the two side by side:

```bash
git clone git@github.com:cooparobla/libcoopa.git
git clone git@github.com:cooparobla/physxcoopa.git
```

### 2. Build

You need CMake 3.20 and a C++20 compiler. There is nothing to compile for the library itself.
Building the repo on its own builds the test program:

```bash
cd physxcoopa
cmake -B build && cmake --build build -j
```

### 3. Add it to your project

Add libcoopa first, then physxcoopa, and link `coopa::physx`:

```cmake
add_subdirectory(path/to/libcoopa)
add_subdirectory(path/to/physxcoopa)
target_link_libraries(my_game PRIVATE coopa::physx)
```

When physxcoopa is added this way, it defines only the `coopa::physx` target, not its tests.

### 4. Simulate without a scene

`PhysicsWorld` needs no scene. You create bodies and step it yourself:

```cpp
#include <physxcoopa/world.h>

using namespace coopa::physx;

PhysicsWorld world;  // gravity defaults to {0, 0, -9.81}

// Static floor: a box with half extents 5 x 5 x 0.5, top face at z = 0.
dynamics::Body ground;
ground.type = dynamics::BodyType::Static;
ground.position = {0.0f, 0.0f, -0.5f};
world.add_body(ground, collision::Shape::make_box({5.0f, 5.0f, 0.5f}));

// Dynamic ball, radius 0.5, dropped from 5 m.
dynamics::Body ball;
ball.position = {0.0f, 0.0f, 5.0f};
ball.mass = 2.0f;
ball.inv_mass = 1.0f / ball.mass;
ball.inv_inertia_local = dynamics::sphere_inverse_inertia(0.5f, ball.mass);
dynamics::BodyId id = world.add_body(ball, collision::Shape::make_sphere(0.5f));

for (int frame = 0; frame < 180; ++frame) {
    world.step(1.0f / 60.0f);  // runs as many fixed substeps as the time calls for
    for (const collision::ContactEvent& ev : world.events()) {
        if (ev.type == collision::ContactEvent::Type::Enter) { /* first touch */ }
    }
}

float z = world.get_body(id)->position.z;  // about 0.5: the ball rests on the floor

query::RaycastHit hit;
geometry::Ray ray{{0.0f, 0.0f, 10.0f}, {0.0f, 0.0f, -1.0f}, 100.0f};
if (world.raycast(ray, hit)) { /* hit.point, hit.normal, hit.distance, hit.body */ }
```

You set a body's mass, inverse mass and inverse inertia yourself. `dynamics/inertia.h` has
helpers for each shape. `Shape::make_box` takes half extents.

### 5. Simulate a scene

With a libcoopa scene, register the component parsers once, load the scene, then install the
system. `Scene::update()` steps physics along with every other scene system:

```cpp
#include <physxcoopa/physx_yaml.h>
#include <coopa/scene/scene_loader.h>

coopa::asset::AssetManager assets;
coopa::physx::register_physics_components(assets);  // before any SceneLoader::load()

coopa::scene::Scene scene = coopa::scene::SceneLoader::load("assets/scene.yaml");
auto* physics = coopa::physx::system::install_physics_system(scene);

// Game loop:
scene.update(dt);
scene.late_update(dt);
```

Colliders bind to bodies, and an object needs a collider for its `Rigidbody` to simulate.
Collider components without a `Rigidbody` are static. Collider sizes are full sizes, and they
scale with the object's transform.

```yaml
scene:
  scene_name: PhysicsHello
  root_objects:
    - name: ground
      components:
        - type: Transform
          position: { x: 0.0, y: 0.0, z: -0.5 }
        - type: BoxCollider
          size: { x: 20.0, y: 20.0, z: 1.0 }
          material: ice                 # loads physics_materials/ice.yaml
    - name: crate
      components:
        - type: Transform
          position: { x: 0.0, y: 0.0, z: 3.0 }
        - type: BoxCollider
          size: { x: 1.0, y: 1.0, z: 1.0 }
          material: { dynamic_friction: 0.4, static_friction: 0.5, restitution: 0.2 }
        - type: Rigidbody
          mass: 4.0
          freeze_rotation: { x: false, y: false, z: true }
    - name: pickup
      components:
        - type: Transform
          position: { x: 3.0, y: 0.0, z: 0.5 }
        - type: SphereCollider
          radius: 0.5
          is_trigger: true
```

A shared material is a small YAML file, `physics_materials/<name>.yaml`, next to the scene:

```yaml
dynamic_friction: 0.05
static_friction: 0.05
restitution: 0.1
friction_combine: Minimum
```

From code, use the components and the system:

```cpp
using namespace coopa::physx;

auto* crate = scene.find_object("crate");
crate->get_component<components::RigidbodyComponent>()->add_impulse({0.0f, 0.0f, 5.0f});

crate->get_component<components::BoxCollider>()->on_collision_enter.connect(
    [](components::Collider& self, const components::Collision& c) {
        // c.object, c.normal, c.impulse, c.contacts[0 .. c.contact_count)
    });

system::PhysicsSystem::RaycastHit hit;   // resolves to collider, object and rigidbody
if (physics->raycast(geometry::Ray{{0, 0, 10}, {0, 0, -1}, 100.0f}, hit)) { /* hit.object */ }
```

For code that must run once per fixed substep, derive from `components::FixedUpdateBehaviour`
and override `fixed_update(PhysicsWorld&, float h)`. Call `physics->refresh()` after adding or
removing colliders at runtime.

## Component reference

Every key is optional.

| Component | Keys |
|---|---|
| All colliders | `center`, `is_trigger`, `enabled`, `layer` (index or layer name), `material` (asset name or inline mapping) |
| `BoxCollider` | `size` |
| `SphereCollider` | `radius` |
| `CapsuleCollider` | `radius`, `height` (total, including the caps), `direction` (0, 1 or 2 for X, Y or Z; default 2) |
| `MeshCollider` | `mesh_path` (loads `meshes/<name>.yaml`, a `vertices:`/`faces:` mesh), `convex` |
| `Rigidbody` | `mass`, `drag`, `angular_drag`, `use_gravity`, `is_kinematic`, `interpolation` (`Interpolate` or `None`), `freeze_position`, `freeze_rotation`, `velocity`, `angular_velocity` |
| `HingeJoint` | `connected_object`, `anchor` and `axis` (in this object's local frame), `limits: { min, max }` (degrees) |
| `Cloth` | `resolution`, `size`, `mass`, `shear`, `stretch_compliance`, `bend_compliance`, `damping`, `thickness`, `friction`, `gravity_scale`, `max_velocity`, `external_acceleration`, `wind`, `wind_turbulence`, `air_drag`, `air_lift`, `self_collision`, `self_distance`, `tether_scale`, `substeps`, `iterations`, `sleep_threshold`, `sleep_time`, `layer`, `anchors: [{ object, point, radius }]` |

A cloth sheet is built in its object's local XY plane, centred on the object. Each anchor pins
the particles within `radius` of `point`. With `object`, the point is in that object's local
frame and the pinned particles follow its rigid body. Without it, the point is in world space and
the particles stay fixed.

Project-wide settings come from a `physics:` YAML block. Parse it with
`util::parse_physics_settings()` and pass the result to `register_physics_components()` (for
layer names) and `install_physics_system()`:

```yaml
physics:
  gravity: { x: 0.0, y: 0.0, z: -9.81 }
  fixed_timestep: 0.016666
  solver: { velocity_iterations: 16, position_iterations: 4, max_substeps: 8 }
  cloth: { substeps: 4, iterations: 1 }
  layers: [Default, Ground, Triggers]
  ignore_layer_collisions:
    - [Triggers, Triggers]
  debug_draw: [Colliders, Contacts]
  parallel_threshold: 64
```

## How a step works

```text
PhysicsWorld::step(dt)
  clamp dt to 0.25 s and add it to the accumulator
  while a full fixed step is due (at most max_substeps times):
    step_fixed(h)
      apply gravity and forces
      broadphase -> layer filter -> narrowphase -> contact manifolds
      emit on_substep
      solve contacts and joints, then update islands and sleep
      integrate positions
  advance every cloth once, on the frame clock

PhysicsSystem::execute(scene, frame)        (UpdatePhase::Physics, order 100)
  after install or refresh(): bind new colliders, joints and cloths; drop removed ones
  apply changed collider and Rigidbody fields
  read transforms in (kinematic velocities, teleports)
  world.step(frame.delta_time)
  write interpolated transforms back, then fire collision and trigger callbacks
```

`step_fixed(h)` runs one substep with no accumulator. It is handy in tests, but it does not
advance cloth. Events from `step()` collect over the whole frame and clear on the next call.

## Testing

```bash
ctest --test-dir build        # or run ./build/physxcoopa directly
```

`test.cpp` holds 86 headless tests. They cover narrowphase math, stacking, friction, restitution,
kinematic platforms, sleep, mesh colliders, queries, triggers, joints, scene binding, materials,
settings, cloth, and determinism.

## Project layout

```
physxcoopa/
├── world.h        PhysicsWorld: bodies, joints, cloths, stepping, queries, debug draw
├── physx_yaml.h   register_physics_components(): YAML parsers and asset loaders
├── util/          PhysicsConfig, PhysicsSettings, math helpers, transform bridge
├── geometry/      AABB, ray, sphere, OBB, capsule, triangle mesh, mesh BVH
├── collision/     shapes, SAT and clipping, contact manifolds, mesh contacts, events
├── broadphase/    dynamic AABB tree, layer matrix, pair cache
├── dynamics/      bodies, inertia, integrator, islands, hinge joints, materials, solver
├── cloth/         cloth data, grid builder, collision, XPBD solver
├── query/         raycast, shape cast and overlap helpers
├── debug/         DebugDraw line output
├── loaders/       asset loaders for triangle meshes and physics materials
├── components/    scene components: colliders, Rigidbody, HingeJoint, Cloth, FixedUpdateBehaviour
└── system/        PhysicsSystem and install_physics_system()
test.cpp           the test suite
```

Only `system/`, `components/` and `physx_yaml.h` use libcoopa's scene module. `PhysicsWorld`
itself uses only glm and libcoopa's signals, logger and job engine.

## Notes

- **Queries during a substep.** Run queries between steps. Inside an `on_substep` callback the
  bodies are mid-solve.
- **Changing bodies during a substep.** From `on_substep` you can apply forces and set
  velocities. `create_body()` and `destroy_body()` are deferred to the start of the next substep.
- **Mesh colliders are static or kinematic.** A `MeshCollider` on a dynamic `Rigidbody` throws
  when it binds. The `convex` key is read but convex mesh colliders are not implemented yet.
- **Physics runs before animation.** At its default order (100), physics reads transforms
  before animation (300) writes them. To let animation drive kinematic bodies, install the
  system at a later order, for example `install_physics_system(scene, 325)`.
- **Joints and cloth bind once.** Changing a hinge's anchor or a cloth's grid or anchors after
  binding has no effect until the component is re-added. A cloth's runtime parameters, such as
  wind and friction, can change at any time through `ClothComponent::cloth_mut()->params`.

## Documentation

- Each folder under [`physxcoopa/`](physxcoopa/) has its own README with module details.
- The headers are documented in Doxygen style. `world.h`, `physx_yaml.h` and
  `system/physics_system.h` are the best places to start.
