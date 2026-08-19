# physxcoopa — Real-Time Rigid Body Physics for the coopa Stack

## Context

`/home/coopa/git/physxcoopa` is an empty repo (`.git/` only). It needs to become the physics sibling of `libcoopa` / `gfxcoopa` / `uicoopa` / `sfxcoopa`: a header-only C++20 rigid body engine with Unity-equivalent semantics — box/capsule/sphere/static-mesh colliders, physics materials, static/kinematic/dynamic bodies, scene queries, triggers, collision layers — consumed by `blendy`.

Exploration confirmed there is **no collision code of any kind** anywhere in the stack: no `AABB`, `Ray`, `Sphere`, or `Capsule` type, no broadphase, no BVH, no 3D raycast. `coopa/util/math.h` has only a `Mat4` wrapper and `MathUtil`. The single ray/primitive routine is `intersect_box` in `gfxcoopa/engine/gi/gi_baker.h:29-73`, welded to GI baking. This is greenfield.

### Settled decisions

- **From scratch, header-only, zero third-party.** Sequential-impulse solver, dynamic AABB tree broadphase, SAT + GJK/EPA narrowphase — all in the coopa house style. Only `glm` and `fkYAML` (already vendored in `libcoopa/includes/`) are used.
- **Extend `coopa::util::Transform` with a quaternion path** rather than converting lossily each step.
- **In scope:** scene queries (raycast/overlap/sweep), triggers + collision callbacks, collision layers + layer matrix, debug collider visualizer.

### Five hard constraints discovered in the existing code

1. **`Transform` is Euler-degrees-only, local-space-only.** `/home/coopa/git/libcoopa/coopa/util/transform.h:220-229` stores `glm::vec3 rotation_degrees_` and composes `R = glm::eulerAngleZYX(z, y, x)` at `:202-206` — so the documented "XYZ" is really `Rz*Ry*Rx`. There is no quaternion field, no `set_world_position`, no basis accessors. `<glm/gtc/quaternion.hpp>` is included at `:16` but `glm::quat` appears **nowhere** in the entire stack. Phase 2 fixes this.
2. **The world is Blender Z-up, not Unity Y-up.** The cube spans z 0..1, `plane.000` is the ground at z=0, `animation_component.h:126-128` animates x/y with `z = height`. The exporter's `unity_axes` flag (`python/blendy/bpy/coords.py:49-54`) is **not** applied to shipped scenes. **Default gravity is `(0, 0, -9.81)`** and every "up" assumption — capsule axis, contact classification, ground checks — is +Z.
3. **No CPU mesh data survives GPU upload.** `gfxcoopa/engine/data/mesh.h:124-268` builds function-local vertex/index vectors, uploads, and lets them die; `Mesh` keeps only two `Buffer`s and `index_count_`. Readback is not viable — `Buffer::vertex`/`index` allocate `HOST_ACCESS_SEQUENTIAL_WRITE_BIT` (`buffer.h:169,185`) and `Buffer` exposes no `download()`. Phase 5 re-parses the mesh YAML independently; **no gfxcoopa change is required.**
4. **No fixed timestep exists.** `blendy/src/blendy/core/time.h:46-51` is a raw variable delta with no clamp, no accumulator. The first frame's `dt` spans all Vulkan and scene init — hundreds of ms. physxcoopa owns the accumulator and clamps.
5. **`SceneLoader`'s parser registry is an exact string match with no `!`-stripping** (`scene_loader.h:527-535`), and unknown tags are **swallowed silently with no diagnostic**. Shipped scenes use `type: Transform` mapping keys; uicoopa registered only `"!RectTransform"`-style tags. **Every physics component must register under both spellings** or it will be silently ignored.

---

## Architecture

```
blendy frame:  poll → time.update() → physics.step(dt) → scene_mgr.update(dt) → render()
                                          │
              ┌───────────────────────────┴────────────────────────────┐
              │  PhysicsWorld::step(dt)                                │
              │    accumulator += min(dt, 0.25)                        │
              │    while (acc >= h && n < 8):  substep(h);  acc -= h   │
              │      1. sync kinematic/teleported transforms → bodies  │
              │      2. integrate forces (gravity, drag) → velocities  │
              │      3. broadphase: refit + query trees → pairs        │
              │      4. layer matrix + kinematic/static pair reject    │
              │      5. narrowphase → persistent manifolds (warm)      │
              │      6. build islands (union-find over contact graph)  │
              │      7. solve velocity (8 it) → position (3 it)        │
              │      8. integrate velocities → positions/orientations  │
              │      9. sleep islands below threshold                  │
              │   alpha = acc / h                                      │
              │   write interpolated transforms; diff + emit events    │
              └────────────────────────────────────────────────────────┘
```

**Everything runs on the main thread.** `coopa::job` is available but blendy never uses it and doesn't even link `Threads::Threads` (`blendy/CMakeLists.txt:88-96`). More decisively, `Transform`'s matrix cache is explicitly **not** thread-safe (only `dirty_` is atomic), so transform write-back must be serial regardless. The solver is structured in islands so a `JobEngine*` (defaulting to `nullptr`) can parallelize island solving later without redesign.

**physxcoopa contains no Vulkan and no gfxcoopa dependency.** The debug visualizer produces a plain line list; blendy renders it.

### File tree to create

```
physxcoopa/
├── CMakeLists.txt · README.md · .coopadocs · .gitignore
├── test.cpp                            headless suite (analytic + determinism)
├── configuration/root_directory.h.in
├── plans/2026-08-17_physxcoopa-realtime-physics.md
└── physxcoopa/
    ├── util/         math.h · error.h · config.h            coopa::physx::util
    ├── geometry/     aabb.h · ray.h · sphere.h · obb.h · capsule.h
    │                 triangle_mesh.h · mesh_bvh.h            coopa::physx::geometry
    ├── collision/    shape.h · gjk.h · epa.h · sat.h · clip.h
    │                 manifold.h · narrowphase.h              coopa::physx::collision
    ├── broadphase/   aabb_tree.h · layer_matrix.h · pair_cache.h
    │                                                          coopa::physx::broadphase
    ├── dynamics/     physics_material.h · body.h · inertia.h
    │                 island.h · solver.h · integrator.h       coopa::physx::dynamics
    ├── query/        queries.h                                coopa::physx::query
    ├── debug/        debug_draw.h                             coopa::physx::debug
    ├── components/   collider.h · box_collider.h · capsule_collider.h
    │                 sphere_collider.h · mesh_collider.h
    │                 rigidbody.h                              coopa::physx::components
    ├── world.h       PhysicsWorld — the one object apps construct
    └── physx_yaml.h  register_physics_components() + PhysicsResources
```

House style is identical to sfxcoopa's plan: `#ifndef PHYSXCOOPA_<DIR>_<FILE>_H`, nested `namespace coopa { namespace physx { namespace <mod> {`, `PascalCase` types / `snake_case` methods / `private_members_`, `enum class`, `inline constexpr k_snake_case`, `throw std::runtime_error("[physxcoopa] ...")`, `coopa::debug::Logger("Physics")`, `coopa::event::Signal` for all callbacks, `@file`/`@class`/`@code` Doxygen on every file and public type, per-module `README.md`. CMake follows `uicoopa/CMakeLists.txt`: no `add_library`, no `install()` — consumers just `include_directories()` the sibling path.

---

## Phase 0 — Scaffolding

`.coopadocs` (`include:\n  - physxcoopa`), `.gitignore` (`build/`, `.docs/`), `configuration/root_directory.h.in` copied from libcoopa, and a `CMakeLists.txt` mirroring uicoopa's: C++20, `ROOT_DIR_PARENT`, `include_directories` for `${LIBCOOPA_DIR}/includes/` (glm, fkYAML) and `${LIBCOOPA_DIR}` (coopa/), `add_executable(physxcoopa test.cpp)`, `enable_testing()`, `add_test(NAME physxcoopa_tests COMMAND physxcoopa)`. No `find_package` beyond nothing — physics links only `pthread m`.

## Phase 1 — Math and geometry primitives

- **`util/math.h`** — `inline constexpr float k_epsilon = 1e-6f`, `k_gravity_z = -9.81f`, `k_default_fixed_dt = 1.0f/60.0f`, plus `safe_normalize`, `orthonormal_basis(n, t1, t2)` (needed per contact for friction), and `glm::mat3 skew(const glm::vec3&)`.
- **`util/config.h`** — all solver tunables in one struct so they're discoverable and testable: `velocity_iterations = 8`, `position_iterations = 3`, `baumgarte = 0.2f`, `linear_slop = 0.005f`, `max_linear_correction = 0.2f`, `restitution_threshold = 1.0f`, `sleep_linear = 0.01f`, `sleep_angular = 0.02f`, `sleep_time = 0.5f`, `aabb_margin = 0.1f`, `max_substeps = 8`.
- **`geometry/aabb.h`** — `struct AABB { glm::vec3 min, max; }` with `merge`, `overlaps`, `contains`, `expand(float)`, `surface_area()` (SAH), `center()`, `extents()`, `transform(const glm::mat4&)`.
- **`geometry/ray.h`** — `struct Ray { glm::vec3 origin, direction; float max_distance; }` + slab test `intersect(const AABB&, float& t)`. Crib the slab structure from `gi_baker.h:29-73` but strip the GI-specific `albedo` payload.
- **`geometry/obb.h` · `sphere.h` · `capsule.h`** — `Capsule` is a segment `{a, b}` plus `radius`, which makes every capsule test a segment-distance problem. Include the three workhorse helpers here: `closest_point_on_segment`, `closest_points_segment_segment` (with the parallel-degenerate branch), `closest_point_on_obb`.
- **`geometry/triangle_mesh.h`** — format-agnostic: `std::vector<glm::vec3> vertices; std::vector<uint32_t> indices;` plus `AABB bounds()`, `weld(float epsilon)`, and precomputed per-triangle normals. Construction from raw arrays; the YAML path is a separate free function so physxcoopa is not welded to blendy's asset format.

## Phase 2 — libcoopa `Transform` quaternion extension

The one cross-repo change. Backwards compatible, ~40 lines in `/home/coopa/git/libcoopa/coopa/util/transform.h`:

```cpp
    void set_rotation_quat(const glm::quat& q);   // sets use_quat_ = true
    const glm::quat& rotation_quat() const;       // derives from Euler when use_quat_ is false
    void set_world_position(const glm::vec3& p);  // applies inverse(parent world) when parented
    void set_world_rotation(const glm::quat& q);
    glm::vec3 world_position() const;
private:
    glm::quat rotation_quat_{1,0,0,0};
    bool      use_quat_ = false;
```

`recompute_()` at `:199-218` gains one branch:
```cpp
        glm::mat4 R = use_quat_ ? glm::mat4_cast(rotation_quat_)
                                : glm::eulerAngleZYX(glm::radians(rotation_degrees_.z),
                                                     glm::radians(rotation_degrees_.y),
                                                     glm::radians(rotation_degrees_.x));
```
`set_rotation(vec3)` sets `use_quat_ = false`, so Euler authoring and every existing scene behave exactly as today. `glm::mat4_cast` comes from the already-present include at `:16`.

Two existing hazards to fix while here: `mark_dirty()` (`:189-194`) recurses over children with **no cycle guard and no depth cap** — a cycle from a mis-authored scene is an infinite recursion, and each physics write is O(subtree). Add a visited guard, or at minimum document the invariant.

Guard the whole addition with a `test.cpp` case in libcoopa asserting `set_rotation(vec3)` still produces the identical matrix it does today.

## Phase 3 — Broadphase

- **`broadphase/aabb_tree.h`** — dynamic AABB tree (the Box2D `b2DynamicTree` design): SAH insertion, rotation-based rebalancing, **fat AABBs** enlarged by `aabb_margin` so a body only re-inserts when it escapes its fat bounds. API: `int32_t create_proxy(const AABB&, void* user_data)`, `bool move_proxy(int32_t, const AABB&, const glm::vec3& displacement)`, `destroy_proxy`, `template<typename Fn> void query(const AABB&, Fn&&)`, `template<typename Fn> void raycast(const Ray&, Fn&&)`.
- **Two trees — this is where the kinematic/static efficiency win lands.** A static tree (colliders with no Rigidbody; built once, refit only on explicit change) and a dynamic tree. Each step queries dynamic-vs-dynamic (self) and dynamic-vs-static (cross). **Static-static, static-kinematic and kinematic-kinematic pairs are never generated at all** — no broadphase test, no narrowphase, no solver constraint. Sleeping dynamic bodies are also skipped as query *initiators* while still being queryable as targets.
- **`broadphase/layer_matrix.h`** — `class LayerMatrix`: 32 layers, `uint32_t mask_[32]`, `bool should_collide(uint32_t a, uint32_t b) const`, `set_layer_collision(a, b, bool)`. Tested in the pair filter before any geometry work.
- **`broadphase/pair_cache.h`** — persistent pair set keyed by ordered `(proxy_a, proxy_b)`; emits added/removed pair deltas so manifolds (and trigger enter/exit) can persist across steps.

## Phase 4 — Narrowphase and contact manifolds

- **`collision/shape.h`** — `enum class ShapeType { Sphere, Box, Capsule, ConvexHull, TriangleMesh }` and a variant-style `Shape` carrying the geometry plus a `glm::vec3 local_center` offset (Unity's collider `center`).
- **Dispatch table by shape pair**, cheapest analytic path first:

  | Pair | Method |
  |---|---|
  | Sphere–Sphere | analytic (center distance) |
  | Sphere–Box | closest point on OBB |
  | Sphere–Capsule | point vs segment |
  | Capsule–Capsule | segment–segment closest points |
  | Box–Box | **SAT**: 6 face axes + 9 edge-cross axes, then reference/incident face selection and Sutherland–Hodgman clipping → up to 4 contact points |
  | Capsule–Box | GJK + EPA, then a face-region check to promote a single point to a 2-point manifold when the capsule lies flat |
  | Convex–Triangle | SAT per triangle (mesh path, Phase 5) |

- **`collision/gjk.h` / `epa.h`** — GJK boolean + closest-distance with a proper simplex-evolution routine and degenerate-simplex handling; EPA polytope expansion for penetration depth and normal. These are the two most numerically delicate files in the project; they get the most unit tests (Phase 4 tests below) and are validated against analytic sphere–sphere where the answer is known exactly.
- **`collision/manifold.h`** — `struct ContactPoint { glm::vec3 position; float penetration; float normal_impulse = 0; float tangent_impulse[2] = {0,0}; uint32_t feature_id; }` and `struct ContactManifold { BodyId a, b; glm::vec3 normal; ContactPoint points[4]; uint8_t count; float friction, restitution; }`. **Warm starting depends on `feature_id` matching across steps** — points are matched by feature id, and only matched points inherit last step's impulses. Without this, stacks jitter and never settle.

## Phase 5 — Static mesh colliders

- **`geometry/mesh_bvh.h`** — SAH-built static BVH over triangles, built once at load. `query(const AABB&, Fn&&)` and `raycast(const Ray&, Fn&&)`.
- **Mesh data source: re-parse the mesh YAML in physxcoopa.** `MeshRenderer::mesh_path()` (`mesh_renderer.h:87`) retains the logical key and `SceneLoader` resolves `scene_dir + "/meshes/" + key + ".yaml"` (`scene_loader.h:283`), so a `MeshCollider` rebuilds the same path and reads `vertices:` + `faces:` via fkYAML. This needs **zero gfxcoopa changes**, and gets the *original un-triangulated, un-duplicated* vertex list — better BVH input than the GPU data would have been (`mesh.h:211-212` emits one unique vertex per face corner, so a cube becomes 36 vertices).
- **Convex-vs-triangle with internal edge correction.** The classic ghost-collision bug: a box sliding across a flat mesh floor catches on the shared edge between coplanar triangles because the per-triangle contact normal points along the edge rather than the surface. Fix by precomputing per-edge adjacency and clamping each contact normal into the triangle's valid normal cone. This must be in from the start — retrofitting it means redoing the manifold generation.
- **Non-convex mesh colliders are static-only**, matching Unity. A `MeshCollider` on a non-kinematic Rigidbody throws at `start()` with a clear message rather than silently misbehaving. `convex = true` builds a hull instead (QuickHull, capped at 255 faces) and is legal on dynamic bodies.

## Phase 6 — Bodies, materials, inertia

- **`dynamics/physics_material.h`** — Unity's `PhysicMaterial`:
  ```cpp
  enum class CombineMode { Average, Minimum, Maximum, Multiply };
  struct PhysicsMaterial {
      float dynamic_friction = 0.6f;
      float static_friction  = 0.6f;
      float restitution      = 0.0f;
      CombineMode friction_combine    = CombineMode::Average;
      CombineMode restitution_combine = CombineMode::Average;
  };
  inline float combine(float a, float b, CombineMode mode);
  ```
  Unity's asymmetric rule applies: when the two materials disagree on combine mode, the higher-priority mode wins (Multiply > Maximum > Minimum > Average). Materials are shared by pointer, loadable from a YAML asset, with a default used when none is assigned.
- **`dynamics/inertia.h`** — analytic inverse inertia tensors for box, sphere and capsule (capsule = cylinder + two hemispheres, parallel-axis-shifted). `inverse_inertia_world = R * I_local⁻¹ * Rᵀ` recomputed each substep from the current orientation.
- **`dynamics/body.h`** — `enum class BodyType { Static, Kinematic, Dynamic }`, plus position, `glm::quat orientation`, linear/angular velocity, force/torque accumulators, `inv_mass`, `inv_inertia_local`, drag, `use_gravity`, a `constraints` bitmask (freeze position/rotation per axis, applied by zeroing the corresponding inverse-mass terms), sleep state and timer, and `prev_position`/`prev_orientation` for render interpolation. Bodies live in a flat `std::vector<Body>` referenced by a generational `BodyId { uint32_t index, generation; }` — the same handle shape as `coopa::job::JobHandle`, so a stale id can never address a recycled body.
- **Kinematic velocity derivation:** a kinematic body's transform is authored externally (script or `AnimationComponent`, which writes positions at `animation_component.h:129`). Each step its velocity is derived as `(new_pos - old_pos) / h` so it imparts *correct* impulses to dynamic bodies instead of acting like an infinitely-heavy teleporter.

## Phase 7 — Solver

`dynamics/solver.h` — sequential impulses, Erin Catto style:

1. **Islands** (`dynamics/island.h`): union-find over the contact graph; static bodies never merge islands. Each island solves independently and **sleeps as a unit** — otherwise the bottom box of a stack sleeps while the top is still settling and the stack sags.
2. **Warm start**: apply last step's `normal_impulse`/`tangent_impulse` for every feature-matched contact point before iterating. This is the single biggest quality difference between a stack that settles and one that jitters forever.
3. **Velocity iterations** (8): per contact point, solve the normal constraint with a Baumgarte bias `max(0, penetration - linear_slop) * baumgarte / h` clamped to `max_linear_correction`, accumulating a clamped impulse `jn >= 0`. Then two tangent directions from `orthonormal_basis(normal)` with the Coulomb clamp `|jt| <= friction * jn` using the **accumulated** normal impulse.
4. **Restitution** applied only when the approach velocity exceeds `restitution_threshold` (1 m/s), captured *before* the solve. Below that, restitution is zero — this is what stops resting contacts from micro-bouncing forever.
5. **Position iterations** (3): pseudo-velocity / non-linear Gauss-Seidel pass to drain remaining penetration without injecting kinetic energy.
6. **Integration**: semi-implicit Euler for position; quaternion integration `q += 0.5 * ω * q * h` followed by renormalization.
7. **Sleeping**: a body accumulates sleep time while both `|v| < sleep_linear` and `|ω| < sleep_angular`; at `sleep_time` (0.5 s) the whole island sleeps and is skipped entirely. Woken by a new contact from an awake body, an applied force/impulse, or a transform teleport.

## Phase 8 — Fixed timestep, interpolation, and the `PhysicsWorld` facade

`world.h` — the one object apps construct:

```cpp
explicit PhysicsWorld(const PhysicsConfig& config = {});
void step(float delta_time);          /**< Accumulates and runs 0..max_substeps fixed substeps. */
void build_from_scene(coopa::scene::Scene& scene);   /**< Walks the tree once, caches a flat body list. */
BodyId add_body(const BodyDesc&);  void remove_body(BodyId);
void set_gravity(const glm::vec3& g);                /**< Default (0, 0, -9.81) — Z-up. */
LayerMatrix& layers();
float interpolation_alpha() const;
```

- **Accumulator**: `accumulator_ += glm::min(delta_time, k_max_frame_time)` with `k_max_frame_time = 0.25f`, then `while (accumulator_ >= h && n++ < max_substeps)`. The clamp is mandatory, not defensive — blendy's first `dt` includes all Vulkan and scene initialization (`time.h:46-51` measures from `Time` construction).
- **Interpolation**: transforms are written as `lerp(prev, current, alpha)` / `slerp` for orientation, so a 60 Hz simulation renders smoothly at any framerate. Per-body `Interpolation { None, Interpolate, Extrapolate }` matching Unity.
- **Transform write-back is a single serial pass** at the end of `step()`, using the Phase 2 `set_world_position`/`set_world_rotation`. Cache a flat `std::vector<RigidbodyComponent*>` at `build_from_scene` — `SceneObject::get_component<T>()` is a `dynamic_cast` linear scan (`scene_object.h:97-105`) and `Scene`'s collectors re-traverse the whole tree per call (`scene.h:131-194`), so neither may be used per step.

## Phase 9 — Components and YAML

`components/` — Unity-equivalent, all deriving `coopa::scene::Component`:

- **`Collider`** (base): `PhysicsMaterial* material`, `bool is_trigger`, `uint32_t layer`, `glm::vec3 center` (local offset), plus six `coopa::event::Signal`s — `on_collision_enter/stay/exit`, `on_trigger_enter/stay/exit`.
- **`BoxCollider`**: `glm::vec3 size` (full extents, Unity semantics; half-extents internally).
- **`CapsuleCollider`**: `float radius, height; int direction` — **default `2` (Z) given the Z-up world**, not Unity's Y default.
- **`SphereCollider`**: `float radius`.
- **`MeshCollider`**: `std::string mesh_path; bool convex = false;`.
- **`RigidbodyComponent`**: `mass`, `drag`, `angular_drag`, `use_gravity`, `is_kinematic`, `interpolation`, `constraints`, `add_force/add_torque/add_force_at_position`, `velocity()`, `angular_velocity()`, `move_position/move_rotation`.

**Scale handling:** boxes scale exactly per-axis from the world matrix; spheres and capsules take the **max** scale component (Unity's rule) since a non-uniformly scaled sphere is no longer a sphere. Documented explicitly — silent non-uniform-scale wrongness is a classic support burden.

**`physx_yaml.h`** — `inline void register_physics_components()`, modelled on `uicoopa/ui_yaml.h:128-140`, registering **both spellings for every component** (`"!BoxCollider"` *and* `"BoxCollider"`) because shipped scenes use `type:` keys and unrecognized tags are dropped without warning at `scene_loader.h:527-535`. Plus a `PhysicsResources` singleton (mirroring `UIResources`) holding the `PhysicsWorld*` and the scene directory, since `ComponentParser` receives only `(node, SceneObject&)` — no world, no device, no scene. Parsers therefore only *record* configuration; body/shape creation and all cross-object resolution happen in `Component::start()`, which `Scene::start()` invokes once the whole tree exists (`scene_loader.h:141`).

## Phase 10 — Scene queries

`query/queries.h` — the API gameplay actually calls:

```cpp
struct RaycastHit { glm::vec3 point, normal; float distance; ColliderId collider; coopa::scene::SceneObject* object; };
bool raycast(const glm::vec3& origin, const glm::vec3& dir, float max_distance, RaycastHit& hit, uint32_t layer_mask = ~0u);
std::vector<RaycastHit> raycast_all(...);
bool sphere_cast(const glm::vec3& origin, float radius, const glm::vec3& dir, float max_distance, RaycastHit& hit, ...);
std::vector<ColliderId> overlap_sphere(const glm::vec3& center, float radius, uint32_t layer_mask = ~0u);
std::vector<ColliderId> overlap_box(const OBB& box, uint32_t layer_mask = ~0u);
```

Ray traversal walks both AABB trees front-to-back with early-out on the current best `t`, then does exact ray-vs-shape (analytic for sphere/box/capsule, BVH descent for meshes). This incidentally gives blendy the **mouse picking it currently lacks**, and the BVH is the natural future home for the frustum culling `PbrRenderPipeline` also lacks (it pushes every renderable untested at `pbr_render_pipeline.h:882-899`).

## Phase 11 — Triggers, collision events, and CCD

- **Trigger colliders** generate overlap records but no solver constraint. Enter/stay/exit are computed by diffing this step's overlap set against last step's, using the same persistent pair keys as `pair_cache.h`.
- **Events are queued during the substep loop and emitted once at the end of `step()`**, on the main thread. `coopa::event::Signal` is documented as not thread-safe and is re-entrancy-safe but not concurrency-safe; more practically, a callback that spawns or destroys bodies mid-solve would invalidate the very arrays being iterated. Queue-then-emit makes destruction inside a callback safe.
- **CCD via speculative contacts** (not in the original list, but tunneling is the failure mode users report first): contacts are generated against an inflated margin, so an approaching fast body gets a constraint *before* it overlaps and the solver stops it at the surface. Cheap and it falls out of the existing manifold path. Bodies flagged `use_ccd` additionally get a conservative linear sweep for the bullet-through-paper case.

## Phase 12 — Debug visualizer

`debug/debug_draw.h` — pure data, **no Vulkan**: `struct DebugLine { glm::vec3 a, b; uint32_t color; }` and `void PhysicsWorld::debug_draw(DebugDraw& out, DebugDrawFlags flags)` emitting collider wireframes, BVH nodes, contact points and normals, with color by state (white awake, green sleeping, red contact normal, yellow trigger).

blendy renders it with a `VK_PRIMITIVE_TOPOLOGY_LINE_LIST` pipeline — `gfxcoopa/pipeline/pipeline.h:37-38` already exposes `topology`, `polygon_mode` and `line_width` (plumbed at `:232`, `:256`, `:260`), so this is a small pass plus two trivial shaders. Note the constraint at `pbr_render_pipeline.h:400-402`: gfxcoopa's `RenderPass` always clears color, so the debug pass must draw inside the already-open swapchain pass via the existing `enable_ui`/`set_ui_draw_list` seam (`:407`, `:427`) rather than opening a second pass.

## Phase 13 — blendy integration and docs

1. `blendy/CMakeLists.txt`: add `set(PHYSXCOOPA_DIR "${ROOT_DIR_PARENT}/physxcoopa")` + `include_directories(${PHYSXCOOPA_DIR})` alongside the existing sibling block at `:48-70`.
2. `blendy/test.cpp`: `register_physics_components()` **before** `scene_mgr.load_scene(...)` at `:204`; `physics.build_from_scene(...)` after; and one line in the loop at `:213-249`:
   ```cpp
   time.update();
   float dt = time.delta_time();
   physics.step(dt);          // fixed accumulator inside; writes interpolated transforms
   scene_mgr.update(dt);      // AnimationComponent writes transforms too — see below
   rp.render(renderer, scene_mgr.get_active_scene());
   ```
   Ordering matches Unity's FixedUpdate-before-Update. **`AnimationComponent::update()` writes `set_position` every frame** (`animation_component.h:129`), so any animated object that also has a collider must be marked kinematic or the two will fight for the transform — call this out in the README.
3. `blendy/python/blendy/bpy/scene.py` (`:146-164` emits `{"type": "Transform", ...}` dicts) and `models.py`: export Blender rigid body settings as `type: BoxCollider` / `type: Rigidbody` components. Optional, last.
4. Per-module `README.md` (mermaid `graph TD` + `### Components` + usage example) and a root README leading with the Z-up gravity default and the kinematic/animation interaction. Then `coopadocs build` warning-free.

---

## Testing and Validation

### Headless suite — `test.cpp`

Hand-rolled macros copied from `/home/coopa/git/uicoopa/test.cpp` (`RUN_TEST`, `ASSERT_TRUE`, `ASSERT_NEAR`), plus `ASSERT_VEC3_NEAR`. Physics is unusually easy to get *plausibly* wrong, so the suite is built around **closed-form answers**, not eyeballing:

| Area | Assertion |
|---|---|
| Geometry | Ray–AABB against hand-computed `t`; `closest_points_segment_segment` on the parallel-segment degenerate case; capsule inertia matches the analytic cylinder+hemispheres value |
| GJK/EPA | Against analytic sphere–sphere where penetration depth and normal are exact, over 10k randomized configurations; assert EPA converges within iteration cap for every one |
| SAT box–box | Known overlaps with hand-derived normal and penetration; face-face contact yields exactly 4 points, edge-edge exactly 1 |
| BVH / broadphase | Tree query returns the **identical pair set** as brute-force O(n²) over 1000 randomized AABBs; proxy move/refit preserves that equivalence |
| Layers | `set_layer_collision(a, b, false)` ⇒ pair never reaches narrowphase (assert via a counter) |
| Free fall | After 1 s at `h=1/60`, `z ≈ -0.5·9.81` within semi-implicit Euler's known error bound |
| Resting contact | A box dropped on a plane converges to `penetration < linear_slop` and `|v| < sleep_linear`, then **sleeps within 1 s** |
| Restitution | `restitution = 1.0` returns to ≥95% of drop height; `0.0` never bounces (max post-impact upward velocity < threshold) |
| Friction | Box on a 30° slope: `μ = 0.8` (> tan30 = 0.577) stays static after 5 s; `μ = 0.3` slides with the analytically expected acceleration |
| Stacking | 10-box tower stable for 1000 steps — no box drifts > 1 cm laterally, all asleep by the end |
| Tunneling | Sphere at 200 m/s into a 0.05 m wall does **not** pass through with CCD enabled |
| Mesh collider | Box slides across a two-triangle floor with **no velocity discontinuity** at the shared edge (internal edge correction regression test) |
| Queries | Raycast at a unit box from a known origin returns exact `t`, point and face normal; `raycast_all` returns hits sorted by distance |
| Triggers | Enter fires exactly once, stay every step inside, exit exactly once on leaving |
| Determinism | Same scene, 600 steps, run twice ⇒ **bit-identical** body states |
| Kinematic | Kinematic platform moving at 2 m/s imparts the correct impulse to a resting dynamic box; static–static and kinematic–kinematic pairs generate **zero** narrowphase calls |
| Perf | 500 dynamic boxes step in < 16 ms single-threaded (reported, not asserted, to avoid a flaky CI gate) |

```bash
cbuild && ctest --test-dir build --output-on-failure     # or ./build/physxcoopa
```

### Integration validation in blendy

Per the project's build convention, use `cbuild --vulkan` and `MAX_FRAMES=n cplay` — **never kill the process**, since blendy saves on exit.

1. `cbuild --vulkan` in `/home/coopa/git/blendy`, then `MAX_FRAMES=300 cplay` on the cube scene with a `Rigidbody` + `BoxCollider` added to `cube.000` and a `BoxCollider` on `plane.000`: the cube must fall along **−Z**, land on the plane, settle without jitter, and sleep.
2. Enable the debug visualizer and confirm collider wireframes align exactly with the rendered meshes — this is the fastest way to catch a scale or center-offset error.
3. Frame-rate independence: compare final resting positions at `MAX_FRAMES=300` under an artificial 30 Hz vs 144 Hz cap; they must agree to within a millimetre.
4. Confirm an existing `sphere_orbit` animated object with a collider marked kinematic pushes a dynamic box correctly and is not itself displaced.
5. Regression: run libcoopa's own `test.cpp` after the Phase 2 `Transform` edit, and re-render the cube scene to confirm `output/output-runtime.png` is unchanged for non-physics scenes.
6. `coopadocs build` reports no warnings; `coopadocs show` renders every module.
