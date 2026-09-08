# physxcoopa/query (`coopa::physx::query`)

Shape-dispatch helpers behind `PhysicsWorld`'s public query API (`raycast`, `raycast_all`,
`sphere_cast`, `overlap_sphere`, `overlap_box` — see `world.h`, which owns the actual broadphase
traversal since it alone has access to the trees). Kept separate so `world.h`'s own body isn't a
wall of per-shape-type raycast/overlap code.

| File | Purpose |
|---|---|
| [`queries.h`](queries.h) | `RaycastHit` (carries a `BodyId`, not a `SceneObject*` — `PhysicsWorld` has no `coopa::scene` dependency), `raycast_shape()` (analytic Sphere/Box/Capsule, BVH descent for `TriangleMesh`), `shape_overlaps_sphere()` / `shape_overlaps_obb()`. |

**Queries are only valid between phases, not from inside `on_substep`** — a raycast issued from a
substep callback observes mid-solve, not-yet-integrated state.
