# physxcoopa/broadphase (`coopa::physx::broadphase`)

Pair generation: which body pairs are even worth narrowphase's exact geometric tests.

| File | Purpose |
|---|---|
| [`aabb_tree.h`](aabb_tree.h) | `AABBTree` — dynamic AABB tree (Box2D's `b2DynamicTree` design), fat AABBs enlarged by `aabb_margin` so a proxy only needs re-insertion once it escapes its own fattened bounds. `create_proxy` / `move_proxy` / `destroy_proxy`, `query()` and `raycast()` (both template-callback, no allocation), `for_each_node()` (debug visualization). `PhysicsWorld` keeps two: a static tree (built once) and a dynamic tree (refit every step). |
| [`layer_matrix.h`](layer_matrix.h) | `LayerMatrix` — 32-layer collision matrix, `should_collide(a, b)` / `set_layer_collision(a, b, bool)`, checked before any geometry work. |
| [`pair_cache.h`](pair_cache.h) | `PairCache` — a sorted, deduplicated `std::vector<ProxyPair>` (never a pointer-keyed hash map — iteration order must not depend on ASLR for determinism). `diff()` / `classify()` compare this step's set against the last `advance()`-rolled one; this is the exact mechanism both manifold warm-start persistence and Phase 9's trigger/collision enter-stay-exit events are built on. |
