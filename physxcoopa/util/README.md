# physxcoopa/util (`coopa::physx::util`)

Shared math helpers, config, error reporting, and the world-space `Transform` bridge into libcoopa.

| File | Purpose |
|---|---|
| [`math.h`](math.h) | `k_epsilon`, `safe_normalize()`, `orthonormal_basis()` (Duff et al.'s branchless tangent-frame construction, used by the solver's friction directions). |
| [`config.h`](config.h) | `PhysicsConfig` — every tunable the solver/world reads: `velocity_iterations`, `relax_iterations`, `position_iterations`, `position_correction`, `linear_slop`, sleep thresholds, `max_substeps`, `aabb_margin`. |
| [`error.h`](error.h) | `throw_error(msg)` — `std::runtime_error` prefixed `[physxcoopa]`, the house convention every other file's failure path uses. |
| [`transform_bridge.h`](transform_bridge.h) | `Trs` + `world_trs()` / `set_world_trs()` — world-space position/rotation/scale read and write for `coopa::util::Transform`, built entirely from its already-public `parent()`/`get_world_matrix()` accessors. This is what keeps `PhysicsSystem` from needing any further libcoopa change beyond the `Transform` quaternion extension. |
