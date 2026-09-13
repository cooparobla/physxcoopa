# physxcoopa/cloth (`coopa::physx::cloth`)

Realtime cloth: small-substep XPBD sheets that drape over rigid colliders and follow rigid bodies
they are pinned to. One-way coupled — a cloth is pushed by the rigid world, never the reverse
(the same choice Unity's `Cloth` makes, and what keeps a 1 g sheet from destabilising a body it
hangs on).

## Where it runs

```text
PhysicsWorld::step_fixed(h)
  1. integrate forces -> velocities
  2. broadphase -> narrowphase -> manifolds
  3. on_substep
  4. dynamics::solve()            -- rigid contacts + joints
  5. integrate velocities -> rigid positions/orientations
  6. step_cloths_(h)              <-- HERE, against the poses step 5 just produced
```

Last, so every cloth collides against this substep's *final* rigid poses rather than poses one
substep stale — which is what stops a sheet visibly lagging a body it is draped over. Nothing
downstream reads the cloth result, because coupling is one-way.

## One cloth substep

```text
for s in 0..cloth_substeps (4):          hs = h / cloth_substeps
  1. aerodynamics    per-triangle drag + lift -> per-particle acceleration
  2. integrate       v += a*hs; v *= 1/(1+damping*hs); prev = p; p += v*hs
  3. anchors         pinned particles written from their body's pose (lerped across substeps)
  4. constraints     stretch then bend, batch by batch, cloth_iterations (1) times
  5. tethers         one-sided clamp on distance to the nearest anchor
  6. self-collision  optional spatial-hash particle separation
  7. collisions      project out of every candidate rigid shape (incl. its swept volume)
  8. finalize        v = (p - prev)/hs, then Coulomb friction at recorded contacts
```

Rigid collision runs **last of the position stages**. Constraints, tethers and self-collision can
all push a particle back into a body, so whichever runs last is the one that holds; visible
interpenetration costs far more than a fraction of a millimetre of constraint violation. Self-
collision is the sharp case -- it can displace a particle by up to half of `self_distance`, which
would otherwise wipe out the entire `thickness` standoff with nothing left to re-project it.

Colliders are projected out of the volume they **sweep** before the cloth is next solved, not just
the pose they start at. The sweep length is `max(0, h - frame_dt)`, i.e. the collider motion the
renderer will show *without* a fresh solve -- zero at 60 Hz (every solve is drawn exactly once) and
zero below 60 Hz (several substeps per frame), non-zero only above it. Getting that to vanish in the
common case is the point: a sweep applied when no un-solved frame will be drawn holds the sheet off
a moving body for nothing, which reads as a gap and is just as wrong as clipping. A swept sphere is
modelled exactly (it is a capsule); a swept box or capsule is probed at its start and end pose with
the deeper correction winning; meshes are never swept, being static-only here.

Velocities are re-derived from positions at stage 8 rather than tracked through the projections.
That is the defining property of PBD — every correction automatically becomes a velocity change,
with no per-stage impulse bookkeeping that could disagree with the positions.

## Why XPBD and not the impulse solver

1. **Stiffness.** Cloth wants to be near-inextensible. A velocity-level spring at that stiffness is
   the stiff-ODE case semi-implicit Euler handles worst. XPBD is unconditionally stable at any
   stiffness, including infinite (`stretch_compliance = 0`, the default).
2. **Resolution independence.** Compliance is a material parameter (m/N), so a 15×15 and a 45×45
   sheet authored identically hang identically. A raw spring constant does not survive a
   resolution change.
3. Unity's `Cloth` is NvCloth, which is PBD — so this is also the "mimic Unity" answer.

## Parallelism and determinism

`make_grid_cloth()` emits constraints in **`ConstraintBatch`es whose particle sets are pairwise
disjoint** (structural by index parity, shear by column parity, bend modulo 4). A batch can
therefore be solved in any order, on any thread, with a bit-identical result — the disjointness,
not just thread safety, is the requirement, because `PhysicsWorld::world_state_hash()` must stay
reproducible. `solve_cloth()` is templated on its work dispatcher so a `JobEngine`-backed one can
be dropped in.

It currently runs serially (`SerialDispatch`), because at realistic sheet sizes that measures
faster: a 25×25 sheet costs ~0.12 ms per fixed substep serially, against which the parallel path
would pay job-dispatch overhead 30+ times per substep. Measured, single-threaded, `-O2`:

| Sheet | Particles | per 1/60 s step | with self-collision |
|---|---|---|---|
| 15×15 | 225 | 0.04 ms | — |
| 25×25 | 625 | 0.12 ms | 0.52 ms |
| 41×41 | 1681 | 0.35 ms | 1.77 ms |

Self-collision is ~4× the cost of everything else, which is why it is opt-in.

## Files

| File | Purpose |
|---|---|
| [`cloth.h`](cloth.h) | `ClothId`, `ClothParticle`, `ClothConstraint` (lambda accumulator on the struct, as `HingeJoint` does), `ClothTether`, `ClothAnchor`, `ClothParams` (Unity-shaped tunables), `ConstraintBatch`, `Cloth`; `compute_bounds()`, `max_particle_speed()`. |
| [`cloth_builder.h`](cloth_builder.h) | `GridClothDesc` + `make_grid_cloth()` (particles, structural/shear/bend constraints, disjoint batches, triangle list); `pin_to_body()`, `pin_static()`, `build_tethers()` (Dijkstra over the stretch graph — *geodesic* distance to the nearest anchor, not Euclidean). |
| [`cloth_collision.h`](cloth_collision.h) | `ClothCollider` pose snapshot + `project_particle()` — sphere analytic, box via `geometry::closest_point_on_obb`, capsule via `closest_point_on_segment`, mesh via `MeshBVH` + `collision::closest_point_on_triangle`. Deliberately *not* through `narrowphase.h`: `ContactManifold` caps at 4 points, and a sheet on a sphere makes hundreds. |
| [`cloth_solver.h`](cloth_solver.h) | `ClothSolverScratch` (allocate-once workspace), `SerialDispatch`, `solve_cloth()`. Allocates nothing per step. |

## Authoring

`components/cloth.h` (`ClothComponent`) + the `"Cloth"` parser in `physx_yaml.h`; anchors name a
sibling object and are resolved by `PhysicsSystem::regather_cloths_()`, never by the parser (the
named object may not exist yet while parsing). Global substep counts come from
`PhysicsConfig::cloth_substeps` / `cloth_iterations`, settable from a scene's `physics.cloth:` block.

```yaml
- type: Cloth
  resolution: { x: 25, y: 25 }
  size: { x: 4.0, y: 4.0 }
  mass: 1.0
  bend_compliance: 0.00005
  thickness: 0.03
  friction: 0.5
  self_collision: true
  anchors:
    - object: ball
      point: { x: 0.0, y: 0.0, z: 1.0 }   # the target's LOCAL frame
      radius: 0.4
```

## Deferred

Two-way coupling (Unity's cloth is one-way by design), cloth-vs-cloth collision, tearing, a
`DebugDrawFlags::Cloth` bit, and importing an arbitrary (non-grid) mesh as a cloth. The solver
itself is topology-agnostic — only `make_grid_cloth()` and the self-collision 1-ring test assume a
grid — so an imported-mesh path is mostly a builder and a batch colouring pass.
