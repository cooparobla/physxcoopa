# physxcoopa/dynamics (`coopa::physx::dynamics`)

Rigid body state, integration, and the sequential-impulse solver — the actual physics.

## Solver Architecture

```text
step_fixed(h)
  1. integrate forces (gravity, drag) -> velocities        [integrator.h]
  2. broadphase -> pairs -> narrowphase -> manifolds        [collision/]
  3. build islands (union-find over the contact graph)      [island.h]
  4. fire on_substep (gameplay may add_force/set_velocity/wake here; forces land NEXT substep,
     impulses -- apply_impulse_at_position() -- reach this substep's solve)
  5. solve():                                                [solver.h]
       warm_start()                    -- reapply last step's cached impulses
       velocity_iterations (16)         -- normal + friction, target v=0, zero positional bias
       relax_iterations (1-3)           -- restitution correction, LAST, target v=-e*v_approach
       solve_position() (4 its)         -- pseudo-velocity penetration drain, no velocity injected
       rebuild warm-start cache
       update_islands_and_sleep()       -- kinematic wake rule, per-island sleep timers
  6. integrate velocities -> positions/orientations         [integrator.h]
```

Split-impulse design: velocity solve never touches position, position solve never touches
velocity — the only place penetration correction happens is step 5's `solve_position()`. Points
within one manifold solve Gauss-Seidel with a **rotating start index**
(`(k + iteration_index) % point_count`), not Jacobi and not a fixed order — see `solver.h`'s own
doc for why (a 10-box stack and a box on a 30° slope only both stabilize with the rotation; either
alone fixes one case and breaks the other).

| File | Purpose |
|---|---|
| [`physics_material.h`](physics_material.h) | `PhysicsMaterial` (Unity's `PhysicMaterial`) — friction/restitution + `CombineMode {Average, Minimum, Maximum, Multiply}`; `combine()` implements Unity's asymmetric tie-break (Multiply > Maximum > Minimum > Average). |
| [`inertia.h`](inertia.h) | Analytic inverse-inertia tensors for box/sphere/capsule (cylinder + two parallel-axis-shifted hemispheres). |
| [`body.h`](body.h) | `BodyId {index, generation}`, `BodyType {Static, Kinematic, Dynamic}`, `Body` — position/orientation/velocities, force/torque accumulators and impulses (`apply_impulse_at_position()` -- with an opt-out of waking, for continuous per-substep fields like buoyancy -- and `velocity_at_point()`), sleep state, `prev_*` (render interpolation) and `last_written_*` (teleport/kinematic-velocity detection) pairs. |
| [`integrator.h`](integrator.h) | `apply_gravity()`, `apply_drag()`, `integrate_velocities()` — semi-implicit Euler for position, first-order quaternion integration for orientation (`q += 0.5*ω*q*h`, renormalized). |
| [`island.h`](island.h) | Union-find over the contact graph; static/sleeping bodies never merge islands, so a whole island sleeps or wakes as a unit. |
| [`solver.h`](solver.h) | `solve()` — see the architecture diagram above; also `capture_restitution_bias()`, `warm_start()`, `solve_velocity_pass()`, `solve_position()`, `update_islands_and_sleep()` (including the kinematic wake rule: a sleeping body touching a kinematic body whose derived velocity goes non-zero wakes immediately). |
