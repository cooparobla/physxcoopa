# physxcoopa/collision (`coopa::physx::collision`)

Per-shape-pair contact generation. Every pair has an exact analytic or SAT solution — there is no
GJK/EPA and no convex-hull builder anywhere in physxcoopa (see the plan's "Design rationale" for
why: v1's shape set is closed, and every pair in it has a cheaper exact answer).

## Dispatch

```text
generate_contacts(shape_a, shape_b) -> ContactManifold
  Sphere-Sphere    analytic center distance
  Sphere-Box       closest point on OBB                         [narrowphase.h]
  Box-Box          SAT (6 face + 9 edge-cross axes) + Sutherland-Hodgman clipping   [sat.h, clip.h]
  Capsule-Capsule  closest points between two segments           [segment.h]
  Capsule-Box      segment clipped against the box's 6 slabs     [segment.h]
  *-TriangleMesh   per-triangle contact + internal-edge correction via adjacency   [mesh_contact.h]
```

`manifold.h`'s `normal` always points from body `a` toward body `b`; every dispatch branch in
`narrowphase.h` is responsible for getting that sign right regardless of which underlying
per-shape function it called (several of those return the opposite convention and get flipped
before returning).

| File | Purpose |
|---|---|
| [`shape.h`](shape.h) | `ShapeType`, `Shape` (world-scale geometry + material/trigger/layer), and the `world_sphere()`/`world_obb()`/`world_capsule()`/`world_bounds()` instancing helpers everything else in this module builds on. |
| [`manifold.h`](manifold.h) | `ContactPoint` (position, penetration, per-axis impulse accumulators, warm-start `feature_id`) and `ContactManifold` (up to 4 points, `normal`, friction/restitution, `is_trigger`). |
| [`contact_event.h`](contact_event.h) | `ContactEvent` — one enter/stay/exit transition, queued by `PhysicsWorld` during the substep loop and drained once per frame by the caller (`PhysicsSystem` fires the actual `Collider` signals from these). |
| [`sat.h`](sat.h) | `generate_box_box_contacts()` — full box-box SAT + face clipping, with `(reference_face, incident_face, clipped_vertex_index)` packed into a stable `feature_id`. |
| [`clip.h`](clip.h) | Sutherland-Hodgman polygon clipping against a box's face planes, `sat.h`'s clipping step. |
| [`segment.h`](segment.h) | `capsule_vs_sphere()`, `capsule_vs_capsule()`, `capsule_vs_box()` — all analytic, no iteration cap that can fail to converge. |
| [`mesh_contact.h`](mesh_contact.h) | Convex-vs-`TriangleMesh` contact generation with internal-edge correction (the classic "box sliding across a tessellated floor catches on the shared edge between coplanar triangles" bug). Sphere/Capsule use a core-plus-radius closest-point test; Box tests every corner against each triangle's plane directly, since a box has no core-radius offset for the closest-point trick to work once it's already overlapping. |
| [`narrowphase.h`](narrowphase.h) | `generate_contacts()` — the dispatch table above, plus `sphere_vs_sphere()`/`sphere_vs_box()`. |

## Usage Example

```cpp
#include <physxcoopa/collision/narrowphase.h>

coopa::physx::collision::ContactManifold m;
bool hit = coopa::physx::collision::generate_contacts(
    id_a, shape_a, pos_a, rot_a,
    id_b, shape_b, pos_b, rot_b,
    m);
if (hit) {
    // m.normal, m.points[0..m.count), m.friction, m.restitution, m.is_trigger
}
```
