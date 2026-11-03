# safety_filter_controller

Chainable `ros2_control` controller for streaming a joint-position policy
output to a UR (or any position-controlled) arm through a real-time safety
layer, instead of going through a `forward_position_controller`.

## Why not chain to `forward_position_controller`?

`forward_command_controller/ForwardCommandController` (which
`forward_position_controller` is a preset of) does **not** implement
`ChainableControllerInterface` — it exposes no reference interfaces, so
nothing can be chained in front of it. Since it adds no logic of its own
(it just forwards a topic to the hardware), there is no reason to keep it
in the chain anyway. `SafetyFilterController` claims the hardware
`<joint>/position` command interfaces directly and is the terminal
controller:

```
policy → [SafetyFilterController] → hardware (UR)
```

If you later want a strict "reference generation" / "low-level control"
separation, chain into a `joint_trajectory_controller` instead — it *is*
chainable — rather than `forward_position_controller`.

## What it checks, every cycle

1. **Delta clamp**: rejects the new reference if any joint moved further
   than `max_delta_per_joint` since the last accepted command.
2. **Horizon collision check**: linearly extrapolates the commanded
   velocity `horizon_steps` steps into the future and runs a broadphase
   (Pinocchio `BroadPhaseManager` over hpp-fcl's `DynamicAABBTreeCollisionManager`)
   collision query at each step.

If either check fails, the last known-safe command is republished instead
of the new reference (the arm holds/continues its last safe trajectory
rather than jumping or stopping abruptly).

## Real-time notes

- The Pinocchio `Model`/`GeometryModel`/`BroadPhaseManager` and all Eigen
  buffers are built and sized once in `on_configure()`. `update_and_write_commands()`
  performs no dynamic allocation.
- Use collision **primitives** (capsules/cylinders) rather than full meshes
  for the arm's collision geometry if the horizon check doesn't fit your
  RT budget at your target rate — this is a URDF/SRDF concern, not a code
  change.
- `hpp-fcl`'s Nesterov-accelerated GJK can be enabled on the collision
  requests for roughly 2x on narrow-phase queries if you're still tight on
  budget; not wired up here to keep the example minimal.
- If the horizon check still doesn't fit your control period, move it to a
  separate non-RT thread running at a lower frequency, and keep only the
  delta clamp (cheap, vectorial) in `update_and_write_commands()`.

## Build requirements

- Pinocchio built with `WITH_COLLISION_SUPPORT=ON` (pulls in hpp-fcl and
  the broadphase headers).
- hpp-fcl ≥ 2.x (for the reintroduced broadphase manager).

## Interfaces

- **Reference interfaces** (exported upstream, one per joint):
  `<controller_name>/<joint>/position` — write your policy output here,
  either via a preceding chained controller or, when running un-chained,
  by publishing a `std_msgs/Float64MultiArray` on `~/commands` (joint
  order must match the `joints` parameter).
- **Command interfaces** (claimed on hardware): `<joint>/position`.
- **State interfaces** (claimed on hardware, read-only): `<joint>/position`,
  `<joint>/velocity`.

## Parameters

See [`config/safety_filter_controller.yaml`](config/safety_filter_controller.yaml).

## Known gaps / next steps

- `collision_objects` is currently just a placeholder parameter — static
  environment geometry (table, box, perception octree cells) needs to
  already be present in the URDF/geometry passed to `urdf_path`. Dynamic
  (re-)insertion into `geom_model_` at runtime isn't implemented yet.
- No parameter reconfiguration at runtime (change joints/limits without a
  controller restart) — not needed for the current use case.
