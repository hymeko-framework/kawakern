# kawakern

Physics over HyMeKo kinematic models, without IR: articulated-body dynamics, contact and its
replayable solvers, the stepping engine with rigid attachments, an energy audit and hydraulic
muscles, force fields and vehicles.

## What belongs here

- `articulated`: the articulated-body algorithm (`AbaPlan`), inverse dynamics, mass solves, kinetic
  and potential energy by link.
- `contact`: ground and collider contacts, constraints, and `contact::replay` with the active-set,
  ADMM and NSGS solvers and their diagnostics.
- `engine`: `Engine` stepping a mechanism with free-flight integrators, `RigidAttachment`, the
  `EnergyAudit` with constraint work, and `engine::muscle` (Song–Geyer activation, McKibben
  hydraulic muscles).
- `force_field`, `vehicle`.

## What does not

- The kinematic model, forward/inverse kinematics and collision geometry: `hymeko_kinematics` in
  the geometry repository, which this crate builds on.
- Reading HyMeKo IR: `hymeko_geometry` in the transformation repository. It re-exports this crate
  under its former paths with Core declaration handles (`hymeko_geometry::engine::Engine` is
  `kawakern::Engine<IrHandles>`).
- Executing a published IR instance and drawing it: Kawamirai.
- Numerical integration and PDE solving as general services: Nagare.

## Handles

The engine is generic over the model's handle family, `Engine<H: Handles>`, because it owns its
`KinematicModel<H>`; everything else takes `&KinematicModel<H>`. Physics never reads the handles.
Hand-built models use `hymeko_kinematics::Indexed`.

## Checks

```
cargo test --offline -j 2
```

## Provenance

Moved out of the transformation repository's `hymeko_geometry::{articulated, contact, engine,
force_field, vehicle}` at 232f720e1824bf4c2822cce74906118743a008a9, unchanged apart from the
handle parameter and `FkPlan` topology read through its accessors; their history is there.
