# kawakern

The HyMeKo physics: two crates.

- `kawakern` (repository root) — physics over kinematic models, without IR: articulated-body
  dynamics, contact and its replayable solvers, the stepping engine with rigid attachments, an
  energy audit and hydraulic muscles, force fields and vehicles.
- `kawakern_dynamics` — a HyMeKo IR read as a port-Hamiltonian system, its analysis and runs
  (fluid, diffusion, soft bodies, Kirchhoff laws), power views over `kawakern` mechanisms, and
  discrete scenarios. See [kawakern_dynamics/README.md](kawakern_dynamics/README.md).

## What belongs in `kawakern`

- `articulated`: the articulated-body algorithm (`AbaPlan`), inverse dynamics, mass solves, kinetic
  and potential energy by link.
- `contact`: ground and collider contacts, constraints, and `contact::replay` with the active-set,
  ADMM and NSGS solvers and their diagnostics.
- `engine`: `Engine` stepping a mechanism with free-flight integrators, `RigidAttachment`, the
  `EnergyAudit` with constraint work, and `engine::muscle` (Song–Geyer activation, McKibben
  hydraulic muscles).
- `force_field`, `vehicle`.
- `track`: closed roads for vehicle scenes — a rational centreline, its arc-length table, wrapped
  sampling, nearest-point projection and static obstacles, built from a plain `TrackSpec`.

## What does not

- The kinematic model, forward/inverse kinematics and collision geometry: `hymeko_kinematics` in
  the geometry repository, which this crate builds on.
- Reading a mechanism or a road out of HyMeKo IR: transformation's `hymeko_mechanism` (with
  `IrHandles`, its Core-declaration handle family) and `hymeko_formats::environment`. Consumers
  use this crate directly, e.g. `kawakern::Engine<IrHandles>`.
- Executing a published IR instance and drawing it: Kawamirai.
- Numerical integration and PDE solving as general services: Nagare.

## Handles

The engine is generic over the model's handle family, `Engine<H: Handles>`, because it owns its
`KinematicModel<H>`; everything else takes `&KinematicModel<H>`. Physics never reads the handles.
Hand-built models use `hymeko_kinematics::Indexed`.

## Checks

```
cargo test --locked -j 2 --workspace
```

## Provenance

Moved out of the transformation repository's `hymeko_geometry::{articulated, contact, engine,
force_field, vehicle}` at 232f720e1824bf4c2822cce74906118743a008a9, unchanged apart from the
handle parameter and `FkPlan` topology read through its accessors; their history is there.

`kawakern_dynamics` is transformation's `hymeko_dynamics` moved at
7d2232fcd26ac4b4479e946626895c808a119fed, renamed, with its power views generic over Core-declaration
handles instead of transformation's facade types.
