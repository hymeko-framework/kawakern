# kawakern_dynamics

A compiled HyMeKo IR read as a port-Hamiltonian system: the projection (J from signed incidence,
fields, hierarchies), modal analysis, generalised Kirchhoff laws, layout, runs, free-surface fluid,
continuum diffusion, small-displacement soft bodies, power views over Kawakern mechanisms, and
source-declared discrete scenarios. Unlike the `kawakern` crate it reads IR (Core 9f533ca); the
in-memory source provider comes from transformation's `hymeko_sources` until it moves into Core.

Power views take any mechanism whose handles are Core declarations
(`H: Handles<Id = DeclId>`), e.g. transformation's `IrHandles`.

Moved unchanged from transformation's `hymeko_dynamics` at
7d2232fcd26ac4b4479e946626895c808a119fed (package renamed; consumers can keep their code with
`hymeko_dynamics = { package = "kawakern_dynamics", git = ... }`); its history is there.

```
cargo test --offline -j 2 -p kawakern_dynamics
```

## Observed inspection task

`kawakern_dynamics::scenario::inspection::InspectionSession` qualifies a declared
sequence of station poses from timestamped observations. It reuses `ScenarioSession`
for discrete progress and bounded transition history. Source/frame/epoch/order,
finite poses, position/orientation/speed thresholds and contiguous sampled dwell
are checked before completion. Source expiry blocks progress until fresh data and
an explicit `resume`; this is not a physical stop command or confirmation.

The [typed source fixture](tests/fixtures/inspection/task.hymeko)
declares three stations, four cycles, home, criteria, workflow bindings and output
leaves. Its positions are conformance data, not a verified UR trajectory. Run the
thin JSON-lines consumer from the kawakern repository root:

```powershell
cargo run --locked -p kawakern_dynamics --example inspection -- kawakern_dynamics/tests/fixtures/inspection/task.hymeko inspection --cbor inspection.cbor
```

It prints initial state, then reads one `observe`, `tick`, `resume` or `history`
operation per input line. For example, `{"operation":"tick","now_ns":0}` returns
the initial missing-observation state. `observe` has `now_ns` plus a `sample`
matching the public `PoseObservation` contract. Invalid operations return explicit
refusals; malformed/oversized input terminates the runner. EOF writes the optional
CBOR using Core's existing writer. The file is current canonical IR evidence,
not a checkpoint of the full observer memory or robot execution state.

`value_updates()` projects state into the declared output leaves for an admitted
host model. The host commits through `CompiledModel::with_value_updates` and its
existing publication transaction, checking model basis and handling failure.
Unknown/stale measurements have explicit validity flags. Sequence halves and
seconds/subsecond nanoseconds are exact numeric fields, avoiding per-sample
interned timestamp strings. Current and last-evaluated target references remain
distinct at a phase boundary.

Focused correctness checks:

```powershell
cargo test --locked -p kawakern_dynamics --test inspection --example inspection
```

ROS/HIVE publication, actual MoveIt sequencing and XR acceptance of this observer
are separate integration work; the standalone runner sends no robot commands.
