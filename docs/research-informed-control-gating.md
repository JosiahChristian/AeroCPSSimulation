# Research-informed control-command gating

AeroCPSSimulation exposes an optional `ControlCommandGate` between control-command proposal and simulated state mutation.

The vertical-flight controller still computes the same deterministic thrust proposal. When a gate is configured, that proposal is presented to an independent gate before the integrator receives it. `Accept` preserves the original path. `Veto` leaves vehicle state unchanged for that time slice, records an explicit veto, and does not silently substitute or reselect another command.

This is a bounded software-architecture transfer from completed Adaptive-Model-Gating work: candidate generation and operational commitment are separated so an independent validation layer can abstain from commitment. The simulation does **not** inherit statistical guarantees, thresholds, robustness claims, or experimental outcome criteria from the research repository.

The still-open Experiment 065 cross-architecture consensus mechanism is intentionally not implemented here. Any future transfer of a specific research mechanism should wait for complete evidence closure and should be introduced through a separately reviewed change.

The gate is optional; constructing `SimulatorEngine` without one preserves the existing vertical-flight behavior and public call pattern.
