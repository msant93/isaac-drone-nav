# Known limitations

This list is updated at every phase. Each entry states the impact and the
planned mitigation.

## NumPy simulator (Phase 2)
| Limitation | Impact | Mitigation |
|---|---|---|
| Flat, rigid ground; no bounce or tipping. Resting contact suppresses sliding and spinning. | Landing dynamics are idealised. | Landings are validated in Isaac Sim (PhysX contacts). |
| No collisions with obstacles. | Collisions must be detected geometrically (distance to obstacles). | Phase 6/11 computes collisions from the scenario geometry. In Isaac Sim, contact reports are used. |
| Linear drag only, no rotor drag/blade flapping, no ground effect, no gyroscopic rotor torques. | Slightly optimistic behaviour at high speed and near the ground. | Same simplifications as Pegasus, so the two simulators stay consistent. |
| Ideal IMU (no noise or bias). | Estimation cannot be tested yet. | Sensor noise models arrive in Phase 4. |
| Iris mass, inertia and rotor positions come from the PX4 Gazebo model. | Possible small mismatch with the Pegasus USD. | Verified against the USD in Phase 3. |
| Diagonal inertia tensor only. | None for symmetric multirotors. | The schema only accepts a diagonal. |
