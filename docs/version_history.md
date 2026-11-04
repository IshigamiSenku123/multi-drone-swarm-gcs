# Version History
Tags v0.1-v0.6 are byte-exact snapshots of the private development archives
(V1-V6 folders); v1.0.0 is the validated release state.
| Tag | Milestone (archive name) | Added | Validation |
|---|---|---|---|
| v0.1 | TakeOff+Formation only-V1 | takeoff + triangular formation | SITL |
| v0.2 | ...-V2 | forward translation + local-frame spawn correction | SITL (right-triangle shape known broken here) |
| v0.3 | ...-V3 | Tkinter GUI launcher | SITL |
| v0.4 | ...-V4 | shape selection (equilateral / right / custom) | SITL T1-T4 |
| v0.6 | ...-V6 | potential-field collision avoidance (default OFF) | SITL T1-T4 contrast |
| v1.0.0 | current workspace | GCS fault detection + isolate_and_continue | F1 validated; F2-F4 pending |
The launch script (scripts/restart_swarm.sh) is provided only in its current
validated form; per-version copies were not preserved in the archives.
Only main / v1.0.0 is supported.
