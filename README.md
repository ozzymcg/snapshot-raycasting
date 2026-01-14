# Snapshot Raycasting (VEX V5)

This project provides a **raycast-based snapshot pose correction** system using **V5 Distance Sensors** and a **segmented field collision map**. It is intended for **one-shot setPose updates** (not continuous localization), and is designed to be easy to integrate into PROS projects.

## What it does
- Reads 2–3 Distance Sensors (median sampling + basic gating)
- Raycasts sensor rays into a configurable collision map (per-sensor field masks supported)
- Solves for a consistent \((x,y)\) pose estimate and applies it to odometry (setPose)

## Documentation
For full details (math, method, figures, defaults, and usage):
- https://github.com/ozzymcg/snapshot-raycasting/blob/main/main.pdf

## Quick start (integration overview)
1. Copy the `snapshot_pose/` headers into `include/` and the `.cpp` sources into `src/`.
2. Configure your sensors, offsets, heading source, and tracker distances in:
   - `src/snapshot_pose/snapshot_bindings.cpp`
3. Call the one-line API in auton when the robot is stable:
   - `snapshot_pose::snapshot_setpose_quadrant(snapshot_pose::Quadrant::TR);`

## Notes
- Best reliability comes from using **perimeter walls only** as landmarks (`MAP_PERIMETER`).
- If you enable interior objects, increase candidate hits per sensor (`candidates_per_sensor = 2`) and use quadrants where possible.
