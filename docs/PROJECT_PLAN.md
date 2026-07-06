# Project Plan — isaaclab-go2-surveillance

10 weeks total, target completion **early August 2026**. Milestones below are
the working plan; they will be refined as the integration proceeds (the stack
has many components that must be brought up one at a time).

> Revised after Week 2: a dedicated **static room scene** step was inserted at
> Week 3 and everything after it shifted by one week. This absorbs the former
> Week 10 buffer (there is no longer a dedicated slack week).

## Milestone table

| Week | Focus | Key deliverables |
|------|-------|------------------|
| 1 | Setup & validation | Git repo, structure, README, project plan; environment verified (GPU, Python, torch CUDA, ROS2); empty Isaac scene (ground + Go2) loads headless. |
| 2 | Locomotion bring-up | Load push-recovery policy in Isaac Sim; drive Go2 from manual velocity commands `[v_x, v_y, omega_z]`. |
| 3 | Static room scene | Author `isaac_scenes/room.usd`: the static room to be mapped (floor, walls, a few obstacles with collisions). Decide build-from-primitives vs. adapting an Isaac Sim environment asset. |
| 4 | ROS2 bridge | Isaac Sim ↔ ROS2 topic bridge (odom, cmd_vel, clock); teleop the Go2 over ROS2. |
| 5 | Sensors | Add RGBD camera (640×480); publish RGB + depth + camera_info to ROS2; verify TF tree. |
| 6 | Mapping (NVBlox) | Bring up Isaac ROS NVBlox; build TSDF/ESDF of the static room from teleop drive; visualize map. |
| 7 | Autonomous exploration | Integrate m-explore frontier exploration + Nav2 global/local planning; **Phase 1 demo**: autonomous mapping. |
| 8 | Perception | Add 1–2 animated humans to the scene as dynamic obstacles; YOLOv8n/v11n person detection on RGB; 2D→3D projection using depth; basic tracker. |
| 9 | Surveillance loop | Freeze static map; patrol behavior + reactive human avoidance via Nav2 local planner; **Phase 2 demo**. |
| 10 | Metrics & polish | Tabulate metrics (mapping coverage, detection rate, avoidance), record demo videos, finalize docs, cross-link to predecessor repo. No dedicated buffer week remains — overruns eat into this week. |

## Notes

- Phase 1 maps the **static** room only; dynamic obstacles appear in Phase 2.
  This is the standard "static map + reactive avoidance" pattern (see
  out-of-scope notes in `CLAUDE.md`).
- Iterative approach: each milestone should be runnable from a fresh checkout
  via documented commands before moving on.
