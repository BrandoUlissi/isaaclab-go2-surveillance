# Project Plan — isaaclab-go2-surveillance

10 weeks total, target completion **early August 2026**, with the final week
as buffer. Milestones below are the working plan; they will be refined as the
integration proceeds (the stack has many components that must be brought up one
at a time).

## Milestone table

| Week | Focus | Key deliverables |
|------|-------|------------------|
| 1 | Setup & validation | Git repo, structure, README, project plan; environment verified (GPU, Python, torch CUDA, ROS2); empty Isaac scene (ground + Go2) loads headless. |
| 2 | Locomotion bring-up | Load push-recovery policy in Isaac Sim; drive Go2 from manual velocity commands `[v_x, v_y, omega_z]`. |
| 3 | ROS2 bridge | Isaac Sim ↔ ROS2 topic bridge (odom, cmd_vel, clock); teleop the Go2 over ROS2. |
| 4 | Sensors | Add RGBD camera (640×480); publish RGB + depth + camera_info to ROS2; verify TF tree. |
| 5 | Mapping (NVBlox) | Bring up Isaac ROS NVBlox; build TSDF/ESDF of the static room from teleop drive; visualize map. |
| 6 | Autonomous exploration | Integrate m-explore frontier exploration + Nav2 global/local planning; **Phase 1 demo**: autonomous mapping. |
| 7 | Perception | YOLOv8n/v11n person detection on RGB; 2D→3D projection using depth; basic tracker. |
| 8 | Surveillance loop | Freeze static map; patrol behavior + reactive human avoidance via Nav2 local planner; **Phase 2 demo**. |
| 9 | Metrics & polish | Tabulate metrics (mapping coverage, detection rate, avoidance), record demo videos, finalize docs and reproducibility commands. |
| 10 | Buffer | Slack for overruns, README narrative, final cross-linking to predecessor repo. |

## Notes

- Phase 1 maps the **static** room only; dynamic obstacles appear in Phase 2.
  This is the standard "static map + reactive avoidance" pattern (see
  out-of-scope notes in `CLAUDE.md`).
- Iterative approach: each milestone should be runnable from a fresh checkout
  via documented commands before moving on.
