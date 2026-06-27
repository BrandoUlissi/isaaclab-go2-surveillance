# isaaclab-go2-surveillance

> **Status: Week 1 — in progress.** Initial repository setup and environment validation.

## Project goal

An autonomous surveillance robot demo in NVIDIA Isaac Sim, built around a
Unitree Go2 quadruped equipped with an RGBD camera. In **Phase 1** the robot
autonomously explores a static room and builds a 3D map (NVBlox TSDF/ESDF)
using a frontier-based exploration strategy. In **Phase 2**, with the static
map frozen, it patrols the room while detecting and tracking moving humans
(YOLO person detection + 2D→3D projection + tracker) and avoiding them in real
time via the Nav2 local planner. The low-level locomotion is handled by a
push-recovery RL policy carried over from the predecessor project.

## Stack

| Layer | Component |
|-------|-----------|
| Simulation | Isaac Sim 4.5.0, Isaac Lab 2.1.1 |
| Locomotion | RL push-recovery policy (rsl_rl 2.3.3), velocity-command interface |
| Mapping | Isaac ROS NVBlox (TSDF/ESDF) |
| Exploration | Frontier-based (m-explore) |
| Navigation | Nav2 (Humble) |
| Perception | YOLOv8n / YOLOv11n (nano), 2D→3D projection, tracker |
| Middleware | ROS2 Humble |
| ML runtime | PyTorch 2.7.0 + CUDA 12.8, Python 3.10 |

## Hardware

Single thin gaming laptop, all-in-one (no distributed runtime):

- NVIDIA RTX 4050 Laptop GPU, **6 GB VRAM**, 30 W TGP
- Intel Core i7-12650H, 16 GB RAM + 16 GB swap
- Ubuntu 22.04.5 LTS, kernel 6.8 (HWE), driver 550.163.01

The tight VRAM budget drives several scope decisions (nano-only YOLO,
640×480 cameras, single GPU). See `CLAUDE.md` for details.

## Predecessor project

This is the follow-up to
[**isaaclab-go2-locomotion**](https://github.com/BrandoUlissi/isaaclab-go2-locomotion),
which produced the trained Go2 locomotion policies (baseline and
push-recovery). The push-recovery policy is reused here as the low-level
controller: it takes velocity commands `[v_x, v_y, omega_z]` and outputs joint
target positions, while the navigation stack produces the velocity commands.

## Repository layout

```
ros2_ws/        ROS2 packages (perception, mapping, navigation)
isaac_scenes/   USD scene files (room, robot, humans)
scripts/        Python wrappers to launch Isaac Sim with a scene
configs/        YAML configs (Nav2, NVBlox, etc.)
docs/           Run notes, architecture, project plan
notebooks/      Optional analysis
logs/           Runtime logs and videos (gitignored)
```

## Documentation

- [`docs/PROJECT_PLAN.md`](docs/PROJECT_PLAN.md) — 10-week milestone plan.
- `CLAUDE.md` — full project context and conventions.

## License

[MIT](LICENSE) © 2026 Brando Ulissi
