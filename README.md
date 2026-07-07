# isaaclab-go2-surveillance

> **Status: Week 4 — ROS2 bridge.** Isaac Sim ↔ ROS2 topic bridge: the Go2 is
> driven over `/cmd_vel` while `/odom`, `/clock` and TF are published to the ROS
> graph. Week 3 (static room scene) is done. Next: Week 5 (RGBD camera).

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

## Running (milestones so far)

Activate the Python env first: `conda activate isaaclab_env`. First Isaac Sim
launch is slow (asset loading, ~1–2 min).

```bash
# Week 1 — create/validate the empty test scene (headless) and export USD
python scripts/create_empty_scene.py --output isaac_scenes/empty_test_scene.usd

# View any saved USD scene in the Isaac Sim GUI (orbit with the mouse)
python scripts/view_scene.py --usd isaac_scenes/empty_test_scene.usd

# Week 2 — drive the Go2 with the push-recovery policy (keyboard teleop, GUI)
python scripts/teleop_keyboard.py
#   W/S = forward/back (v_x)   A/D = strafe (v_y)   Q/E = yaw (omega_z)   L = stop

# Week 3 — build the static room from the YAML layout, then bake it to USD
python scripts/build_room.py --live                        # edit layout live in the GUI
python scripts/build_room.py --export isaac_scenes/room.usd # bake to USD (headless)
python scripts/view_scene.py --usd isaac_scenes/room.usd     # view the baked room
```

Week 2/4 need the predecessor repo checked out at `~/isaaclab-go2-locomotion`
(it provides the push-recovery policy and gym task).

### Week 4 — drive the Go2 over ROS2

The bridge runs Isaac Sim + the policy and hosts an in-process ROS2 node.
Source ROS2 **before** activating conda (order matters):

```bash
source /opt/ros/humble/setup.bash && conda activate isaaclab_env

# terminal A — sim + bridge (subscribes /cmd_vel; publishes /odom, /clock, TF)
python scripts/go2_ros2_bridge.py

# terminal B — publish /cmd_vel (any Twist source works)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# inspect the graph
ros2 topic hz /odom
ros2 topic echo /clock --once
```

## Documentation

- [`docs/PROJECT_PLAN.md`](docs/PROJECT_PLAN.md) — 10-week milestone plan.
- `CLAUDE.md` — full project context and conventions.

## License

[MIT](LICENSE) © 2026 Brando Ulissi
