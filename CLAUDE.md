# Project Context — isaaclab-go2-surveillance

## Project goal

Build an autonomous surveillance robot demo in NVIDIA Isaac Sim, using
a Unitree Go2 quadruped equipped with an RGBD camera. The robot must:

- **Phase 1**: autonomously explore a static room and build a 3D map
  (NVBlox TSDF/ESDF) using a frontier-based exploration strategy (m-explore).
- **Phase 2**: with the static map frozen, patrol the room while detecting
  and tracking moving humans (YOLO person detection + 2D->3D projection +
  tracker), and avoiding them in real time via Nav2 local planner.

Final deliverable is a public GitHub repo `isaaclab-go2-surveillance` with:
- Code + ROS2 packages
- Two demo videos (Phase 1 mapping, Phase 2 surveillance)
- Tabulated metrics in README
- Clear reproducibility documentation
- Cross-link to the predecessor repo `isaaclab-go2-locomotion`

## Predecessor project (important context)

This is the follow-up to `isaaclab-go2-locomotion`
(https://github.com/BrandoUlissi/isaaclab-go2-locomotion).
That repo produced two trained locomotion policies for the Go2:
- baseline (model_1999.pt)
- push-recovery (model_2350.pt)

The push-recovery policy will be used in this project as the low-level
locomotion controller. It accepts velocity commands [v_x, v_y, omega_z]
and outputs joint target positions. The high-level navigation stack
(Nav2 + perception) produces velocity commands; the policy executes them.

The policies live in:
~/isaaclab-go2-locomotion/logs/rsl_rl/unitree_go2_flat_pushrecovery/2026-06-02_23-49-30/model_2350.pt

## Author technical background

Brando Ulissi. M.Sc. Automation Engineering, University of Bologna,

Background:
- Classical control: MPC, LQR, sliding-mode super-twisting, geometric
  SO(3) control
- Perception, sensor fusion
- ROS2 (Humble) — experienced, used in previous projects including
  Isaac ROS NVBlox for an agricultural robotics challenge
- Python (production), MATLAB/Simulink, OpenCV
- C++ reading proficiency, not production-level

Hands-on RL experience came from the predecessor project
(isaaclab-go2-locomotion). No prior experience integrating Isaac Sim
with ROS2-based navigation stacks before this project.

## Hardware constraints (do not forget)

Thin gaming laptop, all-in-one:
- NVIDIA RTX 4050 Laptop GPU, **6 GB VRAM**, 30 W TGP
- Intel Core i7-12650H, 16 GB RAM, 16 GB swap
- Ubuntu 22.04.5 LTS, kernel 6.8 (HWE), driver 550.163.01

Critical hardware-driven scope constraints:
- **VRAM budget is tight** for this project. Expected runtime VRAM
  during Phase 2 execution: ~3.5-6 GB across Isaac Sim + cameras +
  NVBlox + YOLO + locomotion inference + Nav2.
- For YOLO: use **YOLOv8n or YOLOv11n (nano variant)** only. Not s/m/l.
  Nano gives ~0.3-0.5 GB VRAM and is sufficient for person detection
  in this scene.
- For RGB camera: max resolution 640x480, 30 Hz logical (real-time
  expected ~15-25 FPS due to thermal/CPU limits).
- For depth camera: same resolution, used for NVBlox + obstacle avoidance.
- **One single GPU**, no distributed runtime.

## Environment

- Conda environment: `isaaclab_env` (Miniforge), Python 3.10.20.
- Isaac Sim 4.5.0, Isaac Lab 2.1.1 (installed via pip in editable mode
  at ~/IsaacLab/).
- PyTorch 2.7.0+cu128.
- rsl_rl 2.3.3.
- ROS2 Humble native at /opt/ros/humble (do not reinstall via conda;
  it conflicts).
- Isaac ROS NVBlox package (cloned previously, available in
  ~/isaac_ros_ws/src/isaac_ros_nvblox).

To activate the Python env: `conda activate isaaclab_env`.
To activate ROS2: `source /opt/ros/humble/setup.bash`.
For ROS2 + conda combined: source ROS2 first, then activate conda.

## Project structure (planned)

```
isaaclab-go2-surveillance/
├── CLAUDE.md           # this file
├── README.md
├── LICENSE             # MIT
├── ros2_ws/
│   └── src/
│       ├── go2_surveillance_perception/   # YOLO + tracker + 2D->3D
│       ├── go2_surveillance_mapping/      # NVBlox config + launch
│       └── go2_surveillance_navigation/   # Nav2 config + launch + m-explore
├── isaac_scenes/       # USD scene files: room, robot, humans
├── scripts/            # Python wrappers: launch Isaac Sim with scene
├── configs/            # YAML configs (Nav2 params, NVBlox params, etc.)
├── docs/               # RUN_NOTES_phaseN.md, ARCHITECTURE.md
│   └── images/
├── notebooks/          # Optional analysis
└── logs/               # Gitignored runtime logs and videos
```

## Project conventions

- Code, comments, documentation in English (this is a public portfolio
  repo).
- Conversations with the author can be in Italian, but written
  deliverables are English.
- Honest engineering tone in README: no overclaim, no LinkedIn-style hype.
- Iterative approach: small validated steps, no big leaps. Especially
  important here because the stack has many components (Isaac Sim,
  ROS2 bridge, NVBlox, m-explore, Nav2, YOLO, tracker) that need to
  integrate one at a time.
- When asked to implement something, prefer using existing well-supported
  libraries (NVBlox node, Nav2 stack, ultralytics YOLO) over custom
  re-implementations.
- Reproducibility is important: every milestone should be runnable from
  a fresh checkout via documented commands.

## Out of scope (do not pursue)

- UAV/drone applications (DTU NDA).
- Real-robot deployment.
- Multi-robot (single Go2 only).
- Domain randomization or sim-to-real of the navigation stack.
- Multiple scenes (one static room is the only environment).
- Vision-based RL training (laptop VRAM cannot sustain it).
- YOLO fine-tuning / retraining (pre-trained on COCO is sufficient).
- Mapping the environment while dynamic obstacles are present in
  Phase 1 (this is a known limitation of TSDF-based SLAM; the project
  uses the standard "static map + reactive avoidance" pattern instead).
- LinkedIn post writing (out of scope for the AI assistant — author
  handles communications).

## Working pattern

The author works on the "thinking and review" side via Anthropic chat
(claude.ai), and uses Claude Code (this tool) for hands-on coding,
sessions in the terminal.

When in doubt about non-trivial architectural decisions (e.g., choice
of mapping algorithm, tracker design, message topic naming, scope
deviations from the brief), Claude Code should flag back to the
author rather than make silent choices. Specifically:

- Issues estimated to require >100 lines of unscoped code: ask first.
- Decisions that affect 2+ weeks of downstream work: ask first.
- Choices that touch the public README narrative: ask first.

Routine implementation decisions (function naming, code organization
within a single file, choice between two equivalent libraries) can
be made autonomously by Claude Code.

## Timeline

10 weeks total (target completion: early August 2026), with the last
week as buffer. Per-week milestones in `docs/PROJECT_PLAN.md` (to be
created in week 1).

## Critical pitfalls (lessons from the predecessor project)

1. **CC sometimes hangs on overly long initial prompts.** Keep session
   kickoff prompts concise; spread implementation across multiple
   short atomic prompts when possible.
2. **CC tends to skip the Python venv when running subprocesses.** Use
   the full path `/home/msi/miniforge3/envs/isaaclab_env/bin/python`
   in scripts and bash scripts.
3. **Isaac Lab silently ignores `--experiment_name` CLI flag** — do not
   rely on it; use built-in directory naming and timestamps.
4. **Conda + ROS2 sourcing order matters**: source ROS2 first, then
   activate conda. Reverse order corrupts paths.
