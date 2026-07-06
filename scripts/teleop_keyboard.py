"""Keyboard teleop of the Go2 using the push-recovery locomotion policy.

Week 2 milestone: bring up the low-level locomotion controller. Loads the
push-recovery RL policy trained in the predecessor project
(``isaaclab-go2-locomotion``) and drives a single Go2 in Isaac Sim from manual
SE(2) velocity commands ``[v_x, v_y, omega_z]`` typed on the keyboard.

The velocity command is the interface the high-level navigation stack will use
later (Week 3+ feeds it from ROS2 ``cmd_vel`` instead of the keyboard). Here we
override the environment's velocity-command buffer each control step with the
keyboard input, so the policy sees exactly the command we send.

Requires the predecessor repo checked out at ``~/isaaclab-go2-locomotion``
(its ``src`` package registers the push-recovery gym task).

Usage (from project root, isaaclab_env activated):

    /home/msi/miniforge3/envs/isaaclab_env/bin/python \\
        scripts/teleop_keyboard.py

Keyboard (focus the Isaac Sim window):
    Up / Down     : forward / backward   (+/- v_x)
    Left / Right  : strafe left / right  (+/- v_y)
    Z / X         : yaw left / right     (+/- omega_z)
    L             : reset command to zero
Close the window (or Ctrl+C) to quit.
"""

# ── 1. Pre-launch ─────────────────────────────────────────────────────────────

import argparse
import os
import sys

from isaaclab.app import AppLauncher

# Default checkpoint: the push-recovery policy from the predecessor project.
_DEFAULT_CKPT = os.path.expanduser(
    "~/isaaclab-go2-locomotion/logs/rsl_rl/unitree_go2_flat_pushrecovery/"
    "2026-06-02_23-49-30/model_2350.pt"
)
# Predecessor src package (registers the push-recovery gym task).
_PRED_SRC = os.path.expanduser("~/isaaclab-go2-locomotion/src")

_TASK = "Isaac-Velocity-Flat-Unitree-Go2-PushRecovery-Play-v0"

parser = argparse.ArgumentParser(description="Keyboard teleop for the Go2 push-recovery policy.")
parser.add_argument("--checkpoint", type=str, default=_DEFAULT_CKPT, help="Path to policy .pt checkpoint.")
parser.add_argument("--vx_scale", type=float, default=0.8, help="Forward/back velocity per key (m/s).")
parser.add_argument("--vy_scale", type=float, default=0.5, help="Lateral velocity per key (m/s).")
parser.add_argument("--wz_scale", type=float, default=1.0, help="Yaw rate per key (rad/s).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Teleop needs the GUI window (keyboard focus + visualization).
args_cli.headless = False

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── 2. Post-launch imports ────────────────────────────────────────────────────

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.devices import Se2Keyboard  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

import isaaclab_tasks  # noqa: F401, E402  (registers base tasks)
from isaaclab_tasks.utils import load_cfg_from_registry, parse_env_cfg  # noqa: E402

# Register the push-recovery task from the predecessor project.
sys.path.insert(0, _PRED_SRC)
import isaaclab_go2_pushrecovery  # noqa: F401, E402


def main() -> None:
    if not os.path.isfile(args_cli.checkpoint):
        raise FileNotFoundError(f"Checkpoint not found: {args_cli.checkpoint}")

    # Environment config: a single robot, driven by manual commands.
    env_cfg = parse_env_cfg(_TASK, device=args_cli.device, num_envs=1)

    # Make the velocity command fully manual and non-random:
    #  - heading_command=False  -> omega_z is taken directly (not derived from a heading target)
    #  - rel_standing_envs=0    -> the env is never forced to a zero "standing" command
    #  - huge resampling time   -> the sampler never overwrites our injected command
    cmd = env_cfg.commands.base_velocity
    cmd.heading_command = False
    cmd.ranges.heading = None  # silence "heading range set but heading command off" warning
    cmd.rel_standing_envs = 0.0
    cmd.resampling_time_range = (1.0e9, 1.0e9)
    cmd.debug_vis = True  # draw the command arrow in the viewport

    # Build env + load policy (same path as the predecessor's play script).
    agent_cfg = load_cfg_from_registry(_TASK, "rsl_rl_cfg_entry_point")
    env = gym.make(_TASK, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    print(f"[INFO] Loading checkpoint: {args_cli.checkpoint}", flush=True)
    runner.load(args_cli.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # Keyboard teleop device -> [v_x, v_y, omega_z].
    keyboard = Se2Keyboard(
        v_x_sensitivity=args_cli.vx_scale,
        v_y_sensitivity=args_cli.vy_scale,
        omega_z_sensitivity=args_cli.wz_scale,
    )
    keyboard.reset()
    print(keyboard, flush=True)

    # Handle to the velocity-command buffer we overwrite each step.
    cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
    device = env.unwrapped.device

    obs, _ = env.get_observations()
    while simulation_app.is_running():
        # Inject the current keyboard command so the policy observation matches it.
        kb_cmd = keyboard.advance()  # numpy [v_x, v_y, omega_z]
        cmd_term.vel_command_b[:] = torch.tensor(kb_cmd, dtype=torch.float32, device=device)

        with torch.inference_mode():
            actions = policy(obs)
        obs, _, _, _ = env.step(actions)

    env.close()


if __name__ == "__main__":
    main()
    # NOTE: simulation_app.close() can hang on this machine during Isaac Sim
    # teardown (known quirk); force-killing after the window closes is safe.
    simulation_app.close()
