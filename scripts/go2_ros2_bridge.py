"""ROS2 bridge + teleop of the Go2 using the push-recovery locomotion policy.

Week 4 milestone: bring up the Isaac Sim <-> ROS2 topic bridge. Same sim +
policy setup as ``teleop_keyboard.py`` (Week 2), but instead of reading the
keyboard, an in-process ``rclpy`` node exchanges the robot's state with the ROS2
graph:

  subscribes  /cmd_vel   (geometry_msgs/Twist)   -> cmd_term.vel_command_b
  publishes   /odom      (nav_msgs/Odometry)      <- Go2 root state
  publishes   /clock     (rosgraph_msgs/Clock)    <- sim time (use_sim_time)
  broadcasts  TF         odom -> base_link        <- Go2 root pose

The ``/cmd_vel`` -> ``vel_command_b`` write is the exact same seam the keyboard
teleop used (``teleop_keyboard.py``); here a ROS2 subscriber fills it instead of
key presses. Everything lives in one Python process so the node is easy to read,
debug and extend (cameras are added on this same node in Week 5).

The bridge only *executes* velocity commands; drive it from any ``/cmd_vel``
publisher. The standard keyboard teleop works out of the box:

    # terminal A (isaaclab_env, ROS2 sourced): the sim + bridge
    /home/msi/miniforge3/envs/isaaclab_env/bin/python scripts/go2_ros2_bridge.py

    # terminal B (ROS2 sourced): publish /cmd_vel
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Requires the predecessor repo checked out at ``~/isaaclab-go2-locomotion``
(its ``src`` package registers the push-recovery gym task), and ROS2 Humble
sourced before launching (``source /opt/ros/humble/setup.bash``).
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

parser = argparse.ArgumentParser(description="ROS2 bridge for the Go2 push-recovery policy.")
parser.add_argument("--checkpoint", type=str, default=_DEFAULT_CKPT, help="Path to policy .pt checkpoint.")
parser.add_argument("--odom_frame", type=str, default="odom", help="Odometry / TF parent frame id.")
parser.add_argument("--base_frame", type=str, default="base_link", help="Robot body / TF child frame id.")
# --headless (GUI on by default) is added by AppLauncher.add_app_launcher_args below.
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── 2. Post-launch imports ────────────────────────────────────────────────────

import torch  # noqa: E402
import gymnasium as gym  # noqa: E402

from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

import isaaclab_tasks  # noqa: F401, E402  (registers base tasks)
from isaaclab_tasks.utils import load_cfg_from_registry, parse_env_cfg  # noqa: E402

# Register the push-recovery task from the predecessor project.
sys.path.insert(0, _PRED_SRC)
import isaaclab_go2_pushrecovery  # noqa: F401, E402

# ROS2.
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from geometry_msgs.msg import Twist, TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402


def _stamp(t: float):
    """builtin_interfaces/Time from a float number of seconds."""
    sec = int(t)
    nanosec = int(round((t - sec) * 1e9))
    if nanosec >= 1_000_000_000:  # guard rounding into the next second
        sec += 1
        nanosec -= 1_000_000_000
    from builtin_interfaces.msg import Time
    return Time(sec=sec, nanosec=nanosec)


class Go2Bridge(Node):
    """In-process ROS2 node: /cmd_vel in, /odom + /clock + TF out.

    The node holds no threads of its own. ``poll_cmd()`` and ``publish_state()``
    are called by the sim loop so ROS traffic stays in lockstep with physics.
    """

    def __init__(self, odom_frame: str, base_frame: str):
        super().__init__("go2_ros2_bridge")
        self.odom_frame = odom_frame
        self.base_frame = base_frame

        # Latest velocity command from ROS, as [v_x, v_y, omega_z] (body frame).
        self.cmd = torch.zeros(3, dtype=torch.float32)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_bcast = TransformBroadcaster(self)

    def _on_cmd_vel(self, msg: Twist) -> None:
        # SE(2) slice of the Twist: forward, lateral, yaw-rate.
        self.cmd[0] = msg.linear.x
        self.cmd[1] = msg.linear.y
        self.cmd[2] = msg.angular.z

    def poll_cmd(self) -> None:
        """Process any pending /cmd_vel callbacks (non-blocking)."""
        rclpy.spin_once(self, timeout_sec=0.0)

    def publish_state(self, sim_time: float, pos, quat_wxyz, lin_vel_b, ang_vel_b) -> None:
        """Publish /clock, /odom and the odom->base_link TF for this step.

        ``pos``/``quat_wxyz`` are the world pose (world == odom: the robot spawns
        at the origin); ``lin_vel_b``/``ang_vel_b`` are body-frame velocities,
        matching the Odometry twist convention (child frame).
        """
        stamp = _stamp(sim_time)

        # /clock so downstream nodes can run on sim time (use_sim_time:=true).
        clk = Clock()
        clk.clock = stamp
        self.clock_pub.publish(clk)

        # ROS quaternion order is (x, y, z, w); Isaac gives (w, x, y, z).
        qw, qx, qy, qz = (float(v) for v in quat_wxyz)
        px, py, pz = (float(v) for v in pos)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(lin_vel_b[0])
        odom.twist.twist.linear.y = float(lin_vel_b[1])
        odom.twist.twist.linear.z = float(lin_vel_b[2])
        odom.twist.twist.angular.x = float(ang_vel_b[0])
        odom.twist.twist.angular.y = float(ang_vel_b[1])
        odom.twist.twist.angular.z = float(ang_vel_b[2])
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = px
        tf.transform.translation.y = py
        tf.transform.translation.z = pz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_bcast.sendTransform(tf)


def main() -> None:
    if not os.path.isfile(args_cli.checkpoint):
        raise FileNotFoundError(f"Checkpoint not found: {args_cli.checkpoint}")

    # Environment config: a single robot, driven by external velocity commands.
    env_cfg = parse_env_cfg(_TASK, device=args_cli.device, num_envs=1)

    # Make the velocity command fully manual and non-random (same as Week 2 teleop):
    #  - heading_command=False  -> omega_z is taken directly (not from a heading target)
    #  - rel_standing_envs=0    -> the env is never forced to a zero "standing" command
    #  - huge resampling time   -> the sampler never overwrites our injected command
    cmd = env_cfg.commands.base_velocity
    cmd.heading_command = False
    cmd.ranges.heading = None
    cmd.rel_standing_envs = 0.0
    cmd.resampling_time_range = (1.0e9, 1.0e9)
    cmd.debug_vis = True  # draw the command arrow in the viewport

    # Build env + load policy (same path as the Week 2 teleop script).
    agent_cfg = load_cfg_from_registry(_TASK, "rsl_rl_cfg_entry_point")
    env = gym.make(_TASK, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    print(f"[INFO] Loading checkpoint: {args_cli.checkpoint}", flush=True)
    runner.load(args_cli.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # ROS2 node (in-process; spun once per sim step).
    rclpy.init()
    bridge = Go2Bridge(args_cli.odom_frame, args_cli.base_frame)
    print(
        "[INFO] ROS2 bridge up. Subscribing /cmd_vel; publishing /odom, /clock, "
        "TF odom->base_link.\n"
        "       Drive it, e.g.:  ros2 run teleop_twist_keyboard teleop_twist_keyboard",
        flush=True,
    )

    # Handles into the sim.
    cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
    robot = env.unwrapped.scene["robot"]
    device = env.unwrapped.device
    step_dt = env.unwrapped.step_dt  # sim seconds advanced by one env.step()
    sim_time = 0.0

    obs, _ = env.get_observations()
    try:
        while simulation_app.is_running():
            # 1. Pull the latest /cmd_vel and inject it so the policy sees it.
            bridge.poll_cmd()
            cmd_term.vel_command_b[:] = bridge.cmd.to(device)

            # 2. Step the policy + physics.
            with torch.inference_mode():
                actions = policy(obs)
            obs, _, _, _ = env.step(actions)
            sim_time += step_dt

            # 3. Publish the resulting robot state to ROS.
            data = robot.data
            bridge.publish_state(
                sim_time,
                data.root_pos_w[0],
                data.root_quat_w[0],      # (w, x, y, z)
                data.root_lin_vel_b[0],   # body frame -> Odometry twist
                data.root_ang_vel_b[0],
            )
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        env.close()


if __name__ == "__main__":
    main()
    # NOTE: simulation_app.close() can hang during Isaac Sim teardown (known
    # quirk on this machine); force-killing after the window closes is safe.
    simulation_app.close()
