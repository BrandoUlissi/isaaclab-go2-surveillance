"""Create and export the empty test scene: flat ground + one Unitree Go2.

Week 1 validation milestone. Launches Isaac Sim headless, authors a minimal
scene (ground plane, dome light, one Go2 articulation), steps the simulation a
few times to confirm it loads and renders, then exports the stage to a USD
file.

No camera, no policy, no ROS2 — this only confirms the scene is loadable.

Usage (from project root, isaaclab_env activated):

    /home/msi/miniforge3/envs/isaaclab_env/bin/python \\
        scripts/create_empty_scene.py \\
        --output isaac_scenes/empty_test_scene.usd
"""

# ── 1. Pre-launch ─────────────────────────────────────────────────────────────

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Create the empty Go2 test scene.")
parser.add_argument(
    "--output",
    type=str,
    default="isaac_scenes/empty_test_scene.usd",
    help="Path to write the exported USD scene.",
)
parser.add_argument(
    "--steps",
    type=int,
    default=10,
    help="Number of sim steps to run before exporting (render sanity check).",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# This script is a headless validation step; force it regardless of CLI.
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── 2. Post-launch imports ────────────────────────────────────────────────────

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # noqa: E402


def main() -> None:
    # Simulation context.
    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 200.0, device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[2.5, 2.5, 1.5], target=[0.0, 0.0, 0.3])

    # Ground plane.
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)

    # Dome light (so the exported scene is not black).
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.9, 0.9, 0.9))
    light_cfg.func("/World/Light", light_cfg)

    # Go2 robot (reuse the stock Isaac Lab config).
    go2_cfg = UNITREE_GO2_CFG.replace(prim_path="/World/Go2")
    robot = Articulation(go2_cfg)

    # Initialize physics + assets.
    sim.reset()
    # flush=True: when the script is piped, block-buffered stdout can be lost if
    # the process is force-killed during the known Isaac Sim shutdown hang below.
    print(
        f"[INFO] Go2 articulation loaded: {robot.num_joints} joints, "
        f"{robot.num_bodies} bodies.",
        flush=True,
    )

    # Step a few times to confirm the scene simulates and renders headless.
    for _ in range(args_cli.steps):
        sim.step()
        robot.update(sim_cfg.dt)

    # Export the authored stage to USD.
    import omni.usd  # noqa: E402

    out_path = os.path.abspath(args_cli.output)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    stage = omni.usd.get_context().get_stage()
    stage.Export(out_path)
    print(f"[INFO] Scene exported to: {out_path}", flush=True)


if __name__ == "__main__":
    main()
    # NOTE: simulation_app.close() is known to hang on this machine during Isaac
    # Sim teardown. The USD is already on disk by this point, so a force-kill
    # after this line does not affect the artifact.
    simulation_app.close()
