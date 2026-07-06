"""Open a saved USD scene in the Isaac Sim GUI viewport.

Unlike create_empty_scene.py (which runs headless to validate + export), this
script launches the Isaac Sim window so you can look at and orbit around the
scene. It only renders — no physics stepping — so it is light on VRAM.

Usage (from project root, isaaclab_env activated):

    /home/msi/miniforge3/envs/isaaclab_env/bin/python \\
        scripts/view_scene.py --usd isaac_scenes/empty_test_scene.usd

Close the Isaac Sim window (or Ctrl+C in the terminal) to quit.
Mouse in the viewport: left-drag = orbit, scroll = zoom, middle-drag = pan.
"""

# ── 1. Pre-launch ─────────────────────────────────────────────────────────────

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="View a USD scene in Isaac Sim.")
parser.add_argument(
    "--usd",
    type=str,
    default="isaac_scenes/empty_test_scene.usd",
    help="Path to the USD scene to open.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# This is a viewer: force the GUI on regardless of any inherited default.
args_cli.headless = False

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── 2. Post-launch: open the stage and keep rendering ─────────────────────────

import omni.usd  # noqa: E402

usd_path = os.path.abspath(args_cli.usd)
if not os.path.isfile(usd_path):
    raise FileNotFoundError(f"USD not found: {usd_path}")

omni.usd.get_context().open_stage(usd_path)
print(f"[INFO] Opened {usd_path} — close the window to quit.", flush=True)

# Render loop: keep the viewport alive until the user closes the window.
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
