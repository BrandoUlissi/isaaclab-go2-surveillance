"""Build the mapping room scene from a declarative YAML layout.

The room is made entirely of primitives (boxes for the floor, walls, obstacles
and raised platforms) plus one flat dome light — no ceiling, no heavy shadows —
so it is light on VRAM. All boxes are static colliders.

Workflow:
    # 1. open the scene in the Isaac Sim GUI; edits to the YAML update it live
    python scripts/build_room.py --live

    # 2. once you like the layout, bake it to a USD file (headless)
    python scripts/build_room.py --export isaac_scenes/room.usd

    # 3. afterwards, view the baked scene any time
    python scripts/view_scene.py --usd isaac_scenes/room.usd

The layout lives in configs/room_layout.yaml (see that file for the format).
"""

# ── 1. Pre-launch ─────────────────────────────────────────────────────────────

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Build the mapping room scene from YAML.")
parser.add_argument("--layout", type=str, default="configs/room_layout.yaml",
                    help="Path to the room layout YAML.")
parser.add_argument("--live", action="store_true",
                    help="Hot-reload the viewport whenever the YAML changes (GUI mode is always live).")
parser.add_argument("--export", type=str, default=None,
                    help="Headless: build and export the scene to this USD path, then exit.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# GUI when interacting; headless only when baking to USD.
args_cli.headless = args_cli.export is not None

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── 2. Post-launch ────────────────────────────────────────────────────────────

import math  # noqa: E402

import yaml  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Sdf  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

_ROOM_ROOT = "/World/Room"
_FLOOR_THICKNESS = 0.1


def _yaw_quat(yaw_deg: float) -> tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) for a yaw about +z."""
    h = math.radians(yaw_deg) * 0.5
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _spawn_box(path, size, pos, color, quat=None) -> None:
    """Spawn a static (non-rigid) collider box with a flat colour."""
    cfg = sim_utils.CuboidCfg(
        size=tuple(size),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(color)),
    )
    cfg.func(path, cfg, translation=tuple(pos), orientation=quat)


def load_layout(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_room(stage, layout: dict) -> None:
    """(Re)build the room prims under /World/Room from a layout dict."""
    if stage.GetPrimAtPath(_ROOM_ROOT).IsValid():
        stage.RemovePrim(Sdf.Path(_ROOM_ROOT))

    # Floor: a thin box, top face at z = 0.
    floor = layout["floor"]
    _spawn_box(
        f"{_ROOM_ROOT}/floor",
        (floor["size_x"], floor["size_y"], _FLOOR_THICKNESS),
        (0.0, 0.0, -_FLOOR_THICKNESS / 2.0),
        floor.get("color", [0.3, 0.3, 0.33]),
    )

    # Walls.
    h = layout.get("wall_height", 1.5)
    t = layout.get("wall_thickness", 0.15)
    wall_color = layout.get("wall_color", [0.7, 0.7, 0.72])
    for i, (x, y, length, yaw) in enumerate(layout.get("walls", [])):
        _spawn_box(f"{_ROOM_ROOT}/wall_{i}", (length, t, h), (x, y, h / 2.0), wall_color, _yaw_quat(yaw))

    # Obstacles.
    obs_color = layout.get("obstacle_color", [0.8, 0.45, 0.2])
    for i, (x, y, sx, sy, sz) in enumerate(layout.get("obstacles", [])):
        _spawn_box(f"{_ROOM_ROOT}/obstacle_{i}", (sx, sy, sz), (x, y, sz / 2.0), obs_color)

    # Raised platforms.
    plat_color = layout.get("platform_color", [0.25, 0.55, 0.75])
    for i, (x, y, sx, sy, ph) in enumerate(layout.get("platforms", [])):
        _spawn_box(f"{_ROOM_ROOT}/platform_{i}", (sx, sy, ph), (x, y, ph / 2.0), plat_color)

    n = len(layout.get("walls", [])) + len(layout.get("obstacles", [])) + len(layout.get("platforms", []))
    print(f"[INFO] Room built: {n} boxes (+ floor).", flush=True)


def main() -> None:
    sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 60.0, device=args_cli.device))
    stage = omni.usd.get_context().get_stage()

    layout = load_layout(args_cli.layout)

    # One flat dome light (kept outside /World/Room so reloads don't touch it).
    intensity = float(layout.get("light", {}).get("intensity", 1000.0))
    light_cfg = sim_utils.DomeLightCfg(intensity=intensity, color=(0.9, 0.9, 0.9))
    light_cfg.func("/World/Light", light_cfg)

    build_room(stage, layout)

    # -- Headless bake mode ---------------------------------------------------
    if args_cli.export is not None:
        out = os.path.abspath(args_cli.export)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        stage.Export(out)
        print(f"[INFO] Scene exported to: {out}", flush=True)
        return

    # -- Interactive GUI mode (always hot-reloads on YAML change) --------------
    d = max(layout["floor"]["size_x"], layout["floor"]["size_y"])
    sim.set_camera_view(eye=[0.9 * d, 0.9 * d, 0.8 * d], target=[0.0, 0.0, 0.0])
    print(f"[INFO] Editing {args_cli.layout} and saving updates the view live. "
          f"Close the window to quit.", flush=True)

    last_mtime = os.path.getmtime(args_cli.layout)
    while simulation_app.is_running():
        simulation_app.update()
        try:
            mtime = os.path.getmtime(args_cli.layout)
            if mtime != last_mtime:
                last_mtime = mtime
                build_room(stage, load_layout(args_cli.layout))
        except Exception as exc:  # malformed save mid-edit: keep the old scene
            print(f"[WARN] Could not reload layout: {exc}", flush=True)


if __name__ == "__main__":
    main()
    # NOTE: simulation_app.close() can hang during Isaac Sim teardown (known
    # quirk); force-killing after the window closes / export is safe.
    simulation_app.close()
