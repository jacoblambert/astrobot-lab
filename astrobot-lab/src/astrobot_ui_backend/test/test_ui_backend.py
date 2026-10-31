"""Console backend contracts that need no running graph (grid conversion, GT overlay frame)."""
import json
import math
from collections import deque
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from nav_msgs.msg import OccupancyGrid  # noqa: E402

from astrobot_ui_backend.ui_backend import (  # noqa: E402
    STACKS, TRAIL_SPACING_M, UiBridgeNode, load_gt_rocks, occupancy_grid_to_dict, trail_append, windowed_rate)


def grid(width, height, data, frame="map", yaw=0.0):
    msg = OccupancyGrid()
    msg.header.frame_id = frame
    msg.info.width, msg.info.height, msg.info.resolution = width, height, 0.1
    msg.info.origin.orientation.z, msg.info.origin.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    msg.data = data
    return msg


def test_grid_keeps_row_major_cells_and_downsamples_by_block_max():
    out = occupancy_grid_to_dict(grid(4, 2, [0, 1, 2, 3, 4, 5, 6, 7]))
    assert (out["width"], out["height"], out["downsample"], out["data"]) == (4, 2, 1, [0, 1, 2, 3, 4, 5, 6, 7])
    small = occupancy_grid_to_dict(grid(4, 2, [0, 1, 2, 3, 4, 5, 6, 7]), max_cells=2)
    assert (small["width"], small["height"], small["resolution"], small["data"]) == (2, 1, pytest.approx(0.2), [5, 7])


def test_downsampling_keeps_a_lone_occupied_cell_off_the_stride_lattice():
    cells = [0] * 9
    cells[4] = 100  # the centre of a 3 x 3 grid; a stride of 3 samples only the corner
    small = occupancy_grid_to_dict(grid(3, 3, cells), max_cells=1)
    assert small["data"] == [100]
    partial = occupancy_grid_to_dict(grid(3, 3, [-1] * 8 + [0]), max_cells=1)  # padding stays unknown
    assert partial["data"] == [0]


def test_full_v2_mosaic_is_not_decimated():
    out = occupancy_grid_to_dict(grid(600, 600, [0] * 360_000))
    assert (out["width"], out["downsample"]) == (600, 1)


def test_gt_grid_published_at_yaw_pi_is_turned_upright():
    out = occupancy_grid_to_dict(grid(2, 2, [1, 2, 3, 4], frame="gt_map", yaw=math.pi), gt_to_map_transform=(20.0, 20.0, 0.0))
    assert out["frame"] == "map" and abs(out["origin"]["yaw"]) < 1e-9 and out["data"] == [4, 3, 2, 1]


def test_flipped_gt_grid_keeps_each_cell_at_its_world_position():
    msg = grid(3, 2, [100, 0, 0, 0, 0, 0], frame="gt_map", yaw=math.pi)
    msg.info.origin.position.x, msg.info.origin.position.y = 10.0, 5.0
    out = occupancy_grid_to_dict(msg, gt_to_map_transform=(0.0, 0.0, 0.0))
    # source cell (0, 0) spans x in [9.9, 10.0], y in [4.9, 5.0] (the grid extends along -x, -y)
    col = int((9.95 - out["origin"]["x"]) / out["resolution"])
    row = int((4.95 - out["origin"]["y"]) / out["resolution"])
    assert out["data"][row * out["width"] + col] == 100


def test_rates_are_windowed_and_drop_to_zero_when_a_topic_stops():
    stamps = deque([0.0, 0.5, 1.0, 1.5])  # a fast start must not linger in the rate
    stamps.extend(1.5 + 0.1 * i for i in range(1, 101))  # 10 Hz for the last 10 s
    assert windowed_rate(stamps, now=11.5) == pytest.approx(10.0, rel=0.02)
    assert windowed_rate(stamps, now=30.0) == 0.0


def test_trails_keep_one_point_per_spacing_and_track_the_newest_pose():
    trail = []
    for i in range(50):  # 0.01 m steps -> 0.5 m of travel
        trail_append(trail, {"x": 0.01 * i, "y": 0.0, "yaw": 0.0, "stamp": float(i)})
    assert trail[-1]["stamp"] == 49.0
    gaps = [b["x"] - a["x"] for a, b in zip(trail[:-2], trail[1:-1])]
    assert all(g >= TRAIL_SPACING_M - 1e-9 for g in gaps) and len(trail) <= 7


def test_gt_rocks_come_from_the_scene_inventory_in_the_map_frame(tmp_path):
    scene = tmp_path / "natural_scene.json"
    scene.write_text(json.dumps({"objects": [{"path": "r", "minimum": [1.0, 2.0, -0.4], "maximum": [1.5, 2.8, 0.3]}]}))
    assert load_gt_rocks(str(scene), (20.0, 20.0, 0.0)) == [{"x0": 21.0, "y0": 22.0, "x1": 21.5, "y1": 22.8}]
    with pytest.raises(ValueError):
        load_gt_rocks(str(scene), (20.0, 20.0, 0.1))


def test_v2_gt_overlay_uses_the_rtk_home_frame():
    node = SimpleNamespace(_gt_to_map_transform=tuple(STACKS["v2"]["gt_to_map"]))
    pose = UiBridgeNode._gt_in_map(node, {"x": 20.0, "y": 18.0, "yaw": 0.5, "stamp": 3.0})
    assert pose == {"x": 40.0, "y": 38.0, "yaw": 0.5, "stamp": 3.0}
    assert UiBridgeNode._gt_in_map(SimpleNamespace(_gt_to_map_transform=None), pose) is None
