"""System guide contracts: host Python + pytest; no ROS, GPU, archive or motion.

Checks document integrity and critical source-backed distinctions, not runtime
acceptance. Browser layout/interaction smoke checks are separate.
"""

from html.parser import HTMLParser
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit

import pytest


ROOT = Path(__file__).resolve().parents[2]
PAGE = ROOT / "docs/implementation/system-overview.html"


class Guide(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.ids = []
        self.links = []
        self.assets = []
        self.label_targets = []
        self.aria_targets = []
        self.svgs = []
        self.buttons = []
        self.commands = []
        self._in_pre = False
        self._pre = []
        self.feed(PAGE.read_text())
        self.close()

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if "id" in attrs:
            self.ids.append(attrs["id"])
        if tag == "a":
            self.links.append(attrs["href"])
        if "src" in attrs or tag == "link":
            self.assets.append(attrs.get("src", attrs.get("href", "")))
        if tag == "label":
            self.label_targets.append(attrs["for"])
        for key in ("aria-labelledby", "aria-controls"):
            self.aria_targets.extend(attrs.get(key, "").split())
        if tag == "svg":
            self.svgs.append(attrs)
        if tag == "button":
            self.buttons.append(attrs)
        if tag == "pre":
            self._in_pre = True
            self._pre = []

    def handle_endtag(self, tag):
        if tag == "pre":
            self.commands.append("".join(self._pre))
            self._in_pre = False

    def handle_data(self, data):
        if self._in_pre:
            self._pre.append(data)


def test_unique_anchors_and_local_source_links():
    guide = Guide()
    assert len(guide.ids) == len(set(guide.ids))
    assert len(guide.links) > 80
    for link in guide.links:
        parsed = urlsplit(link)
        assert not parsed.scheme and not parsed.netloc, link
        if parsed.path:
            target = (PAGE.parent / unquote(parsed.path)).resolve()
            assert target.is_relative_to(ROOT), link
            assert target.is_file(), link
        else:
            assert parsed.fragment in guide.ids, link


def test_self_contained_no_network_or_motion_side_effects():
    guide = Guide()
    assert not guide.assets
    page = PAGE.read_text()
    assert not re.search(r"url\(\s*['\"]?(?:https?:)?//", page)
    assert "<iframe" not in page
    script = page.split("<script>", 1)[1].split("</script>", 1)[0]
    for forbidden in ("fetch(", "WebSocket(", "XMLHttpRequest", "ROSLIB", "eval("):
        assert forbidden not in script
    assert 'data-guide-ready' not in page  # Readiness is set by executed JS, not static markup.
    assert "dataset.guideReady = 'true'" in script


def test_accessible_controls_and_named_schematics():
    guide = Guide()
    assert guide.label_targets and guide.buttons
    assert all(target in guide.ids for target in guide.label_targets + guide.aria_targets)
    assert len(guide.svgs) == 3
    for svg in guide.svgs:
        assert svg["role"] == "img"
        assert len(svg["aria-labelledby"].split()) == 2
    assert all(button.get("type") == "button" and button.get("aria-label") for button in guide.buttons)
    page = PAGE.read_text()
    assert "prefers-reduced-motion" in page and "@media(max-width:760px)" in page
    assert 'aria-live="polite"' in page


@pytest.mark.parametrize("section", [
    "status", "architecture", "topics", "frames", "timing", "terrain",
    "motion", "evidence", "code", "testing", "failures", "next",
])
def test_major_sections_are_navigable(section):
    guide = Guide()
    assert section in guide.ids
    assert f"#{section}" in guide.links


def test_live_topic_contracts_match_wrapper():
    page = PAGE.read_text()
    source = (ROOT / "tools/live_terrain_shadow.py").read_text()
    for topic in (
        "/astrobot_0/slam_raw/revisions", "/terrain_shadow/native_chunks",
        "/terrain_shadow/native_complete", "/terrain_shadow/costmap", "/terrain_shadow/status",
    ):
        assert topic in page and topic in source
    assert "navigation_authority:false" in page
    assert "navigation_authority=False" in source
    assert "/astrobot_0/slam/bev_costmap" in page
    nav = (ROOT / "astrobot-lab/src/astrobot_launch/config/nav2_phase3_bev.yaml").read_text()
    assert "/astrobot_0/slam/bev_costmap" in nav
    assert "/terrain_shadow/costmap" not in nav


def test_reason_codes_follow_core_enum():
    header = (ROOT / "astrobot-lab/src/slam_mapping/include/slam_mapping/layered_terrain.hpp").read_text()
    enum = header.split("enum TerrainReason", 1)[1].split("};", 1)[0]
    reasons = re.findall(r"([A-Z_]+)\s*=\s*(\d+)", enum)
    assert len(reasons) == 13
    page = PAGE.read_text()
    for name, value in reasons:
        assert f"<td>{value}</td><td>{name}</td>" in page


def test_frame_causality_and_gt_limits_are_explicit():
    page = PAGE.read_text()
    for required in (
        "T_map_lidar(t) = T_map_imu(t) · T_imu_lidar",
        "T_map_base(t)  = T_map_imu(t) · inverse(T_base_imu)",
        "[-20, -20, 0]", "not an assertion of a published TF tree",
        "First publication only", "200 ms", "ODOM revisions still occur",
        "One-time estimator initialization", "Synthetic RTK aiding",
        "Terrain uses native chunks", "Full scans are currently un-deskewed",
        "Never terrain-mapper classification input",
    ):
        assert required in page


def test_latest_failed_run_provenance_and_limits_match_ledger():
    page = PAGE.read_text()
    ledger = (ROOT / "docs/implementation/implementation-plan.md").read_text()
    for run in (
        "revision-379884211d744a1790860f9969775530",
        "revision-38b96ba8739f4cfeb84da7d1200f9ec1",
        "revision-f0db6c401ce845b7800297622753d1d6",
        "revision-76d17e74292e428dac1ac36daa1a5549",
    ):
        assert run in page and run in ledger
    for required in (
        "16 m loop is incomplete", "acceptance:false", "Known shutdown gap",
        "106.25", "109.78", "6.32", "5.58", "frozen_post_turn_connection_v2",
        "MOVES ROBOT IN SIMULATION", "does not satisfy these",
        "September 14 snapshot", "does not replace the retained frozen library",
    ):
        assert required in page


def test_shell_examples_parse_without_execution():
    # Parse only actual command snippets (not transform equations or provenance).
    commands = [snippet for snippet in Guide().commands if any(
        marker in snippet for marker in ("python3 ", "colcon ", "ros2 ", "docker ", "nvidia-smi")
    )]
    assert len(commands) >= 10
    for command in commands:
        result = subprocess.run(["bash", "-n"], input=command, text=True, capture_output=True, timeout=5)
        assert result.returncode == 0, result.stderr


def test_reproduction_flags_exist_and_remain_diagnostic():
    page = PAGE.read_text()
    replay = (ROOT / "tools/replay_live_terrain_shadow.py").read_text()
    capture = (ROOT / "tools/run_shadow_capture.py").read_text()
    assert "--diagnose-heartbeat-prefix" in page and "--diagnose-heartbeat-prefix" in replay
    for flag in ("--require-physics-imu", "--natural-motion", "--terrain-library", "--wall-seconds"):
        assert flag in page and flag in capture
    assert "No Nav2, teleop, other command publisher" in page
    assert "No ROS, GPU, network or archive required to read this page" in page
