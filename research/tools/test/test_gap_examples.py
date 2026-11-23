import sys
import json
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from export_gap_examples import choose_examples, gap_summary
from export_gap_examples import export
from render_gap_report import grid_svg, gap_text, render


def test_pose_report_cannot_cross_runs_or_hide_failed_reference():
    from render_gap_report import pose_reference_notice
    key = "/archive/revision-test/collection.json"
    report = dict(recorder="/archive/revision-test", input_sha256={key: "same-input"})
    pose = dict(input_sha256={key: "same-input"}, native_surface_gate=dict(passed=False, failure="missing <ray>"))
    notice = pose_reference_notice(pose, report)
    assert "FAILED" in notice and "missing &lt;ray&gt;" in notice
    pose["native_surface_gate"]["passed"] = True
    assert pose_reference_notice(pose, report) == ""
    for hashes in ({}, {key: "different-input"}, {"/archive/other/collection.json": "same-input"}):
        pose["input_sha256"] = hashes
        with pytest.raises(ValueError, match="does not match"):
            pose_reference_notice(pose, report)
from replay_local_terrain import sha


def test_fixed_display_times_keep_unreached_and_last():
    chosen, missing = choose_examples([10**9, 6 * 10**9, 17 * 10**9])
    assert chosen == {10**9: ["+0s"], 6 * 10**9: ["+5s"], 17 * 10**9: ["+15s", "last projection receipt"]}
    assert missing == [30, 60, 120, 180, 240]
    with pytest.raises(ValueError):
        choose_examples([1, 1])


def test_unknown_component_dimensions_do_not_include_obstacle():
    classes = np.zeros((60, 60), dtype=np.int8)
    classes[25:35, 20:40] = -1
    classes[0, 0] = 100
    summary = gap_summary(classes, np.array([-3.0, -3.0]), np.eye(4), 0.1)
    patch = summary["base_center_unknown_component"]
    assert patch["width_m"] == 2.0 and patch["height_m"] == 1.0
    assert patch["area_m2"] == pytest.approx(2.0) and patch["cells"] == 200
    assert not patch["touches_grid_edge"]
    assert summary["occupied"] == 1 and summary["unknown"] == 200


def test_supported_center_and_boundary_truncation_are_explicit():
    classes = np.zeros((60, 60), dtype=np.int8)
    assert gap_summary(classes, np.array([-3.0, -3.0]), np.eye(4), 0.1)["base_center_unknown_component"] is None
    classes[:] = -1
    assert gap_summary(classes, np.array([-3.0, -3.0]), np.eye(4), 0.1)["base_center_unknown_component"][
        "touches_grid_edge"
    ]


def test_bad_class_is_not_silently_colored_free():
    with pytest.raises(ValueError):
        gap_summary(np.array([[42]]), np.zeros(2), np.eye(4), 0.1)


def test_svg_preserves_class_colors_scale_and_chassis():
    classes = np.zeros((60, 60), dtype=np.int8)
    classes[20:40, 20:40] = -1
    classes[0, 0], classes[0, 1] = 100, 20
    svg = ET.fromstring(grid_svg(classes, [-3, -3], np.eye(4), 0.1))
    assert svg.attrib["viewBox"] == "-1 -1 62 62"
    assert {r.attrib["fill"] for r in svg.findall("rect")} >= {"#cbd5e1", "#62af97", "#c3cf76", "#d96350"}
    assert len(svg.findall("polygon")) == 3  # chassis, guard, arrow
    zoom = ET.fromstring(grid_svg(classes, [-3, -3], np.eye(4), 0.1, zoom=True))
    assert zoom.attrib["viewBox"] == "15.00000 15.00000 30 30"
    with pytest.raises(ValueError):
        grid_svg(classes, [-3, -3], np.eye(4), 0.2)
    assert "occupied" in gap_text(dict(base_center_unknown_component=None, base_center_class=100))


@pytest.mark.parametrize("fault", [None, "geometry", "class"])
def test_failed_export_keeps_missing_last_receipt_and_actual_last_grid(tmp_path, monkeypatch, fault):
    recorder, mapper, output = [tmp_path / n for n in ("recorder", "mapper", "output")]
    for folder in (recorder, mapper, output, recorder / "bag"):
        folder.mkdir()
    (recorder / "bag/metadata.yaml").write_text("")
    record = dict(
        evidence_ns=10**9,
        publication_ns=1200000000,
        origin=[-3, -3],
        resolution=0.1,
        local_from_map=np.eye(4).tolist(),
        local_from_base=np.eye(4).tolist(),
        compute_ms=20.0,
    )
    (mapper / "projections.jsonl").write_text(
        json.dumps(record) + "\n" + json.dumps(dict(record, evidence_ns=2 * 10**9)) + "\n"
    )
    status = dict(
        archive="/archive/mapper",
        capture="capture",
        completed=False,
        failure=None,
        evidence_ns=2 * 10**9,
        peak_process_rss_mib=200,
    )
    (recorder / "collection.json").write_text(
        json.dumps(
            dict(
                collection_completed=False,
                terrain_shadow_status=status,
                failure="failed <sensor>",
                natural_motion="loop16",
            )
        )
    )
    (mapper / "collection.json").write_text(
        json.dumps(
            dict(
                experiment_completed=False,
                status=status,
                output_sha256={"projections.jsonl": sha(mapper / "projections.jsonl")},
            )
        )
    )
    (mapper / "manifest.json").write_text(json.dumps(dict(request=dict(capture="capture"))))
    ns = types.SimpleNamespace
    msg = ns(
        header=ns(stamp=ns(sec=1, nanosec=0), frame_id="map"),
        info=ns(
            width=60,
            height=60,
            resolution=0.1,
            origin=ns(position=ns(x=-3.0, y=-3.0, z=0.0), orientation=ns(x=0.0, y=0.0, z=0.0, w=1.0)),
        ),
        data=[-1] * 3600,
    )
    if fault == "geometry":
        msg.info.origin.position.x += 0.1
    if fault == "class":
        msg.data[0] = 42

    class Reader:
        def open(self, *args):
            self.messages = [("/terrain_shadow/classes", msg, 123)]

        def set_filter(self, value):
            assert value.topics == ["/terrain_shadow/classes"]

        def has_next(self):
            return bool(self.messages)

        def read_next(self):
            return self.messages.pop(0)

    monkeypatch.setitem(
        sys.modules,
        "rosbag2_py",
        ns(SequentialReader=Reader, StorageOptions=ns, ConverterOptions=lambda *args: None, StorageFilter=ns),
    )
    monkeypatch.setitem(sys.modules, "rclpy.serialization", ns(deserialize_message=lambda raw, cls: raw))
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", ns(OccupancyGrid=object))
    with pytest.raises(ValueError, match="failed source"):
        export(recorder, mapper, output)
    if fault:
        with pytest.raises(ValueError, match="geometry mismatch|invalid class"):
            export(recorder, mapper, output, allow_failed=True)
        return
    report = export(recorder, mapper, output, allow_failed=True)
    assert report["failed_prefix_only"] and not report["acceptance"]
    assert report["missing_projection_stamps"] == [2 * 10**9]
    assert report["expected_projections"] == 2 and report["recorded_projections"] == 1
    assert report["examples"][-1]["labels"] == ["+0s", "last recorded grid"]
    (output / "report.json").write_text(json.dumps(report))
    gallery = tmp_path / "gallery"
    gallery.mkdir()
    render([output], gallery)
    html = (gallery / "gap-report.html").read_text()
    assert "FAILED CAPTURE" in html and "failed &lt;sensor&gt;" in html and "1/2" in html
