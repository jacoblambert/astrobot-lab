import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contact_manifold import decode_contacts, summarize_contacts


def buffers():
    force = np.full((8, 1), np.nan)
    points = np.full((8, 3), np.nan)
    normals = points.copy()
    gaps = force.copy()
    force[3:5] = 2
    points[3:5] = [[1, 2, 0], [1.05, 2.1, 0]]
    normals[3:5] = [0, 0, 1]
    gaps[3:5] = 0
    return [force, points, normals, gaps, np.array([[2]]), np.array([[3]])]


def test_contact_pair_ranges_not_unused_buffer_tail():
    data = decode_contacts(buffers(), capacity=8)
    assert len(data["points_world"]) == 2
    assert data["normal_force_n"] == [2, 2]
    b = buffers()
    b[4][:] = 0
    assert decode_contacts(b, capacity=8)["points_world"] == []


@pytest.mark.parametrize("bad", ["tail", "negative", "float_count", "nonfinite", "overlap", "saturated"])
def test_bad_contact_buffers_rejected(bad):
    b = buffers()
    if bad == "tail":
        b[5][:] = 7
    if bad == "negative":
        b[4][:] = -1
    if bad == "float_count":
        b[4] = np.array([[2.0]])
    if bad == "nonfinite":
        b[1][3, 0] = np.nan
    if bad == "overlap":
        b[4], b[5] = np.array([[2], [2]]), np.array([[3], [3]])
    if bad == "saturated":
        b[4][:], b[5][:] = 8, 0
    with pytest.raises(ValueError):
        decode_contacts(b, capacity=8)


def test_report_transforms_world_contacts_without_certifying_their_hull():
    data = decode_contacts(buffers(), capacity=8)
    row = dict(floor_contacts=[data] * 5, positions=[[1, 2, 0.2]], orientations_wxyz=[[1, 0, 0, 0]])
    result = summarize_contacts([row])
    assert not result["filled_contact_area_certified"]
    for wheel in result["wheels"]:
        assert wheel["missing_contact_frames"] == 0
        np.testing.assert_allclose(wheel["minimum_base_m"], [0, 0, -0.2])
        np.testing.assert_allclose(wheel["maximum_single_frame_span_m"], [0.05, 0.1, 0])
