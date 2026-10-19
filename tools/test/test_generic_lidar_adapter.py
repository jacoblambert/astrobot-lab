from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from generic_lidar_adapter import NativeWindow, DTYPE, validate_wire_payload
from native_chunk_transport import acquisition_stamp_ns


def tick(adapter):
    ids = (np.arange(10661) + adapter.next_sequence * 10661) % 63968
    a, z = adapter.azimuth[ids // 32], -np.deg2rad(adapter.elevations[ids % 32])
    rays = np.column_stack([np.cos(a) * np.cos(z), np.sin(a) * np.cos(z), -np.sin(z)])
    return rays, np.ones(len(ids)), np.full(len(ids), 255, np.uint8), a, z


def test_acquisition_quantization_matches_isaac_clock_not_nearest_nanosecond():
    # Original tick 8475, revision-15f85f441dbf4a2a87ca0322d958be6c.
    stamp = 158.05000824294984
    assert round(stamp * 1e9) == 158050008243
    assert acquisition_stamp_ns(stamp) == 158050008242
    # Isaac's C++ bridge casts timeStamp*1e9 to int64, rather than splitting
    # seconds first. Include integer-second boundaries and the long-run range.
    stamps = np.r_[0.0, np.nextafter(1.0, 0.0), 1.0, np.arange(40001) * float(np.float32(1 / 60))]
    expected = (stamps * 1e9).astype(np.int64)
    np.testing.assert_array_equal([acquisition_stamp_ns(s) for s in stamps], expected)


@pytest.mark.parametrize("stamp", [np.nan, np.inf, -np.inf, -1e-9, 2**31])
def test_invalid_acquisition_times_remain_rejected(stamp):
    with pytest.raises(ValueError, match="acquisition time"):
        acquisition_stamp_ns(stamp)


@pytest.mark.parametrize("policy", ["nearest_ns_v1", "isaac_clock_ns_v2"])
def test_window_headers_and_offsets_share_declared_timestamp_policy(policy):
    adapter = NativeWindow(timestamp_policy=policy)
    stamps = 158.05000824294984 + np.arange(6) * float(np.float32(1 / 60))
    expected_xyz = []
    for i, stamp in enumerate(stamps):
        arrays = tick(adapter)
        expected_xyz.append(arrays[0])
        result = adapter.push(i, float(stamp), *arrays)
    encode_time = round if policy == "nearest_ns_v1" else int
    expected_ns = [encode_time(s * 1e9) for s in stamps]
    rows = np.frombuffer(result["data"], dtype=DTYPE)
    assert result["stamp_ns"] == expected_ns[0]
    assert result["last_tick_ns"] == result["end_ns"] == expected_ns[-1]
    np.testing.assert_array_equal(rows["time"], np.repeat((np.array(expected_ns) - expected_ns[0]) / 1e9, 10661))
    np.testing.assert_array_equal(
        np.column_stack([rows[k] for k in ("x", "y", "z")]), np.concatenate(expected_xyz).astype(np.float32)
    )
    assert result["native_stamps"] == list(stamps)
    adapter.finish()


def test_unknown_timestamp_policy_rejected():
    with pytest.raises(ValueError, match="timestamp policy"):
        NativeWindow(timestamp_policy="guess")


def test_exact_native_times_local_points_and_bounded_six_tick_window():
    adapter = NativeWindow()
    arrays = tick(adapter)
    for i in range(6):
        result = adapter.push(i, 1 + i / 60, *tick(adapter))
        if i < 5:
            assert result is None and len(adapter.pending) == i + 1
    rows = np.frombuffer(result["data"], dtype=DTYPE)
    assert len(rows) == 63966 and not adapter.pending
    np.testing.assert_array_equal(rows["x"][:10661], arrays[0][:, 0].astype(np.float32))
    np.testing.assert_array_equal(np.unique(rows["time"]), [round(i / 60 * 1e9) / 1e9 for i in range(6)])
    assert result["stamp_ns"] == 10**9 and result["end_ns"] == 1083333333
    validate_wire_payload(result)
    assert len(result["data"]) == result["points"] * 32
    assert np.frombuffer(result["data"], np.uint8).reshape(-1, 32)[:, 26:28].sum() == 0
    assert not result["moving_qualified"]
    adapter.finish()
    with pytest.raises(ValueError, match="invalidated"):
        adapter.push(6, 1.1, *tick(adapter))


@pytest.mark.parametrize("fault", ["sequence", "stamp", "angle", "xyz", "count", "nonfinite"])
def test_bad_tick_invalidates_instead_of_dropping_and_resuming(fault):
    adapter = NativeWindow()
    arrays = list(tick(adapter))
    sequence, stamp = 0, 1.0
    if fault == "sequence":
        sequence = 1
    elif fault == "stamp":
        stamp = -1.0
    elif fault == "angle":
        arrays[4][0] += 0.02
    elif fault == "xyz":
        arrays[0][0, 0] += 0.1
    elif fault == "count":
        arrays[0] = arrays[0][:-1]
    else:
        arrays[0][0, 0] = np.nan
    with pytest.raises(ValueError):
        adapter.push(sequence, stamp, *arrays)
    with pytest.raises(ValueError, match="invalidated"):
        adapter.push(0, 1.0, *tick(adapter))


def test_reset_or_gap_and_partial_final_window_are_rejected():
    with pytest.raises(ValueError):
        NativeWindow().finish()
    for stamp in (1.0, 2.0):
        adapter = NativeWindow()
        adapter.push(0, 1.0, *tick(adapter))
        with pytest.raises(ValueError):
            adapter.push(1, stamp, *tick(adapter))
    adapter = NativeWindow()
    adapter.push(0, 1.0, *tick(adapter))
    with pytest.raises(ValueError):
        adapter.finish()


def test_misses_are_not_converted_to_far_geometry():
    adapter = NativeWindow()
    for i in range(6):
        arrays = tick(adapter)
        arrays[2][:10] = 0
        result = adapter.push(i, 1 + i / 60, *arrays)
    assert result["native_samples"] == 63966 and result["points"] == 63906


def test_repeated_packet_with_fresh_header_is_not_new_evidence():
    adapter = NativeWindow()
    arrays = tick(adapter)
    adapter.push(0, 1.0, *arrays)
    with pytest.raises(ValueError, match="beam"):
        adapter.push(1, 1 + 1 / 60, *arrays)


def test_packed_structured_payload_is_rejected_before_ros_or_cpp():
    packed = np.dtype([(name, DTYPE[name]) for name in DTYPE.names])
    assert packed.itemsize == 30  # NumPy 1.26 concatenate's implicit result.
    with pytest.raises(ValueError, match="wire stride"):
        validate_wire_payload(dict(points=7, data=np.zeros(7, dtype=packed).tobytes()))


@pytest.mark.parametrize("noise_m", [0.001045, 0.001484, 0.002, 0.01])
def test_small_xyz_range_difference_is_diagnostic_and_preserves_original_points(noise_m):
    adapter = NativeWindow()
    expected = []
    for i in range(6):
        arrays = list(tick(adapter))
        arrays[0][0, 0] += noise_m
        expected.append(arrays[0].copy())
        result = adapter.push(i, 1 + i / 60, *arrays)
        np.testing.assert_array_equal(arrays[0], expected[-1])
    rows = np.frombuffer(result["data"], dtype=DTYPE)
    actual = np.column_stack([rows[name] for name in ("x", "y", "z")])
    np.testing.assert_array_equal(actual, np.concatenate(expected).astype(np.float32))
    assert adapter.association_diagnostics["hits_over_1mm"] == 6
    assert adapter.association_diagnostics["max_residual_m"] == pytest.approx(noise_m)
    adapter.finish()
