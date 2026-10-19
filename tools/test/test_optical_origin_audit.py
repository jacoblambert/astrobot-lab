import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from probe_lidar_optical_origin import audit_passed


def cases():
    return [
        dict(
            case=name + suffix,
            native_samples=127932,
            geometry_passed=name != "original",
            origin_overlaps=["body"] if name == "original" else [],
            mask_mismatch=0,
            hidden_body_returns=0,
            max_depth_error_m=0.00001,
            valid_returns=100,
        )
        for name in ("original", "raised_045", "raised_065")
        for suffix in ("_min05", "_min0")
    ]


def test_complete_successful_controls_required():
    assert audit_passed(cases())
    assert not audit_passed(cases()[:-1])


@pytest.mark.parametrize(
    "key,value",
    [
        ("native_samples", 1),
        ("geometry_passed", False),
        ("origin_overlaps", ["wheel"]),
        ("mask_mismatch", 1),
        ("hidden_body_returns", 1),
        ("max_depth_error_m", 0.0011),
        ("valid_returns", 0),
    ],
)
def test_candidate_failures_cannot_be_reported_as_success(key, value):
    rows = cases()
    rows[-1][key] = value
    assert not audit_passed(rows)
