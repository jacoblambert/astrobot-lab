import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from render_contact_support import render


def test_report_keeps_partial_progress_and_safety_limits_visible():
    report = dict(
        contract=dict(history_xy_erosion_m=0.02),
        summary=dict(
            tracks=dict(all=dict(samples=33, stationary=33, legacy_progress_or_turn=0, expanded_any_action=1))
        ),
        examples=[],
    )
    page = render([("example</script><b>", report)])
    assert "1/33" in page and "0/33" in page and "33/33" in page
    assert "Offline diagnosis, not permission to drive" in page
    assert "not a calibrated error bound" in page
    assert "not a footprint that follows the robot" in page
    assert "example</script>" not in page
    assert 'fill-rule="evenodd"' in page
