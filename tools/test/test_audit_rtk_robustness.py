import numpy as np
import pytest

import audit_rtk_robustness as rtk


def _status(t, state, permitted, epoch=1, reason=""):
    return (t, rtk.STATES.index(state), epoch, permitted, reason)


def test_profile_terms():
    assert rtk.parse("nominal") == dict(outage=0.0, noise=0.0, outlier=0.0)
    assert rtk.parse("outage30,noise5") == dict(outage=30.0, noise=5.0, outlier=0.0)


def test_supervision_timeline_hold_and_release():
    rows = [_status(t, "HEALTHY", True) for t in np.arange(10, 50, 0.5)]
    rows += [_status(50.0, "HEALTHY", True), _status(52.1, "STOPPING", False, reason="absolute_sensor_outage")]
    rows += [_status(t, "STARTING", False) for t in np.arange(52.5, 70, 0.5)]
    rows += [_status(t, "HEALTHY", True, epoch=2) for t in np.arange(70, 90, 0.5)]
    rec = dict(status=rows, resets=[dict(epoch=1, t=12.0), dict(epoch=2, t=66.0)], clears=4)
    errors = np.array([[t, 0.1 if t < 70 else 0.6, 0.0, 0.0] for t in np.arange(10, 90, 1.0)])
    out = rtk.supervision(rec, errors, dict(start=50.0, end=60.0))
    assert out["faults"] == [dict(t=52.1, state="STOPPING", reason="absolute_sensor_outage")]
    assert out["motion_held_after_s"] == 2.1
    assert out["motion_permitted_again_after_window_s"] == 10.0
    assert out["epochs"] == 2 and out["map_resets"] == 2
    assert out["published_h_max_cm_while_permitted"] == 60.0
    assert 0.6 < out["permitted_fraction_after_first"] < 0.8


def test_latency_summary():
    out = rtk.latency([0.1] * 98 + [0.6, 0.7])
    assert out["p50"] == 0.1 and out["max"] == 0.7 and out["over_0_5s"] == 2


def test_rtk_noise_rescale_keeps_realization_and_offset():
    rng = np.random.default_rng(3)
    truths = rng.uniform(-30, -10, (2000, 3))
    home = np.array([-20.0, -20.0])
    d = np.hypot(*(truths[:, :2] - home).T)
    sigma = 0.05 + 0.01 * d
    unit = rng.normal(size=(2000, 3))
    noise = unit * np.stack([sigma, sigma, 2 * sigma], 1)
    fixes = truths + noise
    fixes[:, :2] -= home
    recovered, offset = rtk.fix_noise(fixes, truths)
    assert offset.tolist() == [20.0, 20.0, 0.0]
    np.testing.assert_allclose(recovered, noise, atol=1e-9)
    cov = [0.0] * 36
    cov[0] = cov[7] = sigma[0] ** 2
    cov[14] = (2 * sigma[0]) ** 2
    new, new_cov = rtk.rescale_fix(recovered[0], cov, d[0], 0.02, 0.001)
    want = 0.02 + 0.001 * d[0]
    np.testing.assert_allclose(new, unit[0] * [want, want, 2 * want])
    assert np.isclose(new_cov[0], want**2) and np.isclose(new_cov[14], 4 * want**2) and new_cov[21] == cov[21]


def test_rtk_noise_rescale_rejects_a_non_cm_home():
    truths = np.zeros((10, 3))
    with pytest.raises(ValueError, match="cm home"):
        rtk.fix_noise(truths + [0.125, 0.0, 0.0], truths)
