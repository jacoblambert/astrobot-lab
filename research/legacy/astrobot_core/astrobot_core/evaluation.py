"""Reference-based mission scoring, independent of online reachability estimates."""
import math


class FixedCoverage:
    def __init__(self, home_x, home_y, radius, cell_size):
        if not all(math.isfinite(v) for v in (home_x, home_y, radius, cell_size)) or radius <= 0 or cell_size <= 0:
            raise ValueError("finite positive mission geometry required")
        self.width = math.ceil(2 * radius / cell_size)
        self.cell_size = cell_size
        self.origin = (home_x - self.width * cell_size / 2, home_y - self.width * cell_size / 2)
        self.cells = {(r, c) for r in range(self.width) for c in range(self.width)
                      if math.hypot(self.origin[0] + (c + .5) * cell_size - home_x,
                                    self.origin[1] + (r + .5) * cell_size - home_y) <= radius}
        self.visited = set()

    def observe(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return
        key = (math.floor((y - self.origin[1]) / self.cell_size), math.floor((x - self.origin[0]) / self.cell_size))
        if key in self.cells:
            self.visited.add(key)

    @property
    def fraction(self):
        return len(self.visited) / len(self.cells) if self.cells else 0.0


def mission_verdict(mode, status, home_distance, coverage, best_sample, home_tol=1.0,
                    coverage_gate=0.7, sample_gate=0.75, reference_valid=True):
    reasons = []
    finished = status.get("state") == "finished"
    if not reference_valid:
        reasons.append("reference_invalid")
    if not finished:
        reasons.append("not_finished")
    if status.get("return_reason") in {"slam_diverged", "localization_fault", "return_failed"}:
        reasons.append(status["return_reason"])
    ok_home = home_distance is not None and math.isfinite(home_distance) and home_distance <= home_tol
    if not ok_home:
        reasons.append(f"home_dist>{home_tol}")
        if finished:
            reasons.append("false_success")
    if mode == "explore_radius":
        if coverage is None or not math.isfinite(coverage) or coverage < coverage_gate:
            reasons.append(f"coverage<{coverage_gate}")
    elif mode == "sample_return":
        if not math.isfinite(best_sample) or best_sample < sample_gate:
            reasons.append(f"best_sample<{sample_gate}")
    else:
        reasons.append(f"unknown_mode:{mode}")
    return {"pass": not reasons, "reasons": reasons,
            "validity": "valid" if reference_valid else "infrastructure_invalid",
            "metrics": {"state": status.get("state"), "return_reason": status.get("return_reason"),
                        "home_dist": home_distance, "best_sample": best_sample,
                        "coverage_gain_fraction": coverage, "mission_coverage_fraction": coverage,
                        "coverage_source": "validated_gt_fixed_region"}}
