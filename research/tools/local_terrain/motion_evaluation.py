"""Independent motion-score accounting, NOT a terrain oracle or drive authority.

The caller freezes expected cases before inference and supplies independently
validated geometry labels. This module does not derive those labels from a map.
Passing these metrics alone never qualifies runtime, geometry or navigation.
"""

from dataclasses import dataclass
import math

from shadow_terrain import integer

STATES = {"feasible", "blocked", "unknown"}


@dataclass(frozen=True)
class MotionCase:
    case_id: str
    family: str
    exposure_id: str  # Correlated frames retain the same physical exposure ID.
    query_ns: int
    revision: str  # Opaque session/epoch/revision identity from the frozen query.

    def __post_init__(self):
        if any(not isinstance(v, str) or not v for v in (self.case_id, self.family, self.exposure_id, self.revision)):
            raise ValueError("motion case needs explicit identity/family/exposure/revision")
        integer(self.query_ns, "query time")


@dataclass(frozen=True)
class MotionDecision:
    case_id: str
    state: str
    evidence_ns: int
    publication_ns: int
    revision: str
    valid: bool

    def __post_init__(self):
        if self.state not in STATES or not isinstance(self.case_id, str) or not self.case_id:
            raise ValueError("invalid motion decision")
        integer(self.evidence_ns, "evidence time")
        integer(self.publication_ns, "publication time")
        if type(self.valid) is not bool or not isinstance(self.revision, str) or not self.revision:
            raise ValueError("invalid decision validity/revision")
        if self.evidence_ns > self.publication_ns:
            raise ValueError("evidence from after publication")


def score_motions(cases, reference, decisions, *, required_families, max_age_ns, source_completed, target=0.95):
    """Count every declared case; reject duplicates, extras and diluted families.

    reference maps case_id to feasible/blocked/unknown from the separate oracle.
    Missing labels/decisions remain visible and prevent metric completion. Age is
    measured at consumption (query), not merely at the mapper's publication.
    """
    max_age_ns = integer(max_age_ns, "operating-contract age limit")
    if not max_age_ns or not math.isfinite(target) or not 0.95 <= target <= 1:
        raise ValueError("positive frozen age bound and availability target >=95% required")
    if type(source_completed) is not bool:
        raise ValueError("explicit source completion required")
    families = tuple(required_families)
    if not families or len(set(families)) != len(families) or any(not isinstance(f, str) or not f for f in families):
        raise ValueError("freeze a nonempty unique scenario-family list")
    cases, decisions = list(cases), list(decisions)
    by_id = {case.case_id: case for case in cases}
    by_decision = {decision.case_id: decision for decision in decisions}
    if not cases or len(by_id) != len(cases) or len(by_decision) != len(decisions):
        raise ValueError("empty or duplicated motion exposure")
    if set(by_decision) - set(by_id) or set(reference) - set(by_id):
        raise ValueError("undeclared reference/decision case")
    if any(value not in STATES for value in reference.values()):
        raise ValueError("invalid independent motion reference")
    if {case.family for case in cases} != set(families):
        raise ValueError("missing or undeclared scenario family")

    def counters():
        return dict(
            cases=0,
            safe_reference=0,
            hazard_reference=0,
            unknown_reference=0,
            missing_decisions=0,
            false_safe=0,
            feasible_without_reference=0,
            useful_safe=0,
            unnecessary_stop=0,
            invalid_feasible=0,
        )

    total = counters()
    groups = {family: counters() for family in families}
    exposures = {family: set() for family in families}
    for case in cases:
        truth = reference.get(case.case_id, "unknown")
        decision = by_decision.get(case.case_id)
        feasible = decision is not None and decision.state == "feasible"
        coherent = decision is not None and (
            decision.valid
            and decision.revision == case.revision
            and decision.publication_ns <= case.query_ns
            and 0 <= case.query_ns - decision.evidence_ns <= max_age_ns
        )
        counts = dict(
            cases=1,
            safe_reference=int(truth == "feasible"),
            hazard_reference=int(truth == "blocked"),
            unknown_reference=int(truth == "unknown"),
            missing_decisions=int(decision is None),
            false_safe=int(feasible and truth == "blocked"),
            feasible_without_reference=int(feasible and truth == "unknown"),
            useful_safe=int(feasible and coherent and truth == "feasible"),
            unnecessary_stop=int(truth == "feasible" and not (feasible and coherent)),
            invalid_feasible=int(feasible and not coherent),
        )
        for target_counts in (total, groups[case.family]):
            for key, value in counts.items():
                target_counts[key] += value
        exposures[case.family].add(case.exposure_id)
    for counts in [total, *groups.values()]:
        counts["safe_motion_availability"] = (
            counts["useful_safe"] / counts["safe_reference"] if counts["safe_reference"] else None
        )
    complete = source_completed and not total["unknown_reference"] and not total["missing_decisions"]
    useful = all(
        group["safe_motion_availability"] is not None and group["safe_motion_availability"] >= target
        for group in [total, *groups.values()]
    )
    metric_pass = bool(
        complete
        and useful
        and total["hazard_reference"] > 0
        and not total["false_safe"]
        and not total["invalid_feasible"]
    )
    return dict(
        motion_metrics_pass=metric_pass,
        acceptance=False,
        navigation_authority=False,
        independent_geometry_validated_by_this_module=False,
        complete=complete,
        source_completed=source_completed,
        target=target,
        max_age_ns=max_age_ns,
        overall=total,
        families=groups,
        physical_exposures=len({case.exposure_id for case in cases}),
        family_exposures={family: len(ids) for family, ids in exposures.items()},
        exposure_note="Frames may be correlated; exposure counts are not independent Bernoulli trials.",
        outstanding_gates=[
            "independent geometry/vehicle contract",
            "sensing/stopping",
            "runtime",
            "stability",
            "recovery",
        ],
    )
