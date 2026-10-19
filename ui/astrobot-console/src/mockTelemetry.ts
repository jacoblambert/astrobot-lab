import type { GridMap, TelemetryFrame } from "./types";

export type MockScenario = "nominal" | "replanning" | "slam_warning" | "critical" | "success";

export interface FleetEntry {
  id: string;
  name: string;
  status: "ok" | "warn" | "err" | "off";
  task: string;
  battery: number;
}

// Mock fleet extras shown in sidebar when not connected to a live backend
export const MOCK_FLEET_EXTRA: FleetEntry[] = [
  { id: "astrobot_1", name: "astrobot_1", status: "ok",   task: "Returning to dock for sample handoff",       battery: 0.82 },
  { id: "astrobot_2", name: "astrobot_2", status: "warn", task: "IMU bias drift detected · recalibrating",    battery: 0.31 },
  { id: "astrobot_3", name: "astrobot_3", status: "ok",   task: "Surveying grid sector D-4",                  battery: 0.91 },
  { id: "astrobot_4", name: "astrobot_4", status: "off",  task: "Comms lost · last seen 00:08:14 ago",        battery: 0.12 },
  { id: "astrobot_5", name: "astrobot_5", status: "err",  task: "Plan failure · path occluded by debris",     battery: 0.54 },
  { id: "astrobot_6", name: "astrobot_6", status: "ok",   task: "Idle at charger station 02",                 battery: 0.99 },
  { id: "astrobot_7", name: "astrobot_7", status: "ok",   task: "Transit to staging area",                    battery: 0.66 },
];

function makeMap(kind: "cost" | "resource" | "mission", width = 120, height = 120): GridMap {
  const data: number[] = [];
  for (let row = 0; row < height; row += 1) {
    for (let col = 0; col < width; col += 1) {
      const x = (col - width / 2) / width;
      const y = (row - height / 2) / height;
      const r0 = Math.hypot(x + 0.18, y - 0.1);
      const r1 = Math.hypot(x - 0.22, y + 0.2);
      const ridge = Math.abs(Math.sin(col * 0.16) + Math.cos(row * 0.13));
      if (kind === "cost") {
        const rock = Math.max(0, 100 - r0 * 900) + Math.max(0, 95 - r1 * 750);
        data.push(Math.min(100, Math.round(rock + ridge * 7)));
      } else if (kind === "resource") {
        const hotspot = Math.max(0, 100 - r0 * 250) + Math.max(0, 80 - r1 * 190);
        data.push(Math.min(100, Math.round(hotspot)));
      } else {
        data.push(r0 < 0.34 || r1 < 0.28 ? 70 : row % 9 === 0 || col % 9 === 0 ? 35 : 0);
      }
    }
  }
  return { frame: "map", resolution: 0.1, width, height, origin: { x: -6, y: -6, yaw: 0 }, data, stamp: Date.now() / 1000 };
}

// Deterministic series generator (no Math.random — same output each call)
function makeSeries(seed: number, n = 30, lo = 0, hi = 1, smooth = 0.55): number[] {
  let v = (lo + hi) / 2;
  const out: number[] = [];
  for (let i = 0; i < n; i++) {
    const target = lo + ((Math.sin(seed + i * 0.5) + 1) / 2) * (hi - lo);
    v = v * smooth + target * (1 - smooth) + Math.sin(seed * 7 + i * 1.7) * (hi - lo) * 0.04;
    out.push(Math.max(lo, Math.min(hi, v)));
  }
  return out;
}

export function makeMockTelemetry(scenario: MockScenario = "nominal"): TelemetryFrame {
  const now = Date.now() / 1000;
  const trajectory = Array.from({ length: 48 }, (_, i) => ({
    x: -4.2 + i * 0.13,
    y: -3.3 + Math.sin(i * 0.22) * 1.25,
    yaw: 0.2,
    stamp: now - (48 - i) * 2,
  }));
  const plan = Array.from({ length: 24 }, (_, i) => ({
    x: 2.0 + i * 0.12,
    y: 1.1 + Math.sin(i * 0.16) * 0.5,
    yaw: 0.1,
    stamp: now,
  }));
  const scenarioState = {
    nominal: {
      severity: "nominal" as const,
      statusText: "navigating to resource frontier",
      missionState: "running",
      failures: 1,
      returningHome: false,
      events: [
        { severity: "nominal"  as const, text: "Navigation nominal",                        source: "nav2"        },
        { severity: "nominal"  as const, text: "Reached waypoint 12/14",                    source: "behavior_tree"},
        { severity: "nominal"  as const, text: "Sample acquisition complete · best 0.78",   source: "mission"     },
        { severity: "nominal"  as const, text: "Loop closure confirmed",                    source: "glim_slam"   },
      ],
    },
    replanning: {
      severity: "warning" as const,
      statusText: "plan rejected, replanning",
      missionState: "replanning",
      failures: 2,
      returningHome: false,
      events: [
        { severity: "warning" as const, text: "Planner precheck rejected candidate",        source: "exploration" },
        { severity: "warning" as const, text: "BEV costmap path quality exceeded threshold",source: "exploration" },
      ],
    },
    slam_warning: {
      severity: "warning" as const,
      statusText: "SLAM drift rising · 0.38 m error",
      missionState: "running",
      failures: 1,
      returningHome: false,
      events: [
        { severity: "warning" as const, text: "SLAM error exceeded soft threshold",        source: "diagnostics" },
        { severity: "warning" as const, text: "slam_gt_increment_mismatch · confidence medium", source: "diagnostics" },
      ],
    },
    critical: {
      severity: "critical" as const,
      statusText: "SLAM divergence · mission paused",
      missionState: "safety_cancelled",
      failures: 4,
      returningHome: false,
      events: [
        { severity: "critical" as const, text: "SLAM pose jump detected · 6.2 m",          source: "diagnostics" },
        { severity: "critical" as const, text: "Goal safety-cancelled",                     source: "exploration" },
      ],
    },
    success: {
      severity: "nominal" as const,
      statusText: "returning home · best sample 0.82",
      missionState: "returning_home",
      failures: 1,
      returningHome: true,
      events: [
        { severity: "nominal" as const, text: "Resource threshold met · returning home",   source: "mission"     },
        { severity: "nominal" as const, text: "Best sample 0.82 ≥ threshold 0.80",          source: "exploration" },
      ],
    },
  }[scenario];

  return {
    schemaVersion: "astrobot.console.v1",
    stamp: now,
    connected: false,
    robots: [
      {
        name: "astrobot_0",
        severity: scenarioState.severity,
        statusText: scenarioState.statusText,
        pose: trajectory[trajectory.length - 1],
        goal: plan[plan.length - 1],
        trajectory,
        plan,
        diagnostics: { primary: "no_active_fault_detected", confidence: "mock" },
      },
    ],
    mission: {
      mode: "sample_return",
      state: scenarioState.missionState,
      elapsedSec: 742,
      coverageFraction: 0.63,
      bestSample: 0.78,
      goals: 14,
      successes: 12,
      failures: scenarioState.failures,
      returningHome: scenarioState.returningHome,
      details: { mission_cell_size_m: 2, blacklist_size: 3, resource_candidates: 18 },
    },
    maps: {
      obstacle: makeMap("cost"),
      water: makeMap("resource"),
      mission: makeMap("mission"),
    },
    cameraTopics: ["/astrobot_0/camera_0/image_raw"],
    series: {
      // Topic Hz — 30-sample deterministic series
      "PointCloud Hz": makeSeries(0.7,  30, 8.6,  10.4),
      "IMU Hz":        makeSeries(1.3,  30, 96,   102),
      "SLAM pose Hz":  makeSeries(2.1,  30, 17,   21),
      "Voxel map Hz":  makeSeries(3.3,  30, 1.7,  2.4),
      "BEV map Hz":    makeSeries(4.5,  30, 1.7,  2.3),
      "Camera Hz":     makeSeries(5.1,  30, 8.5,  10.5),
      // Algorithm metrics
      "SLAM error":    makeSeries(8.2,  30, 0.04, 0.18),
      "Goal distance": makeSeries(9.4,  30, 0.5,  4.5).map((v, i, a) => a[a.length - 1 - i] * 0.8 + v * 0.2),
      "Coverage":      Array.from({ length: 30 }, (_, i) => 0.20 + (i / 29) * 0.43),
      "Resource":      Array.from({ length: 30 }, (_, i) => 0.12 + (i / 29) * 0.66),
    },
    topics: {
      "slam/bev_costmap":               { stamp: now - 0.8,  count: 238,   hz: 2.0  },
      "lidar_0/pointcloud/filtered":    { stamp: now - 0.1,  count: 3120,  hz: 10.0 },
      "imu":                            { stamp: now - 0.02, count: 31200, hz: 100.0 },
      "slam/pose_corrected":            { stamp: now - 0.04, count: 6240,  hz: 20.0 },
      "slam/prob_voxel_map":            { stamp: now - 0.6,  count: 690,   hz: 2.2  },
      "diagnostics":                    { stamp: now - 1.6,  count: 94,    hz: 0.5  },
      "camera_0/image_raw":             { stamp: now - 0.1,  count: 1412,  hz: 10.0 },
      "exploration/status":             { stamp: now - 0.5,  count: 187,   hz: 1.0  },
    },
    events: scenarioState.events.map((event, index) => ({ ...event, stamp: now - index * 14 })),
  };
}
