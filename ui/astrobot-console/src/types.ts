export type Severity = "nominal" | "warning" | "critical" | "unknown";

export interface GridMap {
  frame: string;
  sourceFrame?: string;
  resolution: number;
  width: number;
  height: number;
  sourceWidth?: number;
  sourceHeight?: number;
  downsample?: number;
  origin: { x: number; y: number; yaw: number };
  data: number[];
  stamp?: number;
}

export interface Pose2D {
  x: number;
  y: number;
  yaw: number;
  stamp?: number;
}

export interface Footprint {
  halfLength: number;
  halfWidth: number;
}

export interface RobotState {
  name: string;
  severity: Severity;
  statusText: string;
  pose?: Pose2D;
  goal?: Pose2D;
  trajectory: Pose2D[];
  plan: Pose2D[];
  diagnostics?: Record<string, unknown>;
  footprint?: Footprint;
  /** Ground truth in the display map frame (evaluator overlay; sim only). */
  gtPose?: Pose2D | null;
  gtTrajectory?: Pose2D[];
  /** The exploration manager's ranked goal candidates, best first. */
  candidates?: Array<{ x: number; y: number }>;
}

/** A GT rock footprint (axis-aligned world bounds) in the display map frame. */
export interface GtRock {
  x0: number;
  y0: number;
  x1: number;
  y1: number;
}

export interface LocalizationState {
  state: string;
  motionPermitted: boolean;
  reason: string;
  epoch: number;
  stamp: number;
}

export interface TerrainState {
  valid: boolean;
  failure?: string | null;
  epoch?: number | null;
  received: number;
  projected: number;
  skippedSequences: number;
  lateChunks: number;
  evidenceLagSec?: number | null;
  peakRssMib?: number | null;
  stamp: number;
}

export interface MissionState {
  mode: string;
  state: string;
  elapsedSec: number;
  coverageFraction: number;
  bestSample: number;
  goals: number;
  successes: number;
  failures: number;
  returningHome: boolean;
  details: Record<string, unknown>;
}

export interface TelemetryFrame {
  schemaVersion: string;
  stamp: number;
  connected: boolean;
  robots: RobotState[];
  mission: MissionState;
  maps: Record<string, GridMap>;
  cameraTopics: string[];
  series: Record<string, number[]>;
  topics: Record<string, { stamp: number; count: number; hz?: number }>;
  events: Array<{ stamp: number; severity: Severity; text: string; source: string }>;
  /** Optional (newer backends): "legacy" | "v2". */
  stack?: string;
  simTime?: number | null;
  /** Sim seconds per wall second. */
  simSpeed?: number | null;
  /** Rate series to show, in order. */
  rateSeries?: string[];
  localization?: LocalizationState | null;
  terrain?: TerrainState | null;
  dds?: { domain: string; rmw: string };
  /** Rock footprints from the sim's scene inventory (the gate's contact reference). */
  gtRocks?: GtRock[];
}
