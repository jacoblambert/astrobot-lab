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

export interface RobotState {
  name: string;
  severity: Severity;
  statusText: string;
  pose?: Pose2D;
  goal?: Pose2D;
  trajectory: Pose2D[];
  plan: Pose2D[];
  diagnostics?: Record<string, unknown>;
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
}
