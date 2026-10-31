import React, { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import type { Footprint, GridMap, GtRock, Pose2D, RobotState, Severity, TelemetryFrame } from "./types";
import { makeMockTelemetry, MOCK_FLEET_EXTRA, type FleetEntry, type MockScenario } from "./mockTelemetry";
import "./styles.css";

// ── Constants ────────────────────────────────────────────────────────────────

const LAYER_ORDER = ["obstacle", "explore", "local", "mission", "water", "gt"];

const LAYER_LABELS: Record<string, string> = {
  obstacle: "Cost",
  explore: "Explored",
  local: "Local",
  mission: "Grid",
  water: "Water",
  gt: "GT",
};

const LAYER_TITLES: Record<string, string> = {
  obstacle: "Nav2 cost map",
  explore: "Explored terrain",
  local: "Local planner window",
  mission: "Mission grid",
  water: "Water (GT)",
  gt: "GT obstacles",
};

const DUMMY_CAM_TOPICS = [
  "camera_0/image_raw",
  "camera_0/image_rect",
  "camera_0/depth_preview",
  "/gt/debug/chase_camera",
];

const SCENARIO_LABELS: Record<MockScenario, string> = {
  nominal: "Nominal",
  replanning: "Replanning",
  slam_warning: "SLAM warn",
  critical: "Critical",
  success: "Sample return",
};

// ── Helpers ──────────────────────────────────────────────────────────────────

function ageLabel(stamp: number | undefined, now: number): string {
  if (!stamp || stamp <= 0) return "–";
  const age = Math.max(0, now - stamp);
  if (age < 1) return `${Math.round(age * 1000)} ms`;
  if (age < 60) return `${age.toFixed(1)} s`;
  return `${Math.round(age / 60)} m`;
}

function topicStatus(stamp: number | undefined, now: number): "ok" | "warn" | "err" {
  if (!stamp || stamp <= 0) return "err";
  const age = now - stamp;
  if (age < 1.5) return "ok";
  if (age < 5) return "warn";
  return "err";
}

function severityToStatus(s: Severity): "ok" | "warn" | "err" {
  if (s === "nominal") return "ok";
  if (s === "warning") return "warn";
  return "err";
}

function formatElapsed(sec: number): string {
  const m = Math.floor(sec / 60);
  const s = Math.floor(sec % 60);
  if (m >= 60) return `${Math.floor(m / 60)}h ${m % 60}m`;
  return `${m}:${String(s).padStart(2, "0")}`;
}

function formatNow(d: Date) {
  return {
    timeStr: d.toLocaleTimeString("en-US", { hour: "numeric", minute: "2-digit", second: "2-digit", hour12: true }),
    dateStr: d.toLocaleDateString("en-US", { weekday: "short", month: "short", day: "numeric" }),
  };
}

// ── Map view ─────────────────────────────────────────────────────────────────

/** Initial display from the URL, for bookmarkable demo views: ?layer=explore&view=follow&rocks=1&gt=1 */
const URL_PARAMS = new URLSearchParams(window.location.search);

/** A square world window on the canvas: centre (cx, cy) in map metres, `scale` px per metre. */
interface View {
  cx: number;
  cy: number;
  scale: number;
  W: number;
  H: number;
}

const FOLLOW_SPAN_M = 14;

function worldToCanvas(v: View, x: number, y: number): [number, number] {
  return [v.W / 2 + (x - v.cx) * v.scale, v.H / 2 - (y - v.cy) * v.scale];
}

function canvasToWorld(v: View, px: number, py: number): [number, number] {
  return [v.cx + (px - v.W / 2) / v.scale, v.cy - (py - v.H / 2) / v.scale];
}

function mapBounds(map: GridMap): { x0: number; y0: number; x1: number; y1: number } {
  const w = map.width * map.resolution;
  const h = map.height * map.resolution;
  const c = Math.cos(map.origin.yaw);
  const s = Math.sin(map.origin.yaw);
  const xs: number[] = [];
  const ys: number[] = [];
  for (const [lx, ly] of [[0, 0], [w, 0], [0, h], [w, h]]) {
    xs.push(map.origin.x + c * lx - s * ly);
    ys.push(map.origin.y + s * lx + c * ly);
  }
  return { x0: Math.min(...xs), y0: Math.min(...ys), x1: Math.max(...xs), y1: Math.max(...ys) };
}

type RGBA = [number, number, number, number];

function rgbaForCell(value: number, layer: string): RGBA {
  if (layer === "gt-overlay") return value >= 50 ? [244, 114, 182, 150] : [0, 0, 0, 0];
  if (value < 0) return [10, 18, 28, 170];
  const v = Math.min(100, value) / 100;
  switch (layer) {
    case "water":
      return [30 + v * 50, 120 + v * 115, 200 + v * 55, (0.06 + v * 0.82) * 255];
    case "mission":
      return v > 0.6 ? [94, 234, 212, 77] : v > 0.3 ? [96, 165, 250, 46] : [42, 65, 90, 77];
    case "gt":
      return [160 + v * 60, 20 + v * 60, 200 + v * 30, (0.06 + v * 0.72) * 255];
    case "explore":
      // observed classes: 0 free, 100 occupied (no costs or inflation)
      return value >= 50 ? [251, 191, 36, 235] : [38, 92, 104, 150];
    default:
      // obstacle / local: Nav2 costs, dark slate → purple → magenta (100 = lethal)
      return [60 + v * 168, 40 + v * 57, 100 + v * 149, (0.12 + v * 0.78) * 255];
  }
}

const gridImages = new WeakMap<GridMap, Map<string, HTMLCanvasElement>>();

/** The grid as a 1 px-per-cell image (row 0, the lowest world y, at the bottom), cached per map frame. */
function gridImage(map: GridMap, layer: string): HTMLCanvasElement {
  let byLayer = gridImages.get(map);
  if (!byLayer) {
    byLayer = new Map();
    gridImages.set(map, byLayer);
  }
  const cached = byLayer.get(layer);
  if (cached) return cached;
  const canvas = document.createElement("canvas");
  canvas.width = Math.max(1, map.width);
  canvas.height = Math.max(1, map.height);
  const ctx = canvas.getContext("2d")!;
  const image = ctx.createImageData(canvas.width, canvas.height);
  const px = image.data;
  for (let row = 0; row < map.height; row++) {
    const out = (map.height - 1 - row) * map.width;
    for (let col = 0; col < map.width; col++) {
      const [r, g, b, a] = rgbaForCell(map.data[row * map.width + col] ?? -1, layer);
      const i = (out + col) * 4;
      px[i] = r;
      px[i + 1] = g;
      px[i + 2] = b;
      px[i + 3] = a;
    }
  }
  ctx.putImageData(image, 0, 0);
  byLayer.set(layer, canvas);
  return canvas;
}

function drawGrid(ctx: CanvasRenderingContext2D, v: View, map: GridMap, image: HTMLCanvasElement) {
  const [ox, oy] = worldToCanvas(v, map.origin.x, map.origin.y);
  const s = v.scale * map.resolution;
  ctx.save();
  ctx.translate(ox, oy);
  ctx.rotate(-map.origin.yaw);
  ctx.scale(s, s);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(image, 0, -map.height);
  ctx.restore();
}

function drawPolyline(ctx: CanvasRenderingContext2D, v: View, points: Pose2D[], color: string, width: number, dash: number[]) {
  if (points.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.beginPath();
  points.forEach((p, i) => {
    const [x, y] = worldToCanvas(v, p.x, p.y);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

/** GT rocks as ellipses inscribed in their world bounds (rocks are roughly round). */
function drawGtRocks(ctx: CanvasRenderingContext2D, v: View, rocks: GtRock[]) {
  ctx.fillStyle = "rgba(244,114,182,0.28)";
  ctx.strokeStyle = "rgba(244,114,182,0.95)";
  ctx.lineWidth = 1.25;
  for (const r of rocks) {
    const [cx, cy] = worldToCanvas(v, (r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2);
    ctx.beginPath();
    ctx.ellipse(cx, cy, Math.max(1.5, ((r.x1 - r.x0) / 2) * v.scale), Math.max(1.5, ((r.y1 - r.y0) / 2) * v.scale), 0, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
}

/** Ranked goal candidates: the next goal is #1; the top few are numbered. */
function drawCandidates(ctx: CanvasRenderingContext2D, v: View, candidates: Array<{ x: number; y: number }>) {
  ctx.font = "9px 'JetBrains Mono', monospace";
  for (let i = candidates.length - 1; i >= 0; i--) {
    const [x, y] = worldToCanvas(v, candidates[i].x, candidates[i].y);
    const top = i < 5;
    ctx.fillStyle = top ? "rgba(147,197,253,0.95)" : "rgba(147,197,253,0.4)";
    ctx.beginPath();
    ctx.arc(x, y, top ? 3.5 : 2, 0, Math.PI * 2);
    ctx.fill();
    if (top) ctx.fillText(String(i + 1), x + 5, y - 4);
  }
}

/** The explore_radius mission disc: coverage counts the 2 m grid cells inside it. */
function drawMissionRadius(ctx: CanvasRenderingContext2D, v: View, x: number, y: number, radius: number) {
  const [cx, cy] = worldToCanvas(v, x, y);
  ctx.strokeStyle = "rgba(147,197,253,0.5)";
  ctx.lineWidth = 1;
  ctx.setLineDash([6, 5]);
  ctx.beginPath();
  ctx.arc(cx, cy, radius * v.scale, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "rgba(147,197,253,0.75)";
  ctx.font = "9px 'JetBrains Mono', monospace";
  ctx.fillText(`mission radius ${radius.toFixed(0)} m`, cx + 4, cy - radius * v.scale - 4);
}

/** The rover body at true scale (never below a legible marker), with a heading wedge. */
function drawRover(ctx: CanvasRenderingContext2D, v: View, pose: Pose2D, fp: Footprint, ghost: boolean) {
  const [rx, ry] = worldToCanvas(v, pose.x, pose.y);
  const grow = Math.max(1, 9 / (fp.halfLength * v.scale));
  const hl = fp.halfLength * v.scale * grow;
  const hw = fp.halfWidth * v.scale * grow;
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(-pose.yaw);
  if (ghost) {
    ctx.strokeStyle = "rgba(251,191,36,0.95)";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 3]);
    ctx.strokeRect(-hl, -hw, 2 * hl, 2 * hw);
    ctx.setLineDash([]);
  } else {
    ctx.fillStyle = "rgba(5,10,18,0.55)";
    ctx.strokeStyle = "#5eead4";
    ctx.lineWidth = 1.75;
    ctx.fillRect(-hl, -hw, 2 * hl, 2 * hw);
    ctx.strokeRect(-hl, -hw, 2 * hl, 2 * hw);
    ctx.fillStyle = "#5eead4";
    ctx.beginPath();
    ctx.moveTo(hl, 0);
    ctx.lineTo(hl * 0.35, -hw * 0.6);
    ctx.lineTo(hl * 0.5, 0);
    ctx.lineTo(hl * 0.35, hw * 0.6);
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
}

/** Metric grid lines at a step that stays readable at the current zoom. */
function drawMetricGrid(ctx: CanvasRenderingContext2D, v: View) {
  const span = Math.max(v.W, v.H) / v.scale;
  const step = span > 40 ? 10 : span > 16 ? 5 : span > 6 ? 1 : 0.5;
  const [x0, y1] = canvasToWorld(v, 0, 0);
  const [x1, y0] = canvasToWorld(v, v.W, v.H);
  ctx.strokeStyle = "rgba(94,234,212,0.08)";
  ctx.lineWidth = 0.5;
  for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) {
    const [px] = worldToCanvas(v, x, 0);
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, v.H); ctx.stroke();
  }
  for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) {
    const [, py] = worldToCanvas(v, 0, y);
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(v.W, py); ctx.stroke();
  }
}

function niceScaleMeters(scale: number): number {
  for (const m of [0.5, 1, 2, 5, 10, 20, 50]) if (m * scale >= 50) return m;
  return 100;
}

// ── useTelemetry ─────────────────────────────────────────────────────────────

function useTelemetry(scenario: MockScenario): TelemetryFrame {
  const [frame, setFrame] = useState<TelemetryFrame>(() => makeMockTelemetry(scenario));

  useEffect(() => {
    setFrame((prev) => (prev.connected ? prev : makeMockTelemetry(scenario)));
  }, [scenario]);

  useEffect(() => {
    const proto = window.location.protocol === "https:" ? "wss" : "ws";
    const socket = new WebSocket(`${proto}://${window.location.host}/ws/telemetry`);
    // A socket this effect closed itself (re-render, StrictMode) must not flip the view to mock.
    let disposed = false;
    socket.onmessage = (ev) => {
      if (disposed) return;
      const parsed = JSON.parse(ev.data) as TelemetryFrame;
      setFrame((prev) => ({ ...parsed, maps: { ...(prev.connected ? prev.maps : {}), ...parsed.maps } }));
    };
    socket.onerror = () => {
      if (!disposed) setFrame({ ...makeMockTelemetry(scenario), connected: false });
    };
    socket.onclose = () => {
      if (!disposed) setFrame((prev) => (prev.connected ? { ...prev, connected: false } : prev));
    };
    return () => {
      disposed = true;
      socket.close();
    };
  }, [scenario]);

  return frame;
}

// ── Header ───────────────────────────────────────────────────────────────────

function Header({
  frame,
  fleetTotal,
  fleetOnline,
  scenario,
  setScenario,
}: {
  frame: TelemetryFrame;
  fleetTotal: number;
  fleetOnline: number;
  scenario: MockScenario;
  setScenario: (s: MockScenario) => void;
}) {
  const [now, setNow] = useState(() => formatNow(new Date()));
  useEffect(() => {
    const id = setInterval(() => setNow(formatNow(new Date())), 1000);
    return () => clearInterval(id);
  }, []);

  const robot = frame.robots[0];
  const severity = robot?.severity ?? "unknown";
  const pillClass = severity === "nominal" ? "ok" : severity === "warning" ? "warn" : "err";
  const pillLabel = severity === "nominal" ? "nominal" : severity === "warning" ? "warning" : "critical";

  const mission = frame.mission;
  const modeLabel = mission.mode.replace(/_/g, " ");
  const elapsed = formatElapsed(mission.elapsedSec);

  return (
    <div className="hdr">
      <div className="hdr-brand">
        <div className="hdr-logo">A</div>
        <div className="hdr-title">
          <div className="hdr-eyebrow">Mission Console</div>
          <div className="hdr-name">Astrobot Ops</div>
        </div>
      </div>
      <div className="hdr-divider" />
      <div className="hdr-mission">
        <span className="hdr-mission-tag">FLEET {fleetOnline}/{fleetTotal}</span>
        <span className="hdr-mission-text">
          <strong>{modeLabel}</strong>
          <span className="muted"> · {robot?.statusText ?? "no robot data"}</span>
        </span>
        <span className="hdr-mission-meta">
          {elapsed} elapsed{frame.simSpeed ? ` · sim ${frame.simSpeed.toFixed(2)}×` : ""}
        </span>
      </div>
      <div className="hdr-right">
        <div className={`hdr-pill ${pillClass}`}>
          <span className="dot" />
          {pillLabel}
        </div>
        <div className="hdr-clock">
          <div className="hdr-clock-time">{now.timeStr}</div>
          <div className="hdr-clock-date">{now.dateStr}</div>
        </div>
        {!frame.connected && (
          <>
            <div className="hdr-badge-mock">MOCK</div>
            <select
              className="hdr-scenario"
              value={scenario}
              onChange={(e) => setScenario(e.target.value as MockScenario)}
            >
              {Object.entries(SCENARIO_LABELS).map(([val, label]) => (
                <option key={val} value={val}>{label}</option>
              ))}
            </select>
          </>
        )}
      </div>
    </div>
  );
}

// ── FleetSidebar ─────────────────────────────────────────────────────────────

function FleetSidebar({
  entries,
  selectedId,
  onSelect,
}: {
  entries: FleetEntry[];
  selectedId: string;
  onSelect: (id: string) => void;
}) {
  const counts = entries.reduce<Record<string, number>>(
    (acc, r) => { acc[r.status] = (acc[r.status] ?? 0) + 1; return acc; },
    { ok: 0, warn: 0, err: 0, off: 0 },
  );
  const online = entries.filter((r) => r.status !== "off").length;

  return (
    <div className="fleet">
      <div className="fleet-hdr">
        <div className="fleet-title">Fleet</div>
        <div className="fleet-count"><em>{online}</em>/{entries.length} online</div>
      </div>
      <div className="fleet-summary">
        {(["ok", "warn", "err", "off"] as const).map((s) => (
          <div className="fleet-summary-stat" key={s}>
            <div className={`fleet-summary-num ${s}`}>{counts[s] ?? 0}</div>
            <div className="fleet-summary-lbl">{s === "off" ? "Off" : s.charAt(0).toUpperCase() + s.slice(1)}</div>
          </div>
        ))}
      </div>
      <div className="fleet-collbl"><span>Unit · task</span><span>Power</span></div>
      <div className="fleet-list">
        {entries.map((r) => (
          <div
            key={r.id}
            className={`fleet-row${r.id === selectedId ? " is-selected" : ""}`}
            onClick={() => onSelect(r.id)}
          >
            <span className={`fleet-row-dot ${r.status}`} />
            <div className="fleet-row-body">
              <div className="fleet-row-name">{r.name}</div>
              <div className="fleet-row-task">{r.task}</div>
            </div>
            <div className="fleet-row-batt">
              <div className="fleet-row-batt-num">{r.battery < 0 ? "–" : `${Math.round(r.battery * 100)}%`}</div>
              {r.battery >= 0 && (
                <div className="fleet-row-batt-bar">
                  <div
                    className={`fleet-row-batt-fill ${r.battery < 0.2 ? "crit" : r.battery < 0.4 ? "low" : ""}`}
                    style={{ width: `${r.battery * 100}%` }}
                  />
                </div>
              )}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

// ── BevMapPanel ───────────────────────────────────────────────────────────────

const LEGENDS: Record<string, Array<{ label: string; background: string; border?: string }>> = {
  cost: [
    { label: "free", background: "rgba(60,40,100,0.5)" },
    { label: "cost", background: "linear-gradient(90deg, #6b4fa0, #a855f7)" },
    { label: "lethal", background: "#e4618e" },
    { label: "unknown", background: "#0a121c", border: "1px solid #3d4d63" },
  ],
  mission: [
    { label: "visited = coverage", background: "rgba(94,234,212,0.55)" },
    { label: "seen", background: "rgba(96,165,250,0.45)" },
    { label: "unseen", background: "rgba(42,65,90,0.8)" },
    { label: "outside", background: "#0a121c", border: "1px solid #3d4d63" },
  ],
  explore: [
    { label: "free", background: "rgba(38,92,104,0.9)" },
    { label: "occupied", background: "#fbbf24" },
    { label: "unknown", background: "#0a121c", border: "1px solid #3d4d63" },
  ],
};

function BevMapPanel({
  maps,
  robots,
  connected,
  missionMode,
  missionDetails,
  gtRocks,
  simTime,
  activeLayer,
  setActiveLayer,
  showTrajectory,
  setShowTrajectory,
  showPlan,
  setShowPlan,
}: {
  maps: Record<string, GridMap>;
  robots: RobotState[];
  connected: boolean;
  missionMode: string;
  missionDetails: Record<string, unknown>;
  gtRocks?: GtRock[];
  simTime?: number | null;
  activeLayer: string;
  setActiveLayer: (l: string) => void;
  showTrajectory: boolean;
  setShowTrajectory: (v: boolean) => void;
  showPlan: boolean;
  setShowPlan: (v: boolean) => void;
}) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const wrapRef = useRef<HTMLDivElement | null>(null);
  const [size, setSize] = useState({ W: 0, H: 0 });
  const [displayOpen, setDisplayOpen] = useState(true);
  const [viewMode, setViewMode] = useState<"arena" | "follow">(URL_PARAMS.get("view") === "follow" ? "follow" : "arena");
  const [zoom, setZoom] = useState(1);
  const [showGtOverlay, setShowGtOverlay] = useState(URL_PARAMS.get("rocks") === "1");
  const [showGtPose, setShowGtPose] = useState(URL_PARAMS.get("gt") === "1");
  const [showCandidates, setShowCandidates] = useState(URL_PARAMS.get("candidates") !== "0");
  const [teleopClick, setTeleopClick] = useState(false);
  const [teleopStatus, setTeleopStatus] = useState("");
  const map = maps[activeLayer] ?? maps.obstacle ?? Object.values(maps)[0];
  const robot = robots[0];
  const teleopAvailable = connected && missionMode === "teleop_click" && Boolean(map);

  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const observer = new ResizeObserver(([entry]) => {
      setSize({ W: entry.contentRect.width, H: entry.contentRect.height });
    });
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const onWheel = (ev: WheelEvent) => {
      ev.preventDefault();
      setZoom((z) => Math.min(12, Math.max(0.5, z * (ev.deltaY > 0 ? 1 / 1.15 : 1.15))));
    };
    el.addEventListener("wheel", onWheel, { passive: false });
    return () => el.removeEventListener("wheel", onWheel);
  }, []);

  // The view: the arena (GT map extent, else the cost map) or a window following the rover.
  let view: View | null = null;
  const arenaMap = maps.gt ?? maps.obstacle ?? map;
  if (arenaMap && size.W > 0 && size.H > 0) {
    const b = mapBounds(arenaMap);
    const follow = viewMode === "follow" && robot?.pose;
    const span = (follow ? FOLLOW_SPAN_M : Math.max(b.x1 - b.x0, b.y1 - b.y0) * 1.04) / zoom;
    view = {
      cx: follow ? robot!.pose!.x : (b.x0 + b.x1) / 2,
      cy: follow ? robot!.pose!.y : (b.y0 + b.y1) / 2,
      scale: Math.min(size.W, size.H) / span,
      W: size.W,
      H: size.H,
    };
  }

  async function sendTeleopGoal(ev: React.MouseEvent<HTMLCanvasElement>) {
    if (!teleopClick || !teleopAvailable || !view) return;
    const rect = ev.currentTarget.getBoundingClientRect();
    const [x, y] = canvasToWorld(view, ev.clientX - rect.left, ev.clientY - rect.top);
    const yaw = robot?.pose?.yaw ?? 0;
    setTeleopStatus(`sending ${x.toFixed(1)}, ${y.toFixed(1)}`);
    try {
      const resp = await fetch(`/api/robots/${robot?.name ?? "astrobot_0"}/teleop_goal`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ x, y, yaw, frame_id: "map" }),
      });
      if (!resp.ok) {
        const body = await resp.text();
        throw new Error(body || `HTTP ${resp.status}`);
      }
      setTeleopStatus(`sent ${x.toFixed(1)}, ${y.toFixed(1)}`);
    } catch (err) {
      setTeleopStatus(`send failed`);
      console.error("teleop goal failed", err);
    }
  }

  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");
    if (!canvas || !ctx || !map || !view) return;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.floor(view.W * dpr);
    canvas.height = Math.floor(view.H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#050a12";
    ctx.fillRect(0, 0, view.W, view.H);

    drawGrid(ctx, view, map, gridImage(map, activeLayer));
    // Ground-truth rocks blended over the active layer, for visual alignment/recall checks.
    if (showGtOverlay && gtRocks?.length) drawGtRocks(ctx, view, gtRocks);
    else if (showGtOverlay && maps.gt && activeLayer !== "gt") drawGrid(ctx, view, maps.gt, gridImage(maps.gt, "gt-overlay"));
    drawMetricGrid(ctx, view);
    const home = missionDetails as { home_x?: number | null; home_y?: number | null; home_radius_m?: number };
    if (showCandidates && missionMode === "explore_radius" && typeof home.home_x === "number" && typeof home.home_y === "number" && home.home_radius_m)
      drawMissionRadius(ctx, view, home.home_x, home.home_y, home.home_radius_m);

    robots.forEach((r) => {
      if (showGtPose && r.gtTrajectory) drawPolyline(ctx, view!, r.gtTrajectory, "rgba(251,191,36,0.55)", 1.5, [1, 3]);
      if (showTrajectory) drawPolyline(ctx, view!, r.trajectory, "rgba(94,234,212,0.55)", 2, [2, 4]);
      if (showPlan) drawPolyline(ctx, view!, r.plan, "rgba(232,121,249,0.9)", 2.5, [8, 7]);
      if (showCandidates && r.candidates) drawCandidates(ctx, view!, r.candidates);

      if (r.goal) {
        const [gx, gy] = worldToCanvas(view!, r.goal.x, r.goal.y);
        ctx.strokeStyle = "#e879f9";
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(gx, gy, 14, 0, Math.PI * 2); ctx.stroke();
        ctx.beginPath(); ctx.arc(gx, gy, 5, 0, Math.PI * 2); ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(gx - 20, gy); ctx.lineTo(gx - 8, gy);
        ctx.moveTo(gx + 8, gy); ctx.lineTo(gx + 20, gy);
        ctx.moveTo(gx, gy - 20); ctx.lineTo(gx, gy - 8);
        ctx.moveTo(gx, gy + 8); ctx.lineTo(gx, gy + 20);
        ctx.stroke();
        ctx.fillStyle = "#e879f9";
        ctx.font = "10px 'JetBrains Mono', monospace";
        ctx.fillText("GOAL", gx + 18, gy - 12);
        ctx.fillStyle = "rgba(192,132,252,0.8)";
        ctx.fillText(`${r.goal.x.toFixed(1)}, ${r.goal.y.toFixed(1)}`, gx + 18, gy);
      }

      const fp = r.footprint ?? { halfLength: 0.495, halfWidth: 0.335 };
      if (showGtPose && r.gtPose) drawRover(ctx, view!, r.gtPose, fp, true);
      if (r.pose) {
        drawRover(ctx, view!, r.pose, fp, false);
        const [rx, ry] = worldToCanvas(view!, r.pose.x, r.pose.y);
        ctx.fillStyle = "rgba(94,234,212,0.9)";
        ctx.font = "10px 'JetBrains Mono', monospace";
        ctx.fillText(r.name, rx + Math.max(14, fp.halfLength * view!.scale + 6), ry + 4);
      }
    });
  }, [activeLayer, map, maps, robots, gtRocks, missionDetails, missionMode, showPlan, showTrajectory, showGtOverlay, showGtPose, showCandidates, view?.cx, view?.cy, view?.scale, view?.W, view?.H]);

  // Map stamps are ROS (sim) time when the backend reports a sim clock, wall time in mock mode.
  const age = map?.stamp ? (simTime ?? Date.now() / 1000) - map.stamp : undefined;
  const mapMeta = map
    ? `${map.width} × ${map.height} · ${map.resolution.toFixed(2)} m/cell · age ${age === undefined ? "–" : age < 1 ? `${Math.round(Math.max(0, age) * 1000)} ms` : `${age.toFixed(1)} s`}`
    : "no map data";
  const scaleMeters = view ? niceScaleMeters(view.scale) : 0;
  const legend = LEGENDS[activeLayer === "explore" || activeLayer === "mission" ? activeLayer : "cost"];

  return (
    <div className="panel panel-costmap">
      <div className="costmap" ref={wrapRef}>
        {map ? (
          <canvas
            ref={canvasRef}
            className={`bev-canvas${teleopClick && teleopAvailable ? " is-teleop-click" : ""}`}
            onClick={sendTeleopGoal}
          />
        ) : (
          <div className="costmap-empty">waiting for map</div>
        )}

        <div className="costmap-label">
          <div className="costmap-label-title">{LAYER_TITLES[activeLayer] ?? activeLayer}</div>
          <div className="costmap-label-meta">{mapMeta}</div>
        </div>

        <div className={`costmap-display${displayOpen ? "" : " is-collapsed"}`}>
          <button className="costmap-display-hdr" onClick={() => setDisplayOpen((v) => !v)}>
            <span className="costmap-display-title">Display</span>
            <span className="costmap-display-chevron">{displayOpen ? "–" : "+"}</span>
          </button>
          {displayOpen && (
            <div className="costmap-display-body">
              <div className="costmap-display-tabs">
                {LAYER_ORDER.filter((k) => maps[k] || k === activeLayer).map((key) => (
                  <div
                    key={key}
                    className={`costmap-display-tab${key === activeLayer ? " is-active" : ""}`}
                    onClick={() => setActiveLayer(key)}
                  >
                    {LAYER_LABELS[key] ?? key}
                  </div>
                ))}
              </div>
              <div className="costmap-display-tabs costmap-view-tabs">
                {(["arena", "follow"] as const).map((mode) => (
                  <div
                    key={mode}
                    className={`costmap-display-tab${mode === viewMode ? " is-active" : ""}`}
                    onClick={() => { setViewMode(mode); setZoom(1); }}
                  >
                    {mode === "arena" ? "Arena" : "Follow"}
                  </div>
                ))}
              </div>
              <div className="costmap-display-checks">
                <label className="costmap-display-check">
                  <input type="checkbox" checked={showGtOverlay} onChange={(e) => setShowGtOverlay(e.target.checked)} />
                  GT rocks
                </label>
                <label className="costmap-display-check">
                  <input type="checkbox" checked={showGtPose} onChange={(e) => setShowGtPose(e.target.checked)} />
                  GT pose + trail
                </label>
                <label className="costmap-display-check">
                  <input type="checkbox" checked={showCandidates} onChange={(e) => setShowCandidates(e.target.checked)} />
                  Goal candidates
                </label>
                <label className="costmap-display-check">
                  <input type="checkbox" checked={showTrajectory} onChange={(e) => setShowTrajectory(e.target.checked)} />
                  Trajectory
                </label>
                <label className="costmap-display-check">
                  <input type="checkbox" checked={showPlan} onChange={(e) => setShowPlan(e.target.checked)} />
                  Plan
                </label>
                <label className="costmap-display-check">
                  <input
                    type="checkbox"
                    checked={teleopClick}
                    disabled={!teleopAvailable}
                    onChange={(e) => {
                      setTeleopClick(e.target.checked);
                      setTeleopStatus(e.target.checked ? "click map to command" : "");
                    }}
                  />
                  Teleop click
                </label>
                <div className={`costmap-teleop-status${teleopAvailable ? "" : " is-disabled"}`}>
                  {teleopAvailable ? teleopStatus || "teleop ready" : "teleop requires teleop_click mission"}
                </div>
              </div>
            </div>
          )}
        </div>

        <div className="costmap-legend">
          {legend.map((item) => (
            <div className="costmap-legend-item" key={item.label}>
              <span className="costmap-legend-swatch" style={{ background: item.background, border: item.border }} />
              {item.label}
            </div>
          ))}
          <div className="costmap-legend-item">
            <span className="costmap-legend-swatch" style={{ background: "#e879f9", height: 2, marginTop: 3 }} />
            plan
          </div>
          <div className="costmap-legend-item">
            <span className="costmap-legend-swatch" style={{ background: "#5eead4", height: 2, marginTop: 3 }} />
            trail
          </div>
          {showGtPose && (
            <div className="costmap-legend-item">
              <span className="costmap-legend-swatch" style={{ background: "#fbbf24", height: 2, marginTop: 3 }} />
              GT
            </div>
          )}
          {showGtOverlay && (
            <div className="costmap-legend-item">
              <span className="costmap-legend-swatch" style={{ background: "rgba(244,114,182,0.5)", border: "1px solid #f472b6", borderRadius: "50%" }} />
              GT rock
            </div>
          )}
          {showCandidates && (robot?.candidates?.length ?? 0) > 0 && (
            <div className="costmap-legend-item">
              <span className="costmap-legend-swatch" style={{ background: "#93c5fd", width: 6, height: 6, borderRadius: "50%", marginTop: 2 }} />
              next goals
            </div>
          )}
        </div>

        {view && (
          <div className="costmap-scale">
            0
            <span className="costmap-scale-bar" style={{ width: scaleMeters * view.scale }} />
            {scaleMeters} m
          </div>
        )}
      </div>
    </div>
  );
}

// ── CameraPanel ───────────────────────────────────────────────────────────────

function MockCamFeed({ scene }: { scene: 0 | 1 }) {
  return (
    <svg className="cam-mock-svg" viewBox="0 0 320 180" preserveAspectRatio="xMidYMid slice">
      <defs>
        <linearGradient id={`sky${scene}`} x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%"   stopColor="#0a1828" />
          <stop offset="55%"  stopColor="#162a3f" />
          <stop offset="100%" stopColor="#0a1828" />
        </linearGradient>
        <linearGradient id={`floor${scene}`} x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%"   stopColor="#1a2c3e" />
          <stop offset="100%" stopColor="#06101a" />
        </linearGradient>
      </defs>
      <rect x="0" y="0" width="320" height="180" fill={`url(#sky${scene})`} />
      <line x1="0" y1="108" x2="320" y2="108" stroke="#5eead4" strokeWidth="0.3" opacity="0.3" />
      <rect x="0" y="108" width="320" height="72" fill={`url(#floor${scene})`} />
      {scene === 0 ? (
        <>
          <polygon points="0,108 60,88 140,100 220,82 320,92 320,108" fill="#0b2030" />
          <rect x="44" y="118" width="30" height="22" rx="2" fill="#1f4a6e" stroke="#5eead4" strokeWidth="0.5" />
          <rect x="205" y="114" width="40" height="28" rx="2" fill="#2a5a86" stroke="#5eead4" strokeWidth="0.5" />
          <g>
            <rect x="118" y="96" width="64" height="40" fill="none" stroke="#5eead4" strokeWidth="0.9" strokeDasharray="3 2" />
            <text x="120" y="94" fontFamily="monospace" fontSize="5.5" fill="#5eead4">rock · 0.91</text>
          </g>
          <g stroke="#5eead4" strokeWidth="0.25" opacity="0.15">
            <line x1="160" y1="108" x2="40"  y2="180" />
            <line x1="160" y1="108" x2="110" y2="180" />
            <line x1="160" y1="108" x2="160" y2="180" />
            <line x1="160" y1="108" x2="210" y2="180" />
            <line x1="160" y1="108" x2="280" y2="180" />
            <line x1="0" y1="130" x2="320" y2="130" />
            <line x1="0" y1="155" x2="320" y2="155" />
          </g>
        </>
      ) : (
        <>
          <polygon points="0,108 80,92 200,100 320,86 320,108" fill="#0c2a40" />
          <rect x="64" y="118" width="24" height="20" rx="2" fill="#2a5a86" stroke="#5eead4" strokeWidth="0.5" />
          <rect x="224" y="112" width="46" height="30" rx="2" fill="#1f4a6e" stroke="#5eead4" strokeWidth="0.5" />
          <g stroke="#5eead4" strokeWidth="0.25" opacity="0.15">
            <line x1="160" y1="108" x2="50"  y2="180" />
            <line x1="160" y1="108" x2="120" y2="180" />
            <line x1="160" y1="108" x2="170" y2="180" />
            <line x1="160" y1="108" x2="220" y2="180" />
            <line x1="0" y1="132" x2="320" y2="132" />
            <line x1="0" y1="158" x2="320" y2="158" />
          </g>
        </>
      )}
      <g stroke="#5eead4" strokeWidth="0.5" opacity="0.4">
        <line x1="155" y1="90" x2="165" y2="90" />
        <line x1="160" y1="85" x2="160" y2="95" />
      </g>
    </svg>
  );
}

function CameraPanel({
  index,
  cameraTopics,
  topics,
  connected,
  now,
}: {
  index: number;
  cameraTopics: string[];
  topics: TelemetryFrame["topics"];
  connected: boolean;
  now: number;
}) {
  const allTopics = cameraTopics.length > 0 ? cameraTopics : DUMMY_CAM_TOPICS;
  const [selectedIdx, setSelectedIdx] = useState(index < allTopics.length ? index : 0);
  const [menuOpen, setMenuOpen] = useState(false);
  const [imageOk, setImageOk] = useState(true);
  const [tick, setTick] = useState(0);
  const menuRef = useRef<HTMLDivElement | null>(null);

  const selectedTopic = allTopics[selectedIdx] ?? allTopics[0] ?? "";
  const topicKey = selectedTopic.replace(/^\/astrobot_0\//, "").replace(/^\//, "");
  const freshness = topics[topicKey] ?? topics[selectedTopic.replace(/^\//, "")];
  const status = topicStatus(freshness?.stamp, now);

  const cameraKey = topicKey.split("/", 1)[0] ?? `camera_${index}`;
  const snapshotUrl = `/api/cameras/${cameraKey}/snapshot.jpg?max_width=720&quality=70&t=${tick}`;

  useEffect(() => {
    const timer = window.setInterval(() => setTick((v) => v + 1), 500);
    return () => clearInterval(timer);
  }, []);

  useEffect(() => { setImageOk(true); }, [selectedTopic]);

  useEffect(() => {
    if (!menuOpen) return;
    function handler(e: MouseEvent) {
      if (menuRef.current && !menuRef.current.contains(e.target as Node)) setMenuOpen(false);
    }
    document.addEventListener("mousedown", handler);
    return () => document.removeEventListener("mousedown", handler);
  }, [menuOpen]);

  const shortTopic = selectedTopic.split("/").slice(-2).join("/");
  const showReal = connected && imageOk;

  return (
    <div className={`panel panel-cam-${index + 1}`}>
      <div className="cam-topicbar" ref={menuRef}>
        <span className="cam-topic-icon">
          <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.75">
            <path d="M14.5 4h-5L7 7H4a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V9a2 2 0 0 0-2-2h-3l-2.5-3Z"/>
            <circle cx="12" cy="13" r="4"/>
          </svg>
        </span>
        <button
          className={`cam-topic-select${menuOpen ? " is-open" : ""}`}
          onClick={() => setMenuOpen((v) => !v)}
        >
          <span className="cam-topic-text">{shortTopic || selectedTopic}</span>
          <svg className="cam-topic-caret" width="10" height="10" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.6">
            <path d="M3 4.5 6 7.5 9 4.5"/>
          </svg>
        </button>
        <span className={`cam-tag ${status}`}>
          {status === "ok" ? (freshness ? `${freshness.hz?.toFixed(0)} Hz` : "ok") : status}
        </span>

        {menuOpen && (
          <div className="cam-topic-menu">
            <div className="cam-topic-menu-hdr">{allTopics.length} topics</div>
            {allTopics.map((t, i) => {
              const tk = t.replace(/^\/astrobot_0\//, "").replace(/^\//, "");
              const fs = topics[tk] ?? topics[t.replace(/^\//, "")];
              const s = topicStatus(fs?.stamp, now);
              const shortT = t.split("/").slice(-2).join("/");
              return (
                <button
                  key={t}
                  className={`cam-topic-opt${i === selectedIdx ? " is-sel" : ""}`}
                  onClick={() => { setSelectedIdx(i); setMenuOpen(false); setImageOk(true); }}
                >
                  <span className={`cam-topic-opt-dot ${s}`} />
                  <span className="cam-topic-opt-body">
                    <span className="cam-topic-opt-label">{shortT}</span>
                    <span className="cam-topic-opt-path">{t}</span>
                  </span>
                  {i === selectedIdx && (
                    <svg className="cam-topic-opt-check" width="12" height="12" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="2">
                      <path d="M2.5 6.5 5 9l4.5-5"/>
                    </svg>
                  )}
                </button>
              );
            })}
          </div>
        )}
      </div>

      <div className="cam-feed">
        {showReal ? (
          <img
            className="cam-feed-img"
            src={snapshotUrl}
            alt={`Camera ${index + 1}`}
            onError={() => setImageOk(false)}
          />
        ) : status === "err" ? (
          <div className="cam-nodata"><div className="cam-nodata-msg">no signal</div></div>
        ) : (
          <MockCamFeed scene={(index % 2) as 0 | 1} />
        )}
        <div className="cam-feed-overlay">
          <div className="cam-feed-corner tl"><em>cam</em> {index === 0 ? "front" : "rear"}</div>
          <div className="cam-feed-corner tr"><em>res</em> 1280×720</div>
          <div className="cam-feed-corner bl"><em>exposure</em> auto</div>
          {status === "ok" && <div className="cam-feed-corner br">● {connected ? "LIVE" : "MOCK"}</div>}
        </div>
      </div>
    </div>
  );
}

// ── Stack status panels (v2: supervisor + terrain publisher) ─────────────────

function StatusKv({ label, value, unit, cls }: { label: string; value: React.ReactNode; unit?: string; cls?: string }) {
  return (
    <div className="robot-kv">
      <div className="robot-kv-lbl">{label}</div>
      <div className={`robot-kv-val${cls ? " " + cls : ""}`}>
        {value}
        {unit && <span className="unit">{unit}</span>}
      </div>
    </div>
  );
}

function LocalizationPanel({ frame }: { frame: TelemetryFrame }) {
  const loc = frame.localization;
  const now = Date.now() / 1000;
  const cls = !loc ? "warn" : loc.motionPermitted ? "ok" : loc.state === "faulted" ? "err" : "warn";
  return (
    <div className="panel panel-status">
      <div className="panel-body robot-panel">
        <div className="robot-name-row">
          <div className="robot-name">Localization</div>
          <div className={`robot-status-badge ${cls}`}>{loc ? loc.state.toUpperCase() : "N/A"}</div>
        </div>
        <div className="robot-task">{loc ? loc.reason : "no supervisor status (legacy stack or not started)"}</div>
        <div className="robot-kv-grid">
          <StatusKv label="Motion" value={loc ? (loc.motionPermitted ? "permitted" : "held") : "–"} cls={loc && !loc.motionPermitted ? "warn" : undefined} />
          <StatusKv label="GLIM epoch" value={loc ? loc.epoch : "–"} />
          <StatusKv label="Status age" value={ageLabel(loc?.stamp, now)} />
          <StatusKv label="Sim speed" value={frame.simSpeed ? frame.simSpeed.toFixed(2) : "–"} unit={frame.simSpeed ? "×" : undefined} />
        </div>
      </div>
    </div>
  );
}

function TerrainPanel({ frame }: { frame: TelemetryFrame }) {
  const t = frame.terrain;
  const now = Date.now() / 1000;
  const cls = !t ? "warn" : t.valid ? "ok" : "err";
  const lag = t?.evidenceLagSec;
  return (
    <div className="panel panel-status">
      <div className="panel-body robot-panel">
        <div className="robot-name-row">
          <div className="robot-name">Terrain map</div>
          <div className={`robot-status-badge ${cls}`}>{!t ? "N/A" : t.valid ? "VALID" : "INVALID"}</div>
        </div>
        <div className="robot-task">
          {!t ? "no v2 terrain status" : t.failure ? t.failure : `epoch ${t.epoch ?? "–"} · ${t.projected} windows · status ${ageLabel(t.stamp, now)}`}
        </div>
        <div className="robot-kv-grid">
          <StatusKv label="Chunks in" value={t ? t.received : "–"} />
          <StatusKv label="Skipped / late" value={t ? `${t.skippedSequences} / ${t.lateChunks}` : "–"} cls={t && t.skippedSequences + t.lateChunks > 0 ? "warn" : undefined} />
          <StatusKv label="Evidence lag" value={lag != null ? lag.toFixed(2) : "–"} unit={lag != null ? "s" : undefined} cls={lag != null && lag > 0.5 ? "warn" : undefined} />
          <StatusKv label="Peak RSS" value={t?.peakRssMib != null ? Math.round(t.peakRssMib) : "–"} unit={t?.peakRssMib != null ? "MiB" : undefined} />
        </div>
      </div>
    </div>
  );
}

// ── RobotStatusPanel ─────────────────────────────────────────────────────────

function RobotStatusPanel({ robot, frame }: { robot: RobotState | undefined; frame: TelemetryFrame }) {
  if (!robot) return <div className="panel panel-robot"><div className="panel-body">No robot data</div></div>;
  const statusClass = severityToStatus(robot.severity);
  const statusLabel = robot.severity === "nominal" ? "OK" : robot.severity === "warning" ? "WARN" : "CRIT";

  const headingDeg = robot.pose ? Math.round((robot.pose.yaw * 180) / Math.PI) : null;

  // Derive a rough speed from the last two trajectory points
  let speed: string = "–";
  const traj = robot.trajectory;
  if (traj.length >= 2) {
    const last = traj[traj.length - 1];
    const prev = traj[traj.length - 2];
    const dt = (last.stamp ?? 0) - (prev.stamp ?? 0);
    if (dt > 0) {
      const dist = Math.hypot(last.x - prev.x, last.y - prev.y);
      speed = (dist / dt).toFixed(2);
    }
  }

  const topics = frame.topics;
  const slamFresh = topics["slam/pose_corrected"]?.stamp;
  const lidarFresh = topics["lidar_0/pointcloud/filtered"]?.stamp;
  const topicCount = Object.keys(topics).length;
  const slamError = robot.pose && robot.gtPose ? Math.hypot(robot.pose.x - robot.gtPose.x, robot.pose.y - robot.gtPose.y) : null;

  return (
    <div className="panel panel-robot">
      <div className="panel-body robot-panel">
        <div className="robot-name-row">
          <div className="robot-name">{robot.name}</div>
          <div className={`robot-status-badge ${statusClass}`}>{statusLabel}</div>
        </div>
        <div className="robot-task">{robot.statusText}</div>
        <div className="robot-kv-grid">
          <div className="robot-kv">
            <div className="robot-kv-lbl">Pose (x, y)</div>
            <div className="robot-kv-val">
              {robot.pose ? `${robot.pose.x.toFixed(1)}, ${robot.pose.y.toFixed(1)}` : "–"}
              {robot.pose && <span className="unit">m</span>}
            </div>
          </div>
          <div className="robot-kv">
            <div className="robot-kv-lbl">Goal (x, y)</div>
            <div className="robot-kv-val">
              {robot.goal ? `${robot.goal.x.toFixed(1)}, ${robot.goal.y.toFixed(1)}` : "none"}
              {robot.goal && <span className="unit">m</span>}
            </div>
          </div>
          <div className="robot-kv">
            <div className="robot-kv-lbl">Heading</div>
            <div className="robot-kv-val">{headingDeg !== null ? `${headingDeg}°` : "–"}</div>
          </div>
          <div className="robot-kv">
            <div className="robot-kv-lbl">Speed</div>
            <div className="robot-kv-val">{speed}<span className="unit">m/s</span></div>
          </div>
          <div className="robot-kv">
            <div className="robot-kv-lbl">{slamError !== null ? "SLAM err (GT)" : "Topics up"}</div>
            <div className="robot-kv-val">
              {slamError !== null ? slamError.toFixed(2) : topicCount}
              {slamError !== null && <span className="unit">m</span>}
            </div>
          </div>
          <div className="robot-kv">
            <div className="robot-kv-lbl">SLAM age</div>
            <div className="robot-kv-val">
              {ageLabel(slamFresh, Date.now() / 1000)}
              {lidarFresh && <span className="unit"> · lid {ageLabel(lidarFresh, Date.now() / 1000)}</span>}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

// ── MissionPanel ──────────────────────────────────────────────────────────────

function MissionPanel({ mission }: { mission: TelemetryFrame["mission"] }) {
  const elapsed = formatElapsed(mission.elapsedSec);
  const state = mission.returningHome ? "returning home" : mission.state.replace(/_/g, " ");
  const pulseClass = mission.failures > 3 ? "err" : mission.failures > 0 ? "warn" : "";

  return (
    <div className="panel panel-mission">
      <div className="panel-body mission-panel">
        <div className="mission-head">
          <div className="mission-name">{mission.mode.replace(/_/g, " ")}</div>
          <div className="mission-elapsed">{elapsed}<span className="unit">elapsed</span></div>
        </div>
        <div className="mission-state">
          <span className={`mission-pulse ${pulseClass}`} />
          {state}
        </div>
        <div className="mission-bar">
          <div className="mission-bar-fill" style={{ width: `${Math.max(0, Math.min(100, mission.coverageFraction * 100))}%` }} />
        </div>
        <div className="mission-kv-grid">
          <div className="mission-kv">
            <div className="mission-kv-lbl">Coverage</div>
            <div className="mission-kv-val">{Math.round(mission.coverageFraction * 100)}%</div>
          </div>
          <div className="mission-kv">
            <div className="mission-kv-lbl">Best sample</div>
            <div className="mission-kv-val">{mission.bestSample.toFixed(2)}</div>
          </div>
          <div className="mission-kv">
            <div className="mission-kv-lbl">Goals</div>
            <div className="mission-kv-val">{mission.successes}/{mission.goals}</div>
          </div>
          <div className="mission-kv">
            <div className="mission-kv-lbl">Failures</div>
            <div className={`mission-kv-val${mission.failures > 0 ? " warn" : ""}`}>{mission.failures}</div>
          </div>
        </div>
      </div>
    </div>
  );
}

// ── Sparkline ─────────────────────────────────────────────────────────────────

function Sparkline({ data, color = "#5eead4" }: { data: number[]; color?: string }) {
  const W = 200, H = 34;
  if (!data.length) return null;
  const min = Math.min(...data);
  const max = Math.max(...data);
  const range = max - min || 1;
  const step = W / Math.max(1, data.length - 1);
  const pts = data.map((v, i) => {
    const x = i * step;
    const y = H - 3 - ((v - min) / range) * (H - 6);
    return [x, y] as [number, number];
  });
  const line = pts.map(([x, y], i) => `${i === 0 ? "M" : "L"} ${x.toFixed(1)} ${y.toFixed(1)}`).join(" ");
  const fill = line + ` L ${W} ${H} L 0 ${H} Z`;
  const last = pts[pts.length - 1];
  const fillId = `sf-${color.replace(/[^a-z0-9]/gi, "")}`;
  return (
    <svg viewBox={`0 0 ${W} ${H}`} preserveAspectRatio="none" width="100%" height="100%">
      <defs>
        <linearGradient id={fillId} x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%"   stopColor={color} stopOpacity="0.18" />
          <stop offset="100%" stopColor={color} stopOpacity="0" />
        </linearGradient>
        <pattern id="sg" width="25" height="10" patternUnits="userSpaceOnUse">
          <path d="M 25 0 L 0 0 0 10" fill="none" stroke="#1a2638" strokeWidth="0.3" />
        </pattern>
      </defs>
      <rect x="0" y="0" width={W} height={H} fill="url(#sg)" />
      <path d={fill} fill={`url(#${fillId})`} />
      <path d={line} fill="none" stroke={color} strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
      {last && <circle cx={last[0] - 0.5} cy={last[1]} r="2" fill={color} />}
    </svg>
  );
}

function SparkRow({
  label,
  value,
  unit,
  fmt,
  data,
}: {
  label: string;
  value: number | undefined;
  unit?: string;
  fmt?: (v: number) => string;
  data: number[];
}) {
  const display =
    value === undefined ? "–"
    : fmt ? fmt(value)
    : value.toFixed(value >= 100 ? 0 : value >= 10 ? 1 : 2);
  return (
    <div className="spark">
      <div className="spark-row">
        <div className="spark-lbl">{label}</div>
        <div className="spark-val">
          {display}
          {unit && value !== undefined && <span className="unit">{unit}</span>}
        </div>
      </div>
      <div className="spark-chart">
        <Sparkline data={data} />
      </div>
    </div>
  );
}

// ── TopicFreqsPanel ──────────────────────────────────────────────────────────

const LEGACY_RATE_SERIES = ["LiDAR Hz", "IMU Hz", "SLAM pose Hz", "Voxel map Hz", "BEV map Hz", "Camera Hz"];

function TopicFreqsPanel({ series, rateSeries }: { series: Record<string, number[]>; rateSeries?: string[] }) {
  const keys = [...(rateSeries ?? LEGACY_RATE_SERIES), "Sim speed"].map((key) => ({
    key,
    unit: key === "Sim speed" ? "×" : "Hz",
  }));
  return (
    <div className="panel panel-freqs">
      <div className="panel-hdr">
        <div className="panel-hdr-title accent">Topic Frequencies</div>
        <div className="panel-hdr-meta">per sim second · last 3 min</div>
      </div>
      <div className="sparks-grid">
        {keys.map(({ key, unit }) => {
          const data = series[key] ?? [];
          const last = data[data.length - 1];
          return <SparkRow key={key} label={key} value={last} unit={unit} data={data} />;
        })}
      </div>
    </div>
  );
}

// ── AlgoPerfPanel ─────────────────────────────────────────────────────────────

const ALGO_PERF_KEYS = [
  { key: "SLAM error",   unit: "m"  },
  { key: "Goal distance", unit: "m" },
  { key: "Coverage",     unit: "",  fmt: (v: number) => `${(v * 100).toFixed(0)}%` },
  { key: "Resource",     unit: "",  fmt: (v: number) => v.toFixed(2) },
];

function AlgoPerfPanel({ series }: { series: Record<string, number[]> }) {
  return (
    <div className="panel panel-perf">
      <div className="panel-hdr">
        <div className="panel-hdr-title accent">Algorithm Performance</div>
        <div className="panel-hdr-meta">recent samples</div>
      </div>
      <div className="sparks-grid">
        {ALGO_PERF_KEYS.map(({ key, unit, fmt }) => {
          const data = series[key] ?? [];
          const last = data[data.length - 1];
          return <SparkRow key={key} label={key} value={last} unit={unit} fmt={fmt} data={data} />;
        })}
      </div>
    </div>
  );
}

// ── StatusFooter ──────────────────────────────────────────────────────────────

function StatusFooter({
  frame,
  showEvents,
  onToggleEvents,
}: {
  frame: TelemetryFrame;
  showEvents: boolean;
  onToggleEvents: () => void;
}) {
  const now = Date.now() / 1000;
  const connClass = frame.connected ? "ok" : "warn";
  const connLabel = frame.connected ? "live" : "mock";
  const topicCount = Object.keys(frame.topics).length;

  const slamTopic = frame.topics["slam/pose_corrected"];
  const tfAge = slamTopic ? ageLabel(slamTopic.stamp, now) : "–";
  const tfClass = slamTopic ? topicStatus(slamTopic.stamp, now) : "err";

  const latencyMs = frame.connected ? Math.round((Date.now() / 1000 - frame.stamp) * 1000) : null;

  return (
    <div className="status-bar">
      <div className="status-item">
        <span className="status-lbl">ROS</span>
        <span className={`status-val ${connClass}`}>
          {connLabel} · {topicCount} topics
        </span>
      </div>
      <div className="status-item">
        <span className="status-lbl">DDS</span>
        <span className="status-val ok">{frame.dds ? `domain ${frame.dds.domain} · ${frame.dds.rmw}` : "–"}</span>
      </div>
      <div className="status-item">
        <span className="status-lbl">TF</span>
        <span className={`status-val ${tfClass}`}>
          map → base_link · {tfAge}
        </span>
      </div>
      {latencyMs !== null && (
        <div className="status-item">
          <span className="status-lbl">Latency</span>
          <span className={`status-val ${latencyMs < 200 ? "ok" : "warn"}`}>{latencyMs} ms</span>
        </div>
      )}
      <div className="status-item" style={{ marginLeft: "auto" }}>
        <button onClick={onToggleEvents} style={{ display: "flex", alignItems: "center", gap: 6 }}>
          <span className="status-lbl">Events</span>
          <span className={`status-val ${frame.events.length > 0 ? "warn" : ""}`}>
            {frame.events.length} new {showEvents ? "▾" : "▴"}
          </span>
        </button>
      </div>
    </div>
  );
}

// ── EventsPanel ───────────────────────────────────────────────────────────────

function EventsPanel({ events, onClose }: { events: TelemetryFrame["events"]; onClose: () => void }) {
  return (
    <div className="events-panel">
      <div className="events-hdr">
        <div className="events-title">Event log · live</div>
        <button className="events-close" onClick={onClose}>×</button>
      </div>
      <div className="events-list">
        {events.length === 0 ? (
          <div style={{ padding: "12px", fontFamily: "var(--font-mono)", fontSize: 11, color: "var(--ink-3)" }}>
            No recent events
          </div>
        ) : (
          events.map((e, i) => (
            <div key={i} className="event-row">
              <div className="event-time">
                {new Date(e.stamp * 1000).toLocaleTimeString("en-US", { hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false })}
              </div>
              <div className={`event-dot ${e.severity}`} />
              <div className="event-body">
                {e.text}
                <span className="event-src">{e.source}</span>
              </div>
            </div>
          ))
        )}
      </div>
    </div>
  );
}

// ── App ───────────────────────────────────────────────────────────────────────

function App() {
  const [scenario, setScenario] = useState<MockScenario>("nominal");
  const frame = useTelemetry(scenario);
  const [activeLayer, setActiveLayer] = useState(URL_PARAMS.get("layer") ?? "obstacle");
  const [showTrajectory, setShowTrajectory] = useState(true);
  const [showPlan, setShowPlan] = useState(true);
  const [showEvents, setShowEvents] = useState(false);
  const [selectedRobotId, setSelectedRobotId] = useState("astrobot_0");

  const robot = frame.robots[0];

  // Build fleet entries: real robot first, then mock extras when disconnected
  const liveEntry: FleetEntry | undefined = robot
    ? {
        id: robot.name,
        name: robot.name,
        status: severityToStatus(robot.severity),
        task: robot.statusText,
        battery: frame.connected ? -1 : 0.74, // not in the backend schema: hidden when live
      }
    : undefined;
  const fleetEntries: FleetEntry[] = liveEntry
    ? (frame.connected ? [liveEntry] : [liveEntry, ...MOCK_FLEET_EXTRA])
    : MOCK_FLEET_EXTRA;
  const fleetOnline = fleetEntries.filter((r) => r.status !== "off").length;

  // Banner status
  const bannerSeverity = robot?.severity ?? "unknown";
  const bannerTag = bannerSeverity === "nominal" ? "NAV" : bannerSeverity === "warning" ? "WARN" : "CRIT";
  const bannerClass = bannerSeverity === "warning" ? "warn" : bannerSeverity === "critical" ? "err" : "";
  const bannerText = robot?.statusText ?? "waiting for robot";

  // Map metadata for banner
  const obstacleMap = frame.maps.obstacle;
  const mapMeta = obstacleMap
    ? `${obstacleMap.resolution.toFixed(2)} m/cell · ${obstacleMap.width}×${obstacleMap.height}`
    : null;

  return (
    <div className="app">
      <Header
        frame={frame}
        fleetTotal={fleetEntries.length}
        fleetOnline={fleetOnline}
        scenario={scenario}
        setScenario={setScenario}
      />
      <FleetSidebar entries={fleetEntries} selectedId={selectedRobotId} onSelect={setSelectedRobotId} />
      <div className="main">
        {/* Banner */}
        <div className="banner">
          <span className={`banner-tag ${bannerClass}`}>{bannerTag}</span>
          <span className="banner-msg">
            <strong>{bannerSeverity === "nominal" ? "Navigation nominal" : bannerText}</strong>
            {bannerSeverity === "nominal" && (
              <span className="muted"> · {bannerText}</span>
            )}
          </span>
          {mapMeta && (
            <span className="banner-meta">
              <span>costmap {mapMeta}</span>
            </span>
          )}
        </div>

        {/* Content row: square BEV + right panels */}
        <div className="main-content">
          <BevMapPanel
            maps={frame.maps}
            robots={frame.robots}
            connected={frame.connected}
            missionMode={frame.mission.mode}
            missionDetails={frame.mission.details}
            gtRocks={frame.gtRocks}
            simTime={frame.simTime}
            activeLayer={activeLayer}
            setActiveLayer={setActiveLayer}
            showTrajectory={showTrajectory}
            setShowTrajectory={setShowTrajectory}
            showPlan={showPlan}
            setShowPlan={setShowPlan}
          />
          <div className="right-col">
            <div className="cam-row">
              {/* Live runs without a camera (v2 sim nav stream) show the stack status instead; with
                  one (v2_demo.sh --camera) the camera sits beside the localization supervisor. */}
              {frame.connected && frame.cameraTopics.length === 0 ? (
                <>
                  <LocalizationPanel frame={frame} />
                  <TerrainPanel frame={frame} />
                </>
              ) : frame.connected && frame.localization ? (
                <>
                  <CameraPanel index={0} cameraTopics={frame.cameraTopics} topics={frame.topics} connected={frame.connected} now={frame.stamp} />
                  <LocalizationPanel frame={frame} />
                </>
              ) : (
                <>
                  <CameraPanel index={0} cameraTopics={frame.cameraTopics} topics={frame.topics} connected={frame.connected} now={frame.stamp} />
                  <CameraPanel index={1} cameraTopics={frame.cameraTopics} topics={frame.topics} connected={frame.connected} now={frame.stamp} />
                </>
              )}
            </div>
            <div className="info-row">
              <RobotStatusPanel robot={robot} frame={frame} />
              <MissionPanel mission={frame.mission} />
            </div>
            <AlgoPerfPanel series={frame.series} />
          </div>
        </div>

        {/* Sparklines */}
        <TopicFreqsPanel series={frame.series} rateSeries={frame.rateSeries} />

        {/* Events popover */}
        {showEvents && <EventsPanel events={frame.events} onClose={() => setShowEvents(false)} />}
      </div>

      <StatusFooter frame={frame} showEvents={showEvents} onToggleEvents={() => setShowEvents((v) => !v)} />
    </div>
  );
}

createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);
