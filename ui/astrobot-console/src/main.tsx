import React, { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import type { GridMap, RobotState, Severity, TelemetryFrame } from "./types";
import { makeMockTelemetry, MOCK_FLEET_EXTRA, type FleetEntry, type MockScenario } from "./mockTelemetry";
import "./styles.css";

// ── Constants ────────────────────────────────────────────────────────────────

const LAYER_LABELS: Record<string, string> = {
  obstacle: "Obstacle",
  water: "Water",
  mission: "Grid",
  gt: "GT",
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

function worldToCanvas(map: GridMap, x: number, y: number, w: number, h: number): [number, number] {
  const dx = x - map.origin.x;
  const dy = y - map.origin.y;
  const cos = Math.cos(map.origin.yaw);
  const sin = Math.sin(map.origin.yaw);
  const lx = cos * dx + sin * dy;
  const ly = -sin * dx + cos * dy;
  return [(lx / (map.resolution * map.width)) * w, h - (ly / (map.resolution * map.height)) * h];
}

function canvasToWorld(map: GridMap, px: number, py: number, w: number, h: number): [number, number] {
  const spanX = map.resolution * map.width;
  const spanY = map.resolution * map.height;
  const lx = (px / w) * spanX;
  const ly = (1 - py / h) * spanY;
  const cos = Math.cos(map.origin.yaw);
  const sin = Math.sin(map.origin.yaw);
  const dx = cos * lx - sin * ly;
  const dy = sin * lx + cos * ly;
  return [map.origin.x + dx, map.origin.y + dy];
}

function colorForCell(value: number, layer: string): string {
  if (value < 0) return "rgba(10,18,28,0.7)";
  const v = Math.max(0, Math.min(100, value)) / 100;
  if (layer === "water") {
    return `rgba(${Math.round(30 + v * 50)}, ${Math.round(120 + v * 115)}, ${Math.round(200 + v * 55)}, ${0.06 + v * 0.82})`;
  }
  if (layer === "mission") {
    return v > 0.6 ? "rgba(94,234,212,0.30)" : v > 0.3 ? "rgba(96,165,250,0.18)" : "rgba(42,65,90,0.30)";
  }
  if (layer === "gt") {
    return `rgba(${Math.round(160 + v * 60)}, ${Math.round(20 + v * 60)}, ${Math.round(200 + v * 30)}, ${0.06 + v * 0.72})`;
  }
  // obstacle: dark slate → purple → magenta
  const r = Math.round(60  + v * 168);
  const g = Math.round(40  + v * 57);
  const b = Math.round(100 + v * 149);
  return `rgba(${r},${g},${b},${0.12 + v * 0.78})`;
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
    socket.onmessage = (ev) => {
      const parsed = JSON.parse(ev.data) as TelemetryFrame;
      setFrame((prev) => ({ ...parsed, maps: { ...prev.maps, ...parsed.maps } }));
    };
    socket.onerror = () => setFrame({ ...makeMockTelemetry(scenario), connected: false });
    socket.onclose = () => setFrame((prev) => (prev.connected ? { ...prev, connected: false } : prev));
    return () => socket.close();
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
        <span className="hdr-mission-meta">{elapsed} elapsed</span>
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
              <div className="fleet-row-batt-num">{Math.round(r.battery * 100)}%</div>
              <div className="fleet-row-batt-bar">
                <div
                  className={`fleet-row-batt-fill ${r.battery < 0.2 ? "crit" : r.battery < 0.4 ? "low" : ""}`}
                  style={{ width: `${r.battery * 100}%` }}
                />
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

// ── BevMapPanel ───────────────────────────────────────────────────────────────

function BevMapPanel({
  maps,
  robots,
  connected,
  missionMode,
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
  activeLayer: string;
  setActiveLayer: (l: string) => void;
  showTrajectory: boolean;
  setShowTrajectory: (v: boolean) => void;
  showPlan: boolean;
  setShowPlan: (v: boolean) => void;
}) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [displayOpen, setDisplayOpen] = useState(true);
  const [showGtOverlay, setShowGtOverlay] = useState(false);
  const [teleopClick, setTeleopClick] = useState(false);
  const [teleopStatus, setTeleopStatus] = useState("");
  const map = maps[activeLayer] ?? maps.obstacle ?? Object.values(maps)[0];
  const now = Date.now() / 1000;
  const teleopAvailable = connected && missionMode === "teleop_click" && Boolean(maps.obstacle);

  async function sendTeleopGoal(ev: React.MouseEvent<HTMLCanvasElement>) {
    if (!teleopClick || !teleopAvailable || !maps.obstacle) return;
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const xCanvas = ev.clientX - rect.left;
    const yCanvas = ev.clientY - rect.top;
    const [x, y] = canvasToWorld(maps.obstacle, xCanvas, yCanvas, rect.width, rect.height);
    const robot = robots[0];
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

  // Render OccupancyGrid to canvas
  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");
    if (!canvas || !ctx || !map) return;
    const rect = canvas.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.floor(rect.width * dpr);
    canvas.height = Math.floor(rect.height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = rect.width, H = rect.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#050a12";
    ctx.fillRect(0, 0, W, H);

    // Use the obstacle BEV map as the reference canvas coordinate frame so all
    // layers render at the same world position and scale when toggled.
    const refMap = maps.obstacle ?? map;
    const refW = refMap.resolution * refMap.width;
    const refH = refMap.resolution * refMap.height;

    // Draw grid cells positioned in the refMap coordinate frame
    const cellW = (map.resolution / refW) * W;
    const cellH = (map.resolution / refH) * H;
    const baseX = ((map.origin.x - refMap.origin.x) / refW) * W;
    // topRef: canvas y of world y = map.origin.y (row 0 bottom edge in world).
    // For row r: top of cell in canvas = topRef - (r+1)*cellH  (world +y → canvas -y)
    const topRef = H - ((map.origin.y - refMap.origin.y) / refH) * H;
    for (let row = 0; row < map.height; row++) {
      const cy = topRef - (row + 1) * cellH;
      for (let col = 0; col < map.width; col++) {
        const v = map.data[row * map.width + col] ?? -1;
        ctx.fillStyle = colorForCell(v, activeLayer);
        ctx.fillRect(baseX + col * cellW, cy, Math.ceil(cellW), Math.ceil(cellH));
      }
    }

    // Ground-truth overlay: GT rocks blended semi-transparent over the active
    // layer (same refMap positioning math), for visual alignment/recall checks.
    if (showGtOverlay && maps.gt && activeLayer !== "gt") {
      const gt = maps.gt;
      const gCellW = (gt.resolution / refW) * W;
      const gCellH = (gt.resolution / refH) * H;
      const gBaseX = ((gt.origin.x - refMap.origin.x) / refW) * W;
      const gTopRef = H - ((gt.origin.y - refMap.origin.y) / refH) * H;
      ctx.fillStyle = "rgba(244,114,182,0.45)";
      for (let row = 0; row < gt.height; row++) {
        const cy = gTopRef - (row + 1) * gCellH;
        for (let col = 0; col < gt.width; col++) {
          const v = gt.data[row * gt.width + col] ?? -1;
          if (v >= 50) ctx.fillRect(gBaseX + col * gCellW, cy, Math.ceil(gCellW), Math.ceil(gCellH));
        }
      }
    }

    // Subtle grid lines
    ctx.strokeStyle = "rgba(94,234,212,0.07)";
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= 8; i++) {
      const x = (W * i) / 8, y = (H * i) / 8;
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    }

    robots.forEach((robot) => {
      if (showTrajectory && robot.trajectory.length > 1) {
        ctx.strokeStyle = "rgba(94,234,212,0.50)";
        ctx.lineWidth = 2;
        ctx.setLineDash([2, 4]);
        ctx.beginPath();
        robot.trajectory.forEach((p, i) => {
          const [x, y] = worldToCanvas(refMap, p.x, p.y, W, H);
          i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.setLineDash([]);
      }

      if (showPlan && robot.plan.length > 1) {
        ctx.strokeStyle = "rgba(232,121,249,0.9)";
        ctx.lineWidth = 2.5;
        ctx.setLineDash([8, 7]);
        ctx.beginPath();
        robot.plan.forEach((p, i) => {
          const [x, y] = worldToCanvas(refMap, p.x, p.y, W, H);
          i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.setLineDash([]);
      }

      if (robot.goal) {
        const [gx, gy] = worldToCanvas(refMap, robot.goal.x, robot.goal.y, W, H);
        ctx.strokeStyle = "#e879f9";
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(gx, gy, 18, 0, Math.PI * 2); ctx.stroke();
        ctx.beginPath(); ctx.arc(gx, gy, 6,  0, Math.PI * 2); ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(gx - 26, gy); ctx.lineTo(gx - 10, gy);
        ctx.moveTo(gx + 10, gy); ctx.lineTo(gx + 26, gy);
        ctx.moveTo(gx, gy - 26); ctx.lineTo(gx, gy - 10);
        ctx.moveTo(gx, gy + 10); ctx.lineTo(gx, gy + 26);
        ctx.stroke();
        ctx.fillStyle = "#e879f9";
        ctx.beginPath(); ctx.arc(gx, gy, 2, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = "#e879f9";
        ctx.font = "10px 'JetBrains Mono', monospace";
        ctx.fillText("GOAL", gx + 22, gy - 14);
        ctx.fillStyle = "rgba(192,132,252,0.8)";
        ctx.fillText(`${robot.goal.x.toFixed(1)}, ${robot.goal.y.toFixed(1)}`, gx + 22, gy - 2);
      }

      if (robot.pose) {
        const [rx, ry] = worldToCanvas(refMap, robot.pose.x, robot.pose.y, W, H);
        ctx.save();
        ctx.translate(rx, ry);
        // Canvas y points down, and the rover wedge is drawn along +x.
        ctx.rotate(-(robot.pose.yaw - refMap.origin.yaw));
        // Body circle
        ctx.fillStyle = "rgba(5,10,18,0.75)";
        ctx.strokeStyle = "#5eead4";
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(0, 0, 11, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        // Forward indicator: filled wedge pointing right (= robot +x after rotation)
        ctx.fillStyle = "#5eead4";
        ctx.strokeStyle = "#050a12";
        ctx.lineWidth = 1.0;
        ctx.beginPath();
        ctx.moveTo(16, 0);          // nose
        ctx.lineTo(6, -6);          // top-rear
        ctx.lineTo(9, 0);           // rear-center notch
        ctx.lineTo(6, 6);           // bottom-rear
        ctx.closePath(); ctx.fill(); ctx.stroke();
        ctx.restore();
        ctx.fillStyle = "rgba(94,234,212,0.9)";
        ctx.font = "10px 'JetBrains Mono', monospace";
        ctx.fillText(robot.name, rx + 22, ry + 4);
      }
    });
  }, [activeLayer, map, maps, robots, showPlan, showTrajectory, showGtOverlay]);

  const mapMeta = map
    ? `${map.width} × ${map.height} · ${map.resolution.toFixed(2)} m/cell · age ${ageLabel(map.stamp, now)}`
    : "no map data";

  const scaleMeters = map ? (map.resolution * map.width * 0.1).toFixed(0) + " m" : "–";

  return (
    <div className="panel panel-costmap">
      <div className="costmap">
        {map ? (
          <canvas
            ref={canvasRef}
            className={`bev-canvas${teleopClick && teleopAvailable ? " is-teleop-click" : ""}`}
            onClick={sendTeleopGoal}
          />
        ) : (
          <div className="costmap-empty">waiting for map</div>
        )}

        {/* Label */}
        <div className="costmap-label">
          <div className="costmap-label-title">{LAYER_LABELS[activeLayer] ?? activeLayer} cost</div>
          <div className="costmap-label-meta">{mapMeta}</div>
        </div>

        {/* Display controls */}
        <div className={`costmap-display${displayOpen ? "" : " is-collapsed"}`}>
          <button className="costmap-display-hdr" onClick={() => setDisplayOpen((v) => !v)}>
            <span className="costmap-display-title">Display</span>
            <span className="costmap-display-chevron">{displayOpen ? "–" : "+"}</span>
          </button>
          {displayOpen && (
            <div className="costmap-display-body">
              <div className="costmap-display-tabs">
                {Object.keys({ obstacle: 1, water: 1, mission: 1, gt: 1 })
                  .filter((k) => maps[k] || k === activeLayer)
                  .map((key) => (
                    <div
                      key={key}
                      className={`costmap-display-tab${key === activeLayer ? " is-active" : ""}`}
                      onClick={() => setActiveLayer(key)}
                    >
                      {LAYER_LABELS[key] ?? key}
                    </div>
                  ))}
              </div>
              <div className="costmap-display-checks">
                <label className="costmap-display-check">
                  <input type="checkbox" checked={showGtOverlay} onChange={(e) => setShowGtOverlay(e.target.checked)} />
                  GT overlay
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

        {/* Legend */}
        <div className="costmap-legend">
          <div className="costmap-legend-item">
            <span className="costmap-legend-swatch" style={{ background: "linear-gradient(90deg, #3c2864, #6b4fa0)" }} />
            low
          </div>
          <div className="costmap-legend-item">
            <span className="costmap-legend-swatch" style={{ background: "linear-gradient(90deg, #6b4fa0, #a855f7)" }} />
            medium
          </div>
          <div className="costmap-legend-item">
            <span className="costmap-legend-swatch" style={{ background: "linear-gradient(90deg, #a855f7, #e879f9)" }} />
            high
          </div>
          <div className="costmap-legend-item">
            <span className="costmap-legend-swatch" style={{ background: "#e879f9", height: 2, marginTop: 3 }} />
            plan
          </div>
          <div className="costmap-legend-item">
            <span className="costmap-legend-swatch" style={{ background: "#5eead4", height: 2, marginTop: 3 }} />
            trail
          </div>
        </div>

        {/* Scale */}
        <div className="costmap-scale">
          0
          <span className="costmap-scale-bar" />
          {scaleMeters}
        </div>
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
            <div className="robot-kv-lbl">Topics up</div>
            <div className="robot-kv-val">{topicCount}</div>
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

const TOPIC_FREQ_KEYS = [
  { key: "PointCloud Hz", unit: "Hz" },
  { key: "IMU Hz",        unit: "Hz" },
  { key: "SLAM pose Hz",  unit: "Hz" },
  { key: "Voxel map Hz",  unit: "Hz" },
  { key: "BEV map Hz",    unit: "Hz" },
  { key: "Camera Hz",     unit: "Hz" },
];

function TopicFreqsPanel({ series }: { series: Record<string, number[]> }) {
  return (
    <div className="panel panel-freqs">
      <div className="panel-hdr">
        <div className="panel-hdr-title accent">Topic Frequencies</div>
        <div className="panel-hdr-meta">30 s window</div>
      </div>
      <div className="sparks-grid">
        {TOPIC_FREQ_KEYS.map(({ key, unit }) => {
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
        <div className="panel-hdr-meta">30 s window</div>
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
        <span className="status-val ok">domain 177 · cyclone</span>
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
  const [activeLayer, setActiveLayer] = useState("obstacle");
  const [showTrajectory, setShowTrajectory] = useState(false);
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
        battery: 0.74, // not yet in backend schema; placeholder
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
            activeLayer={activeLayer}
            setActiveLayer={setActiveLayer}
            showTrajectory={showTrajectory}
            setShowTrajectory={setShowTrajectory}
            showPlan={showPlan}
            setShowPlan={setShowPlan}
          />
          <div className="right-col">
            <div className="cam-row">
              <CameraPanel index={0} cameraTopics={frame.cameraTopics} topics={frame.topics} connected={frame.connected} now={frame.stamp} />
              <CameraPanel index={1} cameraTopics={frame.cameraTopics} topics={frame.topics} connected={frame.connected} now={frame.stamp} />
            </div>
            <div className="info-row">
              <RobotStatusPanel robot={robot} frame={frame} />
              <MissionPanel mission={frame.mission} />
            </div>
            <AlgoPerfPanel series={frame.series} />
          </div>
        </div>

        {/* Sparklines */}
        <TopicFreqsPanel series={frame.series} />

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
