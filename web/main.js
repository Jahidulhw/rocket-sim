// Rocket trajectory viewer.
//
// Loads JSON written by sim/export.py and animates it with three.js.
// Simulator frame: x = east, y = north, z = up. three.js is y-up, so a sim
// point (x, y, z) is drawn at (x, z, -y): east -> +X, up -> +Y, north -> -Z.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

window.__viewerStarted = true;

// Diagnostics for automated checks (scripts/check_viewer.py): mirror
// console.error calls and uncaught errors into the DOM.
const _consoleError = console.error.bind(console);
console.error = (...args) => { window.__viewerErrors.push(args.map(String).join(" ")); _consoleError(...args); };
setInterval(() => {
  document.body.dataset.errors = String(window.__viewerErrors.length);
  document.body.dataset.errorLog = JSON.stringify(window.__viewerErrors.slice(0, 5));
}, 250);

const PHASE_COLORS = {
  PAD: "#8a8f98", RAIL: "#a855f7", BOOST: "#f97316", COAST: "#3b82f6",
  DESCENT: "#10b981", LANDED: "#6b7280",
};
const MARKED_EVENTS = {
  apogee: { label: "Apogee", color: "#eab308" },
  deploy: { label: "Deploy", color: "#ec4899" },
  landing: { label: "Landing", color: "#ef4444" },
};
const TRAIL_SECONDS = 0.6;
const CHUTE_INFLATE_S = 0.4;

// ------------------------------------------------------------------ DOM ----
const $ = (id) => document.getElementById(id);
const ui = {
  canvas: $("scene"), stage: $("stage"), dataset: $("dataset"),
  camOrbit: $("cam-orbit"), camFollow: $("cam-follow"),
  play: $("play"), restart: $("restart"), speed: $("speed"), scrub: $("scrub"),
  readout: $("time-readout"), error: $("error"), legend: $("legend"),
  telemetry: $("telemetry"), summary: $("summary"), mcPanel: $("mc-panel"),
  mcStats: $("mc-stats"), mcNote: $("mc-note"), playback: $("playback"),
  tm: { time: $("tm-time"), alt: $("tm-alt"), speed: $("tm-speed"), vz: $("tm-vz"),
        range: $("tm-range"), phase: $("tm-phase") },
};

function setStatus(s) { document.body.dataset.status = s; }

function showError(message) {
  ui.error.textContent = message;
  ui.error.hidden = false;
  setStatus("error");
  console.warn("[viewer]", message);  // warn, not error: a handled condition
}

function clearError() { ui.error.hidden = true; ui.error.textContent = ""; }

// URL parameters (handy for sharing a view and for headless checks):
//   ?data=<id>  ?t=<seconds> (starts paused there)  ?cam=follow
const params = new URLSearchParams(location.search);

// ---------------------------------------------------------------- theme ----
function readTheme() {
  const cs = getComputedStyle(document.documentElement);
  const v = (name) => cs.getPropertyValue(name).trim();
  return {
    bg: v("--scene-bg"), ground: v("--ground"), grid: v("--grid"), gridMajor: v("--grid-major"),
    ring: v("--ring"), label: v("--label"), labelBg: v("--label-bg"), ghost: v("--ghost"),
    ellipse: v("--ellipse"), landingDot: v("--landing-dot"),
  };
}

// ------------------------------------------------------------- renderer ----
const renderer = new THREE.WebGLRenderer({ canvas: ui.canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 100000);
const controls = new OrbitControls(camera, ui.canvas);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.495;  // stay above the ground
scene.add(new THREE.HemisphereLight(0xffffff, 0x556070, 1.5));
const sun = new THREE.DirectionalLight(0xffffff, 1.8);
sun.position.set(400, 900, 300);
scene.add(sun);

const lineMaterials = new Set();  // fat-line materials need the canvas resolution

function resize() {
  const w = ui.stage.clientWidth, h = ui.stage.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  for (const m of lineMaterials) m.resolution.set(w, h);
}
new ResizeObserver(resize).observe(ui.stage);

// --------------------------------------------------------------- helpers ---
const toThree = (x, y, z, out = new THREE.Vector3()) => out.set(x, z, -y);

function niceStep(x) {
  const p = 10 ** Math.floor(Math.log10(x));
  const m = x / p;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
}

function makeLabel(text, { color, bg, size = 0.03, anchor = [0.5, -0.25] } = {}) {
  const fs = 30, pad = 9;
  const c = document.createElement("canvas");
  const ctx = c.getContext("2d");
  const font = `600 ${fs}px system-ui, -apple-system, "Segoe UI", sans-serif`;
  ctx.font = font;
  const w = Math.ceil(ctx.measureText(text).width) + 2 * pad, h = fs + 2 * pad;
  c.width = w; c.height = h;
  ctx.font = font;
  if (bg) {
    ctx.fillStyle = bg;
    if (ctx.roundRect) { ctx.beginPath(); ctx.roundRect(0, 0, w, h, 10); ctx.fill(); }
    else ctx.fillRect(0, 0, w, h);
  }
  ctx.fillStyle = color;
  ctx.textBaseline = "middle";
  ctx.fillText(text, pad, h / 2 + 1);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const mat = new THREE.SpriteMaterial({ map: tex, sizeAttenuation: false, depthTest: false, transparent: true });
  const s = new THREE.Sprite(mat);
  s.scale.set((size * w) / h, size, 1);
  s.center.set(...anchor);
  s.renderOrder = 20;
  return s;
}

let dotTexture = null;
function getDotTexture() {
  if (!dotTexture) {
    const c = document.createElement("canvas");
    c.width = c.height = 64;
    const ctx = c.getContext("2d");
    ctx.beginPath(); ctx.arc(32, 32, 26, 0, 2 * Math.PI);
    ctx.fillStyle = "#fff"; ctx.fill();
    ctx.lineWidth = 6; ctx.strokeStyle = "rgba(0,0,0,0.55)"; ctx.stroke();
    dotTexture = new THREE.CanvasTexture(c);
    dotTexture.colorSpace = THREE.SRGBColorSpace;
  }
  return dotTexture;
}

let discTexture = null;
function getDiscTexture() {  // plain filled circle for THREE.Points
  if (!discTexture) {
    const c = document.createElement("canvas");
    c.width = c.height = 64;
    const ctx = c.getContext("2d");
    ctx.beginPath(); ctx.arc(32, 32, 30, 0, 2 * Math.PI);
    ctx.fillStyle = "#fff"; ctx.fill();
    discTexture = new THREE.CanvasTexture(c);
  }
  return discTexture;
}

function makeDot(color, size = 0.018) {
  getDotTexture();
  const mat = new THREE.SpriteMaterial({ map: dotTexture, color, sizeAttenuation: false, depthTest: false });
  const s = new THREE.Sprite(mat);
  s.scale.set(size, size, 1);
  s.renderOrder = 19;
  return s;
}

function fatLine(points, { colors = null, color = "#ffffff", width = 3, opacity = 1, dashed = false } = {}) {
  const geo = new LineGeometry();
  geo.setPositions(points);
  if (colors) geo.setColors(colors);
  const mat = new LineMaterial({
    linewidth: width, vertexColors: !!colors, color: colors ? 0xffffff : color,
    transparent: opacity < 1, opacity, dashed, dashSize: 6, gapSize: 4,
  });
  mat.resolution.set(ui.stage.clientWidth || 1, ui.stage.clientHeight || 1);
  lineMaterials.add(mat);
  const line = new Line2(geo, mat);
  if (dashed) line.computeLineDistances();
  return line;
}

function disposeTree(root) {
  root.traverse((o) => {
    o.geometry?.dispose?.();
    const mats = Array.isArray(o.material) ? o.material : o.material ? [o.material] : [];
    for (const m of mats) {
      if (m.map && m.map !== dotTexture && m.map !== discTexture) m.map.dispose();
      lineMaterials.delete(m);
      m.dispose();
    }
  });
}

function circlePoints(cx, cz, r, n = 128, y = 0.05) {
  const pts = [];
  for (let i = 0; i <= n; i++) {
    const a = (i / n) * 2 * Math.PI;
    pts.push(cx + r * Math.cos(a), y, cz + r * Math.sin(a));
  }
  return pts;
}

// --------------------------------------------------------- scene pieces ----
function buildGround(group, extent, th, wind) {
  const step = niceStep(extent / 3.5);
  const half = Math.ceil(extent / step) * step;
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(4 * half, 4 * half).rotateX(-Math.PI / 2),
    new THREE.MeshStandardMaterial({ color: th.ground, roughness: 1, metalness: 0 }),
  );
  ground.position.y = -0.05;
  group.add(ground);

  const grid = new THREE.GridHelper(2 * half, Math.round((2 * half) / (step / 2)), th.gridMajor, th.grid);
  grid.material.transparent = true;
  grid.material.opacity = 0.75;
  group.add(grid);

  // Distance rings, labelled on the south side (default wind blows east).
  for (let r = step; r <= half + 1e-6; r += step) {
    const ring = fatLine(circlePoints(0, 0, r), { color: th.ring, width: 1.2, opacity: 0.6 });
    group.add(ring);
    const lbl = makeLabel(`${r} m`, { color: th.label, bg: th.labelBg, size: 0.024 });
    lbl.position.set(0, 0.3, r);
    group.add(lbl);
  }
  for (const [txt, pos] of [["N", [0, 0.3, -half * 1.08]], ["E", [half * 1.08, 0.3, 0]],
                            ["S", [0, 0.3, half * 1.08]], ["W", [-half * 1.08, 0.3, 0]]]) {
    const l = makeLabel(txt, { color: th.label, bg: th.labelBg, size: 0.03 });
    l.position.set(...pos);
    group.add(l);
  }

  // Launch pad marker.
  const pad = new THREE.Mesh(new THREE.CylinderGeometry(1.5, 2.0, 0.5, 24),
                             new THREE.MeshStandardMaterial({ color: "#9aa3ad", roughness: 0.8 }));
  pad.position.y = 0.25;
  group.add(pad);
  group.add(fatLine(circlePoints(0, 0, Math.max(3, step * 0.06), 48, 0.08), { color: "#f59e0b", width: 2.5 }));
  const padLabel = makeLabel("Launch pad", { color: th.label, bg: th.labelBg, size: 0.024 });
  padLabel.position.set(0, 1, 0);
  group.add(padLabel);

  // Wind arrow on the ground, upwind of the pad, pointing where the wind blows.
  if (wind && wind.speed_mps > 0) {
    const a = (wind.toward_deg * Math.PI) / 180;
    const dir = toThree(Math.sin(a), Math.cos(a), 0).normalize();
    const len = step * 1.1;
    const start = dir.clone().multiplyScalar(-half * 0.55).setY(0.4);
    const arrow = new THREE.ArrowHelper(dir, start, len, 0x0ea5e9, len * 0.25, len * 0.12);
    group.add(arrow);
    const wl = makeLabel(`Wind ${wind.speed_mps.toFixed(1)} m/s${wind.shear_exponent ? " (at 10 m, rising)" : ""}`,
                         { color: th.label, bg: th.labelBg, size: 0.022 });
    wl.position.copy(start).add(dir.clone().multiplyScalar(len * 0.5)).setY(1);
    group.add(wl);
  }
  return { half, step };
}

function buildRocket() {
  // Modelled along +Y, origin at the middle of the body; 8 units long.
  const g = new THREE.Group();
  const white = new THREE.MeshStandardMaterial({ color: "#f3f4f6", roughness: 0.5 });
  const red = new THREE.MeshStandardMaterial({ color: "#dc2626", roughness: 0.5 });
  const dark = new THREE.MeshStandardMaterial({ color: "#1f2937", roughness: 0.6 });
  const body = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.5, 6, 20), white);
  g.add(body);
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.5, 2, 20), red);
  nose.position.y = 4;
  g.add(nose);
  for (let i = 0; i < 3; i++) {
    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.08, 1.6, 1.1), dark);
    const a = (i * 2 * Math.PI) / 3;
    fin.position.set(Math.cos(a) * 0.95, -2.3, Math.sin(a) * 0.95);
    fin.rotation.y = -a;
    g.add(fin);
  }
  const flame = new THREE.Mesh(
    new THREE.ConeGeometry(0.42, 2.4, 16).rotateX(Math.PI),
    new THREE.MeshBasicMaterial({ color: "#fb923c", transparent: true, opacity: 0.9 }),
  );
  flame.position.y = -4.2;
  g.add(flame);
  return { group: g, flame };
}

function buildChute() {
  const g = new THREE.Group();
  const canopy = new THREE.Mesh(
    new THREE.SphereGeometry(4, 24, 10, 0, 2 * Math.PI, 0, Math.PI / 2.2),
    new THREE.MeshStandardMaterial({ color: "#ef4444", roughness: 0.7, side: THREE.DoubleSide }),
  );
  g.add(canopy);
  const rimY = 4 * Math.cos(Math.PI / 2.2), rimR = 4 * Math.sin(Math.PI / 2.2);
  const pts = [];
  for (let i = 0; i < 8; i++) {
    const a = (i / 8) * 2 * Math.PI;
    pts.push(new THREE.Vector3(rimR * Math.cos(a), rimY, rimR * Math.sin(a)), new THREE.Vector3(0, -6, 0));
  }
  g.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts),
                               new THREE.LineBasicMaterial({ color: "#64748b" })));
  return g;
}

// ------------------------------------------------------------ flight view --
function validateFlight(d) {
  const tr = d && d.trajectory;
  const cols = ["t", "x", "y", "z", "vx", "vy", "vz", "phase"];
  if (!tr || !cols.every((c) => Array.isArray(tr[c]) && tr[c].length === tr.t.length) || tr.t.length < 2) {
    throw new Error("Flight file is missing trajectory columns.");
  }
  if (!Array.isArray(d.events) || !d.meta || !Array.isArray(d.meta.phases)) {
    throw new Error("Flight file is missing events or metadata.");
  }
}

function createFlightView(data, th) {
  validateFlight(data);
  const tr = data.trajectory, phases = data.meta.phases, n = tr.t.length;
  const T = tr.t[n - 1];
  const group = new THREE.Group();
  const events = Object.fromEntries(data.events.map((e) => [e.name, e]));

  let maxH = 0, apogee = 0;
  for (let i = 0; i < n; i++) {
    maxH = Math.max(maxH, Math.hypot(tr.x[i], tr.y[i]));
    apogee = Math.max(apogee, tr.z[i]);
  }
  const extent = Math.max(60, maxH * 1.15, apogee * 0.5);
  buildGround(group, extent, th, data.meta.config?.wind);

  // Trajectory: faint full "ghost" path plus a phase-coloured path that grows.
  const pos = new Float32Array(3 * n), col = new Float32Array(3 * n);
  const v = new THREE.Vector3(), c = new THREE.Color();
  for (let i = 0; i < n; i++) {
    toThree(tr.x[i], tr.y[i], tr.z[i], v).toArray(pos, 3 * i);
    c.set(PHASE_COLORS[phases[tr.phase[i]]] || "#888").toArray(col, 3 * i);
  }
  const ghost = fatLine(pos, { color: th.ghost, width: 1.2, opacity: 0.25 });
  group.add(ghost);
  const path = fatLine(pos, { colors: col, width: 3.2 });
  group.add(path);

  // Event markers, revealed when the playhead reaches them.
  const markers = [];
  for (const [name, style] of Object.entries(MARKED_EVENTS)) {
    const e = events[name];
    if (!e) continue;
    const m = new THREE.Group();
    const p = toThree(...e.position);
    const dot = makeDot(style.color);
    dot.position.copy(p);
    m.add(dot);
    let text = `${style.label}  ${e.position[2].toFixed(0)} m`;
    if (name === "landing") text = `${style.label}  ${Math.hypot(e.position[0], e.position[1]).toFixed(0)} m from pad`;
    if (name === "deploy" && data.summary?.deployment?.timing) text = `${style.label} (${data.summary.deployment.timing} apogee)`;
    // Apogee label above the point, deploy label to its left (they are often close).
    const anchor = name === "deploy" ? [1.08, 0.5] : [0.5, -0.25];
    const lbl = makeLabel(text, { color: th.label, bg: th.labelBg, size: 0.024, anchor });
    lbl.position.copy(p);
    m.add(lbl);
    if (name === "apogee") {
      const drop = fatLine([p.x, p.y, p.z, p.x, 0.05, p.z], { color: style.color, width: 1.5, opacity: 0.7, dashed: true });
      m.add(drop);
    }
    if (name === "landing") m.add(fatLine(circlePoints(p.x, p.z, Math.max(2, extent * 0.02), 48, 0.1), { color: style.color, width: 2.5 }));
    m.visible = false;
    group.add(m);
    markers.push({ t: e.t, obj: m });
  }

  const rocket = buildRocket();
  group.add(rocket.group);
  const chute = buildChute();
  chute.visible = false;
  group.add(chute);

  // Exhaust trail: points over the last TRAIL_SECONDS of powered flight, fading with age.
  const TRAIL_MAX = Math.ceil(TRAIL_SECONDS * (data.meta.sample_rate_hz || 60)) + 2;
  const trailPos = new Float32Array(3 * TRAIL_MAX), trailCol = new Float32Array(4 * TRAIL_MAX);
  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
  trailGeo.setAttribute("color", new THREE.BufferAttribute(trailCol, 4));
  const trail = new THREE.Points(trailGeo, new THREE.PointsMaterial({
    size: 7, sizeAttenuation: false, vertexColors: true, transparent: true, depthWrite: false,
  }));
  trail.frustumCulled = false;
  group.add(trail);

  const cfgLaunch = data.meta.config?.launch || { tilt_deg: 0, azimuth_deg: 0 };
  const ti = (cfgLaunch.tilt_deg * Math.PI) / 180, az = (cfgLaunch.azimuth_deg * Math.PI) / 180;
  const railDir = toThree(Math.sin(ti) * Math.sin(az), Math.sin(ti) * Math.cos(az), Math.cos(ti)).normalize();
  const tDeploy = events.deploy ? events.deploy.t : Infinity;
  const isPowered = (ph) => ph === "RAIL" || ph === "BOOST";

  const UP = new THREE.Vector3(0, 1, 0);
  const rocketPos = new THREE.Vector3(), velDir = railDir.clone(), tmp = new THREE.Vector3();
  let idxHint = 0;

  function sampleIndex(t) {
    // Playback is mostly monotonic, so search from the last index first.
    let i = Math.min(idxHint, n - 2);
    if (tr.t[i] > t) i = 0;
    while (i < n - 2 && tr.t[i + 1] <= t) i++;
    idxHint = i;
    return i;
  }

  function sample(t) {
    const i = sampleIndex(t);
    const f = Math.min(1, Math.max(0, (t - tr.t[i]) / (tr.t[i + 1] - tr.t[i])));
    const L = (k) => tr[k][i] + f * (tr[k][i + 1] - tr[k][i]);
    return { i, f, x: L("x"), y: L("y"), z: L("z"), vx: L("vx"), vy: L("vy"), vz: L("vz"),
             phase: phases[tr.phase[f < 1 ? i : i + 1]] };
  }

  function update(t) {
    const s = sample(t);
    toThree(s.x, s.y, s.z, rocketPos);
    const scale = THREE.MathUtils.clamp(camera.position.distanceTo(rocketPos) / 170, 0.04, 60);

    // Orientation along the velocity vector (rail direction while stationary).
    toThree(s.vx, s.vy, s.vz, tmp);
    if (tmp.lengthSq() > 0.25) velDir.copy(tmp).normalize();
    else if (s.phase === "PAD" || s.phase === "RAIL") velDir.copy(railDir);
    rocket.group.quaternion.setFromUnitVectors(UP, velDir);
    rocket.group.scale.setScalar(scale);
    // Keep the drawn tail on the pad while it sits there (model origin is mid-body).
    rocket.group.position.copy(rocketPos).addScaledVector(velDir, s.phase === "PAD" ? 4 * scale : 0);
    rocket.group.position.y = Math.max(rocket.group.position.y, 0);

    const powered = isPowered(s.phase);
    rocket.flame.visible = powered;
    if (powered) rocket.flame.scale.set(1, 0.8 + 0.4 * Math.random(), 1);

    // Parachute: inflates over CHUTE_INFLATE_S after deployment.
    if (t >= tDeploy) {
      const k = Math.min(1, (t - tDeploy) / CHUTE_INFLATE_S);
      chute.visible = true;
      chute.scale.setScalar(scale * (0.15 + 0.85 * k * (2 - k)));
      chute.position.copy(rocketPos).add(tmp.set(0, 4 * scale + 6 * chute.scale.y, 0));
    } else chute.visible = false;

    // Growing path: Line2 draws `instanceCount` segments.
    path.geometry.instanceCount = Math.max(0, s.i + (s.f > 0 ? 1 : 0));

    // Exhaust trail.
    let k = 0;
    const exhaust = new THREE.Color("#fb923c"), hot = new THREE.Color("#fde047");
    for (let j = s.i; j >= 0 && k < TRAIL_MAX; j--) {
      const age = t - tr.t[j];
      if (age > TRAIL_SECONDS) break;
      if (!isPowered(phases[tr.phase[j]])) continue;
      toThree(tr.x[j], tr.y[j], tr.z[j], tmp).toArray(trailPos, 3 * k);
      const a = 1 - age / TRAIL_SECONDS;
      c.copy(exhaust).lerp(hot, a).toArray(trailCol, 4 * k);
      trailCol[4 * k + 3] = a * a;
      k++;
    }
    trailGeo.setDrawRange(0, k);
    trailGeo.attributes.position.needsUpdate = true;
    trailGeo.attributes.color.needsUpdate = true;

    for (const m of markers) m.obj.visible = t >= m.t - 1e-9;

    // Telemetry.
    ui.tm.time.textContent = t.toFixed(2);
    ui.tm.alt.textContent = s.z.toFixed(1);
    ui.tm.speed.textContent = Math.hypot(s.vx, s.vy, s.vz).toFixed(1);
    ui.tm.vz.textContent = (s.vz >= 0 ? "+" : "") + s.vz.toFixed(1);
    ui.tm.range.textContent = Math.hypot(s.x, s.y).toFixed(1);
    ui.tm.phase.textContent = s.phase;
    ui.tm.phase.style.background = PHASE_COLORS[s.phase] || "#666";
    return rocketPos;
  }

  const sm = data.summary || {};
  const dep = sm.deployment || {};
  ui.summary.textContent = `${data.meta.label || ""}: apogee ${sm.apogee_m?.toFixed(1)} m, ` +
    `max ${sm.max_speed_mps?.toFixed(1)} m/s, lands ${sm.landing_distance_m?.toFixed(0)} m from pad after ` +
    `${sm.flight_time_s?.toFixed(1)} s.` +
    (dep.deployed ? ` Chute ${dep.timing} apogee at ${dep.speed_mps.toFixed(1)} m/s.` : " No chute.");

  const lp = events.landing ? toThree(...events.landing.position) : new THREE.Vector3();
  const center = new THREE.Vector3(lp.x / 2, apogee * 0.45, lp.z / 2);
  const radius = Math.max(apogee, Math.hypot(lp.x, lp.z), 50);

  return { kind: "flight", group, T, update, center, radius, rocketPos };
}

// -------------------------------------------------------- dispersion view --
function validateDispersion(d) {
  const r = d && d.runs;
  if (!r || !Array.isArray(r.landing_x) || r.landing_x.length === 0 ||
      !Array.isArray(r.landing_y) || r.landing_y.length !== r.landing_x.length || !d.stats?.ellipse) {
    throw new Error("Dispersion file is missing landing points or statistics.");
  }
}

function createDispersionView(data, th) {
  validateDispersion(data);
  const group = new THREE.Group();
  const r = data.runs, st = data.stats, e = st.ellipse, n = r.landing_x.length;

  let maxH = 0, maxZ = 0;
  for (let i = 0; i < n; i++) maxH = Math.max(maxH, Math.hypot(r.landing_x[i], r.landing_y[i]));
  for (const p of data.sample_paths || []) for (const z of p.z) maxZ = Math.max(maxZ, z);
  const extent = Math.max(60, maxH * 1.1, maxZ * 0.5);
  buildGround(group, extent, th, data.meta.config?.wind);

  // All landing points as round sprites-in-a-buffer.
  const lp = new Float32Array(3 * n);
  for (let i = 0; i < n; i++) toThree(r.landing_x[i], r.landing_y[i], 0.2).toArray(lp, 3 * i);
  const pg = new THREE.BufferGeometry();
  pg.setAttribute("position", new THREE.BufferAttribute(lp, 3));
  // A point sprite has one depth value (its centre), so the tilted ground plane
  // would hide its lower half; the dots lie on the ground, so skip depth testing.
  const dots = new THREE.Points(pg, new THREE.PointsMaterial({
    color: th.landingDot, size: 7, sizeAttenuation: false, map: getDiscTexture(), transparent: true,
    alphaTest: 0.5, depthWrite: false, depthTest: false,
  }));
  dots.renderOrder = 5;
  group.add(dots);

  // k-sigma ellipse on the ground.
  const [cx, cy] = e.center_m, a = e.semi_major_m, b = e.semi_minor_m, ang = (e.angle_deg * Math.PI) / 180;
  const ep = [];
  for (let i = 0; i <= 160; i++) {
    const t = (i / 160) * 2 * Math.PI;
    const x = cx + a * Math.cos(t) * Math.cos(ang) - b * Math.sin(t) * Math.sin(ang);
    const y = cy + a * Math.cos(t) * Math.sin(ang) + b * Math.sin(t) * Math.cos(ang);
    ep.push(x, 0.3, -y);
  }
  group.add(fatLine(ep, { color: th.ellipse, width: 3 }));
  const el = makeLabel(`${e.k_sigma}σ ellipse: ${(100 * e.fraction_inside).toFixed(0)}% of landings`,
                       { color: th.label, bg: th.labelBg, size: 0.024 });
  const top = toThree(cx - b * Math.sin(ang), cy + b * Math.cos(ang), 0.5);
  el.position.copy(top);
  group.add(el);

  const meanDot = makeDot(th.ellipse, 0.022);
  meanDot.position.copy(toThree(cx, cy, 0.5));
  group.add(meanDot);
  if (data.nominal?.landing_m) {
    const nd = makeDot("#ef4444", 0.02);
    const np = toThree(data.nominal.landing_m[0], data.nominal.landing_m[1], 0.5);
    nd.position.copy(np);
    group.add(nd);
    const nl = makeLabel("Nominal landing", { color: th.label, bg: th.labelBg, size: 0.022, anchor: [-0.08, 0.5] });
    nl.position.copy(np);
    group.add(nl);
  }

  // A few faint sampled trajectories plus the nominal one.
  const v = new THREE.Vector3();
  const toFlat = (p) => { const out = []; for (let i = 0; i < p.t.length; i++) out.push(...toThree(p.x[i], p.y[i], p.z[i], v).toArray()); return out; };
  for (const p of data.sample_paths || []) group.add(fatLine(toFlat(p), { color: th.ghost, width: 1.2, opacity: 0.35 }));
  if (data.nominal?.path) group.add(fatLine(toFlat(data.nominal.path), { color: PHASE_COLORS.COAST, width: 2.5 }));

  // Stats panel.
  const mc = data.meta.montecarlo || {}, d = mc.dispersion || {};
  const rows = [
    ["Runs", `${st.n_runs} (seed ${mc.seed ?? "?"})`],
    ["Apogee", `${st.apogee.mean_m.toFixed(1)} ± ${st.apogee.std_m.toFixed(1)} m`],
    ["Apogee 5–95%", `${st.apogee.p5_m.toFixed(0)}–${st.apogee.p95_m.toFixed(0)} m`],
    ["Drift mean", `${st.drift.mean_m.toFixed(0)} m`],
    ["Drift 95th pct", `${st.drift.p95_m.toFixed(0)} m`],
    [`${e.k_sigma}σ ellipse`, `${(2 * a).toFixed(0)} × ${(2 * b).toFixed(0)} m`],
    ["Inside ellipse", `${(100 * e.fraction_inside).toFixed(1)}%`],
  ];
  ui.mcStats.innerHTML = rows.map(([k, val]) => `<div><dt>${k}</dt><dd>${val}</dd></div>`).join("");
  ui.mcNote.textContent = `σ: wind ${d.wind_speed_sigma_mps} m/s & ${d.wind_dir_sigma_deg}°, impulse ` +
    `${(100 * d.impulse_sigma_frac).toFixed(0)}%, Cd ${(100 * d.cd_sigma_frac).toFixed(0)}%, mass ` +
    `${(100 * d.dry_mass_sigma_frac).toFixed(0)}%, rail ${d.launch_angle_sigma_deg}°/axis. ` +
    `Blue line: nominal flight; grey: ${data.sample_paths?.length || 0} sampled runs.`;

  const center = toThree(cx / 2, cy / 2, maxZ * 0.25);
  const radius = Math.max(maxH, maxZ, 50);
  return { kind: "montecarlo", group, T: 0, update: () => null, center, radius,
           viewDir: new THREE.Vector3(0.3, 0.85, 0.55) };
}

// ------------------------------------------------------------- legend ------
function renderLegend(kind) {
  const rows = [];
  if (kind === "flight") {
    for (const [p, c] of Object.entries(PHASE_COLORS)) {
      if (p !== "PAD" && p !== "LANDED") rows.push(`<div class="row"><span class="sw" style="background:${c}"></span>${p.toLowerCase()}</div>`);
    }
    for (const s of Object.values(MARKED_EVENTS)) rows.push(`<div class="row"><span class="dot" style="background:${s.color}"></span>${s.label.toLowerCase()}</div>`);
  } else if (kind === "montecarlo") {
    const th = readTheme();
    rows.push(`<div class="row"><span class="dot" style="background:${th.landingDot}"></span>landing point</div>`,
              `<div class="row"><span class="sw" style="background:${th.ellipse}"></span>2σ ellipse / mean</div>`,
              `<div class="row"><span class="dot" style="background:#ef4444"></span>nominal landing</div>`,
              `<div class="row"><span class="sw" style="background:${PHASE_COLORS.COAST}"></span>nominal path</div>`,
              `<div class="row"><span class="sw" style="background:${th.ghost};opacity:.5"></span>sampled runs</div>`);
  }
  ui.legend.innerHTML = rows.join("");
  ui.legend.hidden = rows.length === 0;
}

// --------------------------------------------------------- app state -------
const state = { view: null, entry: null, t: 0, playing: true, speed: 1, cam: "orbit", scrubbing: false };
const cache = new Map();

function frameCamera(view) {
  controls.target.copy(view.center);
  // Portrait screens have a narrow horizontal field of view: back off further.
  const d = (view.radius * 1.55) / Math.min(1, camera.aspect) ** 0.85;
  const dir = (view.viewDir || new THREE.Vector3(0.62, 0.42, 0.78)).clone().normalize();
  camera.position.copy(view.center).addScaledVector(dir, d);
  camera.near = Math.max(0.1, d / 5000);
  camera.far = d * 50;
  camera.updateProjectionMatrix();
  controls.update();
}

function setCamMode(mode) {
  state.cam = mode;
  ui.camOrbit.setAttribute("aria-pressed", String(mode === "orbit"));
  ui.camFollow.setAttribute("aria-pressed", String(mode === "follow"));
  if (!state.view) return;
  if (mode === "follow" && state.view.rocketPos) {
    const p = state.view.update(state.t);
    controls.target.copy(p);
    camera.position.copy(p).add(new THREE.Vector3(45, 22, 55));
  } else {
    frameCamera(state.view);
  }
  controls.update();
}

function setPlaying(on) {
  state.playing = on;
  ui.play.textContent = on ? "Pause" : "Play";
  ui.play.setAttribute("aria-label", on ? "Pause" : "Play");
}

async function fetchJSON(url) {
  let res;
  try {
    res = await fetch(url, { cache: "no-cache" });
  } catch (err) {
    throw new Error(`Could not fetch ${url}: ${err.message}.` +
      (location.protocol === "file:" ? "\nOpen the page through a web server (python -m http.server), not file://." : ""));
  }
  if (!res.ok) throw new Error(`Could not load ${url} (HTTP ${res.status}).\nRun scripts/build_site.py to generate the data files.`);
  try {
    return await res.json();
  } catch (err) {
    throw new Error(`${url} is not valid JSON: ${err.message}`);
  }
}

async function loadEntry(entry) {
  state.entry = entry;
  clearError();
  setStatus("loading");
  let data = cache.get(entry.file);
  try {
    if (!data) {
      data = await fetchJSON(`data/${entry.file}`);
      cache.set(entry.file, data);
    }
    if (state.entry !== entry) return;  // user switched again while loading
    buildView(data);
    setStatus("ready");
  } catch (err) {
    if (state.view) { scene.remove(state.view.group); disposeTree(state.view.group); state.view = null; }
    showError(err.message);
  }
}

function buildView(data) {
  document.body.dataset.rendered = "false";
  if (state.view) { scene.remove(state.view.group); disposeTree(state.view.group); }
  const th = readTheme();
  scene.background = new THREE.Color(th.bg);
  let view;
  if (data.kind === "flight") view = createFlightView(data, th);
  else if (data.kind === "montecarlo") view = createDispersionView(data, th);
  else throw new Error(`Unknown dataset kind "${data.kind}".`);
  state.view = view;
  scene.add(view.group);
  renderLegend(view.kind);

  const isFlight = view.kind === "flight";
  ui.telemetry.hidden = !isFlight;
  ui.mcPanel.hidden = isFlight;
  for (const elx of [ui.play, ui.restart, ui.speed, ui.scrub, ui.camFollow]) elx.disabled = !isFlight;
  if (isFlight) {
    ui.scrub.max = String(view.T);
    const t0 = parseFloat(params.get("t"));
    state.t = Number.isFinite(t0) ? Math.min(Math.max(t0, 0), view.T) : 0;
    setPlaying(!Number.isFinite(t0));
    params.delete("t");  // only applies to the first load
  } else {
    setPlaying(false);
  }
  setCamMode(isFlight ? state.cam : "orbit");
  view.update(state.t);
}

// ----------------------------------------------------------- controls ------
ui.play.addEventListener("click", () => {
  if (!state.view || state.view.kind !== "flight") return;
  if (!state.playing && state.t >= state.view.T) state.t = 0;
  setPlaying(!state.playing);
});
ui.restart.addEventListener("click", () => { state.t = 0; setPlaying(true); });
ui.speed.addEventListener("change", () => { state.speed = parseFloat(ui.speed.value); });
ui.scrub.addEventListener("input", () => { state.t = parseFloat(ui.scrub.value); });
ui.scrub.addEventListener("pointerdown", () => { state.scrubbing = true; });
window.addEventListener("pointerup", () => { state.scrubbing = false; });
ui.camOrbit.addEventListener("click", () => setCamMode("orbit"));
ui.camFollow.addEventListener("click", () => setCamMode("follow"));
ui.dataset.addEventListener("change", () => {
  const entry = state.entries.find((e) => e.id === ui.dataset.value);
  if (entry) {
    const u = new URL(location.href);
    u.searchParams.set("data", entry.id);
    history.replaceState(null, "", u);
    loadEntry(entry);
  }
});
window.addEventListener("keydown", (e) => {
  if (e.code === "Space" && !["INPUT", "SELECT", "BUTTON"].includes(document.activeElement?.tagName)) {
    e.preventDefault();
    ui.play.click();
  }
});
// Rebuild with new colours when the OS switches light/dark.
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => {
  const data = state.entry && cache.get(state.entry.file);
  if (data) { const t = state.t, playing = state.playing; buildView(data); state.t = t; setPlaying(playing); }
});

// ------------------------------------------------------------- loop --------
let last = performance.now();
function tick(now) {
  const dtReal = Math.min(0.1, (now - last) / 1000);
  last = now;
  const view = state.view;
  if (view && view.kind === "flight") {
    if (state.playing && !state.scrubbing) {
      state.t += dtReal * state.speed;
      if (state.t >= view.T) { state.t = view.T; setPlaying(false); }
    }
    const before = state.cam === "follow" ? view.rocketPos.clone() : null;
    const p = view.update(state.t);
    if (before) {  // follow: translate camera with the rocket, user can still orbit/zoom
      camera.position.add(p.clone().sub(before));
      controls.target.copy(p);
    }
    if (!state.scrubbing) ui.scrub.value = String(state.t);
    ui.readout.textContent = `${state.t.toFixed(1)} / ${view.T.toFixed(1)} s`;
  }
  controls.update();
  renderer.render(scene, camera);
  if (view && document.body.dataset.rendered !== "true") document.body.dataset.rendered = "true";
  requestAnimationFrame(tick);
}

// ------------------------------------------------------------- start -------
async function start() {
  resize();
  requestAnimationFrame(tick);
  if (params.get("cam") === "follow") state.cam = "follow";
  let index;
  try {
    index = await fetchJSON("data/index.json");
    if (!Array.isArray(index.datasets) || index.datasets.length === 0) throw new Error("data/index.json lists no datasets.");
  } catch (err) {
    showError(err.message);
    return;
  }
  state.entries = index.datasets;
  ui.dataset.innerHTML = "";
  for (const e of index.datasets) {
    const o = document.createElement("option");
    o.value = e.id;
    o.textContent = e.label;
    o.title = e.description || "";
    ui.dataset.append(o);
  }
  const wanted = index.datasets.find((e) => e.id === params.get("data")) || index.datasets[0];
  ui.dataset.value = wanted.id;
  await loadEntry(wanted);
}

start();
