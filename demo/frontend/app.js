import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';

const $ = (s, r = document) => r.querySelector(s);
const el = (tag, props = {}, ...kids) => {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(props)) {
    if (k === 'class') n.className = v; else if (k === 'text') n.textContent = v;
    else if (k.startsWith('on')) n.addEventListener(k.slice(2), v); else n.setAttribute(k, v);
  }
  for (const c of kids) if (c != null) n.append(c.nodeType ? c : document.createTextNode(c));
  return n;
};
const cssVar = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const fmt = (v, d = 2) => (v == null || Number.isNaN(v)) ? '—' : Number(v).toFixed(d);
const HIST_S = 12;
const JN = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];

const S = { cfg: null, latest: null, lastMsg: 0, tab: 'left_follower', hist: {}, view: null };

/* ------------------------------------------------------------------ theme */
function applyTheme(t) { if (t) document.documentElement.dataset.theme = t; else delete document.documentElement.dataset.theme; }
try { applyTheme(localStorage.getItem('theme')); } catch (e) { /* storage unavailable */ }
$('#btn-theme').addEventListener('click', () => {
  const dark = matchMedia('(prefers-color-scheme: dark)').matches;
  const cur = document.documentElement.dataset.theme || (dark ? 'dark' : 'light');
  const next = cur === 'dark' ? 'light' : 'dark';
  applyTheme(next);
  try { localStorage.setItem('theme', next); } catch (e) { /* ignore */ }
  if (S.view) S.view.restyle();
});

/* ------------------------------------------------------------------ api */
async function api(path, method = 'POST') {
  const r = await fetch(path, { method, headers: { 'x-demo-token': S.cfg?.token || '' } });
  let body = null; try { body = await r.json(); } catch (e) { /* no body */ }
  if (!r.ok) throw new Error(body?.detail || `${r.status} ${r.statusText}`);
  return body;
}

/* ------------------------------------------------------------------ 3D view (URDF preview) */
class View {
  constructor(host, cfg) {
    this.host = host; this.cfg = cfg; this.arms = {}; this.ghostOn = true;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    host.prepend(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(38, 1, 0.02, 20);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.resetCam();
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x666666, 1.1));
    const d = new THREE.DirectionalLight(0xffffff, 1.4); d.position.set(1, 2, 1.5); this.scene.add(d);
    this.world = new THREE.Group(); this.world.rotation.x = -Math.PI / 2; // ROS Z-up -> three Y-up
    this.scene.add(this.world);
    this.grid = new THREE.GridHelper(2, 20); this.grid.rotation.x = Math.PI / 2; this.world.add(this.grid);
    this.world.add(new THREE.AxesHelper(0.15));
    new ResizeObserver(() => this.resize()).observe(host);
    this.resize();
    this.restyle();
    this.initPreview();
    this.renderer.setAnimationLoop(() => { this.controls.update(); this.renderer.render(this.scene, this.camera); });
  }
  resetCam() { this.camera.position.set(0.0, 0.8, 1.55); this.controls.target.set(0, 0.25, -0.12); this.controls.update(); }
  resize() { const w = this.host.clientWidth, h = this.host.clientHeight; this.renderer.setSize(w, h, false); this.camera.aspect = w / Math.max(1, h); this.camera.updateProjectionMatrix(); }
  restyle() {
    this.renderer.setClearColor(new THREE.Color(cssVar('--view-bg') || '#eceae4'), 1);
    const g = this.grid.material; (Array.isArray(g) ? g : [g]).forEach((m, i) => m.color.set(i ? cssVar('--grid') : cssVar('--axis')));
    for (const a of Object.values(this.arms)) a.style?.();
  }
  basePose(side) {
    const b = this.cfg.bases[side] || {}; const p = b.base_xyz_m || [0, 0, 0]; const r = (b.base_rpy_deg || [0, 0, 0]).map((x) => x * Math.PI / 180);
    return { p, r };
  }
  addArm(name, ghost) {
    const side = this.cfg.arms[name].side; const { p, r } = this.basePose(side);
    const group = new THREE.Group(); group.position.set(...p); group.rotation.set(r[0], r[1], r[2], 'ZYX');
    this.world.add(group);
    const rec = { name, ghost, group, robot: null };
    this.arms[name] = rec;
    const loader = new URDFLoader(new THREE.LoadingManager()); loader.packages = { piper_description: this.cfg.package_root };
    const style = () => {
      if (!rec.robot) return;
      rec.robot.traverse((c) => {
        if (!c.isMesh) return;
        c.material = ghost
          ? new THREE.MeshStandardMaterial({ color: new THREE.Color(cssVar('--accent')), transparent: true, opacity: 0.32, depthWrite: false })
          : new THREE.MeshStandardMaterial({ color: new THREE.Color(cssVar('--ink2')), metalness: 0.15, roughness: 0.65 });
      });
    };
    rec.style = style;
    loader.manager.onLoad = style;
    loader.load(this.cfg.urdf, (robot) => { rec.robot = robot; group.add(robot); style(); $('#viewport-msg')?.remove(); },
      undefined, (e) => { $('#viewport-msg').textContent = '模型加载失败：' + (e?.message || e); });
    if (!ghost) { // TCP marker
      rec.tcp = new THREE.Mesh(new THREE.SphereGeometry(0.012, 16, 12), new THREE.MeshBasicMaterial({ color: new THREE.Color(cssVar('--s2')) }));
      group.add(rec.tcp);
    }
    return rec;
  }
  initPreview() {
    this.previewMode = false;
    this.ballG = new THREE.Group();
    const bottle = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.16, 20), new THREE.MeshStandardMaterial({ color: new THREE.Color(cssVar('--s2')), roughness: 0.5 }));
    bottle.rotation.x = Math.PI / 2; // cylinder axis Y -> world Z (the world group is Z-up)
    this.ballG.add(bottle); this.ballG.visible = false; this.world.add(this.ballG);
    this.marks = {};
    for (const [k, v] of Object.entries({ release: '--good', apex: '--s4', catch: '--crit' })) {
      const m = new THREE.Mesh(new THREE.SphereGeometry(0.014, 16, 12), new THREE.MeshBasicMaterial({ color: new THREE.Color(cssVar(v)) }));
      m.visible = false; this.world.add(m); this.marks[k] = m;
    }
    this.path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: new THREE.Color(cssVar('--s2')) })); this.path.visible = false; this.world.add(this.path);
  }
  setMode(mode) { // 'live' | 'throw' | 'manual'
    this.mode = mode;
    const off = mode !== 'live';
    for (const rec of Object.values(this.arms)) { if (rec.ghost) rec.group.visible = !off && rec.group.visible; if (rec.tcp) rec.tcp.visible = !off; }
    const th = mode === 'throw';
    this.ballG.visible = th; this.path.visible = th; for (const m of Object.values(this.marks)) m.visible = th;
  }
  setPreview(on) { this.setMode(on ? 'throw' : 'live'); }
  tcpInfo(name) { // TCP = link6 (flange) + 0.13503 m along its z, read from the loaded URDF model (same convention as the backend FK)
    const rec = this.arms[name]; const l6 = rec?.robot?.links?.link6; if (!l6) return null;
    this.scene.updateMatrixWorld(true);
    const V = (x, y, z) => new THREE.Vector3(x, y, z);
    const tcpW = l6.localToWorld(V(0, 0, 0.13503)), orgW = l6.localToWorld(V(0, 0, 0));
    const toWorld = (v) => this.world.worldToLocal(v.clone()), toBase = (v) => rec.group.worldToLocal(v.clone());
    const dir = toWorld(tcpW).sub(toWorld(orgW)).normalize(), w = toWorld(tcpW), b = toBase(tcpW);
    return { world: [w.x, w.y, w.z], base: [b.x, b.y, b.z], dir: [dir.x, dir.y, dir.z] };
  }
  showPlan(plan) {
    const P = (a) => new THREE.Vector3(a[0], a[1], a[2]);
    this.marks.release.position.copy(P(plan.release.pos_world)); this.marks.apex.position.copy(P(plan.flight.apex_world)); this.marks.catch.position.copy(P(plan.flight.catch_world));
    const pts = []; plan.t.forEach((t, i) => { if (t >= plan.t_release && t <= plan.t_catch) pts.push(P(plan.ball_world[i])); });
    this.path.geometry.setFromPoints(pts);
  }
  frame(plan, i) {
    i = Math.max(0, Math.min(plan.t.length - 1, i));
    this.setPose('left_follower', { present: true, q_deg: plan.left_q_deg[i], gripper_mm: plan.left_gripper_mm[i] });
    this.setPose('right_follower', { present: true, q_deg: plan.right_q_deg, gripper_mm: plan.right_gripper_mm[i] });
    const b = plan.ball_world[i]; this.ballG.position.set(b[0], b[1], b[2]);
  }
  focusWorld(target, cam) { // world (x,y,z up) -> three (x, z, -y)
    this.controls.target.set(target[0], target[2], -target[1]); this.camera.position.set(cam[0], cam[2], -cam[1]); this.controls.update();
  }
  setPose(name, a) {
    const rec = this.arms[name]; if (!rec?.robot) return;
    rec.group.visible = !!a?.present && (!rec.ghost || this.ghostOn);
    if (!a?.present) return;
    a.q_deg.forEach((d, i) => rec.robot.joints[`joint${i + 1}`]?.setJointValue(d * Math.PI / 180));
    const g = (a.gripper_mm || 0) / 2000;
    rec.robot.joints.joint7?.setJointValue(g); rec.robot.joints.joint8?.setJointValue(-g);
    if (rec.tcp && a.tcp_m) rec.tcp.position.set(...a.tcp_m);
  }
}

/* ------------------------------------------------------------------ charts */
const tip = $('#tooltip');
class Chart {
  constructor(root, { title, unit, pick, digits = 2 }) {
    this.pick = pick; this.digits = digits; this.unit = unit; this.hover = null; this.on = new Array(6).fill(true);
    this.canvas = el('canvas'); this.legend = el('div', { class: 'cl' });
    JN.forEach((n, i) => this.legend.append(el('button', { 'aria-pressed': 'true', onclick: (e) => {
      this.on[i] = !this.on[i]; e.currentTarget.setAttribute('aria-pressed', String(this.on[i])); this.draw();
    } }, el('i', { style: `background:var(--s${i + 1})` }), n)));
    root.append(el('div', { class: 'chart' }, el('h3', {}, title, el('span', { text: unit })), this.canvas, this.legend));
    this.canvas.addEventListener('pointermove', (e) => { const r = this.canvas.getBoundingClientRect(); this.hover = { x: e.clientX - r.left, cx: e.clientX, cy: e.clientY }; this.draw(); });
    this.canvas.addEventListener('pointerleave', () => { this.hover = null; tip.hidden = true; this.draw(); });
  }
  draw() {
    const h = S.hist[S.tab]; const c = this.canvas; const dpr = Math.min(devicePixelRatio || 1, 2);
    const W = c.clientWidth, H = c.clientHeight; if (!W) return;
    if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
    const g = c.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    const ML = 44, MR = 8, MT = 6, MB = 16; const pw = W - ML - MR, ph = H - MT - MB;
    const ink2 = cssVar('--muted'), grid = cssVar('--grid'), axis = cssVar('--axis'), surface = cssVar('--surface');
    if (!h || h.t.length < 2) { g.fillStyle = ink2; g.font = '12px system-ui'; g.fillText('等待数据…', ML, MT + 16); return; }
    const tn = h.t[h.t.length - 1]; const t0 = tn - 10;
    let lo = Infinity, hi = -Infinity;
    for (let k = 0; k < h.t.length; k++) { if (h.t[k] < t0) continue; const v = this.pick(h, k); for (let i = 0; i < 6; i++) if (this.on[i] && v[i] != null) { lo = Math.min(lo, v[i]); hi = Math.max(hi, v[i]); } }
    if (!isFinite(lo)) { lo = -1; hi = 1; }
    if (hi - lo < 1e-6) { lo -= 0.5; hi += 0.5; }
    const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
    const step = niceStep((hi - lo) / 4); const y0 = Math.ceil(lo / step) * step;
    const X = (t) => ML + ((t - t0) / 10) * pw, Y = (v) => MT + (1 - (v - lo) / (hi - lo)) * ph;
    g.font = '11px system-ui'; g.textAlign = 'right'; g.textBaseline = 'middle'; g.lineWidth = 1;
    for (let v = y0; v <= hi; v += step) { g.strokeStyle = grid; g.beginPath(); g.moveTo(ML, Y(v)); g.lineTo(W - MR, Y(v)); g.stroke(); g.fillStyle = ink2; g.fillText(trim(v, step), ML - 6, Y(v)); }
    g.strokeStyle = axis; g.beginPath(); g.moveTo(ML, MT + ph); g.lineTo(W - MR, MT + ph); g.stroke();
    g.textAlign = 'center'; g.textBaseline = 'top'; g.fillStyle = ink2;
    for (const s of [0, 2, 4, 6, 8, 10]) g.fillText(s === 10 ? '现在' : `-${10 - s}s`, X(t0 + s), MT + ph + 3);
    g.save(); g.beginPath(); g.rect(ML, MT, pw, ph); g.clip(); g.lineWidth = 2; g.lineJoin = 'round'; g.lineCap = 'round';
    for (let i = 0; i < 6; i++) {
      if (!this.on[i]) continue; g.strokeStyle = cssVar(`--s${i + 1}`); g.beginPath(); let started = false;
      for (let k = 0; k < h.t.length; k++) { if (h.t[k] < t0 - 0.2) continue; const v = this.pick(h, k)[i]; if (v == null) continue; const x = X(h.t[k]), y = Y(v); started ? g.lineTo(x, y) : (g.moveTo(x, y), started = true); }
      g.stroke();
    }
    g.restore();
    if (this.hover && this.hover.x >= ML && this.hover.x <= W - MR) {
      const tt = t0 + ((this.hover.x - ML) / pw) * 10; let best = 0, bd = 1e9;
      for (let k = 0; k < h.t.length; k++) { const d = Math.abs(h.t[k] - tt); if (d < bd) { bd = d; best = k; } }
      const x = X(h.t[best]); g.strokeStyle = ink2; g.lineWidth = 1; g.beginPath(); g.moveTo(x, MT); g.lineTo(x, MT + ph); g.stroke();
      const v = this.pick(h, best); tip.replaceChildren(el('div', { class: 't', text: `${(h.t[best] - tn).toFixed(2)} s` }));
      for (let i = 0; i < 6; i++) {
        if (!this.on[i] || v[i] == null) continue;
        g.fillStyle = cssVar(`--s${i + 1}`); g.strokeStyle = surface; g.lineWidth = 2; g.beginPath(); g.arc(x, Y(v[i]), 4, 0, 7); g.fill(); g.stroke();
        tip.append(el('div', { class: 'r' }, el('span', {}, el('i', { style: `background:var(--s${i + 1})` }), JN[i]), el('b', { text: v[i].toFixed(this.digits) })));
      }
      tip.hidden = false; tip.style.left = Math.min(innerWidth - 160, this.hover.cx + 14) + 'px'; tip.style.top = Math.max(8, this.hover.cy - 20) + 'px';
    }
  }
}
function niceStep(raw) { const p = 10 ** Math.floor(Math.log10(raw)); const f = raw / p; return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p; }
function trim(v, step) { const d = Math.max(0, -Math.floor(Math.log10(step)) + (step < 1 ? 0 : 0)); return v.toFixed(Math.min(3, d)); }

let charts = [];
function buildCharts() {
  const root = $('#charts'); root.replaceChildren();
  charts = [
    new Chart(root, { title: '关节角', unit: '°', pick: (h, k) => h.q[k], digits: 2 }),
    new Chart(root, { title: '电机电流（高速反馈）', unit: 'A', pick: (h, k) => h.cur[k], digits: 3 }),
    new Chart(root, { title: '力矩反馈', unit: 'N·m', pick: (h, k) => h.eff[k], digits: 3 }),
    new Chart(root, { title: '力矩反馈 − 重力预测（MIT 补偿残差）', unit: 'N·m', pick: (h, k) => h.res[k], digits: 3 }),
  ];
}
(function loop() { charts.forEach((c) => c.draw()); setTimeout(() => requestAnimationFrame(loop), 50); })();

/* ------------------------------------------------------------------ data panel */
function pushHistory(msg) {
  const t = msg.t;
  for (const [name, a] of Object.entries(msg.arms)) {
    const h = S.hist[name] ||= { t: [], q: [], cur: [], eff: [], res: [] };
    if (!a.present) continue;
    if (h.t.length && t - h.t[h.t.length - 1] < 0.04) continue;
    h.t.push(t); h.q.push(a.q_deg); h.cur.push(a.current_a); h.eff.push(a.effort_nm); h.res.push(a.effort_minus_gravity_nm || new Array(6).fill(null));
    while (h.t.length && h.t[0] < t - HIST_S) { for (const k of ['t', 'q', 'cur', 'eff', 'res']) h[k].shift(); }
  }
}
function buildTabs() {
  const nav = $('#tabs'); nav.replaceChildren();
  for (const name of ['left_follower', 'right_follower', 'left_master', 'right_master']) {
    nav.append(el('button', { role: 'tab', 'data-name': name, 'aria-selected': String(name === S.tab), onclick: () => { S.tab = name; buildTabs(); renderData(); charts.forEach((c) => c.draw()); } },
      el('span', { class: 'dot' }), S.cfg.arms[name].label));
  }
}
function chip(text, s, label) { return el('span', { class: 'chip', 'data-s': s }, el('i'), label ? el('span', { text: label + ' ' }) : null, el('b', { text })); }
function renderData() {
  const m = S.latest; if (!m) return; const a = m.arms[S.tab];
  document.querySelectorAll('#tabs button').forEach((b) => b.dataset.on = m.arms[b.dataset.name]?.present ? '1' : '0');
  const chips = $('#arm-chips'); chips.replaceChildren();
  const tb = $('#joint-table tbody'); tb.replaceChildren();
  const note = $('#arm-note');
  if (!a || !a.connected) { chips.append(chip(a?.error || '未连接', 'bad', 'USB-CAN')); note.textContent = '该臂的 USB-CAN 适配器未找到。'; return; }
  if (!a.present) {
    chips.append(chip(a.can || '—', 'warn', 'CAN'), chip('无反馈', 'bad', '状态'));
    note.textContent = a.role === 'master' ? '主臂无反馈：请确认主臂已上电。' : '从臂无反馈：请确认从臂已上电。'; return;
  }
  const mit = a.mode_feed === 6;
  chips.append(
    chip(a.can, 'ok', 'CAN'),
    chip(`${fmt(a.hz.joint, 0)} Hz`, a.hz.joint > 150 ? 'ok' : 'warn', '关节'),
    chip(a.ctrl_mode_text, a.ctrl_mode === 1 ? 'ok' : 'warn', '控制'),
    chip(a.mode_feed_text, mit ? 'mit' : 'ok', '运动模式'),
    chip(a.arm_status_text, a.arm_status === 0 ? 'ok' : 'bad', '状态'),
    chip(`${a.enabled.filter(Boolean).length}/6`, a.enabled.every(Boolean) ? 'ok' : 'warn', '使能'),
    chip(`${fmt(a.gripper_mm, 1)} mm`, a.gripper_enabled ? 'ok' : 'warn', '夹爪'),
    chip(a.tcp_m ? a.tcp_m.map((x) => (x * 1000).toFixed(0)).join(', ') + ' mm' : '—', 'ok', 'TCP'),
    chip(`${fmt(a.age_ms?.joint, 0)} ms`, (a.age_ms?.joint ?? 999) < 50 ? 'ok' : 'warn', '延迟'),
  );
  for (let i = 0; i < 6; i++) {
    const hot = (a.motor_temp_c[i] ?? 0) >= 60 || (a.driver_temp_c[i] ?? 0) >= 70;
    const row = el('tr', {}, el('td', {}, el('span', { class: 'sw', style: `background:var(--s${i + 1})` }), JN[i]));
    const cells = [fmt(a.q_deg[i], 2), fmt(a.speed_rad_s[i], 3), fmt(a.current_a[i], 3), fmt(a.effort_nm[i], 3),
      fmt(a.gravity_nm?.[i], 3), fmt(a.effort_minus_gravity_nm?.[i], 3), a.motor_temp_c[i], a.driver_temp_c[i], fmt(a.voltage_v[i], 1)];
    cells.forEach((c, k) => row.append(el('td', { class: hot && (k === 6 || k === 7) ? 'hot' : '', text: String(c) })));
    row.append(el('td', { text: a.enabled[i] ? '✓' : '—', title: a.motor_flags[i].join(', ') }));
    tb.append(row);
  }
  note.textContent = '力矩反馈为固件按固定系数换算值；重力预测按 piper_identified.urdf 静态重力计算（与控制台 MIT 前馈同一公式，腕部电机按 0.813 缩放到线上指令）。差值仅供参考，含摩擦与换算误差。'
    + (a.motor_flags.some((f) => f.length) ? ' 电机标志：' + a.motor_flags.map((f, i) => f.length ? `${JN[i]} ${f.join('/')}` : '').filter(Boolean).join('；') : '');
}

/* ------------------------------------------------------------------ teleop panel */
const STATE_TXT = { idle: '空闲', aligning: '对齐中', starting: '进入 MIT', running: '遥操中', exiting: '退出中', fault: '故障', estopped: '已急停' };
const STATE_S = { idle: 'ok', aligning: 'warn', starting: 'warn', running: 'mit', exiting: 'warn', fault: 'bad', estopped: 'bad' };
const TC = {};
function buildTeleop() {
  const grid = $('#teleop-grid'); grid.replaceChildren();
  const unlocked = S.cfg.allow_control;
  $('#teleop-hint').textContent = unlocked ? '控制已解锁：每个动作都有确认，Esc = 急停' : '服务以只读模式启动：只能预检，不能下发任何运动指令';
  for (const side of ['left', 'right']) {
    const c = {};
    c.state = el('span', { class: 'chip', 'data-s': 'ok' }, el('i'), el('b', { text: '空闲' }));
    c.checks = el('ul', { class: 'checks' }); c.bars = el('div', { class: 'bars' });
    c.err = el('div', { class: 'err' }); c.meta = el('div', { class: 'meta' });
    const run = (fn, ask) => async () => { if (ask && !confirm(ask)) return; try { await fn(); c.err.textContent = ''; } catch (e) { c.err.textContent = e.message; } };
    c.bPre = el('button', { class: 'sec', text: '预检', onclick: run(async () => { await api(`/api/teleop/${side}/preflight`); }) });
    c.bAlign = el('button', { class: 'sec', text: '对齐到主臂（慢速）', onclick: run(() => api(`/api/teleop/${side}/align`), '从臂将以 10% 速度移动到主臂当前姿态。\n确认从臂周围无障碍、主臂已上电并保持不动？') });
    c.bStart = el('button', { class: 'primary', text: '启动 MIT 遥操', onclick: run(() => api(`/api/teleop/${side}/start`), '即将进入 MIT 模式，从臂会实时跟随主臂（速度上限 3 rad/s，TCP 高度下限 0.05 m）。\n确认：主臂已上电并被手扶住，从臂周围无障碍，急停可达？') });
    c.bStop = el('button', { class: 'sec', text: '停止（保持当前位置）', onclick: run(() => api(`/api/teleop/${side}/stop`)) });
    c.bStow = el('button', { class: 'sec', text: '归拢并失能', onclick: run(() => api(`/api/follower/${side}/stow`), '从臂将慢速归拢到已保存的折叠位姿，到位后才失能。继续？') });
    for (const b of [c.bAlign, c.bStart, c.bStop, c.bStow]) if (!unlocked) b.disabled = true;
    grid.append(el('div', { class: 'tcard' },
      el('h3', {}, `${side === 'left' ? '左' : '右'}侧：主臂 → 从臂`, c.state), c.checks, c.bars, c.meta,
      el('div', { class: 'row' }, c.bPre, c.bAlign, c.bStart, c.bStop, c.bStow), c.err));
    TC[side] = c;
  }
}
function renderTeleop() {
  const m = S.latest; if (!m) return;
  for (const side of ['left', 'right']) {
    const c = TC[side], t = m.teleop?.[side], f = m.arms[`${side}_follower`], l = m.arms[`${side}_master`]; if (!c) continue;
    const st = t?.state || 'idle';
    c.state.dataset.s = STATE_S[st] || 'ok'; c.state.lastChild.textContent = t ? (STATE_TXT[st] || st) : '未连接';
    const tol = t?.params?.align_tol_deg ?? 6;
    c.bars.replaceChildren();
    for (let i = 0; i < 6; i++) {
      const e = f?.present && l?.present ? Math.abs(l.q_deg[i] - f.q_deg[i]) : null; const full = tol * 2.5;
      const over = e != null && e > tol;
      c.bars.append(el('div', { class: 'bar' + (over ? ' over' : '') },
        el('div', { class: 'track' }, e != null ? el('div', { class: 'fill', style: `height:${Math.min(100, e / full * 100)}%` }) : null, el('div', { class: 'thr', style: `bottom:${tol / full * 100}%` })),
        `${JN[i]} ${e == null ? '—' : e.toFixed(1)}°`));
    }
    const pf = t?.preflight;
    c.checks.replaceChildren(...(pf?.checks || []).map((k) => el('li', { class: k.ok ? 'ok' : 'bad' }, el('span', { class: 'm', text: k.ok ? '✓' : '✗' }), el('span', { text: k.name + (k.detail ? '：' + k.detail : '') }))));
    if (!pf) c.checks.replaceChildren(el('li', {}, '尚未预检'));
    const lp = t?.loop;
    c.meta.textContent = t ? `${t.firmware ? '固件 ' + t.firmware + ' · ' : ''}kp ${t.kp?.[0]} / kd ${t.kd?.[0]}`
      + (st === 'running' || st === 'starting' ? ` · 环路 ${fmt(lp?.hz, 0)} Hz · 最大迟到 ${fmt(lp?.max_late_ms, 1)} ms · 跟随误差 ${fmt(Math.max(...(t.track_err_deg || [0])), 1)}°` : '') : '该侧主臂或从臂未连接';
    if (t?.error) c.err.textContent = t.error;
    const unlocked = S.cfg.allow_control, active = ['aligning', 'starting', 'running', 'exiting'].includes(st);
    c.bPre.disabled = !t || active; c.bAlign.disabled = !unlocked || !t || active; c.bStart.disabled = !unlocked || !t || active;
    c.bStop.disabled = !unlocked || !t || !active; c.bStow.disabled = !unlocked || !t || active;
  }
}
setInterval(() => { // read-only preflight refresh while idle, heartbeat while a motion session is active
  const m = S.latest; if (!m || !S.cfg) return;
  if (Object.values(m.mittest || {}).some((t) => !['idle', 'fault'].includes(t.state))) return; // keep the bus/CPU quiet during a MIT test
  for (const side of ['left', 'right']) { const st = m.teleop?.[side]?.state; if (m.teleop?.[side] && (!st || st === 'idle' || st === 'fault') && !document.hidden) api(`/api/teleop/${side}/preflight`).catch(() => {}); }
}, 3000);
// "The page is open and alive" heartbeat: unconditional (not tied to the SSE state), so a stalled event stream
// cannot starve a running session. If the tab dies or the network drops, the server holds the arm within 1 s.
function startHeartbeat() {
  // Runs in a Web Worker: not throttled when the tab is in the background and not blocked by page rendering
  // (a busy main thread during fast arm motion starved the old setInterval heartbeat for >1 s).
  const url = location.origin + '/api/teleop/heartbeat';
  try {
    const src = `let tok='';onmessage=e=>{tok=e.data};setInterval(()=>{if(tok)fetch(${JSON.stringify(url)},{method:'POST',headers:{'x-demo-token':tok}}).catch(()=>{})},100)`;
    const w = new Worker(URL.createObjectURL(new Blob([src], { type: 'text/javascript' })));
    w.postMessage(S.cfg.token); S.hbWorker = w;
  } catch (e) { setInterval(() => api('/api/teleop/heartbeat').catch(() => {}), 100); }
}

/* ------------------------------------------------------------------ MIT step tests */
const MT = {};
const MT_TXT = { idle: '空闲', checking: '预检', moving: '移动到准备姿态', entering: '进入 MIT', hold: 'MIT 保持', track: 'MIT 跟踪', speed: 'MIT 速度爬升', speed_j3: 'J3 高速', speed_j5: 'J5 高速', exiting: '退出中', fault: '故障' };
const MT_S = { idle: 'ok', checking: 'warn', moving: 'warn', entering: 'warn', hold: 'mit', track: 'mit', speed: 'mit', speed_j3: 'mit', speed_j5: 'mit', exiting: 'warn', fault: 'bad' };
function buildMitTest() {
  const unlocked = S.cfg.allow_control;
  $('#mittest-hint').textContent = unlocked ? '每一步都有停止条件；任何时候点右上角“急停保持”或按 Esc' : '只读模式：不能运行测试';
  const fw = $('#mittest-fw'); fw.replaceChildren();
  MT.fwMsg = el('span', { class: 'hint', text: 'MIT 要求 V3 固件 ≥ S-V1.8-8' });
  MT.fwBtn = el('button', { class: 'sec', text: '① 固件查询（两臂）', onclick: async () => {
    try { const r = await api('/api/fw'); MT.fwMsg.textContent = Object.entries(r).map(([s, v]) => `${s === 'left' ? '左' : '右'}从臂 ${v.version || '无应答'} ${v.ok ? '✓ 满足' : '✗ 不满足'}`).join('；'); }
    catch (e) { MT.fwMsg.textContent = e.message; }
  } });
  MT.fwBtn.disabled = !unlocked; fw.append(MT.fwBtn, MT.fwMsg);
  const grid = $('#mittest-grid'); grid.replaceChildren();
  for (const side of ['left', 'right']) {
    const c = {}; MT[side] = c;
    c.state = el('span', { class: 'chip', 'data-s': 'ok' }, el('i'), el('b', { text: '空闲' }));
    c.prog = el('div', { class: 'pbar' }, el('i')); c.log = el('div', { class: 'logbox' }); c.res = el('div'); c.err = el('div', { class: 'err' });
    const run = (step, ask) => async () => { if (ask && !confirm(ask)) return; try { await api(`/api/mittest/${side}/${step}`); S.tab = `${side}_follower`; buildTabs(); renderData(); c.err.textContent = ''; } catch (e) { c.err.textContent = e.message; } };
    const pre = '从臂将先以 10% 速度移动到准备姿态（TCP 高度约 0.47 m），再进入 MIT。\n确认：从臂周围无障碍、急停保持/物理急停可达、其它人没有占用这条臂？';
    c.bHold = el('button', { class: 'primary', text: '② MIT 保持 15 s', onclick: run('hold', pre) });
    c.bTrack = el('button', { class: 'primary', text: '③ MIT 小幅跟踪', onclick: run('track', pre + '\n（J2 做 ±3° 阶跃和 ±4° 以内的正弦，幅度很小。）') });
    c.bSpeed = el('button', { class: 'primary', text: '④ MIT 速度爬升', onclick: run('speed',
      '⚠ 这一步动作明显更大更快：J2 单关节往返摆动 50°，从 0.5 rad/s 起逐级提速到 4.5 rad/s，每级都有停止条件。\n\n' + pre + '\n\n请确认：\n• 臂前方和上方（至少 0.5 m 范围）清空\n• 你的手在“急停保持”按键上（或按 Esc）\n• 周围人员已知会') });
    const big = (j, extra) => '⚠ 这一步动作大而快：' + j + ' 单关节往返摆动 50°，从 0.5 rad/s 起逐级提速，每级都有停止条件。\n' + extra + '\n\n' + pre + '\n\n请确认：\n• 臂前方和上方（至少 0.5 m 范围）清空\n• 你的手在“急停保持”按键上（或按 Esc）\n• 周围人员已知会';
    c.bJ3 = el('button', { class: 'primary', text: '⑤ J3 高速', onclick: run('speed_j3', big('J3（肘）', '目标：验证上抛预设需要的约 2.9 rad/s。')) });
    c.bJ5 = el('button', { class: 'primary', text: '⑥ J5 高速', onclick: run('speed_j5', big('J5（腕）', '目标：验证上抛预设需要的约 3.0 rad/s。')) });
    c.bStop = el('button', { class: 'sec', text: '停止并保持', onclick: run('stop') });
    for (const b of [c.bHold, c.bTrack, c.bSpeed, c.bJ3, c.bJ5, c.bStop]) b.disabled = !unlocked;
    grid.append(el('div', { class: 'tcard' }, el('h3', {}, `${side === 'left' ? '左' : '右'}从臂`, c.state),
      el('div', { class: 'row' }, c.bHold, c.bTrack, c.bSpeed, c.bJ3, c.bJ5, c.bStop), c.prog, c.log, c.res, c.err));
  }
}
function table(headers, rows) {
  return el('table', { class: 'mt' }, el('thead', {}, el('tr', {}, ...headers.map((h) => el('th', { text: h })))),
    el('tbody', {}, ...rows.map((r) => el('tr', {}, ...r.map((v) => el('td', { text: String(v) }))))));
}
function renderMitTest() {
  const m = S.latest; if (!m?.mittest) return;
  for (const side of ['left', 'right']) {
    const c = MT[side], t = m.mittest[side]; if (!c) continue;
    if (!t) { c.state.lastChild.textContent = '未连接'; c.state.dataset.s = 'bad'; continue; }
    c.state.dataset.s = MT_S[t.state] || 'ok'; c.state.lastChild.textContent = MT_TXT[t.state] || t.state;
    const active = !['idle', 'fault'].includes(t.state);
    c.bHold.disabled = !S.cfg.allow_control || active || !t.results?.fw?.ok; c.bTrack.disabled = c.bHold.disabled; c.bSpeed.disabled = c.bHold.disabled || !t.results?.track; c.bJ3.disabled = c.bSpeed.disabled; c.bJ5.disabled = c.bSpeed.disabled; c.bStop.disabled = !S.cfg.allow_control || !active;
    c.prog.firstChild.style.width = t.progress ? `${Math.min(100, t.progress.elapsed / t.progress.duration * 100)}%` : '0%';
    c.log.textContent = t.log.join('\n'); c.log.scrollTop = c.log.scrollHeight;
    if (t.error) c.err.textContent = t.error;
    const r = t.results || {}; const kids = [];
    if (r.hold) kids.push(el('div', { class: 'meta', text: `② 保持：${r.hold.verdict}（${r.hold.samples} 样本 · ${r.hold.csv}）` }),
      table(['关节', '平均偏差°', '抖动 std°', '最大偏差°', '电流 A', '力矩 N·m'], JN.map((n, i) => [n, r.hold.mean_err_deg[i], r.hold.std_err_deg[i], r.hold.max_abs_err_deg[i], r.hold.current_mean_a[i], r.hold.effort_mean_nm[i]])));
    if (r.track) kids.push(el('div', { class: 'meta', text: `③ 跟踪 ${r.track.joint}：${r.track.verdict} · 阶跃 稳态误差 ${r.track.step.steady_err_deg}° 超调 ${r.track.step.overshoot_pct}% 上升 ${r.track.step.rise_10_90_ms} ms · ${r.track.csv}` }),
      table(['频率 Hz', '幅值°', '增益', '滞后 ms', 'RMS 误差°', '峰值电流 A'], r.track.sines.map((s) => [s.freq_hz, s.amp_deg, s.gain, s.lag_ms, s.rms_err_deg, s.peak_current_a])));
    for (const [key, title] of [['speed', '④ J2'], ['speed_j3', '⑤ J3'], ['speed_j5', '⑥ J5']]) {
      const sp = r[key]; if (!sp) continue;
      kids.push(el('div', { class: 'meta', text: `${title} 速度爬升：摆动 ${sp.swing_deg}° · 加速度 ${sp.accel_rad_s2} rad/s² · 惯量 ${sp.inertia_kgm2} kg·m² · 前馈力矩峰值 ${sp.ff_peak_nm ?? '—'} N·m${sp.verdict ? ' · ' + sp.verdict : ''}` }),
        table(['指令 rad/s', '实测 rad/s', '末端 m/s', '最大误差°', '本关节电流 A', '最大电流 A', '结论'],
          sp.levels.map((l) => [l.v_cmd, l.v_joint ?? '—', l.tip_m_s ?? '—', l.err_max_deg ?? '—', l.cur_j_a ?? l.cur_j2_a ?? '—', l.cur_max_a ?? '—', l.result])));
    }
    c.res.replaceChildren(...kids);
  }
}

/* ------------------------------------------------------------------ main */
function renderConn() {
  const c = $('#chip-conn'); const age = (Date.now() - S.lastMsg) / 1000;
  const ok = S.lastMsg && age < 1.5; c.dataset.s = ok ? 'ok' : 'bad'; $('b', c).textContent = ok ? `数据流 ${S.latest ? Object.values(S.latest.arms).filter((a) => a.present).length : 0}/4 臂在线` : '数据流中断';
}
setInterval(renderConn, 500);

async function main() {
  S.cfg = await (await fetch('/api/config')).json();
  $('#chip-ctl').dataset.s = S.cfg.allow_control ? 'warn' : 'ok'; $('#chip-ctl b').textContent = S.cfg.allow_control ? '控制已解锁' : '只读';
  if (S.cfg.allow_control) {
    const hold = $('#btn-hold'), es = $('#btn-estop'); hold.hidden = false; es.hidden = false;
    const doHold = async () => { // fires immediately: no confirmation on the safety action
      hold.textContent = '保持中…';
      try { const r = await api('/api/hold'); hold.textContent = '已保持 ✓'; setTimeout(() => { hold.textContent = '急停保持'; }, 2500); S.holdResult = r; }
      catch (e) { hold.textContent = '急停保持'; alert('急停保持失败：' + e.message + '\n请立即按机械臂上的物理急停！'); }
    };
    hold.onclick = doHold;
    es.onclick = () => { if (confirm('固件急停可能让关节失能、机械臂下坠。确定？\n（通常请用“急停保持”）')) api('/api/estop').catch((e) => alert(e.message)); };
    addEventListener('keydown', (e) => { if (e.key === 'Escape') doHold(); });
  }
  buildTabs(); buildCharts(); buildTeleop(); buildMitTest();
  if (S.cfg.allow_control) startHeartbeat();
  initThrow();
  initPose();
  try {
    S.view = new View($('#viewport'), S.cfg);
    for (const name of Object.keys(S.cfg.arms)) if (S.cfg.arms[name].role === 'follower') S.view.addArm(name, false);
    for (const name of Object.keys(S.cfg.arms)) if (S.cfg.arms[name].role === 'master') S.view.addArm(name, true);
    $('#tg-ghost').onchange = (e) => { S.view.ghostOn = e.target.checked; };
    $('#tg-grid').onchange = (e) => { S.view.grid.visible = e.target.checked; };
    $('#btn-reset-cam').onclick = () => S.view.resetCam();
  } catch (e) { $('#viewport-msg').textContent = '3D 预览初始化失败：' + e.message; }
  const es = new EventSource('/api/stream');
  let tick = 0;
  es.onmessage = (ev) => {
    const m = JSON.parse(ev.data); if (!m.t) return; S.latest = m; S.lastMsg = Date.now(); pushHistory(m);
    if (S.view && !TH.active && !MP.active) for (const name of Object.keys(S.cfg.arms)) S.view.setPose(name, m.arms[name]);
    if (MP.active && tick % 10 === 0) mpApply(); // also catches the moment the meshes finish loading
    if (++tick % 2 === 0) { renderData(); renderTeleop(); renderMitTest(); renderPoseMove(); }
  };
}
main().catch((e) => { $('#viewport-msg').textContent = '初始化失败：' + e.message; console.error(e); });


/* ------------------------------------------------------------------ up-toss preview (offline simulation) */
const TH = { plan: null, t: 0, playing: false, last: 0, active: false, timer: null };
function thRows(rows) { return el('table', { class: 'kv' }, el('tbody', {}, ...rows.map(([k, v]) => el('tr', {}, el('th', { text: k }), el('td', { text: v }))))); }
function renderThrowStats(p) {
  const r = p.release, f = p.flight, fx = (a) => a.map((x) => x.toFixed(2)).join(', ');
  const jr = Object.entries(p.joints).map(([n, j]) => [n, `${j.peak_speed_rad_s} rad/s`, `${j.peak_acc_rad_s2} rad/s²`, j.peak_torque_nm == null ? '—' : `${j.peak_torque_nm} N·m`]);
  $('#th-stats').replaceChildren(
    thRows([
      ['出手点（世界系）', `(${fx(r.pos_world)}) m`],
      ['出手速度', `${r.speed_m_s} m/s：向上 ${r.upward_m_s}，水平 ${r.horizontal_m_s}；仰角 ${r.elevation_deg}°，水平方位 ${r.azimuth_deg}°（0° = 朝右臂 +x，90° = 向前 +y）`],
      ['夹爪接近轴', `与竖直夹 ${r.gripper_tilt_deg}°（≈90° 即水平侧抓）`],
      ['抛高 / 飞行', `最高比出手点高 ${(f.apex_above_release_m * 100).toFixed(0)} cm；到接取点 ${f.time_to_catch_height_s} s`],
      ['接取点（世界系）', `(${fx(f.catch_world)}) m，比出手点高 ${(f.catch_above_release_m * 100).toFixed(0)} cm，球速 ${f.catch_speed_m_s} m/s`],
      ['右臂接取姿态', `J1 ${p.right_q_deg[0]}° J2 ${p.right_q_deg[1]}° J3 ${p.right_q_deg[2]}° J5 ${p.right_q_deg[4]}°（水平距离 ${f.right_reach_m} m，${p.right_ik.ok ? 'IK 可达' : 'IK 不可达'}）`],
      ['时间点', `出手 ${p.t_release} s · 最高点 ${p.t_apex} s · 接取 ${p.t_catch} s`],
    ]),
    table(['关节', '峰值速度', '峰值加速度', '峰值力矩（含重力）'], jr));
  $('#th-warn').replaceChildren(...p.warnings.map((w) => el('li', { class: 'warn' }, el('span', { class: 'm', text: '!' }), el('span', { text: w }))));
  $('#th-assume').textContent = p.assumptions;
}
function thApply() {
  const p = TH.plan; if (!p || !S.view) return;
  S.view.frame(p, Math.round(TH.t / p.dt));
  $('#th-time').value = String(TH.t / p.t_end); $('#th-t').textContent = TH.t.toFixed(2) + ' s';
}
async function thLoad(autoplay) {
  const preset = $('#th-preset').value, aim = $('#th-aim').value, cf = $('#th-catch').value;
  $('#th-aim-v').textContent = TH.presets?.[preset]?.kind === 'oblique' ? '由预设决定' : aim + '°'; $('#th-catch-v').textContent = `抛高的 ${Math.round(cf * 100)}%`;
  let r, p; try { r = await fetch(`/api/throw/plan?preset=${preset}&aim=${aim}&catch_frac=${cf}`); p = await r.json(); } catch (e) { $('#th-warn').replaceChildren(el('li', {}, e.message)); return; }
  if (!r.ok) { $('#th-warn').replaceChildren(el('li', { class: 'bad' }, el('span', { class: 'm', text: '✗' }), el('span', { text: p.detail || '规划失败' }))); return; }
  TH.plan = p; TH.t = 0; TH.playing = !!autoplay; TH.active = true;
  if (MP.active) { $('#mp-on').checked = false; mpSet(false); }
  S.view.setMode('throw'); S.view.showPlan(p); setBanner('离线仿真预览 · 不是真机实时姿态');
  if (!TH.focused) { if (p.kind === 'oblique') S.view.focusWorld([0.0, 0.4, 0.3], [1.5, -0.9, 1.05]); else S.view.focusWorld([0.0, 0.25, 0.3], [1.35, -0.95, 0.95]); TH.focused = true; }
  renderThrowStats(p); $('#th-play').textContent = TH.playing ? '⏸ 暂停' : '▶ 播放'; thApply(); mpPresetOptions();
}
function thLoop(ts) {
  if (TH.playing && TH.plan) {
    const rate = Number($('#th-rate').value); TH.t += Math.min(0.1, (ts - (TH.last || ts)) / 1000) * rate;
    if (TH.t >= TH.plan.t_end) { TH.t = TH.plan.t_end; TH.playing = false; $('#th-play').textContent = '▶ 播放'; }
    thApply();
  }
  TH.last = ts; requestAnimationFrame(thLoop);
}
async function initThrow() {
  const sel = $('#th-preset');
  let presets = {}; try { presets = await (await fetch('/api/throw/presets')).json(); } catch (e) { presets = { mid: { label: '中抛', kind: 'vertical', catch_frac: 0.75 } }; }
  TH.presets = presets;
  for (const [k, v] of Object.entries(presets)) sel.append(el('option', { value: k, text: v.label }));
  sel.value = presets.mid ? 'mid' : Object.keys(presets)[0];
  const syncKind = () => {
    const pr = TH.presets[sel.value]; const obl = pr?.kind === 'oblique';
    $('#th-aim').disabled = obl; $('#th-aim').title = obl ? '斜抛的 J1 起点固定为准备姿态的 -3.1°，由预设决定' : '';
    $('#th-catch').value = String(pr?.catch_frac ?? 0.75);
  };
  syncKind();
  const reload = () => { clearTimeout(TH.timer); TH.timer = setTimeout(() => thLoad(false), 150); };
  sel.onchange = () => { syncKind(); TH.focused = false; reload(); }; $('#th-aim').oninput = reload; $('#th-catch').oninput = reload;
  $('#th-play').onclick = async () => {
    if (!TH.plan || !TH.active) { await thLoad(true); return; }
    if (TH.t >= TH.plan.t_end - 1e-6) TH.t = 0;
    TH.playing = !TH.playing; $('#th-play').textContent = TH.playing ? '⏸ 暂停' : '▶ 播放';
  };
  $('#th-reset').onclick = () => { TH.t = 0; TH.playing = false; $('#th-play').textContent = '▶ 播放'; if (TH.plan) thApply(); };
  $('#th-time').oninput = (e) => { if (!TH.plan) return; TH.playing = false; $('#th-play').textContent = '▶ 播放'; TH.t = Number(e.target.value) * TH.plan.t_end; thApply(); };
  $('#th-exit').onclick = () => { TH.active = false; TH.playing = false; $('#th-play').textContent = '▶ 播放'; S.view?.setPreview(false); $('#viewport-banner').hidden = true; };
  requestAnimationFrame(thLoop);
}


/* ------------------------------------------------------------------ manual pose preview (offline) */
const MP = { active: false, arm: 'left', q: { left: [0, 0, 0, 0, 0, 0], right: [0, 0, 0, 0, 0, 0] }, grip: { left: 60, right: 60 }, rows: [] };
const READY_DEG = [0, 28.6, -57.3, 0, 28.6, 0];
function setBanner(text) { const b = $('#viewport-banner'); b.textContent = text; b.hidden = false; }
function mpBuild() {
  const host = $('#mp-rows'); host.replaceChildren(); MP.rows = [];
  for (let i = 0; i < 7; i++) {
    const g = i === 6; const [lo, hi] = g ? [0, 100] : S.cfg.joint_limits_deg[i];
    const range = el('input', { type: 'range', min: lo, max: hi, step: 0.1, 'aria-label': g ? '夹爪开度' : `J${i + 1}` });
    const num = el('input', { type: 'number', min: lo, max: hi, step: 0.1, 'aria-label': (g ? '夹爪开度' : `J${i + 1} 角度`) + '（数值）' });
    range.style.accentColor = g ? 'var(--ink2)' : `var(--s${i + 1})`;
    const put = (v) => { v = Number(v); if (!Number.isFinite(v)) return; mpSetValue(i, Math.min(hi, Math.max(lo, v))); };
    range.oninput = () => put(range.value); num.onchange = () => put(num.value); num.oninput = () => { if (num.value !== '' && num.value !== '-') put(num.value); };
    host.append(el('div', { class: 'mp-row' }, el('span', { class: 'nm' }, g ? null : el('i', { style: `background:var(--s${i + 1})` }), g ? '夹爪' : `J${i + 1}`), range, num,
      el('span', { class: 'lim', text: g ? '0…100 mm' : `${lo}…${hi}°` })));
    MP.rows.push({ range, num });
  }
  $('#mp-rows').classList.toggle('off', !MP.active);
}
function mpSetValue(i, v) { if (i === 6) MP.grip[MP.arm] = v; else MP.q[MP.arm][i] = v; mpApply(); }
function mpSync() {
  const q = MP.q[MP.arm];
  MP.rows.forEach((r, i) => { const v = i === 6 ? MP.grip[MP.arm] : q[i]; r.range.value = v; if (document.activeElement !== r.num) r.num.value = Number(v).toFixed(1); });
}
function mpApply() {
  if (!MP.active || !S.view) return;
  for (const side of ['left', 'right']) S.view.setPose(side + '_follower', { present: true, q_deg: MP.q[side], gripper_mm: MP.grip[side] });
  mpSync(); mpReadout();
}
function mpInfo(side) { return S.view?.tcpInfo(side + '_follower'); }
function mpReadout() {
  const side = MP.arm, info = mpInfo(side), f3 = (a) => a.map((x) => x.toFixed(3)).join(', ');
  const warn = [];
  if (!info) { $('#mp-readout').replaceChildren(el('div', { class: 'hint', text: '模型加载中…' })); $('#mp-warn').replaceChildren(); $('#mp-text').value = ''; return; }
  const tilt = Math.acos(Math.max(-1, Math.min(1, info.dir[2]))) * 180 / Math.PI, az = Math.atan2(info.dir[1], info.dir[0]) * 180 / Math.PI;
  const q = MP.q[side];
  $('#mp-readout').replaceChildren(thRows([
    [`${side === 'left' ? '左' : '右'}臂 TCP（世界系）`, `(${f3(info.world)}) m`],
    ['TCP（该臂基座系）', `(${f3(info.base)}) m`],
    ['夹爪接近轴（世界系）', `(${f3(info.dir)})；与竖直夹 ${tilt.toFixed(0)}°（90° ≈ 水平）；水平方位 ${az.toFixed(0)}°`],
    ['J2+J3+J5', `${(q[1] + q[2] + q[4]).toFixed(1)}°（= -85° 时夹爪朝正上方）`],
  ]));
  if (info.base[2] < 0.15) warn.push(`TCP 距基座平面仅 ${(info.base[2] * 1000).toFixed(0)} mm，低于 150 mm 安全高度（可能碰桌面）`);
  if (info.world[2] < 0.05) warn.push('TCP 世界系高度低于 50 mm');
  warn.push('只检查了关节限位，没有做自碰撞和双臂碰撞检查');
  $('#mp-warn').replaceChildren(...warn.map((w, i) => el('li', { class: 'warn' }, el('span', { class: 'm', text: '!' }), el('span', { text: w }))));
  $('#mp-text').value = mpText();
}
function mpText() {
  const out = [];
  for (const side of ['left', 'right']) {
    const q = MP.q[side], info = mpInfo(side); if (!info) continue;
    const tilt = Math.acos(Math.max(-1, Math.min(1, info.dir[2]))) * 180 / Math.PI;
    out.push(`${side === 'left' ? '左' : '右'}臂：${q.map((v, i) => `J${i + 1}=${v.toFixed(1)}°`).join(' ')}，夹爪 ${MP.grip[side].toFixed(0)} mm；`
      + `TCP 世界系 (${info.world.map((x) => x.toFixed(3)).join(', ')}) m，基座系 (${info.base.map((x) => x.toFixed(3)).join(', ')}) m；夹爪轴与竖直夹 ${tilt.toFixed(0)}°`);
  }
  return out.join('\n');
}
function mpPoseFrom(kind) {
  const side = MP.arm;
  if (kind === 'live') { const a = S.latest?.arms[side + '_follower']; return a?.present ? { q: a.q_deg.slice(), g: a.gripper_mm } : null; }
  if (kind === 'ready') return { q: READY_DEG.slice() };
  if (kind === 'stow') return S.cfg.stow?.[side] ? { q: S.cfg.stow[side].slice() } : null;
  if (kind === 'zero') return { q: [0, 0, 0, 0, 0, 0] };
  const p = TH.plan; if (!p) return null;
  if (kind === 'throw-start') return { arm: 'left', q: p.left_q_deg[0].slice(), g: p.left_gripper_mm[0] };
  if (kind === 'throw-release') { const i = Math.round(p.t_release / p.dt); return { arm: 'left', q: p.left_q_deg[i].slice(), g: p.left_gripper_mm[i] }; }
  if (kind === 'throw-end') { const i = p.t.length - 1; return { arm: 'left', q: p.left_q_deg[i].slice(), g: p.left_gripper_mm[i] }; }
  if (kind === 'throw-catch') return { arm: 'right', q: p.right_q_deg.slice(), g: 70 };
  return null;
}
function mpPresetOptions() {
  const sel = $('#mp-preset'); const cur = sel.value; sel.replaceChildren();
  const add = (v, t) => sel.append(el('option', { value: v, text: t }));
  add('live', '当前实测姿态'); add('ready', '准备姿态（MIT 测试用）'); add('stow', '归拢位（已保存）'); add('zero', '全部归零');
  if (TH.plan) { add('throw-start', '上抛 · 起始姿态（左臂）'); add('throw-release', '上抛 · 出手姿态（左臂）'); add('throw-end', '上抛 · 结束姿态（左臂）'); add('throw-catch', '上抛 · 右臂接取姿态'); }
  if ([...sel.options].some((o) => o.value === cur)) sel.value = cur;
}
function mpSet(on) {
  if (on === MP.active) return;
  if (on) {
    if (TH.active) { TH.active = false; TH.playing = false; $('#th-play').textContent = '▶ 播放'; }
    for (const side of ['left', 'right']) {
      const a = S.latest?.arms[side + '_follower'];
      MP.q[side] = a?.present ? a.q_deg.slice() : (S.cfg.stow?.[side]?.slice() || [0, 0, 0, 0, 0, 0]);
      MP.grip[side] = a?.present ? a.gripper_mm : 60;
    }
    MP.active = true; S.view.setMode('manual'); setBanner('手动位姿预览 · 不是真机实时姿态'); mpApply();
  } else {
    MP.active = false; S.view?.setMode('live'); $('#viewport-banner').hidden = true;
  }
  $('#mp-rows').classList.toggle('off', !MP.active);
}
function initPose() {
  mpBuild(); mpPresetOptions();
  $('#mp-on').onchange = (e) => mpSet(e.target.checked);
  document.querySelectorAll('#mp-arm button').forEach((b) => { b.onclick = () => { MP.arm = b.dataset.arm; document.querySelectorAll('#mp-arm button').forEach((x) => x.setAttribute('aria-selected', String(x === b))); mpApply(); mpSync(); mpReadout(); }; });
  $('#mp-load').onclick = () => {
    if (!MP.active) { $('#mp-on').checked = true; mpSet(true); }
    const r = mpPoseFrom($('#mp-preset').value);
    if (!r) { $('#mp-copied').textContent = '该姿态不可用（没有实测数据或没有规划）'; return; }
    if (r.arm && r.arm !== MP.arm) { MP.arm = r.arm; document.querySelectorAll('#mp-arm button').forEach((x) => x.setAttribute('aria-selected', String(x.dataset.arm === r.arm))); }
    MP.q[MP.arm] = r.q; if (r.g != null) MP.grip[MP.arm] = r.g; $('#mp-copied').textContent = ''; mpApply();
  };
  $('#mp-move').disabled = !S.cfg.allow_control;
  $('#mp-move').onclick = mpMoveReal;
  $('#mp-parse').onclick = () => {
    const r = mpParse($('#mp-paste').value);
    $('#mp-parse-msg').textContent = r.n ? `已载入 ${r.sides.join('、')}` : '没有识别到“左臂：J1=… ”或“右臂：…”这样的内容';
    if (r.n) { if (!MP.active) { $('#mp-on').checked = true; mpSet(true); } for (const side of r.sides.map((x) => (x === '左臂' ? 'left' : 'right'))) { MP.q[side] = r.q[side]; if (r.g[side] != null) MP.grip[side] = r.g[side]; } mpApply(); }
  };
  $('#mp-copy').onclick = async () => {
    const t = mpText(); $('#mp-text').value = t;
    try { await navigator.clipboard.writeText(t); $('#mp-copied').textContent = '已复制 ✓'; }
    catch (e) { $('#mp-text').select(); $('#mp-copied').textContent = '已选中文本，请按 ⌘C / Ctrl+C 复制'; }
  };
}

function mpParse(text) {
  const out = { n: 0, sides: [], q: {}, g: {} };
  for (const line of String(text).split(/\n+/)) {
    const m = line.match(/^\s*(左臂|右臂)\s*[：:]/); if (!m) continue;
    const side = m[1] === '左臂' ? 'left' : 'right'; const q = [0, 0, 0, 0, 0, 0]; let found = 0;
    for (const j of line.matchAll(/J([1-6])\s*=\s*(-?\d+(?:\.\d+)?)/g)) { q[Number(j[1]) - 1] = Number(j[2]); found++; }
    if (found < 6) continue;
    const g = line.match(/夹爪\s*(\d+(?:\.\d+)?)\s*mm/);
    out.q[side] = q; out.g[side] = g ? Number(g[1]) : null; out.sides.push(m[1]); out.n++;
  }
  return out;
}
async function mpMoveReal() {
  const side = MP.arm, box = $('#mp-move-status'); const label = side === 'left' ? '左' : '右';
  const body = JSON.stringify({ q_deg: MP.q[side], gripper_mm: MP.grip[side] });
  const post = (path) => fetch(path, { method: 'POST', headers: { 'content-type': 'application/json', 'x-demo-token': S.cfg.token }, body });
  let c; try { const r = await post(`/api/pose/${side}/check`); c = await r.json(); if (!r.ok) throw new Error(c.detail || r.statusText); } catch (e) { box.replaceChildren(el('div', { class: 'err', text: '检查失败：' + e.message })); return; }
  const lines = [...c.reasons.map((x) => ['bad', '✗', x]), ...c.warnings.map((x) => ['warn', '!', x])];
  box.replaceChildren(thRows([['路径检查', c.ok ? '通过' : '不通过'], ['最大关节行程', `${c.max_travel_deg}°，约 ${c.est_time_s} s（${c.rate_percent}% 速度）`], ['直线关节路径最低 TCP 高度', `${c.tcp_z_min_m} m`],
    ['目标姿态静态重力力矩', c.gravity_wire_nm.map((x) => x.toFixed(1)).join(' / ') + ' N·m']]), el('ul', { class: 'checks' }, ...lines.map(([k, m, t]) => el('li', { class: k }, el('span', { class: 'm', text: m }), el('span', { text: t })))));
  if (!c.ok) return;
  if (!confirm(`${label}臂将以 ${c.rate_percent}% 速度从当前姿态移动到：\nJ1..J6 = ${c.to_deg.join(', ')}°\n最大关节行程 ${c.max_travel_deg}°，约 ${c.est_time_s} s；直线关节路径最低 TCP 高度 ${c.tcp_z_min_m} m。\n到位后臂保持使能并保持姿态。\n\n请确认：臂周围（前方、上方）清空，你的手在“急停保持”上，其它人没有占用这条臂。`)) return;
  try { const r = await post(`/api/pose/${side}/move`); const d = await r.json(); if (!r.ok) throw new Error(d.detail || r.statusText); } catch (e) { box.append(el('div', { class: 'err', text: '启动失败：' + e.message })); }
}
function renderPoseMove() {
  const m = S.latest?.posemove?.[MP.arm]; if (!m) return;
  const box = $('#mp-move-status'); if (!m.log.length && m.state === 'idle') return;
  let st = box.querySelector('.pm-log'); if (!st) { st = el('div', { class: 'logbox pm-log' }); box.append(st); }
  st.textContent = `状态：${{ idle: '空闲', checking: '预检', moving: '移动中', fault: '故障' }[m.state] || m.state}${m.progress ? `（剩余偏差 ${m.progress.err_max_deg}°）` : ''}\n` + m.log.join('\n') + (m.error ? '\n' + m.error : '');
  $('#mp-move').disabled = !S.cfg.allow_control || ['checking', 'moving'].includes(m.state);
}
