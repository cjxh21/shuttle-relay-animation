'use strict';
/*
 * 迎面接力 · 绕杆接棒教学动画
 * 世界坐标系（单位：米）：
 *   x 沿 100m 直道（0 = 起点，100 = 终点），向右（东）为正；
 *   y 垂直于跑道，向下（南）为正，跑道中心线为 y = 0。
 * 标志杆 A 在 (20, 0)，标志杆 B 在 (80, 0)，中间比赛段为 60 m。
 * A 端几何与 B 端几何严格关于 (50, 0) 中心对称（旋转 180°）。
 */
(() => {
  // ---------------------------------------------------------------- math
  const TAU = Math.PI * 2;
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  const lerp = (a, b, t) => a + (b - a) * t;
  const smooth = (a, b, x) => {
    const t = clamp((x - a) / (b - a), 0, 1);
    return t * t * (3 - 2 * t);
  };
  const angLerp = (a, b, t) => {
    const d = ((((b - a) % TAU) + TAU * 1.5) % TAU) - Math.PI;
    return a + d * t;
  };
  const P = (x, y) => ({ x, y });
  const rotP = (p) => P(100 - p.x, -p.y);

  let seed = 20261031;
  const rnd = () => {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    return seed / 4294967296;
  };

  // ---------------------------------------------------------------- constants
  const T = 10;              // 每棒总时长（模拟秒）
  const SWEEP_T = 0.46;      // 接棒后“杆后由右向左引棒”时长（模拟秒）
  const LEGS = 20;           // 两端各 10 人，共 20 棒
  const START = -6;          // 开场体育场鸟瞰推进
  const END = LEGS * T + 9;
  const ARM = 0.75;          // 躯干中心到伸直右手的距离（米）

  const POLE_A = P(20, 0);
  const POLE_B = P(80, 0);
  const COL_A = '#2563eb';
  const COL_B = '#ea580c';
  const SKIN = '#f1c28d';
  const HAIR = '#241710';
  const WRIST_R = '#ef4444'; // 红色护腕标识右手（惯用手）

  /*
   * B 端（x = 80）绕杆交接几何：
   * - 持棒者 A->B 朝东（h = 0）：自身左侧为北（y < 0），自身右侧为南（y > 0）。
   *   跑向自身左侧 y = -0.90，标志杆 (80,0) 在其右手边；跑过杆线至 HAND_B(80.85, -0.90)，
   *   右手向南伸出 0.75m 到达杆后交接点 MEET_B(80.85, -0.15)。
   * - 接棒者 B 面向西（h = π）：自身左侧为南（y > 0），自身右侧为北（y < 0）。
   *   身体站在杆后左侧 READY_B(80.85, +0.60)，右手（惯用手）向右侧（北）伸出 0.75m
   *   同样到达杆后交接点 MEET_B(80.85, -0.15)。
   * - 交接后 0 ~ SWEEP_T 秒内：接棒者在杆后（x > 80.7）将右手从右侧（y = -0.15）
   *   经杆正后方拉向左侧（y = +0.66），接力棒轨迹绕杆呈标准“U”形，随后向西全速起跑！
   */
  const READY_B = P(80.85, 0.60);
  const HAND_B = P(80.85, -0.90);
  const MEET_B = P(80.85, -0.15);
  const RUN_Y = 0.90;

  // 标准 400m 田径场几何（南直道 0..100m）
  const CY = -41.38, CW = 15.61, CE = 100, R0 = 36.5, LW = 1.22, R1 = R0 + 8 * LW;
  const Y_IN = -4.88, Y_OUT = 4.88;

  // ---------------------------------------------------------------- Centripetal Catmull-Rom paths (无过冲)
  function buildPath(pts, sub = 28) {
    const n = pts.length;
    const d0 = Math.hypot(pts[1].x - pts[0].x, pts[1].y - pts[0].y) || 1;
    const dn = Math.hypot(pts[n - 1].x - pts[n - 2].x, pts[n - 1].y - pts[n - 2].y) || 1;
    const Q = [
      P(pts[0].x - (pts[1].x - pts[0].x), pts[0].y - (pts[1].y - pts[0].y)),
      ...pts,
      P(pts[n - 1].x + (pts[n - 1].x - pts[n - 2].x), pts[n - 1].y + (pts[n - 1].y - pts[n - 2].y)),
    ];
    const xs = [], ys = [];
    const knot = (ti, a, b) => ti + Math.pow(Math.max(1e-4, Math.hypot(b.x - a.x, b.y - a.y)), 0.5);
    const mix = (a, b, ta, tb, u) => {
      const d = tb - ta || 1e-6;
      const w = (u - ta) / d;
      return P(a.x + (b.x - a.x) * w, a.y + (b.y - a.y) * w);
    };
    void d0; void dn;
    for (let i = 1; i < n; i++) {
      const p0 = Q[i - 1], p1 = Q[i], p2 = Q[i + 1], p3 = Q[i + 2];
      const t0 = 0, t1 = knot(t0, p0, p1), t2 = knot(t1, p1, p2), t3 = knot(t2, p2, p3);
      for (let k = i === 1 ? 0 : 1; k <= sub; k++) {
        const u = lerp(t1, t2, k / sub);
        const A1 = mix(p0, p1, t0, t1, u);
        const A2 = mix(p1, p2, t1, t2, u);
        const A3 = mix(p2, p3, t2, t3, u);
        const B1 = mix(A1, A2, t0, t2, u);
        const B2 = mix(A2, A3, t1, t3, u);
        const C = mix(B1, B2, t1, t2, u);
        xs.push(C.x);
        ys.push(C.y);
      }
    }
    const cum = [0];
    for (let i = 1; i < xs.length; i++) {
      cum.push(cum[i - 1] + Math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]));
    }
    return { xs, ys, cum, L: cum[cum.length - 1] };
  }

  function pathAt(pa, s) {
    s = clamp(s, 0, pa.L);
    let lo = 0, hi = pa.cum.length - 1;
    while (hi - lo > 1) {
      const m = (lo + hi) >> 1;
      if (pa.cum[m] <= s) lo = m; else hi = m;
    }
    const seg = pa.cum[hi] - pa.cum[lo] || 1e-9;
    const u = (s - pa.cum[lo]) / seg;
    return {
      x: lerp(pa.xs[lo], pa.xs[hi], u),
      y: lerp(pa.ys[lo], pa.ys[hi], u),
      h: Math.atan2(pa.ys[hi] - pa.ys[lo], pa.xs[hi] - pa.xs[lo]),
    };
  }

  function makeProfile(g, Td, L) {
    const N = 800, dt = Td / N, s = [0];
    let acc = 0;
    for (let k = 1; k <= N; k++) {
      acc += g((k - 0.5) * dt) * dt;
      s.push(acc);
    }
    const sc = L / acc;
    for (let k = 0; k <= N; k++) s[k] *= sc;
    return {
      T: Td,
      sAt(t) {
        const i = clamp(t / dt, 0, N);
        const a = Math.floor(i), b = Math.min(a + 1, N);
        return lerp(s[a], s[b], i - a);
      },
      vAt(t) { return sc * g(clamp(t, 0, Td)); },
    };
  }

  function makeDecel(L, v0, vw = 1.45, tau = 0.8) {
    const dt = 0.01, s = [0];
    let acc = 0, t = 0;
    while (acc < L && s.length < 50000) {
      t += dt;
      acc += (vw + (v0 - vw) * Math.exp(-(t - dt / 2) / tau)) * dt;
      s.push(Math.min(acc, L));
    }
    const n = s.length - 1;
    return {
      T: t,
      sAt(tt) {
        const i = clamp(tt / dt, 0, n);
        const a = Math.floor(i), b = Math.min(a + 1, n);
        return lerp(s[a], s[b], i - a);
      },
      vAt(tt) { return tt >= t ? 0 : vw + (v0 - vw) * Math.exp(-tt / tau); },
    };
  }

  // 第 1 棒（t=0 直接从 A 端起跑）与后续接棒起跑路径
  const pathFirstAB = buildPath([
    rotP(READY_B), P(21.6, -0.82), P(24.5, -RUN_Y), P(50, -RUN_Y), P(76, -RUN_Y), HAND_B,
  ]);
  const sprintPtsBA = [P(80.72, RUN_Y), P(76.5, RUN_Y), P(50, RUN_Y), P(23.5, RUN_Y), rotP(HAND_B)];
  const sprintPtsAB = sprintPtsBA.map(rotP);
  const pathSprintBA = buildPath(sprintPtsBA);
  const pathSprintAB = buildPath(sprintPtsAB);

  const firstProf = makeProfile(
    (t) => (1 - Math.exp(-t / 0.65)) * (1 - 0.40 * smooth(T - 1.1, T, t)),
    T,
    pathFirstAB.L
  );
  const RUN_DUR = T - SWEEP_T;
  const sprintProf = makeProfile(
    (t) => (0.28 + 0.72 * (1 - Math.exp(-t / 0.55))) * (1 - 0.40 * smooth(RUN_DUR - 1.1, RUN_DUR, t)),
    RUN_DUR,
    pathSprintBA.L
  );
  const V_END = sprintProf.vAt(RUN_DUR);

  const slotXB = (k) => (k <= 1 ? lerp(READY_B.x, READY_B.x + 1.85, k) : READY_B.x + 0.90 + 0.95 * k);
  function slotPos(side, k) {
    const p = P(slotXB(Math.max(0, k)), READY_B.y);
    return side === 'B' ? p : rotP(p);
  }
  function postPtsB(kk) {
    const sx = slotXB(kk);
    const fx = Math.max(sx + 1.0, 86.2);
    return [HAND_B, P(82.6, -1.20), P(84.8, -2.05), P(fx, -2.25), P(sx + 1.15, -0.85), P(sx, READY_B.y)];
  }

  // 杆后由右向左引棒阶段（0 <= tau <= SWEEP_T）
  function sweepState(p, side, tau) {
    const u = smooth(0, SWEEP_T, tau);
    const bx = lerp(READY_B.x, 80.72, u) + 0.14 * Math.sin(u * Math.PI);
    const by = lerp(READY_B.y, RUN_Y, u);
    const wp = side === 'B' ? P(bx, by) : rotP(P(bx, by));
    const h = side === 'B' ? Math.PI : 0;
    return {
      p,
      x: wp.x,
      y: wp.y,
      h,
      phase: 0,
      amp: 0.30 * u,
      e: 1 - u,
      mode: 'sweep',
    };
  }

  // ---------------------------------------------------------------- people & schedule
  const people = [];
  for (let i = 0; i < 10; i++) people.push({ team: 'A', num: i + 1, leg: 2 * i, start: 'A', end: 'B' });
  for (let i = 0; i < 10; i++) people.push({ team: 'B', num: i + 1, leg: 2 * i + 1, start: 'B', end: 'A' });
  people.forEach((p) => {
    p.name = p.team + p.num;
    p.sprintPath = p.start === 'A' ? pathSprintAB : pathSprintBA;
  });
  const legRunner = [];
  people.forEach((p) => { legRunner[p.leg] = p; });

  const queues = { A: [], B: [] };
  people.forEach((p) => {
    p.startMember = { p, entry: -Infinity, order: p.num, exit: p.leg * T, arrived: false };
    queues[p.start].push(p.startMember);
  });
  for (let i = 0; i < LEGS; i++) {
    const p = legRunner[i], te = (i + 1) * T, side = p.end, est = te + 6;
    const kk = queues[side].filter((m) => m.entry < est && m.exit > est).length;
    const ptsB = postPtsB(kk);
    p.postPath = buildPath(side === 'B' ? ptsB : ptsB.map(rotP));
    p.postProf = makeDecel(p.postPath.L, i === 0 ? firstProf.vAt(T) : V_END);
    p.postDur = p.postProf.T;
    p.arrH = pathAt(p.postPath, p.postPath.L).h;
    p.endMember = { p, entry: te + p.postDur, order: 100 + i, exit: Infinity, arrived: true };
    queues[side].push(p.endMember);
  }

  function qIndex(side, mem, t) {
    let k = 0;
    for (const m of queues[side]) {
      if (m === mem) continue;
      if (m.entry < mem.entry || (m.entry === mem.entry && m.order < mem.order)) {
        k += t < m.exit ? 1 : 1 - smooth(m.exit + 0.45, m.exit + 1.85, t);
      }
    }
    return k;
  }

  function queueState(p, side, mem, t) {
    const k = qIndex(side, mem, t), k2 = qIndex(side, mem, t - 0.05);
    const a = slotPos(side, k), b = slotPos(side, k2);
    const sp = Math.hypot(a.x - b.x, a.y - b.y) / 0.05;
    const face = side === 'B' ? Math.PI : 0;
    const h = mem.arrived ? angLerp(p.arrH, face, smooth(0, 0.9, t - mem.entry)) : face;
    let e = 0;
    const ts = p.leg * T;
    if (side === p.start && p.leg > 0) e = smooth(ts - 2.6, ts - 1.85, t);
    return { p, x: a.x, y: a.y, h, phase: (a.x / 0.8) * TAU, amp: clamp(sp / 2.5, 0, 0.6), e, mode: 'queue', k };
  }

  function stateOf(p, t) {
    const ts = p.leg * T, te = ts + T;
    if (t < ts) return queueState(p, p.start, p.startMember, t);
    if (t < te) {
      const tau = t - ts;
      if (p.leg === 0) {
        const s = firstProf.sAt(tau), pt = pathAt(pathFirstAB, s), v = firstProf.vAt(tau);
        const e = smooth(T - 0.78, T - 0.10, tau);
        return { p, x: pt.x, y: pt.y, h: pt.h, phase: (s / 1.8) * TAU, amp: clamp(v / 5, 0, 1), e, mode: 'run' };
      }
      if (tau < SWEEP_T) return sweepState(p, p.start, tau);
      const rt = tau - SWEEP_T;
      const s = sprintProf.sAt(rt), pt = pathAt(p.sprintPath, s), v = sprintProf.vAt(rt);
      const e = p.leg < LEGS - 1 ? smooth(RUN_DUR - 0.78, RUN_DUR - 0.10, rt) : 0;
      return { p, x: pt.x, y: pt.y, h: pt.h, phase: (s / 1.8) * TAU, amp: clamp(v / 5, 0, 1), e, mode: 'run' };
    }
    const tau = t - te;
    if (tau < p.postDur) {
      const s = p.postProf.sAt(tau), pt = pathAt(p.postPath, s), v = p.postProf.vAt(tau);
      const e = p.leg < LEGS - 1 ? 1 - smooth(0.02, 0.38, tau) : 0;
      return { p, x: pt.x, y: pt.y, h: pt.h, phase: (s / 1.1) * TAU, amp: clamp(v / 5, 0.25, 1), e, mode: 'post' };
    }
    return queueState(p, p.end, p.endMember, t);
  }

  // 计算右手与左手世界坐标（保证 e=1 时不论 mag 大小都精确落在交接点）
  function handsOf(st, mag = 1) {
    const c = Math.cos(st.h), s = Math.sin(st.h);
    const fx = c, fy = s, rx = -s, ry = c;
    const sw = Math.sin(st.phase) * st.amp;
    const lxW = -0.24 * mag * sw * (1 - st.e);
    const lyW = lerp(0.24 * mag, ARM, st.e);
    const llxW = 0.24 * mag * sw;
    const llyW = -0.24 * mag;
    return {
      R: P(st.x + fx * lxW + rx * lyW, st.y + fy * lxW + ry * lyW),
      L: P(st.x + fx * llxW + rx * llyW, st.y + fy * llxW + ry * llyW),
      fx, fy, rx, ry,
    };
  }

  const legAt = (t) => (t < 0 ? 0 : Math.min(Math.floor(t / T), LEGS - 1));
  const holderAt = (t) => legRunner[legAt(t)];

  // ---------------------------------------------------------------- canvas & camera
  const canvas = document.getElementById('stage');
  const ctx = canvas.getContext('2d');
  const wrap = document.getElementById('wrap');
  let W = 1200, H = 620, dpr = 1;
  let cam = { cx: 50, cy: 0, s: 10 };
  let FS = 14;

  function resize() {
    const r = wrap.getBoundingClientRect();
    W = Math.max(300, Math.round(r.width));
    H = W < 640
      ? Math.round(Math.min(W * 0.96, window.innerHeight * 0.58))
      : Math.round(Math.min(W * 9 / 16, window.innerHeight * 0.65));
    H = Math.max(H, 260);
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
    canvas.style.height = H + 'px';
    FS = clamp(W / 80, 11, 15.5);
  }
  window.addEventListener('resize', resize);

  const setWorld = () => ctx.setTransform(
    dpr * cam.s, 0, 0, dpr * cam.s,
    dpr * (W / 2 - cam.cx * cam.s), dpr * (H / 2 - cam.cy * cam.s)
  );
  const setScreen = () => ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const w2s = (p) => P((p.x - cam.cx) * cam.s + W / 2, (p.y - cam.cy) * cam.s + H / 2);
  const px = (n) => n / cam.s;

  function blendCam(a, b, w) {
    const s = Math.exp(lerp(Math.log(a.s), Math.log(b.s), w));
    const k = Math.abs(a.s - b.s) < 1e-6 ? w : (1 / a.s - 1 / s) / (1 / a.s - 1 / b.s);
    return { cx: lerp(a.cx, b.cx, k), cy: lerp(a.cy, b.cy, k), s };
  }
  const overviewCam = () => ({ cx: 58, cy: -38, s: Math.min(W / 250, H / 175) });
  const relayCam = () => ({ cx: 50, cy: 0.5, s: Math.min(W / 97, H / 21) });
  const closeCam = (pole, off) => ({ cx: pole.x + off, cy: -0.05, s: Math.min(W / 12.2, H / 7.0) });

  // ---------------------------------------------------------------- stadium
  function ovalPath(R) {
    ctx.beginPath();
    ctx.moveTo(CW, CY + R);
    ctx.lineTo(CE, CY + R);
    ctx.arc(CE, CY, R, Math.PI / 2, -Math.PI / 2, true);
    ctx.lineTo(CW, CY - R);
    ctx.arc(CW, CY, R, -Math.PI / 2, Math.PI / 2, true);
    ctx.closePath();
  }
  function ovalPoint(R, u) {
    const S = CE - CW;
    if (u < S) return P(CW + u, CY + R);
    u -= S;
    if (u < Math.PI * R) {
      const th = Math.PI / 2 - u / R;
      return P(CE + R * Math.cos(th), CY + R * Math.sin(th));
    }
    u -= Math.PI * R;
    if (u < S) return P(CE - u, CY - R);
    u -= S;
    const th = -Math.PI / 2 - u / R;
    return P(CW + R * Math.cos(th), CY + R * Math.sin(th));
  }
  function distOval(x, y) {
    if (x >= CW && x <= CE) return Math.abs(y - CY);
    return Math.hypot(x - (x < CW ? CW : CE), y - CY);
  }

  const CROWD_COLS = ['#e74c3c', '#3498db', '#f1c40f', '#ecf0f1', '#2ecc71', '#9b59b6', '#e67e22', '#1f2d3d', '#ff8fb1'];
  const crowd = CROWD_COLS.map(() => []);
  for (let R = R1 + 6.9; R < R1 + 30.5; R += 1.25) {
    const per = 2 * (CE - CW) + TAU * R;
    for (let u = 0; u < per; u += 0.85) {
      if (u % 15 < 1.4) continue;
      const pt = ovalPoint(R, u);
      const occ = pt.y > 0 ? 0.72 : 0.5;
      if (rnd() > occ) continue;
      crowd[Math.floor(rnd() * CROWD_COLS.length)].push(pt.x + (rnd() - 0.5) * 0.15, pt.y + (rnd() - 0.5) * 0.15);
    }
  }
  const trees = [];
  for (let i = 0; i < 420; i++) {
    const x = -130 + rnd() * 370, y = -175 + rnd() * 270;
    if (distOval(x, y) < R1 + 40) continue;
    trees.push({ x, y, r: 2 + rnd() * 2.6, c: rnd() });
  }
  const LIGHTS = [P(CW - 56, CY - 62), P(CW - 56, CY + 62), P(CE + 56, CY - 62), P(CE + 56, CY + 62)];

  function drawStadium() {
    const s = cam.s;
    ctx.fillStyle = '#4c7a40';
    ctx.fillRect(-400, -400, 900, 700);
    ovalPath(R1 + 47); ctx.fillStyle = '#9b9a92'; ctx.fill();
    ovalPath(R1 + 40); ctx.fillStyle = '#5d8a4c'; ctx.fill();
    if (s < 30) {
      for (const t of trees) {
        ctx.fillStyle = 'rgba(0,0,0,.22)';
        ctx.beginPath(); ctx.arc(t.x + t.r * 0.45, t.y + t.r * 0.35, t.r, 0, TAU); ctx.fill();
        ctx.fillStyle = t.c < 0.5 ? '#2f6b2b' : '#3a7d33';
        ctx.beginPath(); ctx.arc(t.x, t.y, t.r, 0, TAU); ctx.fill();
        ctx.fillStyle = 'rgba(255,255,255,.08)';
        ctx.beginPath(); ctx.arc(t.x - t.r * 0.3, t.y - t.r * 0.3, t.r * 0.5, 0, TAU); ctx.fill();
      }
      for (const l of LIGHTS) {
        ctx.strokeStyle = 'rgba(0,0,0,.25)'; ctx.lineWidth = 1.2;
        ctx.beginPath(); ctx.moveTo(l.x, l.y); ctx.lineTo(l.x + 16, l.y + 11); ctx.stroke();
        ctx.fillStyle = '#4b5563'; ctx.fillRect(l.x - 3, l.y - 1.6, 6, 3.2);
        ctx.fillStyle = '#fef9c3'; ctx.fillRect(l.x - 2.6, l.y - 1.1, 5.2, 2.2);
      }
    }
    ovalPath(R1 + 33); ctx.fillStyle = '#4a5059'; ctx.fill();
    ovalPath(R1 + 31); ctx.fillStyle = '#8c929b'; ctx.fill();
    if (s < 45) {
      ctx.strokeStyle = 'rgba(35,40,48,.35)'; ctx.lineWidth = Math.max(0.12, px(0.5));
      for (let R = R1 + 6.25; R < R1 + 31; R += 1.25) { ovalPath(R); ctx.stroke(); }
      const sz = 0.6;
      for (let c = 0; c < crowd.length; c++) {
        const arr = crowd[c];
        ctx.fillStyle = CROWD_COLS[c];
        ctx.beginPath();
        for (let i = 0; i < arr.length; i += 2) ctx.rect(arr[i] - sz / 2, arr[i + 1] - sz / 2, sz, sz);
        ctx.fill();
      }
      const sx = CW - R1 - 33.5;
      ctx.fillStyle = '#111827'; ctx.fillRect(sx - 2, CY - 13, 3.5, 26);
      ctx.fillStyle = '#0ea5e9'; ctx.fillRect(sx - 1.4, CY - 12, 1.4, 24);
    }
    // 南侧主看台顶棚
    ctx.fillStyle = 'rgba(232,236,241,.93)';
    ctx.fillRect(22, CY + R1 + 4.2, 72, 27.5);
    ctx.strokeStyle = 'rgba(100,110,125,.6)'; ctx.lineWidth = Math.max(0.15, px(0.6));
    for (let x = 22; x <= 94; x += 6) {
      ctx.beginPath(); ctx.moveTo(x, CY + R1 + 4.2); ctx.lineTo(x, CY + R1 + 31.7); ctx.stroke();
    }
    ctx.beginPath(); ctx.moveTo(22, CY + R1 + 18); ctx.lineTo(94, CY + R1 + 18); ctx.stroke();
    ovalPath(R1 + 5.4); ctx.fillStyle = '#3d434b'; ctx.fill();
    ovalPath(R1 + 5); ctx.fillStyle = '#b8ae9c'; ctx.fill();
    ctx.fillStyle = 'rgba(0,0,0,.10)'; ctx.fillRect(22, 7.6, 72, 2.3);
    // 塑胶跑道
    ctx.fillStyle = '#c4553f';
    ovalPath(R1); ctx.fill();
    ctx.fillRect(-3, Y_IN, CW + 3, Y_OUT - Y_IN);
    ctx.fillRect(CE - 1, Y_IN, 7, Y_OUT - Y_IN);
    // 内场草坪与足球场标线
    ctx.save();
    ovalPath(R0); ctx.clip();
    ctx.fillStyle = '#3f9446'; ctx.fillRect(-30, -90, 160, 90);
    ctx.fillStyle = '#48a24f';
    for (let x = -30; x < 130; x += 12) ctx.fillRect(x, -90, 6, 90);
    const pcx = (CW + CE) / 2;
    ctx.strokeStyle = 'rgba(255,255,255,.8)'; ctx.lineWidth = Math.max(0.12, px(0.8));
    ctx.strokeRect(pcx - 52.5, CY - 34, 105, 68);
    ctx.beginPath(); ctx.moveTo(pcx, CY - 34); ctx.lineTo(pcx, CY + 34); ctx.stroke();
    ctx.beginPath(); ctx.arc(pcx, CY, 9.15, 0, TAU); ctx.stroke();
    for (const sgn of [-1, 1]) {
      const gx = pcx + sgn * 52.5;
      ctx.strokeRect(sgn < 0 ? gx : gx - 16.5, CY - 20.15, 16.5, 40.3);
      ctx.strokeRect(sgn < 0 ? gx : gx - 5.5, CY - 9.15, 5.5, 18.3);
    }
    ctx.restore();
    // 分道线与起终点线
    ctx.strokeStyle = 'rgba(255,255,255,.92)';
    ctx.lineWidth = Math.max(0.05, px(0.7));
    for (let j = 0; j <= 8; j++) {
      ovalPath(R0 + j * LW); ctx.stroke();
      const y = Y_IN + j * LW;
      ctx.beginPath(); ctx.moveTo(-3, y); ctx.lineTo(CW, y); ctx.moveTo(CE, y); ctx.lineTo(CE + 6, y); ctx.stroke();
    }
    ctx.lineWidth = Math.max(0.08, px(1.4));
    ctx.beginPath(); ctx.moveTo(0, Y_IN); ctx.lineTo(0, Y_OUT); ctx.moveTo(100, Y_IN); ctx.lineTo(100, Y_OUT); ctx.stroke();
  }

  function drawRelayMarks(alpha) {
    if (alpha <= 0.01) return;
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.fillStyle = 'rgba(255,220,40,0.12)';
    ctx.fillRect(20, Y_IN, 60, Y_OUT - Y_IN);
    ctx.strokeStyle = '#ffd60a';
    ctx.lineWidth = Math.max(0.07, px(1.6));
    ctx.setLineDash([px(7), px(5)]);
    ctx.beginPath(); ctx.moveTo(20, Y_IN); ctx.lineTo(20, Y_OUT); ctx.moveTo(80, Y_IN); ctx.lineTo(80, Y_OUT); ctx.stroke();
    ctx.setLineDash([]);
    const dy = -6.7;
    ctx.strokeStyle = 'rgba(255,255,255,.9)';
    ctx.lineWidth = px(1.4);
    ctx.beginPath(); ctx.moveTo(20, dy); ctx.lineTo(80, dy); ctx.stroke();
    ctx.strokeStyle = 'rgba(255,255,255,.55)';
    ctx.beginPath(); ctx.moveTo(0, dy); ctx.lineTo(20, dy); ctx.moveTo(80, dy); ctx.lineTo(100, dy); ctx.stroke();
    for (const x of [0, 20, 80, 100]) {
      ctx.beginPath(); ctx.moveTo(x, dy - px(6)); ctx.lineTo(x, dy + px(6)); ctx.stroke();
    }
    ctx.restore();
  }

  function drawRelayLabels(alpha) {
    if (alpha < 0.02) return;
    const f = FS, dy = -6.7;
    pill(w2s(P(50, dy)), '60 m 迎面接力比赛区', { bg: 'rgba(250,204,21,.96)', fg: '#111827', size: f, alpha, bold: true });
    pill(w2s(P(10, dy)), '前段 20 m', { size: f * 0.82, alpha: alpha * 0.88 });
    pill(w2s(P(90, dy)), '后段 20 m', { size: f * 0.82, alpha: alpha * 0.88 });
    pill(w2s(P(2.2, Y_OUT + 1.45)), '100 m 起点 (0 m)', { size: f * 0.78, alpha });
    pill(w2s(P(97.8, Y_OUT + 1.45)), '100 m 终点 (100 m)', { size: f * 0.78, alpha });
    pill(w2s(P(20, Y_OUT + 1.45)), '20 m · 标志杆 A', { size: f * 0.8, alpha, bg: 'rgba(37,99,235,.94)' });
    pill(w2s(P(80, Y_OUT + 1.45)), '80 m · 标志杆 B', { size: f * 0.8, alpha, bg: 'rgba(234,88,12,.94)' });
    pill(w2s(P(13.5, -2.55)), 'A 端纵队 · 10 位同学', { size: f * 0.8, alpha, bg: 'rgba(37,99,235,.90)' });
    pill(w2s(P(86.5, 2.55)), 'B 端纵队 · 10 位同学', { size: f * 0.8, alpha, bg: 'rgba(234,88,12,.90)' });
  }

  function drawPole(p, glow) {
    const r = Math.max(0.085, px(5));
    ctx.strokeStyle = 'rgba(0,0,0,.30)';
    ctx.lineWidth = r * 0.9;
    ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(p.x + 0.85, p.y + 0.52); ctx.stroke();
    if (glow > 0) {
      ctx.fillStyle = `rgba(250,204,21,${0.28 * glow})`;
      ctx.beginPath(); ctx.arc(p.x, p.y, r * 3.2, 0, TAU); ctx.fill();
    }
    ctx.fillStyle = '#ffffff'; ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, TAU); ctx.fill();
    ctx.fillStyle = '#e11d48'; ctx.beginPath(); ctx.arc(p.x, p.y, r * 0.68, 0, TAU); ctx.fill();
    ctx.fillStyle = '#ffffff'; ctx.beginPath(); ctx.arc(p.x, p.y, r * 0.32, 0, TAU); ctx.fill();
    ctx.strokeStyle = '#111827'; ctx.lineWidth = px(1.2); ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, TAU); ctx.stroke();
  }

  // ---------------------------------------------------------------- people & baton
  function drawPerson(st, mag, highlightHand = false) {
    const col = st.p.team === 'A' ? COL_A : COL_B;
    const sw = Math.sin(st.phase) * st.amp;
    ctx.save();
    ctx.translate(st.x, st.y);
    ctx.scale(mag, mag);
    // 投影
    ctx.fillStyle = 'rgba(0,0,0,.22)';
    ctx.beginPath(); ctx.ellipse(0.07, 0.09, 0.30, 0.25, 0, 0, TAU); ctx.fill();
    ctx.rotate(st.h);
    ctx.lineCap = 'round';
    // 双腿与跑鞋
    const fr = 0.32 * sw;
    ctx.strokeStyle = SKIN; ctx.lineWidth = 0.075;
    ctx.beginPath(); ctx.moveTo(0, 0.09); ctx.lineTo(fr, 0.10); ctx.moveTo(0, -0.09); ctx.lineTo(-fr, -0.10); ctx.stroke();
    ctx.fillStyle = '#1f2937';
    ctx.beginPath(); ctx.ellipse(fr + 0.03, 0.10, 0.075, 0.042, 0, 0, TAU); ctx.fill();
    ctx.beginPath(); ctx.ellipse(-fr + 0.03, -0.10, 0.075, 0.042, 0, 0, TAU); ctx.fill();
    // 右臂（+y 局部坐标 = 自身右侧）与左臂（-y 局部坐标 = 自身左侧）
    const rh = P(-0.24 * sw * (1 - st.e), lerp(0.24, ARM / mag, st.e));
    const lh = P(0.24 * sw, -0.24);
    ctx.strokeStyle = SKIN; ctx.lineWidth = 0.072;
    ctx.beginPath(); ctx.moveTo(0, 0.18); ctx.lineTo(rh.x, rh.y); ctx.moveTo(0, -0.18); ctx.lineTo(lh.x, lh.y); ctx.stroke();
    // 右手腕红色护腕（清晰标识惯用手/右手）
    const wdx = rh.x, wdy = rh.y - 0.18, wl = Math.hypot(wdx, wdy) || 1;
    const wx = rh.x - (wdx / wl) * 0.075, wy = rh.y - (wdy / wl) * 0.075;
    ctx.strokeStyle = WRIST_R; ctx.lineWidth = 0.088;
    ctx.beginPath();
    ctx.moveTo(wx - (wdx / wl) * 0.02, wy - (wdy / wl) * 0.02);
    ctx.lineTo(wx + (wdx / wl) * 0.02, wy + (wdy / wl) * 0.02);
    ctx.stroke();
    if (highlightHand) {
      ctx.strokeStyle = 'rgba(250,204,21,.9)';
      ctx.lineWidth = 0.028;
      ctx.beginPath(); ctx.arc(rh.x, rh.y, 0.095, 0, TAU); ctx.stroke();
    }
    ctx.fillStyle = SKIN;
    ctx.beginPath(); ctx.arc(rh.x, rh.y, 0.052, 0, TAU); ctx.fill();
    ctx.beginPath(); ctx.arc(lh.x, lh.y, 0.045, 0, TAU); ctx.fill();
    // 躯干球衣
    ctx.fillStyle = col;
    ctx.beginPath(); ctx.ellipse(0, 0, 0.135, 0.235, 0, 0, TAU); ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,.35)'; ctx.lineWidth = 0.018; ctx.stroke();
    // 头部与朝向帽檐/面部指示
    ctx.fillStyle = '#ffffff';
    ctx.beginPath(); ctx.ellipse(0.085, 0, 0.06, 0.075, 0, 0, TAU); ctx.fill();
    ctx.fillStyle = HAIR;
    ctx.beginPath(); ctx.arc(-0.005, 0, 0.105, 0, TAU); ctx.fill();
    ctx.fillStyle = SKIN;
    ctx.beginPath(); ctx.ellipse(0.062, 0, 0.046, 0.072, 0, 0, TAU); ctx.fill();
    ctx.restore();
  }

  function drawBaton(st, mag, glow) {
    const hd = handsOf(st, mag);
    const bx = hd.R.x, by = hd.R.y;
    const L = 0.18 * mag;
    if (glow > 0) {
      ctx.fillStyle = `rgba(255,214,0,${0.34 * glow})`;
      ctx.beginPath(); ctx.arc(bx, by, 0.30 * mag, 0, TAU); ctx.fill();
    }
    ctx.lineCap = 'round';
    ctx.strokeStyle = '#5c4300'; ctx.lineWidth = 0.09 * mag;
    ctx.beginPath(); ctx.moveTo(bx - hd.fx * L, by - hd.fy * L); ctx.lineTo(bx + hd.fx * L, by + hd.fy * L); ctx.stroke();
    ctx.strokeStyle = '#ffd400'; ctx.lineWidth = 0.058 * mag;
    ctx.beginPath(); ctx.moveTo(bx - hd.fx * L, by - hd.fy * L); ctx.lineTo(bx + hd.fx * L, by + hd.fy * L); ctx.stroke();
    return P(bx, by);
  }

  function ring(st, mag, color, a) {
    ctx.save();
    ctx.strokeStyle = color; ctx.globalAlpha = a; ctx.lineWidth = Math.max(0.04, px(2.2));
    ctx.beginPath(); ctx.arc(st.x, st.y, 0.48 * mag, 0, TAU); ctx.stroke();
    ctx.restore();
  }

  // ---------------------------------------------------------------- UI / annotation primitives
  function roundRect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
  }

  function pill(p, text, o = {}) {
    const size = o.size || FS, alpha = o.alpha == null ? 1 : o.alpha;
    if (alpha <= 0.01) return null;
    if (!o.clamp && (p.x < -60 || p.x > W + 60 || p.y < -40 || p.y > H + 40)) return null;
    setScreen();
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.font = `${o.bold ? '700 ' : '600 '}${size}px -apple-system, "PingFang SC", "Microsoft YaHei", sans-serif`;
    const w = ctx.measureText(text).width + size * 1.12, h = size * 1.72;
    let x = p.x - w / 2, y = p.y - h / 2;
    if (o.anchor === 'left') x = p.x;
    if (o.anchor === 'right') x = p.x - w;
    if (o.clamp !== false) {
      x = clamp(x, 6, W - w - 6);
      y = clamp(y, 6, H - h - 6);
    }
    ctx.fillStyle = o.bg || 'rgba(15,23,42,.88)';
    roundRect(x, y, w, h, h / 2); ctx.fill();
    if (o.border) { ctx.strokeStyle = o.border; ctx.lineWidth = 1.8; ctx.stroke(); }
    ctx.fillStyle = o.fg || '#fff';
    ctx.textBaseline = 'middle'; ctx.textAlign = 'center';
    ctx.fillText(text, x + w / 2, y + h / 2 + 0.5);
    ctx.restore();
    return { x, y, w, h };
  }

  function callout(target, at, text, o = {}) {
    const a = o.alpha == null ? 1 : o.alpha;
    if (a <= 0.01) return;
    const ts = w2s(target), ls = w2s(at);
    if (ts.x < -80 || ts.x > W + 80 || ts.y < -80 || ts.y > H + 80) return;
    setScreen();
    ctx.save();
    ctx.globalAlpha = a;
    ctx.strokeStyle = o.line || 'rgba(255,255,255,.9)';
    ctx.lineWidth = 1.7;
    ctx.beginPath(); ctx.moveTo(ls.x, ls.y); ctx.lineTo(ts.x, ts.y); ctx.stroke();
    ctx.fillStyle = o.line || '#fff';
    ctx.beginPath(); ctx.arc(ts.x, ts.y, 3.2, 0, TAU); ctx.fill();
    ctx.restore();
    pill(ls, text, { ...o, clamp: true });
  }

  function arrowW(pts, color, wpx, alpha, dash) {
    if (alpha <= 0.01) return;
    setWorld();
    ctx.save();
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color; ctx.fillStyle = color;
    ctx.lineWidth = px(wpx); ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    if (dash) ctx.setLineDash([px(dash[0]), px(dash[1])]);
    const n = pts.length;
    ctx.beginPath(); ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < n - 1; i++) {
      const m = P((pts[i].x + pts[i + 1].x) / 2, (pts[i].y + pts[i + 1].y) / 2);
      ctx.quadraticCurveTo(pts[i].x, pts[i].y, m.x, m.y);
    }
    const a = pts[n - 1], b = pts[n - 2];
    const ang = Math.atan2(a.y - b.y, a.x - b.x), hl = px(wpx * 3.6);
    const base = P(a.x - Math.cos(ang) * hl * 0.7, a.y - Math.sin(ang) * hl * 0.7);
    ctx.lineTo(base.x, base.y);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(a.x - hl * Math.cos(ang - 0.46), a.y - hl * Math.sin(ang - 0.46));
    ctx.lineTo(a.x - hl * Math.cos(ang + 0.46), a.y - hl * Math.sin(ang + 0.46));
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }

  function drawZone(isB, alpha, bad = false) {
    if (alpha <= 0.01) return;
    const M = (p) => (isB ? p : rotP(p));
    setWorld();
    ctx.save();
    ctx.globalAlpha = alpha;
    const c = [M(P(80, -2.55)), M(P(83.6, -2.55)), M(P(83.6, 2.55)), M(P(80, 2.55))];
    ctx.fillStyle = bad ? 'rgba(239,68,68,.12)' : 'rgba(34,197,94,.16)';
    ctx.beginPath();
    c.forEach((q, i) => (i ? ctx.lineTo(q.x, q.y) : ctx.moveTo(q.x, q.y)));
    ctx.closePath(); ctx.fill();
    const pxX = isB ? 80 : 20;
    ctx.strokeStyle = bad ? '#ef4444' : '#ffffff';
    ctx.lineWidth = px(2.1);
    ctx.setLineDash([px(8), px(6)]);
    ctx.beginPath(); ctx.moveTo(pxX, Y_IN); ctx.lineTo(pxX, Y_OUT); ctx.stroke();
    ctx.restore();
  }

  function zoneLabels(isB, alpha) {
    if (alpha <= 0.01) return;
    const pxX = isB ? 80 : 20;
    const zoneX = isB ? 82.35 : 17.65;
    pill(w2s(P(zoneX, -2.28)), '杆后（合法交接区）', { size: FS * 0.80, alpha: alpha * 0.94, bg: 'rgba(21,128,61,.92)' });
    pill(w2s(P(pxX, -2.85)), '杆线：必须在杆后完成绕杆交接', { size: FS * 0.78, alpha: alpha * 0.92 });
  }

  function drawNameBadge(st, mag, alpha) {
    if (alpha <= 0.01) return;
    const sp = w2s(st);
    if (sp.x < 18 || sp.x > W - 18 || sp.y < 18 || sp.y > H - 60) return;
    // 名字标在远离中心线 y=0 的外侧，绝不遮挡伸向中间的右手与接力棒
    const outerSign = st.y < 0 ? -1 : 1;
    const offPx = 0.46 * mag * cam.s + 13;
    pill(P(sp.x, sp.y + outerSign * offPx), st.p.name, {
      size: clamp(FS * 0.72, 10, 12.5),
      alpha,
      clamp: false,
      bg: st.p.team === 'A' ? 'rgba(37,99,235,.92)' : 'rgba(234,88,12,.92)',
    });
  }

  // 绕杆交接四步核心标注（dt = 当前时刻 - 交棒时刻）
  function handoffAnnotations(isB, dt, alpha, opts = {}) {
    if (alpha <= 0.01) return;
    const M = (p) => (isB ? p : rotP(p));
    const f = FS;
    const a1 = alpha * (1 - smooth(-0.45, -0.18, dt));
    const a2 = alpha * smooth(-0.32, -0.08, dt) * (1 - smooth(0.50, 0.74, dt));
    const a3 = alpha * smooth(0.56, 0.82, dt);

    // 阶段 1：持棒跑向自己左侧 + 接棒身体在左、右手伸在右侧（杆后）
    arrowW([M(P(74.8, -RUN_Y)), M(P(77.8, -RUN_Y)), M(P(80.85, -RUN_Y))], '#60a5fa', 4, a1, [10, 7]);
    callout(
      M(P(77.2, -RUN_Y)), M(P(76.4, -2.05)),
      '持棒者：跑向自己左侧（杆在右手边）',
      { alpha: a1, size: f * 0.88, bg: 'rgba(29,78,216,.95)', line: '#93c5fd' }
    );
    callout(
      M(READY_B), M(P(82.8, 1.72)),
      '接棒者：身体站在自己左侧',
      { alpha: a1, size: f * 0.88, bg: 'rgba(194,65,12,.95)', line: '#fdba74' }
    );
    callout(
      M(MEET_B), M(P(83.25, -0.58)),
      '接棒手（右手/惯用手）向右侧伸到杆后',
      { alpha: a1, size: f * 0.88, bg: 'rgba(185,28,28,.95)', line: '#fca5a5' }
    );
    callout(M(POLE_B), M(P(78.5, 0.88)), '标志杆', { alpha: a1, size: f * 0.82 });

    // 阶段 2：右手接棒，棒从杆后由右向左绕杆
    arrowW([M(P(80.88, -0.52)), M(P(81.48, 0.08)), M(P(80.88, 0.68))], '#facc15', 5.2, a2);
    callout(
      M(P(81.25, 0.08)), M(P(83.55, -0.12)),
      '右手接棒：棒从杆后 · 从右到左（绕杆）',
      { alpha: a2, size: f * 0.96, bg: 'rgba(250,204,21,.98)', fg: '#111827', bold: true, line: '#facc15' }
    );

    // 阶段 3：绕杆完成，沿自己左侧起跑
    arrowW([M(P(80.5, RUN_Y)), M(P(77.6, RUN_Y)), M(P(74.6, RUN_Y))], '#4ade80', 4.2, a3, [10, 7]);
    callout(
      M(P(76.8, RUN_Y)), M(P(76.4, 2.02)),
      '绕杆完成 → 立即沿自己左侧起跑！',
      { alpha: a3, size: f * 0.90, bg: 'rgba(21,128,61,.95)', line: '#86efac' }
    );
    if (!opts.noReturn) {
      arrowW([M(P(81.8, -1.18)), M(P(83.9, -1.92)), M(P(86.2, -2.22))], 'rgba(226,232,240,.92)', 3, a3, [6, 6]);
      callout(M(P(84.0, -1.92)), M(P(83.7, -2.42)), '交棒后从外侧回队尾', { alpha: a3, size: f * 0.78 });
    }
    pill(w2s(M(P(81.95, 0.02))), '棒绕杆呈 “U” 形轨迹', {
      alpha: a3, size: f * 0.84, bg: 'rgba(250,204,21,.96)', fg: '#111827', bold: true,
    });
  }

  function wrapText(text, maxW, font) {
    ctx.font = font;
    const out = [];
    let line = '';
    for (const ch of text) {
      if (ctx.measureText(line + ch).width > maxW && line) { out.push(line); line = ch; }
      else line += ch;
    }
    if (line) out.push(line);
    return out;
  }

  function captionBar(step, maxStep, title, desc, accent) {
    setScreen();
    const pad = 11, size = clamp(W / 66, 12, 16);
    const lines = wrapText(desc, W - pad * 4 - 8, `${size * 0.88}px -apple-system, "PingFang SC", "Microsoft YaHei", sans-serif`);
    const h = size * 1.55 + lines.length * size * 1.32 + pad * 1.1;
    const y = H - h - 7;
    ctx.fillStyle = 'rgba(10,16,32,.85)';
    roundRect(8, y, W - 16, h, 11); ctx.fill();
    ctx.fillStyle = accent || '#facc15';
    roundRect(8, y, 6, h, 3); ctx.fill();
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.font = `700 ${size}px -apple-system, "PingFang SC", "Microsoft YaHei", sans-serif`;
    ctx.fillStyle = accent || '#facc15';
    ctx.fillText(title, pad + 10, y + pad * 0.62);
    if (step > 0) {
      const tw = ctx.measureText(title).width;
      for (let i = 1; i <= maxStep; i++) {
        ctx.fillStyle = i === step ? '#facc15' : 'rgba(255,255,255,.25)';
        ctx.beginPath();
        ctx.arc(pad + 10 + tw + 14 + (i - 1) * 13, y + pad * 0.62 + size * 0.54, 4.2, 0, TAU);
        ctx.fill();
      }
    }
    ctx.font = `${size * 0.88}px -apple-system, "PingFang SC", "Microsoft YaHei", sans-serif`;
    ctx.fillStyle = '#e2e8f0';
    lines.forEach((ln, i) => ctx.fillText(ln, pad + 10, y + pad * 0.62 + size * 1.46 + i * size * 1.32));
  }

  // ---------------------------------------------------------------- UI state
  let mode = 'relay'; // 'relay' | 'focus' | 'demo'
  let simT = START, focusT = 0, demoT = 0;
  let playing = true, speed = 1;
  let closeOn = true, trailOn = true;
  let focusStep = 'loop';

  const $ = (s) => document.querySelector(s);
  const playBtn = $('#play'), nextBtn = $('#next'), scrub = $('#scrub'), clock = $('#clock');
  const focusStepsBar = $('#focus-steps');
  scrub.min = START; scrub.max = END;

  function handoffInfo(t) {
    const j = Math.round(t / T);
    if (j < 1 || j > LEGS - 1) return null;
    const dt = t - j * T;
    return { j, dt, isB: j % 2 === 1 };
  }
  function closeWeights(t) {
    const h = handoffInfo(t);
    if (!h || !closeOn) return { w: 0, ws: 0, h };
    const w = smooth(-3.3, -2.3, h.dt) * (1 - smooth(2.0, 3.0, h.dt));
    const ws = smooth(-1.5, -0.9, h.dt) * (1 - smooth(0.95, 1.55, h.dt));
    return { w, ws, h };
  }
  function rateAt(t) {
    const { w, ws } = closeWeights(t);
    return lerp(1, 0.55, w) * lerp(1, 0.38, ws);
  }

  // ---------------------------------------------------------------- Mode 1: 完整接力演示
  function renderRelay() {
    const t = simT;
    const intro = smooth(-5.2, -2.8, t);
    let c = blendCam(overviewCam(), relayCam(), intro);
    const { w, ws, h } = closeWeights(t);
    if (h && w > 0) {
      const pole = h.isB ? POLE_B : POLE_A;
      c = blendCam(c, closeCam(pole, h.isB ? 0.55 : -0.55), w);
    }
    cam = c;
    const mag = clamp(8.2 / (0.44 * cam.s), 1, 2.3);

    setScreen(); ctx.clearRect(0, 0, W, H);
    setWorld();
    drawStadium();
    drawRelayMarks(smooth(-4.5, -3.0, t));
    if (h) drawZone(h.isB, w);
    drawPole(POLE_A, h && !h.isB ? w : 0);
    drawPole(POLE_B, h && h.isB ? w : 0);

    // 接力棒运动轨迹（展示绕杆 U 形）
    if (trailOn && t > -0.2) {
      const N = 85, dtS = 0.036 + 0.018 * (1 - w);
      let prev = null;
      ctx.lineCap = 'round';
      for (let k = N; k >= 0; k--) {
        const tt = t - k * dtS;
        if (tt < 0) continue;
        const ho = holderAt(tt), st = stateOf(ho, tt), hd = handsOf(st, mag);
        const pt = hd.R;
        if (prev) {
          ctx.strokeStyle = `rgba(255,214,0,${0.90 * (1 - k / N)})`;
          ctx.lineWidth = Math.max(0.055, px(3.4)) * (0.38 + 0.62 * (1 - k / N));
          ctx.beginPath(); ctx.moveTo(prev.x, prev.y); ctx.lineTo(pt.x, pt.y); ctx.stroke();
        }
        prev = pt;
      }
    }

    const states = people.map((p) => stateOf(p, t));
    const holder = holderAt(t);
    const li = legAt(t);
    const receiver = t >= 0 && li < LEGS - 1 ? legRunner[li + 1] : null;
    const pulse = 0.62 + 0.38 * Math.sin(performance.now() / 180);

    setWorld();
    for (const st of states) {
      if (st.p === holder) ring(st, mag, '#facc15', 0.95);
      else if (st.p === receiver && t > (li + 1) * T - 4) ring(st, mag, '#4ade80', pulse);
    }
    const order = states.slice().sort((a, b) => (a.mode === 'queue') - (b.mode === 'queue')).reverse();
    for (const st of order) {
      const hiHand = w > 0.45 && (st.p === holder || st.p === receiver);
      drawPerson(st, mag, hiHand);
    }
    const hs = states.find((s) => s.p === holder);
    drawBaton(hs, mag, w);

    setScreen();
    drawRelayLabels(smooth(-3.6, -2.6, t) * (1 - w));
    if (cam.s > 6) {
      const tagA = 1 - smooth(0.2, 0.6, w);
      for (const st of states) {
        const isKey = st.p === holder || st.p === receiver;
        const a = w > 0.5 ? (isKey ? 1 : 0.78) : (isKey ? tagA : 0);
        drawNameBadge(st, mag, a);
      }
    }
    if (h) {
      zoneLabels(h.isB, smooth(0.5, 0.9, w));
      handoffAnnotations(h.isB, h.dt, smooth(0.55, 0.95, w));
    }

    // 顶部状态栏
    setScreen();
    if (t >= 0 && t < LEGS * T + 2) {
      const r = legRunner[li];
      pill(P(12, 22), `第 ${li + 1} 棒 / 共 ${LEGS} 棒 · ${r.name} 右手持棒（${r.start} 端 → ${r.end} 端）`, {
        anchor: 'left', size: FS * 0.88,
      });
    }
    if (ws > 0.15) {
      pill(P(W - 12, 22), `特写慢放 ${(speed * rateAt(t)).toFixed(2)}×`, {
        anchor: 'right', size: FS * 0.88, bg: 'rgba(220,38,38,.92)', alpha: smooth(0.15, 0.4, ws),
      });
    } else if (h && w > 0.5) {
      pill(P(W - 12, 22), `第 ${h.j} 次绕杆交接 · ${h.isB ? 'B' : 'A'} 端`, {
        anchor: 'right', size: FS * 0.88, alpha: w,
      });
    }

    if (t > -2.6 && t < 0.45) {
      const word = t < -1.75 ? '各就位' : t < -0.9 ? '预备' : '跑！';
      const ph = t < -1.75 ? t + 2.6 : t < -0.9 ? t + 1.75 : t + 0.9;
      const a = smooth(0, 0.15, ph) * (1 - smooth(0.7, 0.85, ph + (word === '跑！' ? -0.50 : 0)));
      pill(P(W / 2, H * 0.30), word, {
        size: clamp(W / 19, 28, 58), bold: true, alpha: a,
        bg: word === '跑！' ? 'rgba(220,38,38,.94)' : 'rgba(15,23,42,.88)',
      });
    }

    if (t < -2.6) {
      captionBar(0, 0, '比赛场地：100 m 直道中间 60 m', '室外体育场 100 m 跑道取中间 60 m（20 m 处与 80 m 处各立标志杆），两端各站 10 位同学。红腕带代表右手（惯用手）。', '#facc15');
    } else if (t >= LEGS * T + 1.5) {
      pill(P(W / 2, H * 0.30), '🎉 20 棒迎面接力完成！', {
        size: clamp(W / 22, 24, 50), bold: true, bg: 'rgba(22,163,74,.94)',
        alpha: smooth(LEGS * T + 1.5, LEGS * T + 2.2, t),
      });
      captionBar(0, 0, '全员完成', '两侧各 10 位同学全部完成接力。记住核心要领：跑向左侧、接棒身在左右手伸右、棒从杆后由右向左绕杆再起跑！', '#4ade80');
    } else if (h && w > 0.5) {
      const step = h.dt < -0.25 ? 1 : h.dt < 0.62 ? 2 : 3;
      const caps = [
        ['① 跑向左侧 · 身在左右手伸右', '持棒同学跑向自己的左侧（标志杆在右手边）；接棒同学身体站在左侧，右手（惯用手）向右侧伸到标志杆正后方等候。'],
        ['② 杆后交接：右手接棒 · 从右到左', '持棒同学跑过杆线，右手对右手交棒！接棒同学握棒后在杆后把棒从右侧拉向左侧——完成“绕杆”关键一步。'],
        ['③ 绕杆完成 · 立即起跑', '接力棒绕过标志杆走出“U”形后，接棒同学立即沿自己的左侧加速起跑；交完棒的同学从外侧慢跑回队尾。'],
      ];
      captionBar(step, 3, caps[step - 1][0], caps[step - 1][1], '#facc15');
    } else if (t >= 0) {
      const r = legRunner[li];
      captionBar(0, 0, `第 ${li + 1} 棒：${r.name} 跑向 ${r.end} 端`, '右手持棒沿自己左侧奔跑。快到对面标志杆时保持在左侧跑过杆线，准备在杆后右手交接。可点击上方“🔍 绕杆接棒分解”逐步细看。', r.team === 'A' ? '#60a5fa' : '#fb923c');
    }
  }

  // ---------------------------------------------------------------- Scenario builder for Focus & Demo
  const SCENE_LEN = 6.8, SCENE_TH = 2.6;
  function buildHandoffScene(correct) {
    const dx = correct ? 0 : -1.75; // 错误示范：在杆前 x = 79.10 处提前交接，未绕杆
    const sh = (p) => P(p.x + dx, p.y);
    const runPath = buildPath([P(65, -RUN_Y), P(71, -RUN_Y), P(76.5, -RUN_Y), sh(HAND_B)]);
    const runProf = makeProfile((t) => 1 - 0.42 * smooth(SCENE_TH - 1.1, SCENE_TH, t), SCENE_TH, runPath.L);
    const postPath = buildPath(postPtsB(4).map(sh));
    const postProf = makeDecel(postPath.L, runProf.vAt(SCENE_TH));
    const recSprint = buildPath(sprintPtsBA.map(sh));
    const runner = { team: 'A', num: 1, name: '持棒同学' };
    const rec = { team: 'B', num: 1, name: '接棒同学' };
    const qs = [1, 2, 3].map((k) => ({ team: 'B', num: 1 + k, name: 'B' + (1 + k), k }));

    return {
      correct, dx, sh,
      states(tau) {
        const out = [];
        // 持棒来人
        if (tau < SCENE_TH) {
          const s = runProf.sAt(tau), pt = pathAt(runPath, s);
          out.push({
            p: runner, x: pt.x, y: pt.y, h: pt.h,
            phase: (s / 1.8) * TAU, amp: clamp(runProf.vAt(tau) / 5, 0, 1),
            e: smooth(SCENE_TH - 0.78, SCENE_TH - 0.08, tau), role: 'runner',
          });
        } else {
          const tt = tau - SCENE_TH, s = postProf.sAt(tt), pt = pathAt(postPath, s);
          const done = tt >= postProf.T;
          out.push({
            p: runner, x: pt.x, y: pt.y,
            h: done ? angLerp(pt.h, Math.PI, smooth(postProf.T, postProf.T + 0.9, tt)) : pt.h,
            phase: (s / 1.1) * TAU, amp: done ? 0 : clamp(postProf.vAt(tt) / 5, 0.25, 1),
            e: 1 - smooth(0.02, 0.38, tt), role: 'runner',
          });
        }
        // 接棒同学
        if (tau < SCENE_TH) {
          const r0 = sh(READY_B);
          out.push({
            p: rec, x: r0.x, y: r0.y, h: Math.PI, phase: 0, amp: 0,
            e: smooth(SCENE_TH - 2.2, SCENE_TH - 1.4, tau), role: 'rec',
          });
        } else if (tau < SCENE_TH + SWEEP_T) {
          const sw = sweepState(rec, 'B', tau - SCENE_TH);
          sw.x += dx;
          sw.role = 'rec';
          out.push(sw);
        } else {
          const rt = tau - SCENE_TH - SWEEP_T;
          const s = sprintProf.sAt(rt), pt = pathAt(recSprint, s);
          out.push({
            p: rec, x: pt.x, y: pt.y, h: pt.h,
            phase: (s / 1.8) * TAU, amp: clamp(sprintProf.vAt(rt) / 5, 0, 1),
            e: 0, role: 'rec',
          });
        }
        // 纵队等候同学（错例时队列仍在杆后，只有接棒人违例越线/杆前接棒）
        for (const q of qs) {
          const k = q.k - smooth(SCENE_TH + 0.45, SCENE_TH + 1.85, tau);
          const a = slotPos('B', k), b = slotPos('B', q.k - smooth(SCENE_TH + 0.45, SCENE_TH + 1.85, tau - 0.05));
          out.push({
            p: q, x: a.x, y: a.y, h: Math.PI,
            phase: (a.x / 0.8) * TAU, amp: clamp(Math.abs(a.x - b.x) / 0.05 / 2.5, 0, 0.6),
            e: 0, role: 'q',
          });
        }
        return out;
      },
    };
  }

  const sceneCorrect = buildHandoffScene(true);
  const sceneWrong = buildHandoffScene(false);
  const sceneRate = (tau) => 0.48 * lerp(1, 0.36, smooth(SCENE_TH - 1.1, SCENE_TH - 0.5, tau) * (1 - smooth(SCENE_TH + 0.8, SCENE_TH + 1.4, tau)));

  function drawSceneTrail(scene, tau) {
    if (!trailOn) return;
    const N = 110;
    let prev = null;
    setWorld();
    ctx.lineCap = 'round';
    for (let k = N; k >= 0; k--) {
      const tt = tau - k * 0.032;
      if (tt < 0) continue;
      const ss = scene.states(tt);
      const hst = tt < SCENE_TH ? ss[0] : ss[1];
      const hd = handsOf(hst, 1);
      if (prev) {
        ctx.strokeStyle = `rgba(255,214,0,${0.92 * (1 - k / N)})`;
        ctx.lineWidth = px(4.2) * (0.38 + 0.62 * (1 - k / N));
        ctx.beginPath(); ctx.moveTo(prev.x, prev.y); ctx.lineTo(hd.R.x, hd.R.y); ctx.stroke();
      }
      prev = hd.R;
    }
  }

  // 在分解模式下绘制人物自身“左/右侧”方位提示，彻底消除迎面镜像混淆
  function drawBodySideHints(runnerSt, recSt, tau) {
    const f = FS * 0.76;
    if (tau < SCENE_TH + 0.25) {
      // 持棒者（朝右跑）：上方是其左侧，下方是其右侧
      pill(w2s(P(runnerSt.x, runnerSt.y - 0.56)), '持棒者左侧', {
        size: f, bg: 'rgba(30,64,175,.88)', alpha: 0.92, clamp: false,
      });
      // 接棒者（朝左站）：下方是其左侧（身体所在），上方是其右侧（右手伸出）
      pill(w2s(P(recSt.x, recSt.y + 0.56)), '接棒者左侧（身体在左）', {
        size: f, bg: 'rgba(154,52,18,.92)', alpha: 0.95, clamp: false,
      });
    }
  }

  // ---------------------------------------------------------------- Mode 2: 绕杆接棒分解
  const FOCUS_KEYFRAMES = { 1: 1.25, 2: 2.08, 3: 2.80, 4: 3.58 };
  const focusGuidePts = [];
  for (let tt = 1.0; tt <= 4.3; tt += 0.04) {
    const ss = sceneCorrect.states(tt);
    focusGuidePts.push(handsOf(tt < SCENE_TH ? ss[0] : ss[1], 1).R);
  }

  function renderFocus() {
    const tau = focusT;
    cam = { cx: 80.35, cy: 0.0, s: Math.min(W / 11.2, H / 6.5) };
    setScreen(); ctx.clearRect(0, 0, W, H);
    setWorld();
    drawStadium();
    drawRelayMarks(1);
    drawZone(true, 1, false);

    // 预先画出理想绕杆 U 形虚线引导轨（与真实接力棒轨迹严格重合）
    setWorld();
    ctx.save();
    ctx.strokeStyle = 'rgba(250,204,21,.34)';
    ctx.lineWidth = px(3);
    ctx.setLineDash([px(7), px(6)]);
    ctx.beginPath();
    focusGuidePts.forEach((pt, idx) => (idx ? ctx.lineTo(pt.x, pt.y) : ctx.moveTo(pt.x, pt.y)));
    ctx.stroke();
    ctx.restore();

    drawPole(POLE_A, 0);
    drawPole(POLE_B, 1);

    const states = sceneCorrect.states(tau);
    drawSceneTrail(sceneCorrect, tau);

    setWorld();
    const activeHolder = tau < SCENE_TH ? states[0] : states[1];
    ring(activeHolder, 1, '#facc15', 0.95);
    if (tau < SCENE_TH) ring(states[1], 1, '#4ade80', 0.85);
    for (const st of states.slice().reverse()) {
      drawPerson(st, 1, st.role === 'runner' || st.role === 'rec');
    }
    drawBaton(activeHolder, 1, 1);

    setScreen();
    for (const st of states) {
      if (st.role === 'q') drawNameBadge(st, 1, 0.85);
    }
    drawBodySideHints(states[0], states[1], tau);
    zoneLabels(true, 1);

    const dt = tau - SCENE_TH;
    handoffAnnotations(true, dt, 1);

    const stepIdx = tau < 1.95 ? 1 : tau < SCENE_TH - 0.02 ? 2 : tau < SCENE_TH + SWEEP_T + 0.12 ? 3 : 4;
    pill(P(12, 22), '🔍 绕杆接棒慢速分解（红腕带 = 右手/惯用手）', {
      anchor: 'left', size: FS * 0.88, bg: 'rgba(15,23,42,.92)',
    });
    pill(P(W - 12, 22), playing ? `循环慢放中 · 步骤 ${stepIdx}/4` : `已定格 · 步骤 ${stepIdx}/4（按空格继续）`, {
      anchor: 'right', size: FS * 0.86, bg: playing ? 'rgba(37,99,235,.92)' : 'rgba(217,119,6,.94)',
    });

    const stepCaps = [
      ['步骤 ①：带着棒跑向自己的左侧', '蓝衣持棒同学向对面跑来时，主动偏向自己的左侧跑道，使标志杆处于其右手一侧；橙衣接棒同学站在杆后，身体位于自己的左侧。'],
      ['步骤 ②：身体在左侧，右手伸在右侧（杆后）', '接棒同学保持身体在杆后左侧，将惯用手（右手，红腕带）向自己的右侧伸出，手停在标志杆正后方等待来棒。'],
      ['步骤 ③：右手接棒，棒从杆后由右向左！', '持棒同学跑过杆线完成右手交右手；接棒同学握紧棒后，让棒在标志杆后方从自己的右侧拉向左侧——完成最关键的“绕杆”！'],
      ['步骤 ④：绕杆完成，沿左侧立即起跑', '接力棒完整绕过标志杆呈“U”形后，接棒同学立即沿自己的左侧冲刺起跑；交完棒的同学从外侧绕回队尾。'],
    ];
    captionBar(stepIdx, 4, stepCaps[stepIdx - 1][0], stepCaps[stepIdx - 1][1], '#facc15');
  }

  // ---------------------------------------------------------------- Mode 3: 正误对比
  function renderDemo() {
    const part = demoT < SCENE_LEN ? 0 : 1; // 0 = 错误示范, 1 = 正确示范
    const tau = demoT - part * SCENE_LEN;
    const D = part === 0 ? sceneWrong : sceneCorrect;
    cam = { cx: 80.1, cy: 0.0, s: Math.min(W / 13.0, H / 7.0) };

    setScreen(); ctx.clearRect(0, 0, W, H);
    setWorld();
    drawStadium();
    drawRelayMarks(1);
    drawZone(true, 1, !D.correct && tau > SCENE_TH - 0.15);
    drawPole(POLE_A, 0);
    drawPole(POLE_B, 1);

    const states = D.states(tau);
    drawSceneTrail(D, tau);

    setWorld();
    const activeHolder = tau < SCENE_TH ? states[0] : states[1];
    ring(activeHolder, 1, '#facc15', 0.95);
    for (const st of states.slice().reverse()) {
      drawPerson(st, 1, st.role === 'runner' || st.role === 'rec');
    }
    drawBaton(activeHolder, 1, 1);

    setScreen();
    for (const st of states) drawNameBadge(st, 1, 0.9);

    const dt = tau - SCENE_TH;
    if (D.correct) {
      zoneLabels(true, 1);
      handoffAnnotations(true, dt, 1);
      pill(P(W / 2, 24), '✓ 正确示范：绕过标志杆，棒从杆后由右向左交接后起跑', {
        size: FS * 0.96, bold: true, bg: 'rgba(22,163,74,.95)',
      });
      const step = dt < -0.25 ? 1 : dt < 0.62 ? 2 : 3;
      const caps = [
        ['① 跑向左侧 · 杆后等棒', '持棒者跑向自己左侧并跑过杆线；接棒者站杆后左侧，右手向右侧伸到杆后。'],
        ['② 杆后右手交接 · 由右向左', '接力棒严格从标志杆后方经过，从接棒者右侧拉向左侧，绕杆走出饱满的“U”形。'],
        ['③ 绕杆后起跑', '确认棒绕过标志杆后全速起跑，动作规范、路线清晰不碰撞！'],
      ];
      captionBar(step, 3, caps[step - 1][0], caps[step - 1][1], '#4ade80');
    } else {
      pill(P(W / 2, 24), '✗ 错误示范：未绕杆！在标志杆前提前交接抢跑', {
        size: FS * 0.96, bold: true, bg: 'rgba(220,38,38,.95)',
      });
      const rec = states[1];
      const a1 = 1 - smooth(-0.25, 0.05, dt);
      callout(P(rec.x, rec.y), P(rec.x - 2.3, 1.95), '错误：接棒同学越过杆线站到了杆前', {
        alpha: a1, size: FS * 0.88, bg: 'rgba(185,28,28,.95)', line: '#fca5a5',
      });
      callout(P(77.2, -RUN_Y), P(76.2, -2.05), '错误：持棒同学未跑过标志杆就交棒', {
        alpha: a1, size: FS * 0.88, bg: 'rgba(185,28,28,.95)', line: '#fca5a5',
      });
      if (dt > -0.10) {
        const foulPt = P(MEET_B.x + D.dx, MEET_B.y);
        const m = w2s(foulPt);
        const a = smooth(-0.10, 0.14, dt);
        setScreen();
        ctx.save();
        ctx.globalAlpha = a;
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 5.5;
        ctx.lineCap = 'round';
        const r = clamp(W / 44, 15, 26);
        ctx.beginPath();
        ctx.moveTo(m.x - r, m.y - r); ctx.lineTo(m.x + r, m.y + r);
        ctx.moveTo(m.x + r, m.y - r); ctx.lineTo(m.x - r, m.y + r);
        ctx.stroke();
        ctx.restore();
        callout(foulPt, P(76.2, -1.55), '犯规：棒在杆前折返，漏掉了“绕杆”！', {
          alpha: a, size: FS * 0.95, bold: true, bg: 'rgba(220,38,38,.97)', line: '#fca5a5',
        });
      }
      captionBar(0, 0, '为什么算犯规？', '迎面接力规定必须绕过标志杆交接：棒若在杆前折返，既少跑了距离，又容易在杆前发生迎面冲撞。务必做到“棒从杆后、从右到左”！', '#f87171');
    }
    if (Math.abs(dt) < 0.85) {
      pill(P(W - 12, 24), '慢动作对比', { anchor: 'right', size: FS * 0.84, bg: 'rgba(220,38,38,.92)' });
    }
  }

  // ---------------------------------------------------------------- main loop & controls
  function fmt(t) {
    const s = Math.max(0, t - START);
    return `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
  }

  function setPlaying(v) {
    playing = v;
    playBtn.textContent = v ? '⏸ 暂停' : '▶ 播放';
  }

  function setFocusStepUI(val) {
    focusStep = val;
    document.querySelectorAll('.step-btn').forEach((b) => {
      const on = b.dataset.step === String(val);
      b.classList.toggle('active', on);
      b.setAttribute('aria-pressed', on ? 'true' : 'false');
    });
  }

  function setMode(m) {
    mode = m;
    document.querySelectorAll('.mode').forEach((b) => {
      const on = b.dataset.mode === m;
      b.classList.toggle('active', on);
      b.setAttribute('aria-pressed', on ? 'true' : 'false');
    });
    focusStepsBar.hidden = m !== 'focus';
    nextBtn.textContent = m === 'relay' ? '⏭ 下一次接棒' : m === 'focus' ? '⏭ 下一步骤' : '⏭ 切换正/误';
    if (m === 'focus') {
      focusT = 0;
      setFocusStepUI('loop');
    } else if (m === 'demo') {
      demoT = 0;
    }
    setPlaying(true);
  }

  function restart() {
    if (mode === 'relay') simT = START;
    else if (mode === 'focus') { focusT = 0; setFocusStepUI('loop'); }
    else demoT = 0;
    setPlaying(true);
  }

  function nextAction() {
    if (mode === 'demo') {
      demoT = demoT < SCENE_LEN ? SCENE_LEN : 0;
      setPlaying(true);
      return;
    }
    if (mode === 'focus') {
      const order = ['1', '2', '3', '4'];
      const idx = order.indexOf(String(focusStep));
      const next = order[(idx + 1) % order.length];
      setFocusStepUI(next);
      focusT = FOCUS_KEYFRAMES[next];
      setPlaying(false);
      return;
    }
    for (let j = 1; j < LEGS; j++) {
      const target = j * T - (closeOn ? 3.4 : 2.8);
      if (target > simT + 0.05) {
        simT = target;
        setPlaying(true);
        return;
      }
    }
    simT = LEGS * T - 3.2;
  }

  playBtn.addEventListener('click', () => {
    if (mode === 'relay' && simT >= END) simT = START;
    if (mode === 'focus' && !playing && focusStep !== 'loop') setFocusStepUI('loop');
    setPlaying(!playing);
  });
  $('#restart').addEventListener('click', restart);
  nextBtn.addEventListener('click', nextAction);
  $('#speed').addEventListener('change', (e) => { speed = parseFloat(e.target.value); });
  $('#closeup').addEventListener('change', (e) => { closeOn = e.target.checked; });
  $('#trail').addEventListener('change', (e) => { trailOn = e.target.checked; });

  scrub.addEventListener('input', () => {
    const v = parseFloat(scrub.value);
    if (mode === 'relay') {
      simT = v;
    } else if (mode === 'focus') {
      focusT = clamp(((v - START) / (END - START)) * SCENE_LEN, 0, SCENE_LEN - 0.001);
    } else {
      demoT = clamp(((v - START) / (END - START)) * SCENE_LEN * 2, 0, SCENE_LEN * 2 - 0.001);
    }
  });

  document.querySelectorAll('.mode').forEach((b) => {
    b.addEventListener('click', () => setMode(b.dataset.mode));
  });

  document.querySelectorAll('.step-btn').forEach((b) => {
    b.addEventListener('click', () => {
      const st = b.dataset.step;
      setFocusStepUI(st);
      if (st === 'loop') {
        setPlaying(true);
      } else {
        focusT = FOCUS_KEYFRAMES[st];
        setPlaying(false);
      }
    });
  });

  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'SELECT' || (e.target.tagName === 'INPUT' && e.target.type !== 'range')) return;
    if (e.code === 'Space') {
      e.preventDefault();
      if (mode === 'focus' && !playing && focusStep !== 'loop') setFocusStepUI('loop');
      setPlaying(!playing);
    } else if (e.code === 'ArrowRight') {
      e.preventDefault();
      nextAction();
    } else if (e.code === 'KeyR') {
      restart();
    }
  });

  let last = performance.now();
  function frame(now) {
    const dtR = Math.min(0.05, (now - last) / 1000);
    last = now;
    if (playing) {
      if (mode === 'relay') {
        simT += dtR * speed * (closeOn ? rateAt(simT) : 1);
        if (simT >= END) { simT = END; setPlaying(false); }
      } else if (mode === 'focus') {
        focusT = (focusT + dtR * speed * sceneRate(focusT)) % SCENE_LEN;
      } else {
        const tau = demoT % SCENE_LEN;
        demoT = (demoT + dtR * speed * sceneRate(tau)) % (SCENE_LEN * 2);
      }
    }

    if (mode === 'relay') {
      renderRelay();
      if (document.activeElement !== scrub) scrub.value = simT;
      clock.textContent = `${fmt(simT)} / ${fmt(END)}`;
    } else if (mode === 'focus') {
      renderFocus();
      if (document.activeElement !== scrub) {
        scrub.value = START + (focusT / SCENE_LEN) * (END - START);
      }
      clock.textContent = `分解 ${focusT.toFixed(1)}s / ${SCENE_LEN}s`;
    } else {
      renderDemo();
      if (document.activeElement !== scrub) {
        scrub.value = START + (demoT / (SCENE_LEN * 2)) * (END - START);
      }
      clock.textContent = demoT < SCENE_LEN ? '错误示范' : '正确示范';
    }
    requestAnimationFrame(frame);
  }

  // URL 参数支持（便于预览与课堂定位）：?mode=focus&step=3&paused=1
  const qp = new URLSearchParams(location.search);
  if (qp.get('mode') === 'focus' || qp.get('mode') === 'demo') setMode(qp.get('mode'));
  if (qp.has('t')) simT = clamp(parseFloat(qp.get('t')), START, END);
  if (qp.has('ft')) focusT = clamp(parseFloat(qp.get('ft')), 0, SCENE_LEN - 0.001);
  if (qp.has('step') && FOCUS_KEYFRAMES[qp.get('step')]) {
    setFocusStepUI(qp.get('step'));
    focusT = FOCUS_KEYFRAMES[qp.get('step')];
  }
  if (qp.has('dt')) demoT = clamp(parseFloat(qp.get('dt')), 0, SCENE_LEN * 2 - 0.001);
  if (qp.get('closeup') === '0') { closeOn = false; $('#closeup').checked = false; }
  if (qp.get('paused') === '1') setPlaying(false);

  resize();
  requestAnimationFrame(frame);
})();
