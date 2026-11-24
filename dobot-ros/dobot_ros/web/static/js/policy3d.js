'use strict';

// ── Policy simulator 3D layer ────────────────────────────────────
// Draws the simulated table + objects inside the robot's Z-up frame and
// replays policy traces (joints, gripper, object poses) through the
// hooks robot3d.js exposes on window.robot3d. Classic script; all access
// to the module is lazy so load order doesn't matter.

(function () {
  const r3 = () => window.robot3d;
  const COLORS = {
    red: 0xd9363e, green: 0x3fb950, blue: 0x388bfd, yellow: 0xe3b341, orange: 0xf0883e,
    purple: 0xa371f7, white: 0xdddddd, black: 0x333333, gray: 0x888888, cyan: 0x39c5cf,
    pink: 0xff7b9c, unknown: 0x999999,
  };

  let group = null;
  let framed = false;
  let tableMesh = null;
  let outline = null;
  const meshes = new Map();   // id → Mesh

  function ensureGroup() {
    const r = r3();
    if (!r || !r.getRobotRoot()) return null;
    if (!group) {
      group = new r.THREE.Group();
      group.name = 'policy-scene';
      r.getRobotRoot().add(group);
    }
    return group;
  }

  function setTable(table) {
    const g = ensureGroup();
    if (!g || !table) return;
    const T = r3().THREE;
    const ws = table.workspace;
    const w = (ws.x_max - ws.x_min) / 1000, h = (ws.y_max - ws.y_min) / 1000;
    const cx = (ws.x_max + ws.x_min) / 2000, cy = (ws.y_max + ws.y_min) / 2000;
    const z = table.surface_z / 1000;
    if (tableMesh) { g.remove(tableMesh); g.remove(outline); }
    tableMesh = new T.Mesh(
      new T.PlaneGeometry(w, h),
      new T.MeshStandardMaterial({ color: 0x2b3340, roughness: 0.9, metalness: 0.0,
                                   transparent: true, opacity: 0.85, side: T.DoubleSide }));
    tableMesh.position.set(cx, cy, z - 0.0005);
    g.add(tableMesh);
    outline = new T.LineSegments(
      new T.EdgesGeometry(new T.PlaneGeometry(w, h)),
      new T.LineBasicMaterial({ color: 0x58a6ff }));
    outline.position.set(cx, cy, z);
    g.add(outline);
    // Frame the table the first time it appears; the user's orbit is kept after that.
    if (!framed) { framed = true; r3().frame?.(cx, cy, z, 1.4); }
  }

  function setObjects(objs) {
    const g = ensureGroup();
    if (!g) return;
    const T = r3().THREE;
    const seen = new Set();
    (objs || []).forEach(o => {
      seen.add(o.id);
      let m = meshes.get(o.id);
      const size = o.size || [0.04, 0.04, 0.04].map(v => v * 1000);
      if (!m) {
        m = new T.Mesh(
          new T.BoxGeometry(size[0] / 1000, size[1] / 1000, size[2] / 1000),
          new T.MeshStandardMaterial({ color: COLORS[o.color] ?? COLORS.unknown, roughness: 0.5, metalness: 0.1 }));
        m.userData.size = size.slice();
        g.add(m);
        meshes.set(o.id, m);
      }
      m.position.set(o.position[0] / 1000, o.position[1] / 1000, o.position[2] / 1000);
      m.rotation.set(0, 0, (o.yaw_deg || 0) * Math.PI / 180);
      m.material.emissive = new T.Color(o.support === 'gripper' ? 0x335577 : 0x000000);
    });
    for (const [id, m] of meshes) {
      if (!seen.has(id)) { g.remove(m); meshes.delete(id); }
    }
  }

  function clear() {
    if (!group) return;
    for (const m of meshes.values()) group.remove(m);
    meshes.clear();
  }

  function applySample(ev) {
    const r = r3();
    if (!r) return;
    if (ev.joints) r.setJointsDirect(ev.joints);
    if (ev.gripper != null) r.updateGripperPosition(ev.gripper / 1000);
    if (ev.objects) setObjects(ev.objects);
  }

  function setLive(on) { const r = r3(); if (r) r.setExternalDrive(on); }

  // ── Replay ───────────────────────────────────────────────────
  const replay = {
    events: [], samples: [], t: 0, duration: 0, playing: false, speed: 1.0,
    _raf: 0, _last: 0, onTick: null,
    load(record) {
      this.pause();
      this.events = record.events || [];
      this.samples = this.events.filter(e => e.kind === 'sample');
      this.duration = this.events.length ? this.events[this.events.length - 1].t : 0;
      if (record.initial_objects) setObjects(record.initial_objects);
      this.seek(0);
    },
    seek(frac) {
      this.t = Math.max(0, Math.min(1, frac)) * this.duration;
      this._apply();
    },
    seekTime(t) { this.t = Math.max(0, Math.min(this.duration, t)); this._apply(); },
    _apply() {
      let s = null;
      for (const e of this.samples) { if (e.t <= this.t) s = e; else break; }
      if (s) { setLive(true); applySample(s); }
      if (this.onTick) this.onTick(this.t, this.duration);
    },
    play() {
      if (!this.samples.length) return;
      if (this.t >= this.duration) this.t = 0;
      this.playing = true;
      this._last = performance.now();
      const step = (now) => {
        if (!this.playing) return;
        this.t += (now - this._last) / 1000 * this.speed;
        this._last = now;
        if (this.t >= this.duration) { this.t = this.duration; this._apply(); this.playing = false; return; }
        this._apply();
        this._raf = requestAnimationFrame(step);
      };
      this._raf = requestAnimationFrame(step);
    },
    pause() { this.playing = false; if (this._raf) cancelAnimationFrame(this._raf); this._raf = 0; },
  };

  window.policy3d = { setTable, setObjects, clear, applySample, setLive, replay };
})();
