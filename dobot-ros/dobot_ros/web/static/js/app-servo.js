'use strict';

// ── Dobot CR5 — Servo Tester tab ─────────────────────────────────
// Extracted from app.js. Loaded as a classic script AFTER app.js so
// window.DobotUI.{logActivity,showToast,debounce} are available.

(function () {
  const ui = () => (window.DobotUI || {});
  const logActivity = (msg, type) => ui().logActivity?.(msg, type);
  const showToast   = (msg, type) => ui().showToast?.(msg, type);
  const debounce    = (fn, ms)   => ui().debounce?.(fn, ms) || fn;

  // ── Servo Tester ───────────────────────────────────────────────

  async function servoPost(path, body) {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    return r.json();
  }

  function fmtVec(v, digits = 1) {
    if (!v || !v.length) return '--';
    return '[' + v.map(x => Number(x).toFixed(digits)).join(', ') + ']';
  }

  async function servoRefreshStatus() {
    try {
      const r = await fetch('/api/servo/status');
      const s = await r.json();
      document.getElementById('srv-mode').textContent = s.mode || '--';
      document.getElementById('srv-step').textContent = s.step || 0;
      document.getElementById('srv-lat').textContent = (s.last_latency_ms ?? 0).toFixed(1);
      document.getElementById('srv-pct').textContent =
        `${(s.p50_latency_ms ?? 0).toFixed(1)}/${(s.p95_latency_ms ?? 0).toFixed(1)}`;
      document.getElementById('srv-clamps').textContent = s.clamps || 0;
      document.getElementById('srv-overruns').textContent = s.overruns || 0;
      document.getElementById('srv-anchor').textContent = fmtVec(s.anchor_pose);
      document.getElementById('srv-commanded').textContent = fmtVec(s.last_commanded_pose);
      document.getElementById('srv-error').textContent = s.last_error || '';

      document.getElementById('btn-srv-start').disabled = !!s.running;
      document.getElementById('btn-srv-stop').disabled = !s.running;
    } catch (e) {
      // Modal may be closed.
    }
  }

  // Throttled offset pusher: coalesce rapid slider events into 10 Hz updates.
  const servoOffsetState = {
    offset: [0, 0, 0, 0, 0, 0],
    dirty: false,
    lastSent: 0,
  };
  async function servoMaybePushOffset() {
    if (!servoOffsetState.dirty) return;
    const now = performance.now();
    if (now - servoOffsetState.lastSent < 80) return;  // ~12.5 Hz ceiling
    servoOffsetState.lastSent = now;
    servoOffsetState.dirty = false;
    try {
      await servoPost('/api/servo/target', { offset: servoOffsetState.offset });
    } catch (e) { /* ignore */ }
  }

  function initServoTester() {
    const modal = document.getElementById('servoModal') || document.getElementById('tab-servo');
    if (!modal) return;

    const byId = id => document.getElementById(id);

    // Tuning sliders — update only their value label on input.
    // Persistence is explicit: the Save button POSTs to /api/servo/config;
    // Reset restores the HTML defaults (and persists on next Save).
    const tuneMap = [
      ['srv-rate', 'srv-rate-val', v => v],
      ['srv-t', 'srv-t-val', v => v],
      ['srv-gain', 'srv-gain-val', v => v],
      ['srv-ahead', 'srv-ahead-val', v => v],
    ];
    for (const [sliderId, valId, fmt] of tuneMap) {
      const el = byId(sliderId);
      if (!el) continue;
      el.addEventListener('input', () => {
        byId(valId).textContent = fmt(el.value);
      });
    }

    // Helper to apply a {servo_rate_hz, t, gain, aheadtime} dict to the sliders
    // and their value labels. Used on load and Reset.
    const applyTuning = (cfg) => {
      const fields = [
        ['srv-rate',  'srv-rate-val',  cfg.servo_rate_hz],
        ['srv-t',     'srv-t-val',     cfg.t],
        ['srv-gain',  'srv-gain-val',  cfg.gain],
        ['srv-ahead', 'srv-ahead-val', cfg.aheadtime],
      ];
      for (const [sid, vid, val] of fields) {
        if (val === undefined || val === null) continue;
        const el = byId(sid);
        if (!el) continue;
        el.value = val;
        byId(vid).textContent = val;
      }
    };

    // Load saved tuning on init so sliders reflect persisted state.
    (async () => {
      try {
        const r = await fetch('/api/servo/config');
        const j = await r.json();
        if (j && j.success && j.config) applyTuning(j.config);
      } catch (e) { /* keep HTML defaults */ }
    })();

    // Save button — explicit persist.
    const tuningFeedback = byId('srv-tuning-feedback');
    const flashTuning = (msg, ok = true) => {
      if (!tuningFeedback) return;
      tuningFeedback.textContent = msg;
      tuningFeedback.className = 'small ' + (ok ? 'text-success' : 'text-danger');
      setTimeout(() => { if (tuningFeedback.textContent === msg) tuningFeedback.textContent = ''; }, 1800);
    };
    byId('btn-srv-save-tuning')?.addEventListener('click', async () => {
      const cfg = {
        servo_rate_hz: parseFloat(byId('srv-rate').value),
        t: parseFloat(byId('srv-t').value),
        gain: parseFloat(byId('srv-gain').value),
        aheadtime: parseFloat(byId('srv-ahead').value),
      };
      try {
        const res = await servoPost('/api/servo/config', cfg);
        flashTuning(res.success ? 'Saved' : `Save failed: ${res.error}`, !!res.success);
      } catch (e) {
        flashTuning(`Save failed: ${e.message}`, false);
      }
    });

    // Reset button — pull server-side defaults and apply to sliders.
    // Does not auto-save; user must click Save to persist.
    byId('btn-srv-reset-tuning')?.addEventListener('click', async () => {
      try {
        const r = await fetch('/api/servo/config');
        const j = await r.json();
        if (j && j.defaults) {
          applyTuning(j.defaults);
          flashTuning('Reset to defaults — click Save to persist');
        }
      } catch (e) {
        flashTuning(`Reset failed: ${e.message}`, false);
      }
    });

    // Jog sliders — push offset target.
    const jogLabels = ['srv-jog-x-val', 'srv-jog-y-val', 'srv-jog-z-val',
                       'srv-jog-rx-val', 'srv-jog-ry-val', 'srv-jog-rz-val'];
    const jogOffsetPusher = setInterval(servoMaybePushOffset, 50);
    document.querySelectorAll('.srv-jog').forEach(s => {
      s.addEventListener('input', () => {
        const axis = parseInt(s.dataset.axis, 10);
        const v = parseFloat(s.value);
        servoOffsetState.offset[axis] = v;
        servoOffsetState.dirty = true;
        byId(jogLabels[axis]).textContent = v;
      });
      // Spring-back to 0 on release for translational axes.
      s.addEventListener('change', () => {
        const axis = parseInt(s.dataset.axis, 10);
        if (axis < 3) {
          s.value = 0;
          servoOffsetState.offset[axis] = 0;
          servoOffsetState.dirty = true;
          byId(jogLabels[axis]).textContent = 0;
        }
      });
    });

    byId('btn-srv-center')?.addEventListener('click', () => {
      document.querySelectorAll('.srv-jog').forEach(s => { s.value = 0; });
      servoOffsetState.offset = [0, 0, 0, 0, 0, 0];
      servoOffsetState.dirty = true;
      for (let i = 0; i < 6; i++) byId(jogLabels[i]).textContent = 0;
    });

    // Patterns.
    document.querySelectorAll('[data-pattern]').forEach(btn => {
      btn.addEventListener('click', async () => {
        const name = btn.dataset.pattern;
        const params = {
          circle:    { radius_mm: 40, period_s: 4, plane: 'xy' },
          lissajous: { amplitude_x_mm: 40, amplitude_y_mm: 40,
                       freq_x_hz: 0.25, freq_y_hz: 0.33, phase_deg: 90 },
          square:    { axis: 'x', amplitude: 30, period_s: 4 },
          sine:      { axis: 'x', amplitude: 30, freq_hz: 0.5 },
        }[name] || {};
        const res = await servoPost('/api/servo/pattern', { name, params });
        logActivity(`Servo: pattern ${name} ${res.success ? 'running' : 'failed'}`,
                    res.success ? 'info' : 'error');
      });
    });
    byId('btn-srv-clear-pattern')?.addEventListener('click', async () => {
      await servoPost('/api/servo/target', { offset: servoOffsetState.offset });
    });

    // Start / Stop / E-Stop.
    byId('btn-srv-start')?.addEventListener('click', async () => {
      const res = await servoPost('/api/servo/start', {
        servo_rate_hz: parseFloat(byId('srv-rate').value),
        t: parseFloat(byId('srv-t').value),
        gain: parseFloat(byId('srv-gain').value),
        aheadtime: parseFloat(byId('srv-ahead').value),
        csv_log: byId('srv-csv').checked,
      });
      logActivity(res.success ? 'Servo tester started' : `Servo start failed: ${res.error}`,
                  res.success ? 'info' : 'error');
      servoRefreshStatus();
    });
    byId('btn-srv-stop')?.addEventListener('click', async () => {
      await servoPost('/api/servo/stop');
      logActivity('Servo tester stopped', 'info');
      servoRefreshStatus();
    });
    byId('btn-srv-estop')?.addEventListener('click', async () => {
      await servoPost('/api/servo/estop');
      logActivity('Servo E-STOP engaged', 'warn');
      servoRefreshStatus();
    });

    // Poll status while the Servo tab (or legacy modal) is visible.
    const trigger = document.getElementById('tab-servo-btn') || modal;
    const showEvt = trigger.id === 'servoModal' ? 'shown.bs.modal' : 'shown.bs.tab';
    const hideEvt = trigger.id === 'servoModal' ? 'hidden.bs.modal' : 'hidden.bs.tab';
    let poll = null;
    trigger.addEventListener(showEvt, () => {
      servoRefreshStatus();
      if (poll) clearInterval(poll);
      poll = setInterval(servoRefreshStatus, 500);
    });
    trigger.addEventListener(hideEvt, () => {
      if (poll) { clearInterval(poll); poll = null; }
    });
  }


  // Register for app.js init() flow.
  window.DobotUI = window.DobotUI || {};
  window.DobotUI.initServoTester = initServoTester;
})();
