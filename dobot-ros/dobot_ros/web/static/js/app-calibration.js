'use strict';

// ── Dobot CR5 — Calibration tab auto-refresh + object overlay + pick ─
// Extracted from app.js. Loaded as a classic script AFTER app.js so
// window.DobotUI.{logActivity,showToast} are available before any
// handler here runs.

(function () {
  // Proxy helpers through window.DobotUI (populated by app.js).
  const ui = () => (window.DobotUI || {});
  const logActivity = (msg, type) => ui().logActivity?.(msg, type);
  const showToast   = (msg, type) => ui().showToast?.(msg, type);

  // ── Calibration tab: auto-refresh + object overlay + pick flow ───

  const _calPickState = {
    target: null,              // { kind: "pixel"|"object", px, py, object_id, object, plan }
    objects: [],
    pollTimer: null,
  };

  function initCalibrationTab() {
    const btn = document.getElementById('tab-calibration-btn');
    if (!btn) return;

    btn.addEventListener('shown.bs.tab', () => {
      // Freshen the camera frame directly (not via .click() which depends on
      // the handler being in scope). Also refresh phase indicator + objects.
      const img = document.getElementById('cal-camera-feed');
      if (img) {
        img.src = '/api/calibration/camera-frame?' + Date.now();
        img.style.display = 'block';
      }
      calRefreshPhaseIndicator();
    });
    btn.addEventListener('hidden.bs.tab', () => {
      calStopObjectPolling();
    });

    // Solve vision transform (local SVD).
    document.getElementById('btn-cal-solve')?.addEventListener('click', async () => {
      logActivity('Solving vision transform (local SVD)...', 'info');
      const r = await fetch('/api/vision/calibrate', { method: 'POST' });
      const res = await r.json();
      if (res.success) {
        logActivity(`Vision transform solved: error=${res.error_mm}mm, ${res.num_points} pts`, 'success');
        showToast(`Solved: ${res.error_mm}mm error`, 'success');
        const sr = document.getElementById('cal-solve-result');
        if (sr) {
          sr.style.display = '';
          document.getElementById('cal-solve-error').textContent = res.error_mm + ' mm';
          document.getElementById('cal-solve-points').textContent = res.num_points;
          document.getElementById('cal-solve-intrinsics').textContent =
            `fx=${res.intrinsics?.fx?.toFixed(0)} fy=${res.intrinsics?.fy?.toFixed(0)}`;
          document.getElementById('cal-solve-table-z').textContent = res.table_z?.toFixed(1) + ' mm';
        }
        calRefreshPhaseIndicator();
      } else {
        showToast(`Solve failed: ${res.error}`, 'danger');
        logActivity(`Solve failed: ${res.error}`, 'error');
      }
    });

    // Also solve on camera server (backward compat).
    document.getElementById('btn-cal-solve-camera')?.addEventListener('click', async () => {
      const r = await fetch('/api/calibration/solve', { method: 'POST' });
      const res = await r.json();
      if (res.success || res.solved) {
        showToast(`Camera server solved: ${res.error_mm}mm`, 'success');
      } else {
        showToast(`Camera solve failed: ${res.error || 'unknown'}`, 'danger');
      }
    });

    // Single-touch table Z record.
    document.getElementById('btn-cal-table-z')?.addEventListener('click', async () => {
      const r = await fetch('/api/calibration/table/z', { method: 'POST' });
      const res = await r.json();
      if (res.success) {
        logActivity(`Table Z recorded: ${res.table_z.toFixed(1)} mm`, 'success');
        showToast('Table Z saved', 'success');
        calRefreshPhaseIndicator();
      } else {
        showToast(`Record failed: ${res.error}`, 'danger');
      }
    });

    // Object overlay toggle.
    document.getElementById('cal-show-objects')?.addEventListener('change', () => {
      calDrawObjectOverlay();
    });
    document.getElementById('btn-cal-refresh-objects')?.addEventListener('click', calFetchObjects);

    // Pick preview + execute + cancel.
    document.getElementById('btn-cal-pick-execute')?.addEventListener('click', calPickExecute);
    document.getElementById('btn-cal-pick-cancel')?.addEventListener('click', () => {
      _calPickState.target = null;
      calRenderPickPreview(null);
    });

    // Click on the camera image: if inside an object bbox → select that
    // object; otherwise let the existing pixel-select flow run and also
    // trigger a pixel-pick preview.
    document.getElementById('cal-camera-container')?.addEventListener('click', (ev) => {
      const img = document.getElementById('cal-camera-feed');
      if (!img || !img.naturalWidth) return;
      const rect = img.getBoundingClientRect();
      const scaleX = img.naturalWidth / rect.width;
      const scaleY = img.naturalHeight / rect.height;
      const px = Math.round((ev.clientX - rect.left) * scaleX);
      const py = Math.round((ev.clientY - rect.top) * scaleY);

      const hit = _calPickState.objects.find(o => {
        const [bx, by, bw, bh] = o.bbox || [0, 0, 0, 0];
        return px >= bx && px <= bx + bw && py >= by && py <= by + bh;
      });
      if (hit) {
        calPickPlan({ kind: 'object', object_id: hit.id, object: hit });
      } else {
        // After the existing mouseup handler updates cal-pixel, preview this pixel.
        setTimeout(() => calPickPlan({ kind: 'pixel', px, py }), 60);
      }
    });
  }

  function calStartObjectPolling() {
    calFetchObjects();
    if (_calPickState.pollTimer) clearInterval(_calPickState.pollTimer);
    _calPickState.pollTimer = setInterval(calFetchObjects, 1500);
  }
  function calStopObjectPolling() {
    if (_calPickState.pollTimer) { clearInterval(_calPickState.pollTimer); _calPickState.pollTimer = null; }
  }

  async function calFetchObjects() {
    try {
      const r = await fetch('/api/detection/objects');
      const d = await r.json();
      _calPickState.objects = d.objects || [];
      calDrawObjectOverlay();
    } catch (e) { /* ignore */ }
  }

  function calDrawObjectOverlay() {
    const img = document.getElementById('cal-camera-feed');
    const canvas = document.getElementById('cal-obj-overlay');
    if (!img || !canvas || img.style.display === 'none') return;
    canvas.width = img.naturalWidth || 640;
    canvas.height = img.naturalHeight || 360;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const show = document.getElementById('cal-show-objects')?.checked !== false;
    if (!show) return;

    _calPickState.objects.forEach(o => {
      const [x, y, w, h] = o.bbox || [0, 0, 0, 0];
      // Pick target gets a brighter box; others a subtle one.
      const isTarget = _calPickState.target?.kind === 'object'
                    && _calPickState.target.object_id === o.id;
      ctx.strokeStyle = isTarget ? '#00ff88' : '#ffcc00';
      ctx.lineWidth = isTarget ? 3 : 2;
      ctx.strokeRect(x, y, w, h);
      ctx.fillStyle = ctx.strokeStyle;
      ctx.font = 'bold 14px sans-serif';
      ctx.fillText(`#${o.id} ${o.label || ''}`.trim(), x + 4, Math.max(14, y - 4));
    });
  }

  async function calPickPlan(target) {
    const body = target.kind === 'object'
      ? { object_id: target.object_id }
      : { px: target.px, py: target.py };
    try {
      const r = await fetch('/api/pick/plan', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const res = await r.json();
      if (res.success) {
        _calPickState.target = { ...target, plan: res };
        calRenderPickPreview(res);
        calDrawObjectOverlay();
      } else {
        showToast(`Plan failed: ${res.error}`, 'danger');
        _calPickState.target = null;
        calRenderPickPreview(null);
      }
    } catch (e) {
      showToast(`Plan error: ${e}`, 'danger');
    }
  }

  function fmtXYZ(p, digits = 1) {
    if (!p) return '--';
    if (p.length === 6) {
      return `(${p[0].toFixed(digits)}, ${p[1].toFixed(digits)}, ${p[2].toFixed(digits)}) mm`;
    }
    return '[' + p.map(v => Number(v).toFixed(digits)).join(', ') + ']';
  }

  function calRenderPickPreview(plan) {
    const target = document.getElementById('cal-pick-target');
    const preview = document.getElementById('cal-pick-preview');
    const warnings = document.getElementById('cal-pick-warnings');
    if (!plan) {
      target.textContent = 'Click an object or pixel in the camera view to select a target.';
      preview.style.display = 'none';
      return;
    }
    const t = plan.target;
    const summary = t.object
      ? `Object #${t.object.id} ${t.object.label || ''} @ px ? (rotation ${t.rotation_deg.toFixed(0)}°)`
      : `Pixel target`;
    target.textContent = summary;
    document.getElementById('cal-pick-cam-xyz').textContent =
      `(${t.camera_xyz[0].toFixed(3)}, ${t.camera_xyz[1].toFixed(3)}, ${t.camera_xyz[2].toFixed(3)}) m`;
    document.getElementById('cal-pick-rob-xyz').textContent = fmtXYZ(t.robot_xyz);
    document.getElementById('cal-pick-approach').textContent = fmtXYZ(plan.waypoints.approach);
    document.getElementById('cal-pick-grasp').textContent = fmtXYZ(plan.waypoints.grasp);
    document.getElementById('cal-pick-retract').textContent = fmtXYZ(plan.waypoints.retract);
    if (plan.warnings && plan.warnings.length) {
      warnings.style.display = '';
      warnings.innerHTML = plan.warnings.map(w => '⚠ ' + w).join('<br>');
    } else {
      warnings.style.display = 'none';
      warnings.innerHTML = '';
    }
    preview.style.display = '';
  }

  async function calPickExecute() {
    const t = _calPickState.target;
    if (!t) return;
    const summary = t.kind === 'object' ? `object #${t.object_id}` : `pixel (${t.px}, ${t.py})`;
    if (!confirm(`Execute pick on ${summary}?\n\nApproach → descend to grasp → close gripper → retract.`)) return;
    const body = t.kind === 'object'
      ? { object_id: t.object_id, confirm: true }
      : { px: t.px, py: t.py, confirm: true };
    try {
      const r = await fetch('/api/pick/execute', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const res = await r.json();
      if (res.success) {
        logActivity(`Pick complete: ${summary}`, 'success');
        showToast('Pick complete', 'success');
      } else {
        showToast(`Pick failed: ${res.error}`, 'danger');
        logActivity(`Pick failed: ${res.error}`, 'error');
      }
    } catch (e) {
      showToast(`Pick error: ${e}`, 'danger');
    }
  }

  async function calRefreshPhaseIndicator() {
    const tableOk = await fetch('/api/calibration/table').then(r => r.json()).catch(() => ({}));
    const hasTable = (tableOk.table_z !== undefined) ||
                     (tableOk.plane && tableOk.plane.mean_z !== undefined);
    const tableZ = tableOk.table_z ?? tableOk.plane?.mean_z;
    const p1 = document.getElementById('cal-phase-1');
    const p1s = document.getElementById('cal-phase-1-state');
    if (p1 && p1s) {
      p1.className = 'badge ' + (hasTable ? 'bg-success' : 'bg-secondary');
      p1s.textContent = hasTable ? `✔ ${tableZ.toFixed(1)} mm` : 'needed';
    }
    const disp = document.getElementById('cal-table-z-display');
    if (disp) disp.textContent = hasTable ? `${tableZ.toFixed(1)} mm` : '--';

    const camStat = await fetch('/api/calibration/status').then(r => r.json()).catch(() => ({}));
    const hasPoints = (camStat.num_points || 0) >= 3;
    const p2 = document.getElementById('cal-phase-2');
    const p2s = document.getElementById('cal-phase-2-state');
    if (p2 && p2s) {
      p2.className = 'badge ' + (hasPoints ? 'bg-success' : 'bg-secondary');
      p2s.textContent = hasPoints ? `✔ ${camStat.num_points} pts` : `${camStat.num_points || 0}/3`;
    }

    const visStat = await fetch('/api/vision/status').then(r => r.json()).catch(() => ({}));
    const hasSolved = !!visStat.calibrated;
    const p3 = document.getElementById('cal-phase-3');
    const p3s = document.getElementById('cal-phase-3-state');
    if (p3 && p3s) {
      p3.className = 'badge ' + (hasSolved ? 'bg-success' : 'bg-secondary');
      p3s.textContent = hasSolved ? '✔ solved' : 'needed';
    }

    const ready = hasTable && hasPoints && hasSolved;
    const badge = document.getElementById('cal-status-badge');
    if (badge) {
      badge.textContent = ready ? 'Calibrated' : 'Not Calibrated';
      badge.className = ready ? 'text-success small' : 'text-warning small';
    }
  }


  // Register for app.js init() flow.
  window.DobotUI = window.DobotUI || {};
  window.DobotUI.initCalibrationTab = initCalibrationTab;
})();
