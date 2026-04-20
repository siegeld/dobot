'use strict';

// ── Dobot CR5 — Vision tab + Strategy management ─────────────
// Extracted from app.js. Loaded as a classic script AFTER app.js so
// window.DobotUI.{logActivity,showToast} are available before any
// handler here runs.

(function () {
  // Proxy helpers through window.DobotUI (populated by app.js).
  // Aliased here so the moved code can stay byte-identical.
  const ui = () => (window.DobotUI || {});
  const logActivity = (msg, type) => ui().logActivity?.(msg, type);
  const showToast   = (msg, type) => ui().showToast?.(msg, type);

  // ── Vision tab ─────────────────────────────────────────────────

  const _visState = {
    objects: [], target: null, plan: null, pollTimer: null, grid: null,
    strategies: [], activeSlug: null, activeParams: {},
  };

  function initVisionTab() {
    const btn = document.getElementById('tab-vision-btn');
    if (!btn) return;

    btn.addEventListener('shown.bs.tab', () => {
      visRefreshFeed();
      visRefreshStatus();
      visStartPolling();
      visFetchGrid();
      visLoadStrategies();
    });
    btn.addEventListener('hidden.bs.tab', visStopPolling);

    document.getElementById('btn-vis-refresh')?.addEventListener('click', () => {
      visRefreshFeed(); visFetchObjects(); visFetchGrid();
    });
    document.getElementById('vis-show-objects')?.addEventListener('change', visDrawOverlay);
    document.getElementById('vis-show-grid')?.addEventListener('change', () => { visFetchGrid(); visDrawOverlay(); });
    document.getElementById('vis-strategy-select')?.addEventListener('change', visOnStrategyChange);
    document.getElementById('btn-vis-save-params')?.addEventListener('click', visSaveParams);
    document.getElementById('btn-vis-reset-params')?.addEventListener('click', visResetParams);
    document.getElementById('btn-vis-cancel')?.addEventListener('click', () => {
      _visState.target = null; _visState.plan = null;
      visRenderPreview(null); visDrawOverlay();
    });
    document.getElementById('btn-vis-simulate')?.addEventListener('click', visSimulate);
    document.getElementById('btn-vis-execute')?.addEventListener('click', visExecute);

    // Click on camera view → select target.
    document.getElementById('vis-camera-container')?.addEventListener('click', (ev) => {
      const img = document.getElementById('vis-camera-feed');
      if (!img || !img.naturalWidth) return;
      const rect = img.getBoundingClientRect();
      const sx = img.naturalWidth / rect.width;
      const sy = img.naturalHeight / rect.height;
      const px = Math.round((ev.clientX - rect.left) * sx);
      const py = Math.round((ev.clientY - rect.top) * sy);
      document.getElementById('vis-pixel').textContent = `${px}, ${py}`;

      const hit = _visState.objects.find(o => {
        const [bx, by, bw, bh] = o.bbox || [0,0,0,0];
        return px >= bx && px <= bx+bw && py >= by && py <= by+bh;
      });
      visPlanTarget(hit ? { kind: 'object', object_id: hit.id } : { kind: 'pixel', px, py });
    });
  }

  function visRefreshFeed() {
    const img = document.getElementById('vis-camera-feed');
    if (img) { img.src = '/api/calibration/camera-frame?' + Date.now(); img.style.display='block'; }
  }

  async function visRefreshStatus() {
    try {
      const s = await fetch('/api/vision/status').then(r => r.json());
      const b = document.getElementById('vis-cal-badge');
      if (b) {
        b.textContent = s.calibrated ? 'Calibrated' : 'Not Calibrated';
        b.className = 'badge ' + (s.calibrated ? 'bg-success' : 'bg-warning');
      }
      const tz = document.getElementById('vis-table-z');
      if (tz) tz.textContent = s.table_z != null ? s.table_z.toFixed(1) + ' mm' : '--';
      const intr = document.getElementById('vis-intrinsics');
      if (intr) intr.textContent = s.intrinsics ? `${s.intrinsics.fx.toFixed(0)}×${s.intrinsics.fy.toFixed(0)}` : 'default';
    } catch (e) {}
  }

  function visStartPolling() {
    visFetchObjects();
    if (_visState.pollTimer) clearInterval(_visState.pollTimer);
    _visState.pollTimer = setInterval(visFetchObjects, 1500);
  }
  function visStopPolling() {
    if (_visState.pollTimer) { clearInterval(_visState.pollTimer); _visState.pollTimer = null; }
  }

  async function visFetchObjects() {
    try {
      const d = await fetch('/api/detection/objects').then(r => r.json());
      _visState.objects = d.objects || [];
      visDrawOverlay();
    } catch (e) {}
  }

  async function visFetchGrid() {
    if (!document.getElementById('vis-show-grid')?.checked) { _visState.grid = null; return; }
    try {
      const d = await fetch('/api/vision/grid').then(r => r.json());
      _visState.grid = d.success ? d.grid : null;
      visDrawOverlay();
    } catch (e) { _visState.grid = null; }
  }

  function visDrawOverlay() {
    const img = document.getElementById('vis-camera-feed');
    const canvas = document.getElementById('vis-overlay');
    if (!img || !canvas || img.style.display === 'none') return;
    canvas.width = img.naturalWidth || 640;
    canvas.height = img.naturalHeight || 360;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Table-plane grid (projected from robot frame to pixel coords).
    if (_visState.grid && document.getElementById('vis-show-grid')?.checked) {
      ctx.strokeStyle = 'rgba(100, 200, 255, 0.3)';
      ctx.lineWidth = 1;
      _visState.grid.forEach(row => {
        if (row.length < 2) return;
        ctx.beginPath();
        ctx.moveTo(row[0].px[0], row[0].px[1]);
        for (let i = 1; i < row.length; i++) ctx.lineTo(row[i].px[0], row[i].px[1]);
        ctx.stroke();
      });
      // Vertical lines (connect same column index across rows).
      const cols = _visState.grid[0]?.length || 0;
      for (let c = 0; c < cols; c++) {
        ctx.beginPath();
        let started = false;
        _visState.grid.forEach(row => {
          if (c < row.length) {
            if (!started) { ctx.moveTo(row[c].px[0], row[c].px[1]); started = true; }
            else ctx.lineTo(row[c].px[0], row[c].px[1]);
          }
        });
        ctx.stroke();
      }
      // Label the origin (0,0) if visible.
      const center = _visState.grid.find(row => row.find(p => p.robot[0] === 0 && p.robot[1] === 0));
      const originPt = center?.find(p => p.robot[0] === 0 && p.robot[1] === 0);
      if (originPt) {
        ctx.fillStyle = 'rgba(100, 200, 255, 0.7)';
        ctx.beginPath(); ctx.arc(originPt.px[0], originPt.px[1], 4, 0, 2*Math.PI); ctx.fill();
        ctx.font = '11px sans-serif';
        ctx.fillText('(0,0)', originPt.px[0] + 6, originPt.px[1] - 4);
      }
    }

    // Object bounding boxes.
    if (document.getElementById('vis-show-objects')?.checked !== false) {
      _visState.objects.forEach(o => {
        const [x, y, w, h] = o.bbox || [0,0,0,0];
        const isSel = _visState.target?.kind === 'object' && _visState.target.object_id === o.id;
        ctx.strokeStyle = isSel ? '#00ff88' : '#ffcc00';
        ctx.lineWidth = isSel ? 3 : 2;
        ctx.strokeRect(x, y, w, h);
        ctx.fillStyle = ctx.strokeStyle;
        ctx.font = 'bold 13px sans-serif';
        ctx.fillText(`#${o.id} ${o.label || ''}`.trim(), x + 3, Math.max(14, y - 4));
      });
    }

    // Selected target crosshair.
    const plan = _visState.plan;
    if (plan && plan.overlay_pixels?.target) {
      const [tx, ty] = plan.overlay_pixels.target;
      ctx.strokeStyle = '#ff4444';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(tx - 20, ty); ctx.lineTo(tx + 20, ty);
      ctx.moveTo(tx, ty - 20); ctx.lineTo(tx, ty + 20);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Waypoint markers on the image.
    if (plan && plan.overlay_pixels) {
      const markers = [
        ['approach', '#44aaff', 'A'],
        ['grasp', '#ff4444', 'G'],
        ['retract', '#44ff44', 'R'],
      ];
      markers.forEach(([key, color, lbl]) => {
        const px = plan.overlay_pixels[key];
        if (!px) return;
        ctx.fillStyle = color;
        ctx.beginPath(); ctx.arc(px[0], px[1], 6, 0, 2*Math.PI); ctx.fill();
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 10px sans-serif';
        ctx.fillText(lbl, px[0] - 3, px[1] + 4);
      });
    }
  }

  // ── Strategy management ────────────────────────────────────────

  async function visLoadStrategies() {
    try {
      const [listRes, activeRes] = await Promise.all([
        fetch('/api/strategies/list').then(r => r.json()),
        fetch('/api/strategies/active').then(r => r.json()),
      ]);
      _visState.strategies = listRes.strategies || [];
      _visState.activeSlug = activeRes.slug;
      _visState.activeParams = activeRes.params || {};

      const sel = document.getElementById('vis-strategy-select');
      if (sel) {
        sel.innerHTML = _visState.strategies.map(s =>
          `<option value="${s.slug}" ${s.slug === _visState.activeSlug ? 'selected' : ''}>${s.name}</option>`
        ).join('');
      }
      visRenderStrategyForm();
    } catch (e) {
      console.warn('Failed to load strategies:', e);
    }
  }

  async function visOnStrategyChange() {
    const sel = document.getElementById('vis-strategy-select');
    const slug = sel?.value;
    if (!slug || slug === _visState.activeSlug) return;
    await fetch('/api/strategies/active', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ slug }),
    });
    _visState.activeSlug = slug;
    const s = _visState.strategies.find(st => st.slug === slug);
    _visState.activeParams = s?.current_params || {};
    visRenderStrategyForm();
    logActivity(`Strategy changed to: ${s?.name || slug}`, 'info');
  }

  function visRenderStrategyForm() {
    const container = document.getElementById('vis-strategy-params');
    const descEl = document.getElementById('vis-strategy-desc');
    if (!container) return;
    const s = _visState.strategies.find(st => st.slug === _visState.activeSlug);
    if (!s) { container.innerHTML = ''; return; }
    if (descEl) descEl.textContent = s.description || '';

    const params = s.parameters || [];
    const vals = _visState.activeParams;
    container.innerHTML = params.map(p => {
      const val = vals[p.name] ?? p.default;
      const unit = p.unit ? ` <small class="text-muted">${p.unit}</small>` : '';
      const tip = p.description ? ` data-bs-toggle="tooltip" title="${p.description.replace(/"/g, '&quot;')}"` : '';

      if (p.type === 'bool') {
        return `<div class="form-check mb-1"${tip}>
          <input class="form-check-input strat-param" type="checkbox" data-param="${p.name}" ${val ? 'checked' : ''}>
          <label class="form-check-label small">${p.name.replace(/_/g, ' ')}</label>
        </div>`;
      }
      if (p.type === 'choice') {
        const opts = (p.choices || []).map(c =>
          `<option value="${c}" ${c === val ? 'selected' : ''}>${c}</option>`).join('');
        return `<div class="mb-1"${tip}>
          <label class="form-label small mb-0">${p.name.replace(/_/g, ' ')}${unit}</label>
          <select class="form-select form-select-sm strat-param" data-param="${p.name}">${opts}</select>
        </div>`;
      }
      // float or int
      const attrs = [
        `type="number"`, `value="${val}"`, `data-param="${p.name}"`,
        p.min != null ? `min="${p.min}"` : '',
        p.max != null ? `max="${p.max}"` : '',
        p.step != null ? `step="${p.step}"` : '',
      ].filter(Boolean).join(' ');
      return `<div class="mb-1"${tip}>
        <label class="form-label small mb-0">${p.name.replace(/_/g, ' ')}${unit}</label>
        <input class="form-control form-control-sm strat-param" ${attrs}>
      </div>`;
    }).join('');

    // Re-init tooltips for the new elements.
    if (typeof bootstrap !== 'undefined' && bootstrap.Tooltip) {
      container.querySelectorAll('[data-bs-toggle="tooltip"]').forEach(el => {
        bootstrap.Tooltip.getOrCreateInstance(el, { container: 'body', delay: { show: 400, hide: 100 } });
      });
    }
  }

  function visCollectParams() {
    const params = {};
    document.querySelectorAll('.strat-param').forEach(el => {
      const key = el.dataset.param;
      if (!key) return;
      if (el.type === 'checkbox') params[key] = el.checked;
      else if (el.type === 'number') params[key] = parseFloat(el.value);
      else params[key] = el.value;
    });
    return params;
  }

  async function visSaveParams() {
    const slug = _visState.activeSlug;
    if (!slug) return;
    const params = visCollectParams();
    const r = await fetch(`/api/strategies/${slug}/params`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(params),
    });
    const res = await r.json();
    if (res.success) {
      _visState.activeParams = res.params;
      showToast('Strategy params saved', 'success');
      logActivity(`Saved params for ${slug}`, 'info');
    } else {
      showToast(`Save failed: ${res.error || r.status}`, 'danger');
    }
  }

  async function visResetParams() {
    const slug = _visState.activeSlug;
    if (!slug) return;
    const s = _visState.strategies.find(st => st.slug === slug);
    if (!s) return;
    const defaults = {};
    (s.parameters || []).forEach(p => { defaults[p.name] = p.default; });
    const r = await fetch(`/api/strategies/${slug}/params`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(defaults),
    });
    const res = await r.json();
    if (res.success) {
      _visState.activeParams = res.params;
      visRenderStrategyForm();
      showToast('Reset to defaults', 'info');
    }
  }

  async function visPlanTarget(target) {
    _visState.target = target;
    // Strategy params are read server-side from the active strategy's JSON.
    // We just send the target — no hardcoded approach/grasp/lift here.
    const body = target.kind === 'object'
      ? { object_id: target.object_id }
      : { px: target.px, py: target.py };

    try {
      const r = await fetch('/api/vision/plan', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const res = await r.json();
      if (res.success) {
        _visState.plan = res;
        visRenderPreview(res);
        visDrawOverlay();
      } else {
        showToast(`Plan: ${res.error}`, 'danger');
        _visState.plan = null;
        visRenderPreview(null);
      }
    } catch (e) {
      showToast(`Plan error: ${e}`, 'danger');
    }
  }

  function visRenderPreview(plan) {
    const hint = document.getElementById('vis-target-hint');
    const preview = document.getElementById('vis-preview');
    if (!plan) {
      if (hint) hint.style.display = '';
      if (preview) preview.style.display = 'none';
      return;
    }
    if (hint) hint.style.display = 'none';
    if (preview) preview.style.display = '';

    const t = plan.target;
    const d = plan.depth_check;
    const c = plan.confidence;
    const wp = plan.waypoints;

    const fmtP = (p) => p ? `(${p[0]}, ${p[1]}, ${p[2]}) mm` : '--';
    const $ = id => document.getElementById(id);

    // Show which strategy produced this plan.
    const stratName = plan.strategy?.name || _visState.activeSlug || '?';
    const planCard = document.querySelector('#vis-preview')?.closest('.card');
    const planHeader = planCard?.querySelector('.card-header');
    if (planHeader) {
      planHeader.textContent = '';
      const icon = document.createElement('i');
      icon.className = 'bi bi-hand-index-thumb';
      const badge = document.createElement('span');
      badge.className = 'badge bg-info ms-2';
      badge.textContent = stratName;
      planHeader.appendChild(icon);
      planHeader.appendChild(document.createTextNode(' Pick target '));
      planHeader.appendChild(badge);
    }

    $('vis-p-pixel').textContent = t.pixel ? `(${t.pixel[0]}, ${t.pixel[1]})` : '--';
    $('vis-p-robot-xy').textContent = t.robot_xy ? `(${t.robot_xy[0]}, ${t.robot_xy[1]}) mm` : '--';
    $('vis-p-rotation').textContent = `${t.table_rot_deg}° table (${t.image_rot_deg}° image)`;
    $('vis-p-present').innerHTML = d.object_present
      ? '<span class="text-success">Yes</span>' : '<span class="text-danger">No</span>';
    $('vis-p-height').textContent = d.object_height_mm ? d.object_height_mm.toFixed(1) + ' mm' : '--';
    $('vis-p-exp-depth').textContent = d.expected_depth_m ? d.expected_depth_m + ' m' : '--';
    $('vis-p-meas-depth').textContent = d.measured_depth_m ? d.measured_depth_m + ' m' : '--';

    const confBadge = $('vis-p-conf-badge');
    if (confBadge) {
      const colors = { green: 'bg-success', yellow: 'bg-warning', red: 'bg-danger' };
      confBadge.className = 'badge ' + (colors[c.level] || 'bg-secondary');
      confBadge.textContent = `${c.score}/${c.max} ${c.level.toUpperCase()}`;
    }
    const confDetail = $('vis-p-conf-detail');
    if (confDetail && c.signals) {
      confDetail.textContent = Object.entries(c.signals)
        .map(([k, v]) => `${k}:${v}`).join(' · ');
    }

    $('vis-p-approach').textContent = fmtP(wp.approach);
    $('vis-p-grasp').textContent = fmtP(wp.grasp);
    $('vis-p-retract').textContent = fmtP(wp.retract);

    const w = document.getElementById('vis-p-warnings');
    if (plan.warnings?.length) {
      w.style.display = '';
      w.innerHTML = plan.warnings.map(x => '<i class="bi bi-exclamation-triangle"></i> ' + x).join('<br>');
    } else {
      w.style.display = 'none';
    }
  }

  async function visSimulate() {
    const plan = _visState.plan;
    if (!plan) { showToast('Select a target first', 'warning'); return; }
    const wp = plan.waypoints;
    const poses = [wp.approach, wp.grasp, wp.grasp, wp.retract];
    try {
      const r = await fetch('/api/inverse-kin', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ poses }),
      });
      const res = await r.json();
      if (!res.success) {
        showToast(`IK failed: ${res.error}`, 'danger');
        // Fallback: show plan card without animation.
        viShowPlanSteps(wp, null);
        return;
      }
      const steps = [
        { joints: res.joints[0], gripper_pos: 800, label: '1. Approach (gripper open)' },
        { joints: res.joints[1], gripper_pos: 800, label: '2. Descend to grasp' },
        { joints: res.joints[2], gripper_pos: 200, label: '3. Close gripper' },
        { joints: res.joints[3], gripper_pos: 200, label: '4. Retract' },
      ];
      viShowPlanSteps(wp, steps);
      if (window.simulateRobotMotion) {
        window.simulateRobotMotion(steps, {
          stepDuration: 1.5,
          onComplete: () => logActivity('Simulation complete', 'info'),
        });
      }
    } catch (e) {
      showToast(`Simulate error: ${e}`, 'danger');
    }
  }

  function viShowPlanSteps(wp, steps) {
    const card = document.getElementById('vis-plan-card');
    const list = document.getElementById('vis-plan-steps');
    if (!card || !list) return;
    card.style.display = '';
    const fmtP = p => p ? `(${p[0].toFixed(1)}, ${p[1].toFixed(1)}, ${p[2].toFixed(1)})` : '--';
    const allWp = wp.all || [];
    if (allWp.length) {
      list.innerHTML = allWp.map((w, i) => {
        let desc = w.label;
        if (w.gripper_action) desc += ` [gripper ${w.gripper_action} → ${w.gripper_pos}]`;
        desc += `: ${fmtP(w.pose)}`;
        return `<li>${desc}</li>`;
      }).join('');
    } else {
      list.innerHTML = [
        `<li>Approach: ${fmtP(wp.approach)}</li>`,
        `<li>Grasp: ${fmtP(wp.grasp)}</li>`,
        `<li>Retract: ${fmtP(wp.retract)}</li>`,
      ].join('');
    }
  }

  async function visExecute() {
    const t = _visState.target;
    if (!t || !_visState.plan) { showToast('Select a target first', 'warning'); return; }
    const conf = _visState.plan.confidence;
    if (conf?.level === 'red') {
      alert('Pick confidence is RED — too risky. Check the flagged signals.');
      return;
    }
    const desc = t.kind === 'object' ? `object #${t.object_id}` : `pixel (${t.px}, ${t.py})`;
    if (!confirm(`Execute pick on ${desc}?\n\nOpen → approach → descend → close → retract.`)) return;

    const body = t.kind === 'object'
      ? { object_id: t.object_id, confirm: true }
      : { px: t.px, py: t.py, confirm: true };

    try {
      logActivity(`Executing pick: ${desc}...`, 'info');
      const r = await fetch('/api/vision/execute', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const res = await r.json();
      if (res.success) {
        logActivity('Pick complete!', 'success');
        showToast('Pick complete', 'success');
      } else {
        logActivity(`Pick failed: ${res.error}`, 'error');
        showToast(`Pick failed: ${res.error}`, 'danger');
      }
    } catch (e) {
      showToast(`Execute error: ${e}`, 'danger');
    }
  }


  // Register the init for app.js's init() flow.
  window.DobotUI = window.DobotUI || {};
  window.DobotUI.initVisionTab = initVisionTab;
})();
