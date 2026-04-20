'use strict';

// ── Dobot CR5 Dashboard App ────────────────────────────────────

(function () {

  // ── State ──────────────────────────────────────────────────

  const state = {
    connected: false,
    linearStep: 5,
    angularStep: 5,
    jogSpeed: 5,
    jogMode: 'user',
    gripperOpen: true,
    ws: null,
    wsRetryDelay: 1000,
    updateCount: 0,
    lastRateTime: Date.now(),
  };

  // ── DOM refs ───────────────────────────────────────────────

  const $ = (sel) => document.querySelector(sel);
  const $$ = (sel) => document.querySelectorAll(sel);

  // ── Utilities ──────────────────────────────────────────────

  function formatTime() {
    const d = new Date();
    return d.toLocaleTimeString('en-US', { hour12: false }) + '.' +
      String(d.getMilliseconds()).padStart(3, '0');
  }

  function formatUptime(seconds) {
    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    const s = seconds % 60;
    return `${h}h ${m}m ${s}s`;
  }

  function logActivity(msg, type = '') {
    const log = $('#activity-log');
    const entry = document.createElement('div');
    entry.className = `log-entry ${type ? 'log-' + type : ''}`;
    // Use textContent for the message to prevent XSS from server error strings.
    const timeSpan = document.createElement('span');
    timeSpan.className = 'log-time';
    timeSpan.textContent = formatTime();
    const msgSpan = document.createElement('span');
    msgSpan.className = 'log-msg';
    msgSpan.textContent = msg;
    entry.appendChild(timeSpan);
    entry.appendChild(msgSpan);
    log.appendChild(entry);
    log.scrollTop = log.scrollHeight;
    // Keep max 200 entries
    while (log.children.length > 200) log.removeChild(log.firstChild);
  }

  function showToast(message, type = 'info') {
    const container = $('#toast-container');
    const colors = { info: 'text-bg-info', success: 'text-bg-success', warning: 'text-bg-warning', danger: 'text-bg-danger' };
    const toast = document.createElement('div');
    toast.className = `toast align-items-center ${colors[type] || colors.info} border-0`;
    toast.setAttribute('role', 'alert');
    // Build DOM nodes to prevent XSS from server error messages.
    const d = document.createElement('div');
    d.className = 'd-flex';
    const body = document.createElement('div');
    body.className = 'toast-body';
    body.textContent = message;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'btn-close btn-close-white me-2 m-auto';
    btn.setAttribute('data-bs-dismiss', 'toast');
    d.appendChild(body);
    d.appendChild(btn);
    toast.appendChild(d);
    container.appendChild(toast);
    const bsToast = new bootstrap.Toast(toast, { delay: 3000 });
    bsToast.show();
    toast.addEventListener('hidden.bs.toast', () => toast.remove());
  }

  // ── API calls ──────────────────────────────────────────────

  async function api(endpoint, method = 'GET', body = null) {
    const opts = { method, headers: {} };
    if (body) {
      opts.headers['Content-Type'] = 'application/json';
      opts.body = JSON.stringify(body);
    }
    const res = await fetch(`/api/${endpoint}`, opts);
    return res.json();
  }

  // ── WebSocket ──────────────────────────────────────────────

  function connectWebSocket() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${location.host}/ws/state`;

    state.ws = new WebSocket(wsUrl);
    state.ws.onopen = () => {
      state.wsRetryDelay = 1000;
      logActivity('WebSocket connected', 'success');
    };

    state.ws.onmessage = (evt) => {
      try {
        const data = JSON.parse(evt.data);
        updateDisplay(data);
      } catch (e) { /* ignore parse errors */ }
    };

    state.ws.onclose = () => {
      updateConnectionBadge(false);
      setTimeout(connectWebSocket, Math.min(state.wsRetryDelay, 10000));
      state.wsRetryDelay *= 1.5;
    };

    state.ws.onerror = () => { state.ws.close(); };
  }

  // ── Display Updates ────────────────────────────────────────

  function updateConnectionBadge(connected, modeName) {
    const badge = $('#badge-connection');
    if (connected) {
      badge.className = 'status-badge badge-connected';
      badge.innerHTML = '<i class="bi bi-circle-fill"></i> Connected';
    } else {
      badge.className = 'status-badge badge-disconnected';
      badge.innerHTML = '<i class="bi bi-circle-fill"></i> Disconnected';
    }

    const modeBadge = $('#badge-mode');
    if (modeName) {
      modeBadge.textContent = modeName;
      const modeColors = {
        'ENABLE': 'bg-success', 'RUNNING': 'bg-success',
        'DISABLED': 'bg-secondary', 'POWEROFF': 'bg-secondary',
        'INIT': 'bg-info', 'BRAKE_OPEN': 'bg-info',
        'BACKDRIVE': 'bg-info', 'SINGLE_STEP': 'bg-info',
        'ERROR': 'bg-danger', 'COLLISION': 'bg-danger',
        'PAUSE': 'bg-warning',
      };
      modeBadge.className = `badge ${modeColors[modeName] || 'bg-secondary'}`;
    }
  }

  // Joint angle ranges for bar visualization (approximate for CR5)
  const JOINT_LIMITS = [
    [-360, 360], // J1
    [-360, 360], // J2
    [-160, 160], // J3
    [-360, 360], // J4
    [-360, 360], // J5
    [-360, 360], // J6
  ];

  // Cartesian approximate ranges for bar visualization
  const CART_LIMITS = [
    [-900, 900],   // X mm
    [-900, 900],   // Y mm
    [-200, 900],   // Z mm
    [-180, 180],   // RX deg
    [-180, 180],   // RY deg
    [-180, 180],   // RZ deg
  ];

  function setBarWidth(id, value, min, max) {
    const el = document.getElementById(id);
    if (!el) return;
    // Map value to 0-100% range
    const pct = Math.max(0, Math.min(100, ((value - min) / (max - min)) * 100));
    el.style.width = pct + '%';
  }

  function updateDisplay(data) {
    state.connected = data.connected;
    updateConnectionBadge(data.connected, data.robot_mode_name);

    // Update rate
    state.updateCount++;
    const now = Date.now();
    if (now - state.lastRateTime >= 1000) {
      const hz = (state.updateCount / ((now - state.lastRateTime) / 1000)).toFixed(1);
      $('#update-rate').textContent = hz + ' Hz';
      state.updateCount = 0;
      state.lastRateTime = now;
    }

    // Joint angles
    const jointIds = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
    data.joint.forEach((v, i) => {
      const el = document.getElementById(jointIds[i]);
      if (el) el.textContent = v.toFixed(2);
      setBarWidth(jointIds[i] + '-bar', v, JOINT_LIMITS[i][0], JOINT_LIMITS[i][1]);
    });

    // Cartesian position
    const cartIds = ['px', 'py', 'pz', 'prx', 'pry', 'prz'];
    const barIds = ['px-bar', 'py-bar', 'pz-bar', 'prx-bar', 'pry-bar', 'prz-bar'];
    data.cartesian.forEach((v, i) => {
      const el = document.getElementById(cartIds[i]);
      if (el) el.textContent = v.toFixed(2);
      setBarWidth(barIds[i], v, CART_LIMITS[i][0], CART_LIMITS[i][1]);
    });

    // Robot info
    $('#info-mode').textContent = data.robot_mode_name || '--';

    // Gripper
    if (data.gripper_position >= 0) {
      const gPos = data.gripper_position;
      $('#gripper-current-pos').textContent = gPos;
      const slider = $('#gripper-pos-slider');
      if (slider && document.activeElement !== slider) {
        slider.value = gPos;
        $('#gripper-pos-label').textContent = gPos;
      }
      const gBar = document.getElementById('gripper-bar-fill');
      if (gBar) gBar.style.width = (gPos / 10) + '%';
    }

    const stateNames = { 0: 'Moving', 1: 'Reached', 2: 'Object Caught', 3: 'Object Dropped' };
    $('#gripper-state-text').textContent = stateNames[data.gripper_state] || '--';

    const gBadge = $('#gripper-status-badge');
    if (data.gripper_initialized) {
      gBadge.textContent = 'Ready';
      gBadge.className = 'badge bg-success';
    } else {
      gBadge.textContent = 'Not Init';
      gBadge.className = 'badge bg-secondary';
    }

    if (data.error) {
      $('#info-mode').textContent = data.robot_mode_name + ' (ERR)';
    }

    // Drive 3D visualization (joints + gripper)
    if (window.updateRobotJoints && data.joint) {
      window.updateRobotJoints(data.joint);
    }
    if (window.updateRobotGripper && data.gripper_position >= 0) {
      window.updateRobotGripper(data.gripper_position);
    }
  }

  // ── Robot Commands ─────────────────────────────────────────

  async function robotCommand(name, endpoint, method = 'POST', body = null) {
    logActivity(`${name}...`, 'info');
    try {
      const result = await api(endpoint, method, body);
      if (result.success) {
        logActivity(`${name}: OK`, 'success');
        showToast(name + ' successful', 'success');
      } else {
        logActivity(`${name}: ${result.error}`, 'error');
        showToast(result.error, 'danger');
      }
      return result;
    } catch (e) {
      logActivity(`${name}: ${e.message}`, 'error');
      showToast(e.message, 'danger');
      return { success: false, error: e.message };
    }
  }

  // ── Jog ────────────────────────────────────────────────────

  async function doJog(axis, direction) {
    const isJoint = axis.startsWith('j');
    const isAngular = isJoint || ['rx', 'ry', 'rz'].includes(axis);
    const step = isAngular ? state.angularStep : state.linearStep;
    const distance = step * direction;
    const mode = document.querySelector('input[name="jogCoordMode"]:checked')?.value || 'user';

    const unit = isAngular ? '\u00B0' : 'mm';
    logActivity(`Jog ${axis.toUpperCase()} ${distance > 0 ? '+' : ''}${distance}${unit} (${state.jogSpeed}%)`, 'info');

    try {
      const result = await api('jog', 'POST', {
        axis, distance, speed: state.jogSpeed, mode
      });
      if (result.success) {
        logActivity(`Jog ${axis.toUpperCase()}: sent`, 'success');
      } else {
        logActivity(`Jog failed: ${result.error}`, 'error');
        showToast(result.error, 'danger');
      }
    } catch (e) {
      logActivity(`Jog error: ${e.message}`, 'error');
    }
  }

  // ── Gripper Commands ───────────────────────────────────────

  function getGripperParams() {
    return {
      speed: 100,  // AG-95 has no speed register; speed is controlled by force
      force: parseInt($('#gripper-force').value),
    };
  }

  // ── Config Loading ─────────────────────────────────────────

  async function loadConfig() {
    try {
      const cfg = await api('config');
      $('#cfg-robot-ip').textContent = cfg.robot_ip || '--';
      $('#cfg-namespace').textContent = cfg.ros_namespace || '(none)';
      $('#cfg-timeout').textContent = cfg.service_timeout ? cfg.service_timeout + 's' : '--';
      $('#cfg-jog-dist').textContent = cfg.jog_distance ? cfg.jog_distance + ' mm' : '--';
      $('#cfg-jog-rot').textContent = cfg.jog_rotation ? cfg.jog_rotation + '\u00B0' : '--';
      $('#cfg-jog-mode').textContent = cfg.jog_mode || '--';
      $('#cfg-sync').textContent = cfg.sync_mode ? 'ON' : 'OFF';
      $('#cfg-tol-deg').textContent = cfg.motion_tolerance_deg ? cfg.motion_tolerance_deg + '\u00B0' : '--';
      $('#cfg-tol-mm').textContent = cfg.motion_tolerance_mm ? cfg.motion_tolerance_mm + ' mm' : '--';
      $('#cfg-motion-timeout').textContent = cfg.motion_timeout ? cfg.motion_timeout + 's' : '--';
      $('#info-config').textContent = cfg.robot_ip || '--';
    } catch (e) { /* ignore */ }
  }

  // ── Status Polling (fallback + uptime) ─────────────────────

  async function pollStatus() {
    try {
      const s = await api('status');
      $('#info-uptime').textContent = formatUptime(s.uptime_seconds || 0);
    } catch (e) { /* ignore */ }
  }

  // ── Keyboard Shortcuts ─────────────────────────────────────

  const linearSteps = [1, 5, 10, 25, 50];
  const angularSteps = [1, 5, 10, 25, 45];

  function handleKeyboard(e) {
    // Don't capture when typing in inputs
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

    const key = e.key.toLowerCase();

    // Movement keys
    switch (key) {
      case 'a': doJog('x', -1); e.preventDefault(); break;
      case 'd': doJog('x', 1); e.preventDefault(); break;
      case 'w': doJog('y', 1); e.preventDefault(); break;
      case 's': doJog('y', -1); e.preventDefault(); break;
      case 'q': doJog('z', -1); e.preventDefault(); break;
      case 'e': doJog('z', 1); e.preventDefault(); break;
      case ' ':
        robotCommand('STOP', 'stop');
        e.preventDefault();
        break;
      case 'g':
        // Toggle gripper
        if (state.gripperOpen) {
          const p = getGripperParams();
          robotCommand('Gripper Close', `gripper/close?speed=${p.speed}&force=${p.force}`);
        } else {
          const p = getGripperParams();
          robotCommand('Gripper Open', `gripper/open?speed=${p.speed}&force=${p.force}`);
        }
        state.gripperOpen = !state.gripperOpen;
        e.preventDefault();
        break;
    }

    // Number keys for step sizes
    if (!e.shiftKey && key >= '1' && key <= '5') {
      const idx = parseInt(key) - 1;
      state.linearStep = linearSteps[idx];
      $$('.step-btn[data-type="linear"]').forEach(b =>
        b.classList.toggle('active', parseFloat(b.dataset.step) === state.linearStep));
      logActivity(`Linear step: ${state.linearStep} mm`, 'info');
      e.preventDefault();
    }
    if (e.shiftKey && e.code >= 'Digit1' && e.code <= 'Digit5') {
      const idx = parseInt(e.code.slice(-1)) - 1;
      state.angularStep = angularSteps[idx];
      $$('.step-btn[data-type="angular"]').forEach(b =>
        b.classList.toggle('active', parseFloat(b.dataset.step) === state.angularStep));
      logActivity(`Angular step: ${state.angularStep}\u00B0`, 'info');
      e.preventDefault();
    }
  }

  // ── Init ───────────────────────────────────────────────────

  function init() {
    // Robot control buttons
    $('#btn-enable').addEventListener('click', () => robotCommand('Enable Robot', 'enable'));
    $('#btn-disable').addEventListener('click', () => robotCommand('Disable Robot', 'disable'));
    $('#btn-clear-error').addEventListener('click', () => robotCommand('Clear Error', 'clear'));
    $('#btn-stop').addEventListener('click', () => robotCommand('STOP', 'stop'));
    $('#btn-orient-vertical').addEventListener('click',
      () => robotCommand('Gripper vertical', 'orient/vertical'));

    // Lock-vertical toggle. When on, the servo tester re-projects every
    // ServoP target to RX=180, RY=0 so SpaceMouse jogging can't tip the
    // tool off perpendicular. Persisted via /api/servo/config so it
    // survives reloads / container restarts.
    const lockChk = $('#chk-lock-vertical');
    if (lockChk) {
      (async () => {
        try {
          const r = await fetch('/api/servo/config');
          const j = await r.json();
          if (j.success && j.config && typeof j.config.lock_vertical === 'boolean') {
            lockChk.checked = j.config.lock_vertical;
          }
        } catch (_) {}
      })();
      lockChk.addEventListener('change', async () => {
        const res = await api('servo/config', 'POST', { lock_vertical: lockChk.checked });
        if (res.success) {
          logActivity(`Lock vertical: ${lockChk.checked ? 'ON' : 'off'}`, 'info');
        } else {
          showToast(`Lock toggle failed: ${res.error}`, 'danger');
          // resync
          try {
            const r = await fetch('/api/servo/config');
            const j = await r.json();
            if (j.success && j.config) lockChk.checked = !!j.config.lock_vertical;
          } catch (_) {}
        }
      });
    }

    // Main drag toggle
    let mainDragActive = false;
    $('#btn-drag-toggle').addEventListener('click', async () => {
      if (mainDragActive) {
        await api('drag/stop', 'POST');
        mainDragActive = false;
        $('#btn-drag-toggle').className = 'btn btn-outline-warning btn-sm flex-fill';
        $('#btn-drag-toggle').textContent = 'Drag';
        showToast('Drag mode off', 'info');
      } else {
        const result = await api('drag/start', 'POST');
        if (result.success) {
          mainDragActive = true;
          $('#btn-drag-toggle').className = 'btn btn-warning btn-sm flex-fill';
          $('#btn-drag-toggle').textContent = 'Drag ON';
          showToast('Drag mode on', 'success');
        } else {
          showToast(result.error || 'Failed', 'danger');
        }
      }
    });

    // Speed slider
    const speedSlider = $('#speed-slider');
    speedSlider.addEventListener('input', () => {
      $('#speed-label').textContent = speedSlider.value;
    });
    speedSlider.addEventListener('change', () => {
      const speed = parseInt(speedSlider.value);
      robotCommand(`Speed ${speed}%`, 'speed', 'POST', { speed });
    });

    // Tool-frame selector. Sets the robot's active tool coordinate system
    // (Tool 0 = wrist/flange, Tool 1 = fingertip). Reads current value
    // on load and pushes to /api/tool on change. Any failure (e.g.
    // workspace-protection error on switch to Tool 1) is surfaced via
    // the usual toast + activity log.
    const toolSelect = $('#tool-select');
    if (toolSelect) {
      (async () => {
        try {
          const r = await fetch('/api/tool');
          const j = await r.json();
          if (typeof j.active_index === 'number') toolSelect.value = String(j.active_index);
        } catch (_) { /* keep HTML default */ }
      })();
      toolSelect.addEventListener('change', async () => {
        const index = parseInt(toolSelect.value, 10);
        const label = toolSelect.options[toolSelect.selectedIndex].textContent;
        const res = await api('tool', 'POST', { index });
        if (res.success) {
          logActivity(`Tool frame → ${label}`, 'info');
        } else {
          showToast(`Tool switch failed: ${res.error}`, 'danger');
          // Try to resync from the server so the dropdown reflects actual state.
          try {
            const r = await fetch('/api/tool');
            const j = await r.json();
            if (typeof j.active_index === 'number') toolSelect.value = String(j.active_index);
          } catch (_) {}
        }
      });
    }

    // Jog speed slider
    const jogSpeedSlider = $('#jog-speed');
    jogSpeedSlider.addEventListener('input', () => {
      state.jogSpeed = parseInt(jogSpeedSlider.value);
      $('#jog-speed-label').textContent = state.jogSpeed;
    });

    // Step size buttons
    $$('.step-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const type = btn.dataset.type;
        const step = parseFloat(btn.dataset.step);
        if (type === 'linear') {
          state.linearStep = step;
          $$('.step-btn[data-type="linear"]').forEach(b => b.classList.remove('active'));
        } else {
          state.angularStep = step;
          $$('.step-btn[data-type="angular"]').forEach(b => b.classList.remove('active'));
        }
        btn.classList.add('active');
      });
    });

    // Jog buttons
    $$('.jog-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const axis = btn.dataset.axis;
        const dir = parseInt(btn.dataset.dir);
        doJog(axis, dir);
      });
    });

    // Gripper buttons
    $('#btn-gripper-init').addEventListener('click', () =>
      robotCommand('Gripper Init', 'gripper/init'));
    $('#btn-gripper-open').addEventListener('click', () => {
      const p = getGripperParams();
      robotCommand('Gripper Open', `gripper/open?speed=${p.speed}&force=${p.force}`);
      state.gripperOpen = true;
    });
    $('#btn-gripper-close').addEventListener('click', () => {
      const p = getGripperParams();
      robotCommand('Gripper Close', `gripper/close?speed=${p.speed}&force=${p.force}`);
      state.gripperOpen = false;
    });
    $('#btn-gripper-move').addEventListener('click', () => {
      const pos = parseInt($('#gripper-pos-slider').value);
      const p = getGripperParams();
      robotCommand(`Gripper Move ${pos}`, 'gripper/move', 'POST', {
        position: pos, speed: p.speed, force: p.force
      });
    });

    // Gripper sliders
    $('#gripper-pos-slider').addEventListener('input', (e) => {
      $('#gripper-pos-label').textContent = e.target.value;
    });
    $('#gripper-force').addEventListener('input', (e) => {
      $('#gripper-force-label').textContent = e.target.value;
    });

    // 3D camera save/reset
    const saveBtn = $('#btn-save-camera');
    if (saveBtn) {
      saveBtn.addEventListener('click', () => {
        if (window.saveRobotCamera) {
          window.saveRobotCamera();
          showToast('Camera view saved', 'success');
        }
      });
    }
    const resetBtn = $('#btn-reset-camera');
    if (resetBtn) {
      resetBtn.addEventListener('click', () => {
        if (window.resetRobotCamera) window.resetRobotCamera();
      });
    }

    // Clear log
    $('#btn-clear-log').addEventListener('click', () => {
      $('#activity-log').innerHTML = '';
    });

    // ── Calibration ──────────────────────────────────────────
    let calSelectedPx = 320, calSelectedPy = 180;
    let calRecordedPixels = [];  // [{px, py, index}]

    function calDrawOverlay() {
      const img = $('#cal-camera-feed');
      const canvas = $('#cal-overlay');
      if (!img || !canvas || img.style.display === 'none') return;
      canvas.width = img.naturalWidth || 640;
      canvas.height = img.naturalHeight || 360;
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Draw recorded points
      calRecordedPixels.forEach((pt) => {
        const x = pt.px, y = pt.py;
        // Crosshair
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x - 10, y); ctx.lineTo(x + 10, y);
        ctx.moveTo(x, y - 10); ctx.lineTo(x, y + 10);
        ctx.stroke();
        // Label
        ctx.fillStyle = '#00ff00';
        ctx.font = 'bold 12px monospace';
        ctx.fillText(pt.index, x + 12, y - 4);
      });

      // Draw current selection
      if (calSelectedPx >= 0) {
        const x = calSelectedPx, y = calSelectedPy;
        ctx.strokeStyle = '#ff4444';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(x - 15, y); ctx.lineTo(x + 15, y);
        ctx.moveTo(x, y - 15); ctx.lineTo(x, y + 15);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }

    async function calRefreshStatus() {
      try {
        const data = await api('calibration/status');
        const badge = $('#cal-status-badge');
        $('#cal-num-points').textContent = data.num_points || 0;
        if (data.solved) {
          badge.className = 'badge bg-success';
          badge.textContent = `Calibrated (${data.error_mm}mm)`;
          $('#cal-error').textContent = data.error_mm + 'mm';
          if ($('#cal-main-cam-status')) {
            $('#cal-main-cam-status').textContent = `${data.num_points} pts, error=${data.error_mm}mm`;
            $('#cal-main-cam-status').className = 'text-success';
          }
        } else {
          badge.className = 'badge bg-secondary';
          badge.textContent = 'Not Calibrated';
          $('#cal-error').textContent = '--';
          if ($('#cal-main-cam-status')) {
            $('#cal-main-cam-status').textContent = data.num_points > 0 ? `${data.num_points} pts (not solved)` : 'Not set';
            $('#cal-main-cam-status').className = 'text-muted';
          }
        }
        // Point list
        const list = $('#cal-points-list');
        if (data.points && data.points.length > 0) {
          list.innerHTML = data.points.map((p, i) => {
            const c = p.camera_xyz;
            const r = p.robot_xyz;
            const depth = Math.sqrt(c[0]*c[0] + c[1]*c[1] + c[2]*c[2]);
            const depthIn = depth / 0.0254;
            return `<div class="text-muted mb-1 d-flex justify-content-between align-items-center">${i+1}. depth=${depth.toFixed(3)}m (${depthIn.toFixed(1)}in) → robot(${r[0].toFixed(1)}, ${r[1].toFixed(1)}, ${r[2].toFixed(1)})mm <i class="bi bi-x-circle text-danger cal-delete-point" role="button" data-index="${i}" style="cursor:pointer; flex-shrink:0;"></i></div>`;
          }).join('');
        } else {
          list.innerHTML = '<div class="text-muted">No points recorded</div>';
        }
      } catch(e) { /* ignore */ }
    }

    function calRefreshFeed() {
      const img = $('#cal-camera-feed');
      img.src = '/api/calibration/camera-frame?' + Date.now();
      img.style.display = 'block';
      img.onload = calDrawOverlay;
    }

    $('#btn-cal-refresh-feed').addEventListener('click', calRefreshFeed);

    // Zoom & pan state for camera image
    let calZoom = 1, calPanX = 0, calPanY = 0, calDragging = false, calDragStart = {};

    function calApplyTransform() {
      const img = $('#cal-camera-feed');
      const canvas = $('#cal-overlay');
      const t = `scale(${calZoom}) translate(${calPanX}px, ${calPanY}px)`;
      img.style.transform = t;
      img.style.transformOrigin = '0 0';
      canvas.style.transform = t;
      canvas.style.transformOrigin = '0 0';
    }

    // Scroll wheel to zoom
    $('#cal-camera-container').addEventListener('wheel', (e) => {
      e.preventDefault();
      const delta = e.deltaY > 0 ? 0.9 : 1.1;
      const newZoom = Math.max(1, Math.min(8, calZoom * delta));
      // Zoom toward mouse position
      const rect = e.currentTarget.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      calPanX -= (mx / calZoom - mx / newZoom);
      calPanY -= (my / calZoom - my / newZoom);
      calZoom = newZoom;
      if (calZoom <= 1) { calPanX = 0; calPanY = 0; }
      calApplyTransform();
    }, { passive: false });

    // Left-drag to pan when zoomed, click (no drag) to select pixel
    let calMouseDownPos = null;
    $('#cal-camera-container').addEventListener('mousedown', (e) => {
      if (e.button !== 0) return;
      calMouseDownPos = { x: e.clientX, y: e.clientY };
      if (calZoom > 1) {
        calDragging = true;
        calDragStart = { x: e.clientX - calPanX * calZoom, y: e.clientY - calPanY * calZoom };
      }
      e.preventDefault();
    });
    document.addEventListener('mousemove', (e) => {
      if (!calDragging) return;
      calPanX = (e.clientX - calDragStart.x) / calZoom;
      calPanY = (e.clientY - calDragStart.y) / calZoom;
      calApplyTransform();
    });
    document.addEventListener('mouseup', (e) => {
      if (e.button !== 0) return;
      const wasDrag = calMouseDownPos &&
        (Math.abs(e.clientX - calMouseDownPos.x) > 3 || Math.abs(e.clientY - calMouseDownPos.y) > 3);
      calDragging = false;
      // Only select pixel if it was a click, not a drag
      if (!wasDrag && calMouseDownPos) {
        const img = $('#cal-camera-feed');
        const rect = img.getBoundingClientRect();
        const scaleX = (img.naturalWidth || 640) / rect.width;
        const scaleY = (img.naturalHeight || 360) / rect.height;
        calSelectedPx = Math.round((e.clientX - rect.left) * scaleX);
        calSelectedPy = Math.round((e.clientY - rect.top) * scaleY);
        $('#cal-pixel').textContent = `${calSelectedPx}, ${calSelectedPy}`;
        calDrawOverlay();
        // Live depth polling at selected pixel
        calStartDepthPolling();
      }
      calMouseDownPos = null;
    });
    $('#cal-camera-container').addEventListener('contextmenu', (e) => e.preventDefault());

    // Table point — robot position only, no camera
    $('#btn-cal-table-point').addEventListener('click', async () => {
      const result = await api('calibration/table/point', 'POST');
      if (result.success) {
        const p = result.point;
        showToast(`Table point ${result.num_points}: Z=${p[2].toFixed(1)}mm`, 'success');
        logActivity(`Table point ${result.num_points}: (${p[0].toFixed(1)}, ${p[1].toFixed(1)}, ${p[2].toFixed(1)})mm`, 'success');
        if (result.plane) {
          logActivity(`Table plane solved: mean Z=${result.plane.mean_z.toFixed(1)}mm`, 'success');
        }
        calRefreshTable();
      } else {
        showToast(result.error || 'Failed', 'danger');
      }
    });

    // Camera calibration point — needs pixel click + robot position
    $('#btn-cal-record').addEventListener('click', async () => {
      logActivity(`Recording camera point at pixel (${calSelectedPx}, ${calSelectedPy})...`, 'info');
      const result = await api('calibration/record', 'POST', {
        px: calSelectedPx, py: calSelectedPy
      });
      if (result.success) {
        const r = result.robot_xyz;
        calRecordedPixels.push({ px: calSelectedPx, py: calSelectedPy, index: result.total_points });
        showToast(`Camera point ${result.total_points} recorded — depth ${result.depth_m}m`, 'success');
        logActivity(`Camera point ${result.total_points}: robot(${r[0].toFixed(1)},${r[1].toFixed(1)},${r[2].toFixed(1)})mm | depth=${result.depth_m}m`, 'success');
        calRefreshStatus();
        calDrawOverlay();
      } else {
        showToast(result.error || 'Record failed', 'danger');
      }
    });

    $('#btn-cal-solve').addEventListener('click', async () => {
      const result = await api('calibration/solve', 'POST');
      if (result.solved) {
        showToast(`Calibration solved! Error: ${result.error_mm}mm`, 'success');
        logActivity(`Calibration solved: ${result.num_points} points, error=${result.error_mm}mm`, 'success');
      } else {
        showToast(`Need at least 3 points (have ${result.num_points || 0})`, 'warning');
      }
      calRefreshStatus();
    });

    $('#btn-cal-clear').addEventListener('click', async () => {
      if (!confirm('Clear all workspace and calibration data?')) return;
      await api('calibration/clear', 'POST');
      await api('calibration/table/clear', 'POST');
      calRecordedPixels = [];
      showToast('All calibration cleared', 'info');
      calRefreshStatus();
      calRefreshTable();
      calDrawOverlay();
      $('#cal-test-results').style.display = 'none';
    });

    $('#btn-cal-test').addEventListener('click', async () => {
      const result = await api('calibration/test');
      const div = $('#cal-test-results');
      if (!result.success) {
        showToast(result.error || 'Test failed', 'danger');
        return;
      }
      const actual = result.actual_robot_xyz;
      let html = `<div class="text-info mb-1">Robot at: (${actual[0].toFixed(1)}, ${actual[1].toFixed(1)}, ${actual[2].toFixed(1)})mm</div>`;
      if (result.objects.length === 0) {
        html += '<div class="text-muted">No objects detected</div>';
      }
      result.objects.forEach(obj => {
        const r = obj.robot_xyz;
        const dx = (r[0] - actual[0]).toFixed(1);
        const dy = (r[1] - actual[1]).toFixed(1);
        const dz = (r[2] - actual[2]).toFixed(1);
        const dist = Math.sqrt(dx*dx + dy*dy + dz*dz).toFixed(1);
        html += `<div>#${obj.id} → (${r[0].toFixed(1)}, ${r[1].toFixed(1)}, ${r[2].toFixed(1)})mm — delta: ${dist}mm (dx=${dx} dy=${dy} dz=${dz})</div>`;
      });
      div.innerHTML = html;
      div.style.display = 'block';
    });

    // Drag mode toggle
    let dragActive = false;
    $('#btn-cal-drag-toggle').addEventListener('click', async () => {
      if (dragActive) {
        await api('drag/stop', 'POST');
        dragActive = false;
        $('#btn-cal-drag-toggle').className = 'btn btn-outline-warning btn-sm w-100 mb-2';
        $('#btn-cal-drag-toggle').innerHTML = '<i class="bi bi-hand-index"></i> Enable Drag Mode';
        showToast('Drag mode off', 'info');
      } else {
        const result = await api('drag/start', 'POST');
        if (result.success) {
          dragActive = true;
          $('#btn-cal-drag-toggle').className = 'btn btn-warning btn-sm w-100 mb-2';
          $('#btn-cal-drag-toggle').innerHTML = '<i class="bi bi-hand-index-fill"></i> Drag Mode ON — move robot by hand';
          showToast('Drag mode on — move robot by hand', 'success');
        } else {
          showToast(result.error || 'Failed to start drag', 'danger');
        }
      }
    });

    // Calibration jog buttons
    document.querySelectorAll('.cal-jog').forEach(btn => {
      btn.addEventListener('click', async () => {
        const axis = btn.dataset.axis;
        const dir = parseInt(btn.dataset.dir);
        const step = parseInt($('#cal-jog-step').value) * dir;
        await api('jog', 'POST', { axis, distance: step, speed: state.jogSpeed, mode: 'user' });
        // Update position display
        try {
          const st = await api('status');
          const c = st.cartesian;
          $('#cal-robot-pos').textContent = `X:${c[0].toFixed(1)} Y:${c[1].toFixed(1)} Z:${c[2].toFixed(1)}`;
        } catch(e) {}
      });
    });

    // Live depth polling at selected pixel
    let calDepthInterval = null;
    function calStartDepthPolling() {
      if (calDepthInterval) clearInterval(calDepthInterval);
      calPollDepth();
      calDepthInterval = setInterval(calPollDepth, 500);
    }
    async function calPollDepth() {
      try {
        const resp = await fetch(`/api/calibration/depth-at?x=${calSelectedPx}&y=${calSelectedPy}`);
        const data = await resp.json();
        const d = data.distance || 0;
        const inches = d / 0.0254;
        const el = $('#cal-depth-live');
        if (d > 0) {
          el.textContent = `${d.toFixed(3)}m (${inches.toFixed(1)}in)`;
          el.className = d > 2.0 ? 'text-danger' : 'text-warning';
        } else {
          el.textContent = 'no depth';
          el.className = 'text-danger';
        }
      } catch(e) {
        $('#cal-depth-live').textContent = 'err';
      }
    }

    // Table plane calibration
    async function calRefreshTable() {
      try {
        const data = await api('calibration/table');
        const count = (data.points || []).length;
        $('#cal-table-count').textContent = count;
        $('#cal-min-clearance').value = data.min_clearance_mm || 25;

        // Modal status
        if (data.plane) {
          const clearance = data.min_clearance_mm || 25;
          $('#cal-table-status').textContent = `${count} pts, Z=${data.plane.mean_z.toFixed(1)}mm, clearance=${clearance}mm`;
          $('#cal-table-status').className = 'text-success';
        } else {
          $('#cal-table-status').textContent = count > 0 ? `${count}/3 points` : '--';
          $('#cal-table-status').className = '';
        }

        // Main card status
        const mainTable = $('#cal-main-table-status');
        const mainSize = $('#cal-main-table-size');
        if (mainTable) {
          if (data.plane) {
            mainTable.textContent = `${count} pts, Z=${data.plane.mean_z.toFixed(1)}mm`;
            mainTable.className = 'text-success';
          } else {
            mainTable.textContent = count > 0 ? `${count} points (need 3+)` : 'Not set';
            mainTable.className = 'text-muted';
          }
        }
        if (mainSize && data.points && data.points.length >= 2) {
          const xs = data.points.map(p => p[0]);
          const ys = data.points.map(p => p[1]);
          const xRange = Math.max(...xs) - Math.min(...xs);
          const yRange = Math.max(...ys) - Math.min(...ys);
          mainSize.textContent = `${xRange.toFixed(0)} x ${yRange.toFixed(0)} mm (${(xRange/25.4).toFixed(1)} x ${(yRange/25.4).toFixed(1)} in)`;
        } else if (mainSize) {
          mainSize.textContent = '--';
        }
      } catch(e) {}
    }

    // Workspace test moves with countdown
    document.querySelectorAll('.cal-ws-move').forEach(btn => {
      btn.addEventListener('click', async () => {
        const pos = btn.dataset.pos;
        // Read height from whichever input is visible (modal or main card)
        const modalHeight = $('#cal-ws-height');
        const mainHeight = $('#cal-main-ws-height');
        const height = parseInt((modalHeight && modalHeight.offsetParent !== null ? modalHeight : mainHeight || modalHeight).value);
        // Disable all move buttons during countdown
        document.querySelectorAll('.cal-ws-move').forEach(b => b.disabled = true);
        for (let i = 5; i > 0; i--) {
          showToast(`Moving to ${pos} in ${i}...`, 'warning');
          logActivity(`Moving to ${pos} in ${i}...`, 'info');
          await new Promise(r => setTimeout(r, 1000));
        }
        logActivity(`Moving to ${pos} at ${height}mm above table`, 'info');
        const result = await api('calibration/workspace-move', 'POST', {
          position: pos, height_mm: height
        });
        document.querySelectorAll('.cal-ws-move').forEach(b => b.disabled = false);
        if (result.success) {
          const t = result.target;
          showToast(`Moving to ${pos}: (${t[0].toFixed(0)}, ${t[1].toFixed(0)}, ${t[2].toFixed(0)})mm`, 'success');
          logActivity(`Target: (${t[0].toFixed(1)}, ${t[1].toFixed(1)}, ${t[2].toFixed(1)})mm, table Z=${result.table_z}mm`, 'success');
        } else {
          showToast(result.error || 'Move failed', 'danger');
        }
      });
    });

    $('#btn-cal-save-clearance').addEventListener('click', async () => {
      const val = parseInt($('#cal-min-clearance').value);
      const result = await api('calibration/table/clearance', 'POST', { min_clearance_mm: val });
      if (result.success) {
        showToast(`Min clearance set to ${val}mm`, 'success');
        calRefreshTable();
      }
    });

    calRefreshTable();

    // Delete calibration point
    $('#cal-points-list').addEventListener('click', async (e) => {
      const el = e.target.closest('.cal-delete-point');
      if (!el) return;
      const index = parseInt(el.dataset.index);
      const result = await api('calibration/delete-point', 'POST', { index });
      if (result.success) {
        showToast(`Point deleted (${result.remaining} remaining)`, 'info');
        calRecordedPixels = calRecordedPixels.filter((_, i) => i !== index);
        calRefreshStatus();
        calDrawOverlay();
      } else {
        showToast(result.error || 'Delete failed', 'danger');
      }
    });

    // Initial calibration status
    calRefreshStatus();

    // Auto-refresh camera feed when calibration modal is open
    let calFeedInterval = null;
    const calModal = document.getElementById('calibrationModal');
    if (calModal) {
      calModal.addEventListener('shown.bs.modal', () => {
        calRefreshFeed();
        calRefreshStatus();
        calFeedInterval = setInterval(calRefreshFeed, 1000);
      });
      calModal.addEventListener('hidden.bs.modal', () => {
        clearInterval(calFeedInterval);
        calFeedInterval = null;
        clearInterval(calDepthInterval);
        calDepthInterval = null;
      });
    }

    // Keyboard
    document.addEventListener('keydown', handleKeyboard);

    // Load config
    loadConfig();

    // Start WebSocket
    connectWebSocket();

    // Poll status every 5s for uptime etc
    setInterval(pollStatus, 5000);

    // Feature tabs live in extracted app-*.js modules loaded after this
    // file; they each register window.DobotUI.init* hooks.
    window.DobotUI?.initVLA?.();
    window.DobotUI?.initServoTester?.();
    window.DobotUI?.initSettings?.();
    window.DobotUI?.initPopupButtons?.();
    window.DobotUI?.initHashNav?.();
    window.DobotUI?.initTooltips?.();
    window.DobotUI?.initCalibrationTab?.();
    window.DobotUI?.initVisionTab?.();
    window.DobotUI?.initSidebarReorder?.();

    // Global E-STOP in the navbar — halts any active servo streaming and
    // calls ros.stop() regardless of current tab. Safe to click anytime.
    const gEstop = document.getElementById('btn-global-estop');
    if (gEstop) {
      gEstop.addEventListener('click', async () => {
        // Halt any active servo streaming first; then issue a regular ros.stop().
        try { await fetch('/api/servo/estop', { method: 'POST' }); } catch (e) {}
        try { await fetch('/api/stop', { method: 'POST' }); } catch (e) {}
        logActivity('E-STOP engaged (servo halted, ros.stop() called)', 'warn');
      });
    }

    logActivity('Dashboard initialized', 'info');
  }

  // ── VLA ────────────────────────────────────────────────────────
  // Extracted to /static/js/app-vla.js — that file registers
  // window.DobotUI.initVLA, invoked from init() above.

  // ── Servo Tester ───────────────────────────────────────────────
  // Extracted to /static/js/app-servo.js — that file registers
  // window.DobotUI.initServoTester, invoked from init() above.

  function debounce(fn, ms) {
    let h;
    return (...args) => {
      clearTimeout(h);
      h = setTimeout(() => fn.apply(null, args), ms);
    };
  }

  // ── Calibration tab ────────────────────────────────────────────
  // Extracted to /static/js/app-calibration.js — that file registers
  // window.DobotUI.initCalibrationTab, invoked from init() above.

  // ── Vision tab + Strategy management ───────────────────────────
  // Extracted to /static/js/app-vision.js — that file registers
  // window.DobotUI.initVisionTab, invoked from init() above.

  // ── Tooltips ──────────────────────────────────────────────────
  // Bootstrap tooltips for every element that carries `data-bs-toggle=
  // "tooltip"` or a `title=` inside a content tab. Initializes once on load
  // and re-scans after each tab switch so dynamically-shown controls get wired.

  // ── Sidebar DnD + Tooltips + Popups + HashNav ───────────────
  // Extracted to app-sidebar.js and app-misc.js.
  // They register window.DobotUI.initSidebarReorder, initTooltips,
  // initPopupButtons, initHashNav — invoked from init() above.

  // Extracted to /static/js/app-settings.js — that file registers
  // window.DobotUI.initSettings, invoked from init() above.

  // Expose shared helpers for extracted modules (app-vision.js, etc).
  // Extracted modules are plain <script>s loaded AFTER this file; they
  // register their init function on DobotUI and consume helpers from it.
  window.DobotUI = window.DobotUI || {};
  Object.assign(window.DobotUI, {
    api, state, logActivity, showToast, debounce,
  });

  // Boot
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

})();
