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
    entry.innerHTML = `<span class="log-time">${formatTime()}</span><span class="log-msg">${msg}</span>`;
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
    toast.innerHTML = `
      <div class="d-flex">
        <div class="toast-body">${message}</div>
        <button type="button" class="btn-close btn-close-white me-2 m-auto" data-bs-dismiss="toast"></button>
      </div>`;
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

    initVLA();
    initServoTester();
    initSettings();
    initPendantModal();
    initHashNav();
    initTooltips();
    initCalibrationTab();
    initVisionTab();

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

  // ── VLA (Vision-Language-Action) ───────────────────────────────

  async function vlaRefreshStatus() {
    try {
      const r = await fetch('/api/vla/status');
      const s = await r.json();
      const urlEl = document.getElementById('vla-server-url');
      const statEl = document.getElementById('vla-server-status');
      const modelEl = document.getElementById('vla-server-model');
      if (urlEl) urlEl.textContent = s.server_url || '--';
      if (statEl) {
        const server = s.server || {};
        const ok = server.status === 'ok';
        statEl.innerHTML = ok
          ? `<span class="text-success">OK</span> (${server.mode || '?'})`
          : `<span class="text-danger">${server.status || 'unknown'}</span>`;
      }
      if (modelEl) modelEl.textContent = (s.server && s.server.model) || '--';

      const recStatus = document.getElementById('vla-record-status');
      if (recStatus) {
        recStatus.textContent = s.recording ? 'Recording…' : 'Not recording';
      }
      document.getElementById('btn-vla-record-start').disabled = !!s.recording;
      document.getElementById('btn-vla-record-stop').disabled = !s.recording;

      const execStatus = document.getElementById('vla-execute-status');
      if (execStatus) {
        if (s.executing && s.executor) {
          const e = s.executor;
          execStatus.innerHTML =
            `Running • step ${e.step} • ${e.last_latency_ms.toFixed(1)} ms ` +
            `• clamps ${e.clamps}` +
            (e.last_error ? ` • <span class="text-danger">${e.last_error}</span>` : '');
        } else {
          execStatus.textContent = 'Idle';
        }
      }
      document.getElementById('btn-vla-execute-start').disabled = !!s.executing;
      document.getElementById('btn-vla-execute-stop').disabled = !s.executing;
    } catch (e) {
      // Non-fatal — modal may be closed.
    }
  }

  async function vlaRefreshEpisodes() {
    const r = await fetch('/api/vla/episodes');
    const { episodes } = await r.json();
    const el = document.getElementById('vla-episodes-list');
    if (!el) return;
    if (!episodes || !episodes.length) {
      el.innerHTML = '<em class="text-muted">No episodes yet.</em>';
      return;
    }
    const rows = episodes.map(e => {
      const dur = (e.started_at && e.ended_at)
        ? `${(e.ended_at - e.started_at).toFixed(1)}s` : '—';
      return `<tr>
        <td><code>${e.name}</code></td>
        <td class="text-muted">${e.instruction || ''}</td>
        <td class="text-end">${e.num_steps || 0}</td>
        <td class="text-end">${dur}</td>
      </tr>`;
    }).join('');
    el.innerHTML = `<table class="table table-sm table-borderless mb-0">
      <thead class="small text-muted">
        <tr><th>Name</th><th>Instruction</th><th class="text-end">Steps</th><th class="text-end">Dur</th></tr>
      </thead>
      <tbody>${rows}</tbody>
    </table>`;
  }

  async function vlaPost(path, body) {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: body ? JSON.stringify(body) : '{}',
    });
    return r.json();
  }

  function initVLA() {
    // Tab trigger button fires Bootstrap tab events; fall back to the legacy
    // modal element if the old markup is still in place.
    const trigger = document.getElementById('tab-vla-btn') || document.getElementById('vlaModal');
    if (!trigger) return;

    const showEvt = trigger.id === 'vlaModal' ? 'shown.bs.modal' : 'shown.bs.tab';
    const hideEvt = trigger.id === 'vlaModal' ? 'hidden.bs.modal' : 'hidden.bs.tab';

    let poll = null;
    trigger.addEventListener(showEvt, () => {
      vlaRefreshStatus();
      vlaRefreshEpisodes();
      if (poll) clearInterval(poll);
      poll = setInterval(vlaRefreshStatus, 1000);
    });
    trigger.addEventListener(hideEvt, () => {
      if (poll) { clearInterval(poll); poll = null; }
    });

    const byId = id => document.getElementById(id);
    byId('btn-vla-refresh-episodes')?.addEventListener('click', vlaRefreshEpisodes);

    byId('btn-vla-record-start')?.addEventListener('click', async () => {
      const instr = byId('vla-record-instruction').value.trim();
      if (!instr) { logActivity('VLA: instruction required', 'warn'); return; }
      const res = await vlaPost('/api/vla/record/start', { instruction: instr });
      if (res.success) logActivity(`VLA: recording → ${res.episode_dir}`, 'info');
      else logActivity(`VLA: record start failed — ${res.error}`, 'error');
      vlaRefreshStatus();
    });
    byId('btn-vla-record-stop')?.addEventListener('click', async () => {
      const res = await vlaPost('/api/vla/record/stop');
      if (res.success) logActivity(`VLA: sealed ${res.episode_dir || '(none)'}`, 'info');
      else logActivity(`VLA: stop failed — ${res.error}`, 'error');
      vlaRefreshStatus();
      vlaRefreshEpisodes();
    });

    byId('btn-vla-execute-start')?.addEventListener('click', async () => {
      const instr = byId('vla-execute-instruction').value.trim();
      if (!instr) { logActivity('VLA: instruction required', 'warn'); return; }
      const res = await vlaPost('/api/vla/execute/start', { instruction: instr });
      if (res.success) logActivity(`VLA: executing ${instr}`, 'info');
      else logActivity(`VLA: execute start failed — ${res.error}`, 'error');
      vlaRefreshStatus();
    });
    byId('btn-vla-execute-stop')?.addEventListener('click', async () => {
      const res = await vlaPost('/api/vla/execute/stop');
      logActivity(`VLA: stopped executor`, res.success ? 'info' : 'error');
      vlaRefreshStatus();
    });
  }

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

    // Tuning sliders — live-update their value display and push config.
    const tunePush = debounce(async () => {
      const cfg = {
        servo_rate_hz: parseFloat(byId('srv-rate').value),
        t: parseFloat(byId('srv-t').value),
        gain: parseFloat(byId('srv-gain').value),
        aheadtime: parseFloat(byId('srv-ahead').value),
      };
      try { await servoPost('/api/servo/config', cfg); } catch (e) {}
    }, 150);

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
        tunePush();
      });
    }

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

  function debounce(fn, ms) {
    let h;
    return (...args) => {
      clearTimeout(h);
      h = setTimeout(() => fn.apply(null, args), ms);
    };
  }

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
      // Freshen the camera frame on every tab show.
      document.getElementById('btn-cal-refresh-feed')?.click();
      calRefreshPhaseIndicator();
      calStartObjectPolling();
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

  // ── Vision tab ─────────────────────────────────────────────────

  const _visState = { objects: [], target: null, plan: null, pollTimer: null };

  function initVisionTab() {
    const btn = document.getElementById('tab-vision-btn');
    if (!btn) return;

    btn.addEventListener('shown.bs.tab', () => {
      visRefreshFeed();
      visRefreshStatus();
      visStartPolling();
    });
    btn.addEventListener('hidden.bs.tab', visStopPolling);

    document.getElementById('btn-vis-refresh')?.addEventListener('click', () => {
      visRefreshFeed(); visFetchObjects();
    });
    document.getElementById('vis-show-objects')?.addEventListener('change', visDrawOverlay);
    document.getElementById('vis-show-grid')?.addEventListener('change', visDrawOverlay);
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

  function visDrawOverlay() {
    const img = document.getElementById('vis-camera-feed');
    const canvas = document.getElementById('vis-overlay');
    if (!img || !canvas || img.style.display === 'none') return;
    canvas.width = img.naturalWidth || 640;
    canvas.height = img.naturalHeight || 360;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);

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

  async function visPlanTarget(target) {
    _visState.target = target;
    const body = target.kind === 'object'
      ? { object_id: target.object_id }
      : { px: target.px, py: target.py };
    body.approach_mm = parseFloat(document.getElementById('vis-approach-mm')?.value || 100);
    body.grasp_clearance_mm = parseFloat(document.getElementById('vis-grasp-mm')?.value || 5);
    body.lift_mm = parseFloat(document.getElementById('vis-lift-mm')?.value || 150);

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
    const fmtP = p => p ? `(${p[0]}, ${p[1]}, ${p[2]})` : '--';
    list.innerHTML = [
      `<li>Open gripper to 800</li>`,
      `<li>Move to approach: ${fmtP(wp.approach)}</li>`,
      `<li>Descend to grasp: ${fmtP(wp.grasp)}</li>`,
      `<li>Close gripper to 200</li>`,
      `<li>Retract to: ${fmtP(wp.retract)}</li>`,
    ].join('');
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
    body.approach_mm = parseFloat(document.getElementById('vis-approach-mm')?.value || 100);
    body.grasp_clearance_mm = parseFloat(document.getElementById('vis-grasp-mm')?.value || 5);
    body.lift_mm = parseFloat(document.getElementById('vis-lift-mm')?.value || 150);

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

  // ── Tooltips ──────────────────────────────────────────────────
  // Bootstrap tooltips for every element that carries `data-bs-toggle=
  // "tooltip"` or a `title=` inside a content tab. Initializes once on load
  // and re-scans after each tab switch so dynamically-shown controls get wired.

  function initTooltips() {
    // eslint-disable-next-line no-undef
    if (typeof bootstrap === 'undefined' || !bootstrap.Tooltip) return;

    const wire = () => {
      document.querySelectorAll('[data-bs-toggle="tooltip"]').forEach(el => {
        // eslint-disable-next-line no-undef
        bootstrap.Tooltip.getOrCreateInstance(el, {
          container: 'body',
          html: el.dataset.bsHtml === 'true',
          placement: el.dataset.bsPlacement || 'auto',
          delay: { show: 400, hide: 100 },
        });
      });
    };

    wire();
    // Some tooltip targets live inside tab panes that aren't attached to the
    // layout tree until shown; re-scan after each tab change.
    document.querySelectorAll('[data-bs-toggle="tab"]').forEach(btn => {
      btn.addEventListener('shown.bs.tab', () => wire());
    });
  }

  // ── Pendant modal ─────────────────────────────────────────────
  // Lazy-loads the /pendant page into an iframe on first open so the main
  // dashboard doesn't pay the pendant's startup cost up front. Unloads the
  // iframe on close so any polling/timers stop when the modal is hidden.

  function initPendantModal() {
    const modal = document.getElementById('pendantModal');
    const frame = document.getElementById('pendantFrame');
    if (!modal || !frame) return;

    modal.addEventListener('show.bs.modal', () => {
      // Append a cache-buster so edits to pendant.html show up without a
      // manual hard-reload.
      frame.src = '/pendant?t=' + Date.now();
    });
    modal.addEventListener('hidden.bs.modal', () => {
      // Drop the iframe source so any intervals / WebSockets inside the
      // pendant page are torn down.
      frame.src = 'about:blank';
    });
  }

  // ── Hash-based tab navigation ─────────────────────────────────
  // URL fragments persist the active tab across refreshes and support
  // browser back/forward. Short hash forms map to existing tab button IDs.

  const HASH_TO_TAB = {
    'dashboard':   'tab-dashboard-btn',
    'calibration': 'tab-calibration-btn',
    'vla':         'tab-vla-btn',
    'vision':      'tab-vision-btn',
    'servo':       'tab-servo-btn',
    'settings':    'tab-settings-btn',
  };
  const TAB_TO_HASH = Object.fromEntries(
    Object.entries(HASH_TO_TAB).map(([h, id]) => [id, h])
  );

  function hashToTabId(hash) {
    const key = (hash || '').replace(/^#/, '').toLowerCase();
    return HASH_TO_TAB[key] || null;
  }

  function activateTabFromHash() {
    const btnId = hashToTabId(window.location.hash);
    if (!btnId) return;
    const btn = document.getElementById(btnId);
    if (!btn) return;
    // Only show if not already active — avoids unnecessary shown/hidden events.
    if (!btn.classList.contains('active')) {
      // eslint-disable-next-line no-undef
      bootstrap.Tab.getOrCreateInstance(btn).show();
    }
  }

  function initHashNav() {
    // Update hash when a tab is shown.
    document.querySelectorAll('[data-bs-toggle="tab"]').forEach(btn => {
      btn.addEventListener('shown.bs.tab', (ev) => {
        const hash = TAB_TO_HASH[ev.target.id];
        if (!hash) return;
        const desired = '#' + hash;
        if (window.location.hash !== desired) {
          // Use replaceState so tab switches don't bloat browser history.
          history.replaceState(null, '', window.location.pathname + desired);
        }
      });
    });

    // Respond to browser navigation and external hash changes.
    window.addEventListener('hashchange', activateTabFromHash);

    // Activate the tab named in the URL on initial load (deferred until the
    // DOM is in place and Bootstrap is available).
    activateTabFromHash();
  }

  // ── Settings tab ───────────────────────────────────────────────

  async function settingsRefreshList() {
    const el = document.getElementById('settings-saved-list');
    if (!el) return;
    try {
      const r = await fetch('/api/settings/saved');
      const { saved } = await r.json();
      if (!saved || !saved.length) {
        el.innerHTML = '<em class="text-muted">No saved settings yet.</em>';
        return;
      }
      const rows = saved.map(s => {
        const upd = s.updated_at ? s.updated_at.replace('T', ' ').replace('Z', '') : '';
        const desc = (s.description || '').replace(/</g, '&lt;');
        return `<tr data-name="${s.name}">
          <td><code>${s.name}</code></td>
          <td class="text-muted small">${desc}</td>
          <td class="text-muted small">${upd}</td>
          <td class="text-end">
            <div class="btn-group btn-group-sm">
              <button class="btn btn-outline-success btn-xs" data-act="load">Load</button>
              <button class="btn btn-outline-info btn-xs" data-act="rename">Rename</button>
              <button class="btn btn-outline-info btn-xs" data-act="desc">Edit desc</button>
              <a class="btn btn-outline-secondary btn-xs" href="/api/settings/saved/${encodeURIComponent(s.name)}/download" download="${s.name}.json">Download</a>
              <button class="btn btn-outline-danger btn-xs" data-act="delete">Delete</button>
            </div>
          </td>
        </tr>`;
      }).join('');
      el.innerHTML = `<table class="table table-sm table-borderless mb-0">
        <thead class="text-muted small">
          <tr><th>Name</th><th>Description</th><th>Updated</th><th class="text-end">Actions</th></tr>
        </thead>
        <tbody>${rows}</tbody>
      </table>`;
    } catch (e) {
      el.innerHTML = `<span class="text-danger">Failed to load: ${e}</span>`;
    }
  }

  async function settingsRefreshCurrent() {
    const el = document.getElementById('settings-current-preview');
    if (!el) return;
    try {
      const r = await fetch('/api/settings/current');
      const { settings } = await r.json();
      el.textContent = JSON.stringify(settings, null, 2);
    } catch (e) {
      el.textContent = `// failed to load: ${e}`;
    }
  }

  async function settingsSaveCurrent() {
    const name = document.getElementById('settings-save-name').value.trim();
    const desc = document.getElementById('settings-save-desc').value.trim();
    if (!name) { showToast('Name is required', 'warning'); return; }
    const r = await fetch('/api/settings/saved', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, description: desc }),
    });
    const res = await r.json();
    if (res.success) {
      logActivity(`Settings saved: ${name}`, 'success');
      showToast('Settings saved', 'success');
      document.getElementById('settings-save-name').value = '';
      document.getElementById('settings-save-desc').value = '';
      settingsRefreshList();
    } else {
      showToast(`Save failed: ${res.error || r.status}`, 'danger');
    }
  }

  async function settingsLoadByName(name) {
    if (!confirm(`Load "${name}" into current settings? This will replace all current settings.`)) return;
    const r = await fetch(`/api/settings/saved/${encodeURIComponent(name)}/load`, { method: 'POST' });
    const res = await r.json();
    if (r.status === 409 && res.reasons) {
      const msg = 'Blocked by safety check:\n\n' + res.reasons.map(x => ' • ' + x).join('\n');
      alert(msg);
      logActivity(`Load blocked: ${res.reasons.join('; ')}`, 'warn');
      return;
    }
    if (res.success) {
      logActivity(`Loaded settings: ${name}`, 'success');
      showToast('Settings loaded', 'success');
      settingsRefreshCurrent();
    } else {
      showToast(`Load failed: ${res.error || r.status}`, 'danger');
    }
  }

  async function settingsRenameByName(name) {
    const newName = prompt(`Rename "${name}" to:`, name);
    if (newName == null || newName === name || !newName.trim()) return;
    const r = await fetch(`/api/settings/saved/${encodeURIComponent(name)}`, {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ new_name: newName.trim() }),
    });
    if (r.ok) { settingsRefreshList(); logActivity(`Renamed: ${name} → ${newName}`, 'info'); }
    else { const res = await r.json().catch(() => ({})); showToast(`Rename failed: ${res.detail || r.status}`, 'danger'); }
  }

  async function settingsEditDescByName(name) {
    const cur = await fetch(`/api/settings/saved/${encodeURIComponent(name)}`).then(r => r.json()).catch(() => ({}));
    const desc = prompt(`Description for "${name}":`, cur.description || '');
    if (desc == null) return;
    const r = await fetch(`/api/settings/saved/${encodeURIComponent(name)}`, {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ description: desc }),
    });
    if (r.ok) { settingsRefreshList(); }
    else { showToast(`Update failed: ${r.status}`, 'danger'); }
  }

  async function settingsDeleteByName(name) {
    if (!confirm(`Delete "${name}"? This cannot be undone.`)) return;
    const r = await fetch(`/api/settings/saved/${encodeURIComponent(name)}`, { method: 'DELETE' });
    if (r.ok) { settingsRefreshList(); logActivity(`Deleted: ${name}`, 'info'); }
    else { showToast(`Delete failed: ${r.status}`, 'danger'); }
  }

  async function settingsImport() {
    const input = document.getElementById('settings-import-file');
    const f = input?.files?.[0];
    if (!f) { showToast('Choose a JSON file first', 'warning'); return; }
    let bundle;
    try {
      const text = await f.text();
      bundle = JSON.parse(text);
    } catch (e) {
      showToast(`Invalid JSON: ${e}`, 'danger'); return;
    }
    const r = await fetch('/api/settings/saved/import', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(bundle),
    });
    const res = await r.json();
    if (res.success) {
      logActivity(`Imported: ${res.saved?.name || f.name}`, 'success');
      showToast('Import complete', 'success');
      input.value = '';
      settingsRefreshList();
    } else {
      showToast(`Import failed: ${res.error || r.status}`, 'danger');
    }
  }

  function initSettings() {
    const trigger = document.getElementById('tab-settings-btn');
    if (!trigger) return;
    trigger.addEventListener('shown.bs.tab', () => {
      settingsRefreshList();
      settingsRefreshCurrent();
    });

    document.getElementById('btn-settings-save')?.addEventListener('click', settingsSaveCurrent);
    document.getElementById('btn-settings-refresh')?.addEventListener('click', settingsRefreshList);
    document.getElementById('btn-settings-refresh-current')?.addEventListener('click', settingsRefreshCurrent);
    document.getElementById('btn-settings-import')?.addEventListener('click', settingsImport);

    // Event-delegation on the saved list for action buttons.
    document.getElementById('settings-saved-list')?.addEventListener('click', async (ev) => {
      const btn = ev.target.closest('button[data-act]');
      if (!btn) return;
      const tr = btn.closest('tr[data-name]');
      const name = tr?.dataset?.name;
      if (!name) return;
      const act = btn.dataset.act;
      if (act === 'load') await settingsLoadByName(name);
      else if (act === 'rename') await settingsRenameByName(name);
      else if (act === 'desc') await settingsEditDescByName(name);
      else if (act === 'delete') await settingsDeleteByName(name);
    });
  }

  // Boot
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

})();
