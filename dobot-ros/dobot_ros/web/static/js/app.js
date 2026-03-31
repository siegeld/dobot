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
        'ENABLE': 'bg-success', 'RUNNING': 'bg-success', 'JOG': 'bg-success',
        'DISABLED': 'bg-secondary', 'INIT': 'bg-info',
        'ERROR': 'bg-danger', 'PAUSE': 'bg-warning',
        'TEACH': 'bg-info', 'BACKDRIVE': 'bg-info',
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

    // Drive 3D visualization
    if (window.updateRobotJoints && data.joint) {
      window.updateRobotJoints(data.joint);
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
      speed: parseInt($('#gripper-speed').value),
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
    $('#gripper-speed').addEventListener('input', (e) => {
      $('#gripper-speed-label').textContent = e.target.value;
    });

    // Reset 3D camera
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

    // Keyboard
    document.addEventListener('keydown', handleKeyboard);

    // Load config
    loadConfig();

    // Start WebSocket
    connectWebSocket();

    // Poll status every 5s for uptime etc
    setInterval(pollStatus, 5000);

    logActivity('Dashboard initialized', 'info');
  }

  // Boot
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

})();
