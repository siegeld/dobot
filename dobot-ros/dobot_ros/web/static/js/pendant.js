'use strict';

// ── Dobot CR5 Teach Pendant ───────────────────────────────────

(function () {

  const state = {
    connected: false,
    linearStep: 5,
    angularStep: 5,
    jogSpeed: 50,
    jogMode: 'user',
    jogTab: 'cartesian',
    ws: null,
    wsRetryDelay: 1000,
  };

  const $ = (sel) => document.querySelector(sel);
  const $$ = (sel) => document.querySelectorAll(sel);

  // ── API ────────────────────────────────────────────────────

  async function api(endpoint, method = 'GET', body = null) {
    const opts = { method, headers: {} };
    if (body) {
      opts.headers['Content-Type'] = 'application/json';
      opts.body = JSON.stringify(body);
    }
    const res = await fetch(`/api/${endpoint}`, opts);
    return res.json();
  }

  // ── Toast ──────────────────────────────────────────────────

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

  // ── WebSocket ──────────────────────────────────────────────

  function connectWebSocket() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    state.ws = new WebSocket(`${protocol}//${location.host}/ws/state`);

    state.ws.onopen = () => {
      state.wsRetryDelay = 1000;
    };

    state.ws.onmessage = (evt) => {
      try {
        updateDisplay(JSON.parse(evt.data));
      } catch (e) { /* ignore */ }
    };

    state.ws.onclose = () => {
      updateConnectionBadge(false);
      setTimeout(connectWebSocket, Math.min(state.wsRetryDelay, 10000));
      state.wsRetryDelay *= 1.5;
    };

    state.ws.onerror = () => { state.ws.close(); };
  }

  // ── Display ────────────────────────────────────────────────

  const MODE_COLORS = {
    'ENABLE': 'bg-success', 'RUNNING': 'bg-success', 'JOG': 'bg-success',
    'DISABLED': 'bg-secondary', 'INIT': 'bg-info',
    'ERROR': 'bg-danger', 'PAUSE': 'bg-warning',
    'TEACH': 'bg-info', 'BACKDRIVE': 'bg-info',
  };

  const GRIP_STATES = { 0: 'Moving', 1: 'Reached', 2: 'Caught', 3: 'Dropped' };

  function updateConnectionBadge(connected, modeName) {
    const badge = $('#badge-connection');
    const text = $('#conn-text');
    if (connected) {
      badge.className = 'status-badge badge-connected';
      text.textContent = 'Connected';
    } else {
      badge.className = 'status-badge badge-disconnected';
      text.textContent = 'Disconnected';
    }
    if (modeName) {
      const mb = $('#badge-mode');
      mb.textContent = modeName;
      mb.className = `badge ${MODE_COLORS[modeName] || 'bg-secondary'}`;
    }
  }

  function updateDisplay(data) {
    state.connected = data.connected;
    updateConnectionBadge(data.connected, data.robot_mode_name);

    // Joint angles
    const jIds = ['tp-j1', 'tp-j2', 'tp-j3', 'tp-j4', 'tp-j5', 'tp-j6'];
    data.joint.forEach((v, i) => {
      const el = document.getElementById(jIds[i]);
      if (el) el.textContent = v.toFixed(2);
    });

    // Cartesian
    const cIds = ['tp-px', 'tp-py', 'tp-pz', 'tp-prx', 'tp-pry', 'tp-prz'];
    data.cartesian.forEach((v, i) => {
      const el = document.getElementById(cIds[i]);
      if (el) el.textContent = v.toFixed(2);
    });

    // Speed
    $('#speed-display').textContent = data.speed_factor || '--';

    // Error
    const errDisp = $('#error-display');
    if (data.error) {
      errDisp.style.display = '';
      $('#error-text').textContent = data.error;
    } else {
      errDisp.style.display = 'none';
    }

    // Gripper
    if (data.gripper_position >= 0) {
      $('#tp-gripper-pos').textContent = data.gripper_position;
      $('#tp-gripper-bar').style.width = (data.gripper_position / 10) + '%';
    }
    $('#tp-gripper-state').textContent = GRIP_STATES[data.gripper_state] || '--';

    const gBadge = $('#tp-gripper-badge');
    if (data.gripper_initialized) {
      gBadge.textContent = 'Ready';
      gBadge.className = 'badge bg-success ms-1';
    } else {
      gBadge.textContent = 'Not Init';
      gBadge.className = 'badge bg-secondary ms-1';
    }
  }

  // ── Commands ───────────────────────────────────────────────

  async function robotCommand(name, endpoint, method = 'POST', body = null) {
    try {
      const result = await api(endpoint, method, body);
      if (result.success) {
        showToast(name + ' OK', 'success');
      } else {
        showToast(result.error || name + ' failed', 'danger');
      }
    } catch (e) {
      showToast(e.message, 'danger');
    }
  }

  async function doJog(axis, direction) {
    const isJoint = axis.startsWith('j');
    const isAngular = isJoint || ['rx', 'ry', 'rz'].includes(axis);
    const step = isAngular ? state.angularStep : state.linearStep;
    const distance = step * direction;

    try {
      const result = await api('jog', 'POST', {
        axis, distance, speed: state.jogSpeed, mode: state.jogMode,
      });
      if (!result.success) {
        showToast(result.error || 'Jog failed', 'danger');
      }
    } catch (e) {
      showToast(e.message, 'danger');
    }
  }

  // ── Init ───────────────────────────────────────────────────

  function init() {
    connectWebSocket();

    // Jog buttons
    $$('.tp-jog-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        doJog(btn.dataset.axis, parseInt(btn.dataset.dir));
      });
    });

    // Tab switching
    $$('.tp-tab').forEach((tab) => {
      tab.addEventListener('click', () => {
        $$('.tp-tab').forEach(t => t.classList.remove('active'));
        tab.classList.add('active');
        state.jogTab = tab.dataset.tab;
        $('#jog-cartesian').style.display = state.jogTab === 'cartesian' ? '' : 'none';
        $('#jog-joint').style.display = state.jogTab === 'joint' ? '' : 'none';
      });
    });

    // Linear step buttons
    $$('#linear-steps .tp-step-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        $$('#linear-steps .tp-step-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        state.linearStep = parseInt(btn.dataset.step);
      });
    });

    // Angular step buttons
    $$('#angular-steps .tp-step-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        $$('#angular-steps .tp-step-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        state.angularStep = parseInt(btn.dataset.step);
      });
    });

    // Robot controls
    $('#btn-tp-enable').addEventListener('click', () => robotCommand('Enable', 'enable'));
    $('#btn-tp-disable').addEventListener('click', () => robotCommand('Disable', 'disable'));
    $('#btn-tp-stop').addEventListener('click', () => robotCommand('STOP', 'stop'));
    $('#btn-tp-clear').addEventListener('click', () => robotCommand('Clear Error', 'clear'));

    // Robot speed slider
    const speedSlider = $('#tp-speed-slider');
    const speedVal = $('#tp-speed-val');
    speedSlider.addEventListener('input', () => {
      speedVal.textContent = speedSlider.value + '%';
    });
    speedSlider.addEventListener('change', () => {
      robotCommand('Speed ' + speedSlider.value + '%', 'speed', 'POST', { speed: parseInt(speedSlider.value) });
    });

    // Jog speed slider
    const jogSpeedSlider = $('#tp-jog-speed');
    const jogSpeedVal = $('#tp-jog-speed-val');
    jogSpeedSlider.addEventListener('input', () => {
      state.jogSpeed = parseInt(jogSpeedSlider.value);
      jogSpeedVal.textContent = jogSpeedSlider.value + '%';
    });

    // Coordinate mode
    $$('.tp-mode-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        $$('.tp-mode-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        state.jogMode = btn.dataset.mode;
      });
    });

    // Gripper
    $('#btn-tp-gripper-init').addEventListener('click', () => robotCommand('Gripper Init', 'gripper/init'));
    $('#btn-tp-gripper-open').addEventListener('click', () => robotCommand('Gripper Open', 'gripper/open?speed=50&force=50'));
    $('#btn-tp-gripper-close').addEventListener('click', () => robotCommand('Gripper Close', 'gripper/close?speed=50&force=50'));
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

})();
