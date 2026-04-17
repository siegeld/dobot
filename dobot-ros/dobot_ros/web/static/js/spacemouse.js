(function () {
  'use strict';

  const SETTING_KEYS = [
    'max_velocity_xyz', 'max_velocity_rpy', 'deadband',
    'max_excursion_xyz', 'max_excursion_rpy', 'idle_auto_disarm_s',
    'button_debounce_ms', 'gripper_force',
  ];

  const root = document.getElementById('sm-root');
  const devicePill = document.getElementById('sm-device-pill');
  const deviceStatus = document.getElementById('sm-device-status');
  const batteryWrap = document.getElementById('sm-battery');
  const batteryPct = document.getElementById('sm-battery-pct');
  const armedPill = document.getElementById('sm-armed-pill');
  const banner = document.getElementById('sm-banner');
  const armBtn = document.getElementById('sm-arm-btn');
  const disarmBtn = document.getElementById('sm-disarm-btn');
  const estopBtn = document.getElementById('sm-estop-btn');
  const anchorEl = document.getElementById('sm-anchor');
  const eventAgeEl = document.getElementById('sm-event-age');
  const buttonNote = document.getElementById('sm-button-note');
  const signMapEl = document.getElementById('sm-sign-map');
  const settingsForm = document.getElementById('sm-settings-form');

  let currentSignMap = [1, 1, 1, 1, 1, 1];

  // ── WebSocket ─────────────────────────────────────────────────
  function connectWs() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws/spacemouse`);
    ws.onmessage = (ev) => {
      try { render(JSON.parse(ev.data)); } catch (e) { console.error(e); }
    };
    ws.onclose = () => setTimeout(connectWs, 1000);
    ws.onerror = () => {};
  }

  function render(s) {
    const status = s.device_status || 'disconnected';
    deviceStatus.textContent = status.replace('_', ' ');
    devicePill.classList.remove('bg-secondary', 'bg-success', 'bg-warning', 'bg-danger');
    if (status === 'connected') {
      devicePill.classList.add('bg-success');
    } else if (status === 'lost') {
      devicePill.classList.add('bg-warning');
    } else if (status === 'permission_denied') {
      devicePill.classList.add('bg-danger');
    } else {
      devicePill.classList.add('bg-secondary');
    }

    if (typeof s.battery_pct === 'number') {
      batteryWrap.style.display = '';
      batteryPct.textContent = s.battery_pct;
    } else {
      batteryWrap.style.display = 'none';
    }

    if (s.armed) {
      armedPill.style.display = '';
      armBtn.style.display = 'none';
      disarmBtn.style.display = '';
      buttonNote.style.display = 'none';
    } else {
      armedPill.style.display = 'none';
      armBtn.style.display = '';
      disarmBtn.style.display = 'none';
      buttonNote.style.display = '';
    }
    armBtn.disabled = (status !== 'connected');

    for (let i = 0; i < 6; i++) {
      const row = root.querySelector(`.sm-axis[data-axis="${i}"]`);
      if (!row) continue;
      const v = (s.axes && s.axes[i]) || 0;
      const fill = row.querySelector('.sm-bar-fill');
      const val = row.querySelector('.sm-val');
      if (val) val.textContent = v.toFixed(2);
      const pct = Math.min(100, Math.abs(v) * 100);
      if (v >= 0) {
        fill.style.left = '50%';
        fill.style.width = (pct / 2) + '%';
      } else {
        fill.style.left = (50 - pct / 2) + '%';
        fill.style.width = (pct / 2) + '%';
      }
      row.setAttribute('data-deadband', Math.abs(v) < 0.01 ? 'true' : 'false');
    }

    for (let b = 0; b < 2; b++) {
      const el = root.querySelector(`.sm-btn-indicator[data-btn="${b}"]`);
      if (!el) continue;
      el.setAttribute('data-active', (s.buttons && s.buttons[b]) ? 'true' : 'false');
    }

    if (s.anchor && s.anchor.length === 6) {
      const [x, y, z, rx, ry, rz] = s.anchor;
      anchorEl.innerHTML =
        `X ${x.toFixed(1)} &nbsp; Y ${y.toFixed(1)} &nbsp; Z ${z.toFixed(1)}<br>` +
        `RX ${rx.toFixed(1)} &nbsp; RY ${ry.toFixed(1)} &nbsp; RZ ${rz.toFixed(1)}`;
    } else {
      anchorEl.innerHTML = '<em class="text-muted">not armed</em>';
    }
    for (let i = 0; i < 6; i++) {
      const el = root.querySelector(`[data-offset="${i}"]`);
      if (el) el.textContent = ((s.offset && s.offset[i]) || 0).toFixed(1);
    }

    eventAgeEl.textContent = ((s.last_event_age_ms || 0).toFixed(0)) + ' ms';

    if (s.error && status !== 'connected') {
      banner.style.display = '';
      banner.className = 'alert alert-warning';
      banner.textContent = s.error;
    } else {
      banner.style.display = 'none';
    }
  }

  // ── Buttons ───────────────────────────────────────────────────
  async function post(path, body) {
    const res = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: body ? JSON.stringify(body) : '{}',
    });
    return res.json();
  }

  armBtn.addEventListener('click', async () => {
    const r = await post('/api/spacemouse/arm');
    if (!r.success) showToast(r.error || 'arm failed');
  });
  disarmBtn.addEventListener('click', () => post('/api/spacemouse/disarm'));
  estopBtn.addEventListener('click', () => post('/api/spacemouse/estop'));
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') post('/api/spacemouse/estop');
  });

  function showToast(msg) {
    banner.style.display = '';
    banner.className = 'alert alert-danger';
    banner.textContent = msg;
    setTimeout(() => { if (banner.textContent === msg) banner.style.display = 'none'; }, 4000);
  }

  // ── Settings ──────────────────────────────────────────────────
  async function loadSettings() {
    const r = await fetch('/api/spacemouse/settings').then(r => r.json());
    if (!r.success) return;
    for (const k of SETTING_KEYS) {
      const el = settingsForm.querySelector(`[name="${k}"]`);
      if (el && r.settings[k] !== undefined) el.value = r.settings[k];
    }
    currentSignMap = (r.settings.sign_map && r.settings.sign_map.slice()) || [1, 1, 1, 1, 1, 1];
    renderSignMap();
  }

  function renderSignMap() {
    const labels = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ'];
    signMapEl.innerHTML = '';
    for (let i = 0; i < 6; i++) {
      const wrap = document.createElement('span');
      wrap.className = 'sm-sign-toggle';
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'btn btn-sm ' + (currentSignMap[i] > 0 ? 'btn-outline-success' : 'btn-outline-danger');
      btn.textContent = (currentSignMap[i] > 0 ? '+' : '−') + labels[i];
      btn.addEventListener('click', () => {
        currentSignMap[i] *= -1;
        renderSignMap();
      });
      wrap.appendChild(btn);
      signMapEl.appendChild(wrap);
    }
  }

  settingsForm.addEventListener('submit', async (ev) => {
    ev.preventDefault();
    const body = {};
    for (const k of SETTING_KEYS) {
      const v = settingsForm.querySelector(`[name="${k}"]`).value;
      if (v !== '') body[k] = Number(v);
    }
    body.sign_map = currentSignMap.slice();
    const r = await post('/api/spacemouse/settings', body);
    if (!r.success) showToast(r.error || 'save failed');
  });

  // ── Boot ──────────────────────────────────────────────────────
  loadSettings();
  connectWs();
})();
