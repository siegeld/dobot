'use strict';

// ── Dobot CR5 — VLA (Vision-Language-Action) tab ─────────────
// Extracted from app.js. Loaded as a classic script AFTER app.js.

(function () {
  const ui = () => (window.DobotUI || {});
  const logActivity = (msg, type) => ui().logActivity?.(msg, type);
  const showToast   = (msg, type) => ui().showToast?.(msg, type);

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


  window.DobotUI = window.DobotUI || {};
  window.DobotUI.initVLA = initVLA;
})();
