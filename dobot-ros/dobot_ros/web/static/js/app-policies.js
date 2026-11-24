'use strict';

// ── Dobot CR5 — Policies tab ─────────────────────────────────────
// Hand-coded policies run on the kinematic simulator or the robot.
// Backend: web/policies_api.py. 3D: policy3d.js. See POLICIES.md.

(function () {
  const ui = () => (window.DobotUI || {});
  const logActivity = (msg, type) => ui().logActivity?.(msg, type);
  const showToast = (msg, type) => ui().showToast?.(msg, type);
  const $ = (id) => document.getElementById(id);
  const p3 = () => window.policy3d;

  const S = {
    policies: [], slug: null, scenes: [], scene: null, sceneObjects: [], table: null,
    runId: null, runTarget: null, running: false, liveEvents: [], sourceLines: [],
    traces: [], currentTrace: null, batch: null,
  };

  async function getJSON(url) { const r = await fetch(url); return r.json(); }
  async function postJSON(url, body) {
    const r = await fetch(url, { method: body === undefined ? 'POST' : 'POST',
      headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body ?? {}) });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(j.error || j.detail || r.statusText);
    return j;
  }

  // ── Policies + params ────────────────────────────────────────
  async function loadPolicies() {
    const d = await getJSON('/api/policies/list');
    S.policies = d.policies || [];
    const sel = $('pol-select');
    sel.innerHTML = '';
    S.policies.forEach(m => {
      const o = document.createElement('option');
      o.value = m.slug; o.textContent = m.name; sel.appendChild(o);
    });
    if (!S.slug || !S.policies.some(m => m.slug === S.slug)) S.slug = S.policies[0]?.slug || null;
    sel.value = S.slug || '';
    renderPolicy();
    renderStatus(d.runner);
  }

  function meta() { return S.policies.find(m => m.slug === S.slug); }

  function renderPolicy() {
    const m = meta();
    $('pol-desc').textContent = m ? m.description : '';
    const form = $('pol-params');
    form.innerHTML = '';
    if (!m) return;
    m.parameters.forEach(pd => {
      const cur = m.current_params[pd.name] ?? pd.default;
      const row = document.createElement('div');
      row.className = 'd-flex align-items-center gap-2 mb-1';
      const lbl = document.createElement('label');
      lbl.className = 'small text-muted flex-grow-1';
      lbl.textContent = pd.description + (pd.unit ? ` (${pd.unit})` : '');
      lbl.title = pd.name;
      let inp;
      if (pd.type === 'choice') {
        inp = document.createElement('select');
        inp.className = 'form-select form-select-sm w-auto';
        (pd.choices || []).forEach(c => { const o = document.createElement('option'); o.value = c; o.textContent = c; inp.appendChild(o); });
        inp.value = cur;
      } else if (pd.type === 'bool') {
        inp = document.createElement('input'); inp.type = 'checkbox'; inp.className = 'form-check-input';
        inp.checked = !!cur;
      } else {
        inp = document.createElement('input'); inp.type = 'number';
        inp.className = 'form-control form-control-sm'; inp.style.width = '6.5em';
        if (pd.min != null) inp.min = pd.min; if (pd.max != null) inp.max = pd.max;
        inp.step = pd.step ?? (pd.type === 'int' ? 1 : 'any');
        inp.value = cur;
      }
      inp.dataset.param = pd.name; inp.dataset.type = pd.type;
      row.appendChild(lbl); row.appendChild(inp); form.appendChild(row);
    });
    loadSource();
  }

  function collectParams() {
    const out = {};
    $('pol-params').querySelectorAll('[data-param]').forEach(el => {
      const t = el.dataset.type;
      out[el.dataset.param] = t === 'bool' ? el.checked : (t === 'choice' ? el.value : Number(el.value));
    });
    return out;
  }

  async function saveParams() {
    if (!S.slug) return;
    try {
      await postJSON(`/api/policies/${S.slug}/params`, collectParams());
      showToast('Policy parameters saved', 'success');
      await loadPolicies();
    } catch (e) { showToast(`Save failed: ${e.message}`, 'danger'); }
  }

  async function loadSource() {
    if (!S.slug) return;
    try {
      const d = await getJSON(`/api/policies/${S.slug}/source`);
      $('pol-source-path').textContent = d.path;
      const pre = $('pol-source');
      pre.innerHTML = '';
      S.sourceLines = d.source.split('\n');
      S.sourceLines.forEach((line, i) => {
        const span = document.createElement('span');
        span.className = 'pol-src-line'; span.dataset.line = i + 1;
        span.textContent = `${String(i + 1).padStart(3)}  ${line}\n`;
        pre.appendChild(span);
      });
    } catch (e) { /* ignore */ }
  }

  function highlightLine(n) {
    const pre = $('pol-source');
    pre.querySelectorAll('.pol-src-line.active').forEach(el => el.classList.remove('active'));
    if (!n) return;
    const el = pre.querySelector(`[data-line="${n}"]`);
    if (el) { el.classList.add('active'); el.scrollIntoView({ block: 'nearest' }); }
  }

  // ── Scenes ───────────────────────────────────────────────────
  async function loadScenes(keep) {
    const d = await getJSON('/api/sim/scenes');
    S.scenes = d.scenes || [];
    const sel = $('pol-scene');
    const prev = keep || sel.value;
    sel.innerHTML = '';
    S.scenes.forEach(sc => { const o = document.createElement('option'); o.value = sc.name; o.textContent = `${sc.name} (${sc.object_count})`; sel.appendChild(o); });
    if (S.scenes.some(sc => sc.name === prev)) sel.value = prev;
    if (d.noise_defaults && !$('pol-noise-xy').value) {
      $('pol-noise-xy').value = d.noise_defaults.xy_sigma_mm;
      $('pol-noise-yaw').value = d.noise_defaults.yaw_sigma_deg;
      $('pol-noise-miss').value = d.noise_defaults.miss_prob;
    }
    await loadScene();
  }

  async function loadScene() {
    const name = $('pol-scene').value;
    if (!name) return;
    const d = await getJSON(`/api/sim/scenes/${encodeURIComponent(name)}`);
    S.scene = d.scene; S.sceneObjects = d.objects; S.table = d.table;
    renderScene();
  }

  function renderScene() {
    p3()?.setTable(S.table);
    p3()?.setObjects(S.sceneObjects);
    const tb = $('pol-scene-objects');
    tb.innerHTML = '';
    S.sceneObjects.forEach((o, i) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `<td><span class="pol-swatch" style="background:${cssColor(o.color)}"></span>${o.color} ${o.label}</td>
        <td><input type="number" class="form-control form-control-sm pol-obj" data-i="${i}" data-k="0" value="${o.position[0].toFixed(0)}"></td>
        <td><input type="number" class="form-control form-control-sm pol-obj" data-i="${i}" data-k="1" value="${o.position[1].toFixed(0)}"></td>
        <td><input type="number" class="form-control form-control-sm pol-obj" data-i="${i}" data-k="yaw" value="${(o.yaw_deg || 0).toFixed(0)}"></td>
        <td class="text-muted small">${o.size.map(v => v.toFixed(0)).join('×')}</td>`;
      tb.appendChild(tr);
    });
    tb.querySelectorAll('.pol-obj').forEach(inp => inp.addEventListener('change', onObjEdit));
  }

  function cssColor(c) {
    return ({ red: '#d9363e', green: '#3fb950', blue: '#388bfd', yellow: '#e3b341', orange: '#f0883e',
      purple: '#a371f7', white: '#ddd', black: '#333', gray: '#888', cyan: '#39c5cf', pink: '#ff7b9c' })[c] || '#999';
  }

  function onObjEdit(e) {
    const i = +e.target.dataset.i, k = e.target.dataset.k, v = Number(e.target.value);
    const o = S.sceneObjects[i];
    if (k === 'yaw') o.yaw_deg = v; else o.position[+k] = v;
    p3()?.setObjects(S.sceneObjects);
  }

  function currentSceneDict() {
    return { name: S.scene?.name || 'custom', objects: S.sceneObjects.map(o => ({
      id: o.id, label: o.label, color: o.color, size: o.size,
      position: [o.position[0], o.position[1]], yaw_deg: o.yaw_deg })) };
  }

  async function randomizeScene() {
    const seed = $('pol-seed').value === '' ? null : Number($('pol-seed').value);
    try {
      const d = await postJSON('/api/sim/scenes/randomize', { scene: currentSceneDict(), seed });
      S.sceneObjects = d.objects; S.table = d.table;
      renderScene();
    } catch (e) { showToast(`Randomize failed: ${e.message}`, 'danger'); }
  }

  async function saveScene() {
    const name = prompt('Save scene as:', S.scene?.name || 'my_scene');
    if (!name) return;
    try {
      const d = await postJSON('/api/sim/scenes', { name, scene: currentSceneDict() });
      showToast(`Scene saved as ${d.name}`, 'success');
      await loadScenes(d.name);
    } catch (e) { showToast(`Save failed: ${e.message}`, 'danger'); }
  }

  function noiseDict() {
    return { enabled: $('pol-noise-on').checked, xy_sigma_mm: Number($('pol-noise-xy').value),
      yaw_sigma_deg: Number($('pol-noise-yaw').value), miss_prob: Number($('pol-noise-miss').value) };
  }

  // ── Run / stop / batch ───────────────────────────────────────
  function target() { return $('pol-target-robot').checked ? 'robot' : 'sim'; }

  async function runPolicy() {
    if (!S.slug) return;
    const tgt = target();
    if (tgt === 'robot' && !confirm(`Run "${meta().name}" on the REAL robot?\n\nThe engine will rehearse it in the simulator against the live scene first, then ask you to confirm each motion.`)) return;
    const body = {
      slug: S.slug, target: tgt, params: collectParams(),
      scene: currentSceneDict(), seed: $('pol-seed').value === '' ? null : Number($('pol-seed').value),
      noise: noiseDict(), pace: Number($('pol-pace').value), confirm_mode: $('pol-confirm-mode').value,
    };
    try {
      clearLog();
      p3()?.replay.pause();
      if (tgt === 'sim') { p3()?.setLive(true); p3()?.setObjects(S.sceneObjects); }
      else { p3()?.setLive(false); p3()?.clear(); }
      const d = await postJSON('/api/policies/run', body);
      S.runId = d.run_id; S.runTarget = tgt; S.running = true; S.liveEvents = [];
      setRunning(true);
      $('pol-run-status').innerHTML = `<span class="text-info">Running</span> ${S.slug} on ${tgt} <span class="text-muted">(${d.run_id})</span>`;
      logActivity(`Policy ${S.slug} started on ${tgt}`, 'info');
    } catch (e) {
      p3()?.setLive(false);
      showToast(`Run failed: ${e.message}`, 'danger');
    }
  }

  async function stopPolicy() {
    try { await postJSON('/api/policies/stop'); logActivity('Policy stop requested', 'warning'); }
    catch (e) { showToast(`Stop failed: ${e.message}`, 'danger'); }
  }

  async function runBatch() {
    if (!S.slug) return;
    const n = Number($('pol-batch-n').value) || 20;
    try {
      const d = await postJSON('/api/policies/batch', {
        slug: S.slug, n, params: collectParams(), scene: currentSceneDict(),
        seed: Number($('pol-seed').value) || 0, noise: noiseDict() });
      S.batch = { id: d.batch_id, n, done: 0, ok: 0 };
      setRunning(true);
      $('pol-batch-result').innerHTML = `<div class="progress" style="height:6px"><div id="pol-batch-bar" class="progress-bar" style="width:0%"></div></div><div class="small text-muted mt-1" id="pol-batch-line">0 / ${n}</div>`;
    } catch (e) { showToast(`Batch failed: ${e.message}`, 'danger'); }
  }

  function setRunning(on) {
    S.running = on;
    $('btn-pol-run').disabled = on; $('btn-pol-batch').disabled = on; $('btn-pol-stop').disabled = !on;
  }

  function renderStatus(st) {
    if (!st) return;
    if (st.running && st.current) {
      setRunning(true);
      $('pol-run-status').innerHTML = `<span class="text-info">Running</span> ${st.current.slug} on ${st.current.target} <span class="text-muted">(${st.current.elapsed_s}s)</span>`;
    }
  }

  // ── Live events ──────────────────────────────────────────────
  function clearLog() { $('pol-log').innerHTML = ''; $('pol-result').innerHTML = ''; highlightLine(null); }

  function fmtArgs(a) {
    if (!a) return '';
    return Object.entries(a).map(([k, v]) => `${k}=${Array.isArray(v) ? '[' + v.map(x => typeof x === 'number' ? x.toFixed(0) : x).join(',') + ']' : (typeof v === 'number' ? +v.toFixed(1) : JSON.stringify(v))}`).join(' ');
  }

  function appendLog(ev) {
    const k = ev.kind;
    if (k === 'sample') return;
    const el = $('pol-log');
    const div = document.createElement('div');
    div.className = `log-entry pol-ev pol-ev-${k}`;
    div.dataset.t = ev.t;
    let txt = '';
    if (k === 'call') txt = `→ ${ev.name} ${fmtArgs(ev.args)}`;
    else if (k === 'return') { if (ev.result == null) return; txt = `← ${ev.name} ${typeof ev.result === 'object' ? JSON.stringify(ev.result) : ev.result}`; }
    else if (k === 'log') txt = ev.message;
    else if (k === 'error') { txt = `✖ ${ev.name}: ${ev.message}`; div.classList.add('log-error'); }
    else if (k === 'result') { renderResult(ev.result); return; }
    div.innerHTML = `<span class="log-time">${(ev.t ?? 0).toFixed(1)}s</span> <span class="log-msg">${escapeHtml(txt)}</span>`;
    el.appendChild(div);
    el.scrollTop = el.scrollHeight;
    if (k === 'call' && ev.line) highlightLine(ev.line);
  }

  function escapeHtml(s) { return String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c])); }

  function renderResult(res) {
    if (!res) return;
    const ok = res.ok;
    $('pol-result').innerHTML = `<span class="badge ${ok ? 'bg-success' : 'bg-danger'}">${ok ? 'SUCCESS' : 'FAILED'}</span>
      <span class="small ms-2">${escapeHtml(res.reason || '')} attempts=${res.attempts} ${res.elapsed_s ? res.elapsed_s.toFixed(1) + 's' : ''}
      ${res.observations && Object.keys(res.observations).length ? escapeHtml(JSON.stringify(res.observations)) : ''}</span>`;
  }

  function onWs(e) {
    const d = e.detail;
    if (!d || !d.type || !d.type.startsWith('policy_')) return;
    if (d.type === 'policy_event') {
      const ev = d.event;
      if (!S.runId || d.run_id === S.runId || !S.running) { S.runId = d.run_id; S.runTarget = d.target; }
      S.liveEvents.push(ev);
      if (ev.kind === 'sample' && d.target === 'sim') { p3()?.setLive(true); p3()?.applySample(ev); }
      appendLog(ev);
    } else if (d.type === 'policy_done') {
      setRunning(false);
      $('pol-run-status').innerHTML = `${d.result?.ok ? '<span class="text-success">Done</span>' : '<span class="text-danger">Failed</span>'} ${d.slug} on ${d.target} <span class="text-muted">(${d.run_id})</span>`;
      renderResult(d.result);
      logActivity(`Policy ${d.slug}: ${d.result?.ok ? 'success' : 'failed — ' + (d.result?.reason || '')}`, d.result?.ok ? 'success' : 'error');
      if (d.target === 'sim') {
        // Keep the final sim pose on screen; the live feed takes over again
        // when the user picks a trace or clicks "Live".
      } else { p3()?.setLive(false); }
      loadTraces();
    } else if (d.type === 'policy_batch') {
      const bar = $('pol-batch-bar'), line = $('pol-batch-line');
      if (bar) bar.style.width = `${(d.done / d.total * 100).toFixed(0)}%`;
      if (line) line.textContent = `${d.done} / ${d.total} — ${d.ok} ok`;
    } else if (d.type === 'policy_batch_done') {
      setRunning(false);
      renderBatch(d.summary, d.runs);
      loadTraces();
    }
  }

  function renderBatch(s, runs) {
    const rate = (s.success_rate * 100).toFixed(0);
    const reasons = Object.entries(s.reasons || {}).map(([r, c]) => `<div class="small"><b>${c}</b> ${escapeHtml(r)}</div>`).join('');
    const fails = (runs || []).filter(r => !r.ok);
    $('pol-batch-result').innerHTML = `
      <div><span class="badge ${s.success_rate >= 0.95 ? 'bg-success' : s.success_rate >= 0.8 ? 'bg-warning text-dark' : 'bg-danger'}">${rate}%</span>
        <span class="small ms-2">${s.ok}/${s.n} ok · mean attempts ${s.mean_attempts} · ${s.wall_s}s</span></div>
      ${reasons}
      ${fails.length ? `<div class="small mt-1">Failing seeds: ${fails.map(r => `<a href="#" class="pol-trace-link" data-id="${r.trace_id}">${r.seed}</a>`).join(', ')}</div>` : ''}`;
    $('pol-batch-result').querySelectorAll('.pol-trace-link').forEach(a => a.addEventListener('click', ev => { ev.preventDefault(); openTrace(a.dataset.id); }));
  }

  // ── Traces + replay ──────────────────────────────────────────
  async function loadTraces() {
    const d = await getJSON('/api/policies/traces?limit=40');
    S.traces = d.traces || [];
    const el = $('pol-traces');
    el.innerHTML = '';
    S.traces.filter(t => !t.batch).forEach(t => {
      const a = document.createElement('a');
      a.href = '#'; a.className = 'list-group-item list-group-item-action py-1 px-2 small d-flex justify-content-between';
      a.innerHTML = `<span><span class="badge ${t.ok ? 'bg-success' : 'bg-danger'} me-1">${t.ok ? 'ok' : 'fail'}</span>${t.slug} · ${t.target}${t.seed != null ? ' · seed ' + t.seed : ''}</span><span class="text-muted">${(t.started_utc || '').slice(11, 19)}</span>`;
      a.title = t.reason || '';
      a.addEventListener('click', ev => { ev.preventDefault(); openTrace(t.id); });
      el.appendChild(a);
    });
  }

  async function openTrace(id) {
    const rec = await getJSON(`/api/policies/traces/${encodeURIComponent(id)}`);
    if (rec.error) { showToast(rec.error, 'danger'); return; }
    S.currentTrace = rec;
    clearLog();
    (rec.events || []).forEach(appendLog);
    renderResult(rec.result);
    $('pol-run-status').innerHTML = `<span class="text-secondary">Replay</span> ${rec.slug} on ${rec.target} <span class="text-muted">(${rec.id})</span>`;
    if (rec.scene) { S.sceneObjects = rec.initial_objects || S.sceneObjects; }
    if (rec.target === 'sim') p3()?.setLive(true);
    const R = p3()?.replay;
    if (R) {
      R.onTick = (t, dur) => {
        $('pol-replay-slider').value = dur ? (t / dur * 1000) : 0;
        $('pol-replay-time').textContent = `${t.toFixed(1)} / ${dur.toFixed(1)} s`;
        // highlight the latest call at or before t
        let last = null;
        document.querySelectorAll('#pol-log .pol-ev').forEach(el => { if (+el.dataset.t <= t) last = el; el.classList.remove('active'); });
        if (last) { last.classList.add('active'); }
        let line = null;
        for (const ev of rec.events) { if (ev.t <= t && ev.kind === 'call' && ev.line) line = ev.line; if (ev.t > t) break; }
        highlightLine(line);
      };
      R.load(rec);
      $('pol-replay').classList.remove('d-none');
    }
  }

  function goLive() {
    p3()?.replay.pause();
    p3()?.setLive(false);
    $('pol-replay').classList.add('d-none');
    $('pol-run-status').textContent = 'Live robot view';
  }

  // ── Safety panel ─────────────────────────────────────────────
  async function loadSafety() {
    try {
      const d = await getJSON('/api/policies/safety');
      const g = d.guards;
      $('pol-safety').innerHTML = `
        <table class="table table-sm table-borderless info-table mb-1">
          <tr><td class="text-muted">Table</td><td>${d.table_calibrated ? 'calibrated' : '<span class="text-danger">not calibrated</span>'} · table_z ${g.table_z.toFixed(1)}</td></tr>
          <tr><td class="text-muted">Floor clearance</td><td>${g.min_clearance_mm} mm above table (tip)</td></tr>
          <tr><td class="text-muted">Tilt envelope</td><td>±${g.max_tilt_deg}° from vertical</td></tr>
          <tr><td class="text-muted">Per-move travel</td><td>≤ ${g.max_move_mm} mm</td></tr>
          <tr><td class="text-muted">Speed / force caps</td><td>≤ ${g.max_speed}% · ≤ ${g.max_force}%</td></tr>
          <tr><td class="text-muted">Workspace</td><td>X[${g.workspace.x_min.toFixed(0)}, ${g.workspace.x_max.toFixed(0)}] Y[${g.workspace.y_min.toFixed(0)}, ${g.workspace.y_max.toFixed(0)}]</td></tr>
        </table>
        <ul class="small text-muted mb-0 ps-3">${d.layers.map(l => `<li>${escapeHtml(l)}</li>`).join('')}</ul>`;
    } catch (e) { /* ignore */ }
  }

  // ── Init ─────────────────────────────────────────────────────
  function init() {
    if (!$('tab-policies')) return;
    $('pol-select').addEventListener('change', e => { S.slug = e.target.value; renderPolicy(); });
    $('btn-pol-save-params').addEventListener('click', saveParams);
    $('btn-pol-reload').addEventListener('click', async () => { await postJSON('/api/policies/reload'); await loadPolicies(); showToast('Policies reloaded', 'success'); });
    $('pol-scene').addEventListener('change', loadScene);
    $('btn-pol-randomize').addEventListener('click', randomizeScene);
    $('btn-pol-save-scene').addEventListener('click', saveScene);
    $('btn-pol-run').addEventListener('click', runPolicy);
    $('btn-pol-stop').addEventListener('click', stopPolicy);
    $('btn-pol-batch').addEventListener('click', runBatch);
    $('btn-pol-live').addEventListener('click', goLive);
    $('btn-pol-replay-play').addEventListener('click', () => { const R = p3()?.replay; if (!R) return; R.playing ? R.pause() : R.play(); });
    $('pol-replay-slider').addEventListener('input', e => { p3()?.replay.pause(); p3()?.replay.seek(Number(e.target.value) / 1000); });
    $('pol-replay-speed').addEventListener('change', e => { const R = p3()?.replay; if (R) R.speed = Number(e.target.value); });
    document.querySelectorAll('input[name="pol-target"]').forEach(r => r.addEventListener('change', () => {
      const robot = target() === 'robot';
      $('pol-robot-opts').classList.toggle('d-none', !robot);
      $('pol-sim-opts').classList.toggle('d-none', robot);
    }));
    document.addEventListener('dobot-ws', onWs);
    loadPolicies().then(loadScenes).then(loadTraces).then(loadSafety).catch(e => console.warn('policies init', e));
  }

  window.DobotUI = window.DobotUI || {};
  window.DobotUI.initPoliciesTab = init;
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
