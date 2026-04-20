'use strict';

// ── Dobot CR5 — Settings tab ────────────────────────────────────
// Extracted from app.js. Loaded as a classic script AFTER app.js.

(function () {
  const ui = () => (window.DobotUI || {});
  const logActivity = (msg, type) => ui().logActivity?.(msg, type);
  const showToast   = (msg, type) => ui().showToast?.(msg, type);

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
      el.textContent = `Failed to load: ${e}`;
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


  window.DobotUI = window.DobotUI || {};
  window.DobotUI.initSettings = initSettings;
})();
