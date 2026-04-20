'use strict';

// ── Dobot CR5 — Sidebar drag-and-drop reordering ────────────────
// Extracted from app.js. Loaded as a classic script AFTER app.js so
// window.DobotUI.debounce is available.

(function () {
  const ui = () => (window.DobotUI || {});
  const debounce = (fn, ms) => ui().debounce?.(fn, ms) || fn;

  // ── Sidebar drag-and-drop reordering ──────────────────────────
  // The 4 persistent sidebar cards (Robot View, Robot Control, Gripper,
  // Position) can be reordered by dragging any card by its header. The
  // new order is persisted to the settings store under "ui_sidebar" and
  // restored on page load. HTML5 drag-and-drop, no library.

  async function initSidebarReorder() {
    const sidebar = document.querySelector('.app-sidebar');
    if (!sidebar) return;

    // Apply persisted order before wiring handlers so the page settles in
    // its saved layout before any DnD events fire.
    try {
      const r = await fetch('/api/settings/current');
      const j = await r.json();
      const saved = j?.settings?.ui_sidebar?.card_order;
      if (Array.isArray(saved) && saved.length > 0) {
        const cards = Array.from(sidebar.querySelectorAll('.sidebar-card'));
        const byId = new Map(cards.map(c => [c.dataset.cardId, c]));
        // Append each saved card in order; any card not in the saved list
        // (new additions since last save) keeps its current position
        // relative to the end.
        for (const id of saved) {
          const el = byId.get(id);
          if (el) sidebar.appendChild(el);
        }
      }
    } catch (e) { /* keep default order on failure */ }

    const getOrder = () =>
      Array.from(sidebar.querySelectorAll('.sidebar-card'))
        .map(c => c.dataset.cardId)
        .filter(Boolean);

    const persistOrder = debounce(async () => {
      const order = getOrder();
      try {
        await fetch('/api/settings/current', {
          method: 'PATCH',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ ui_sidebar: { card_order: order } }),
        });
      } catch (e) { /* silent — next drop retries */ }
    }, 300);

    let dragging = null;

    sidebar.querySelectorAll('.sidebar-card').forEach(card => {
      card.addEventListener('dragstart', (e) => {
        dragging = card;
        card.classList.add('dragging');
        // Firefox needs this to start the drag.
        try { e.dataTransfer.setData('text/plain', card.dataset.cardId || ''); } catch (_) {}
        e.dataTransfer.effectAllowed = 'move';
      });

      card.addEventListener('dragend', () => {
        card.classList.remove('dragging');
        sidebar.querySelectorAll('.sidebar-card').forEach(c => {
          c.classList.remove('drop-above', 'drop-below');
        });
        dragging = null;
      });

      card.addEventListener('dragover', (e) => {
        if (!dragging || dragging === card) return;
        e.preventDefault();  // required to allow drop
        e.dataTransfer.dropEffect = 'move';
        const rect = card.getBoundingClientRect();
        const above = (e.clientY - rect.top) < rect.height / 2;
        card.classList.toggle('drop-above', above);
        card.classList.toggle('drop-below', !above);
      });

      card.addEventListener('dragleave', () => {
        card.classList.remove('drop-above', 'drop-below');
      });

      card.addEventListener('drop', (e) => {
        e.preventDefault();
        if (!dragging || dragging === card) return;
        const rect = card.getBoundingClientRect();
        const above = (e.clientY - rect.top) < rect.height / 2;
        card.classList.remove('drop-above', 'drop-below');
        if (above) {
          sidebar.insertBefore(dragging, card);
        } else {
          sidebar.insertBefore(dragging, card.nextSibling);
        }
        persistOrder();
      });
    });
  }


  window.DobotUI = window.DobotUI || {};
  window.DobotUI.initSidebarReorder = initSidebarReorder;
})();
