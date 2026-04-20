'use strict';

// ── Dobot CR5 — Misc UI wiring (tooltips, popup buttons, hash nav) ─
// Extracted from app.js. Loaded as a classic script AFTER app.js.

(function () {

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

  // ── Pendant / SpaceMouse popup windows ───────────────────────
  // These open /pendant and /spacemouse in resizable OS-level browser
  // windows so the main dashboard stays fully usable behind them.
  // Reusing a fixed window name means a second click focuses the
  // existing window instead of spawning a duplicate.

  function openPopup(url, name, width, height) {
    // popup=yes triggers chromeless window mode in Chromium/Firefox; the
    // explicit no-chrome flags cover older browsers that honor them.
    const features = [
      'popup=yes',
      `width=${width}`,
      `height=${height}`,
      'resizable=yes',
      'scrollbars=yes',
      'location=no',
      'toolbar=no',
      'menubar=no',
      'status=no',
      'directories=no',
      'titlebar=no',
    ].join(',');
    const w = window.open(url, name, features);
    if (w) { try { w.focus(); } catch (_) {} }
    return w;
  }

  function initPopupButtons() {
    const pBtn = document.getElementById('btn-open-pendant');
    if (pBtn) pBtn.addEventListener('click', () => openPopup('/pendant', 'dobot-pendant', 900, 900));

    const smBtn = document.getElementById('btn-open-spacemouse');
    if (smBtn) smBtn.addEventListener('click', () => openPopup('/spacemouse', 'dobot-spacemouse', 525, 820));

    const smLink = document.getElementById('link-open-spacemouse');
    if (smLink) smLink.addEventListener('click', (e) => {
      e.preventDefault();
      openPopup('/spacemouse', 'dobot-spacemouse', 525, 820);
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


  window.DobotUI = window.DobotUI || {};
  window.DobotUI.initTooltips = initTooltips;
  window.DobotUI.initPopupButtons = initPopupButtons;
  window.DobotUI.initHashNav = initHashNav;
})();
