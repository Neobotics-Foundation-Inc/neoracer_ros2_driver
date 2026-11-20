/* The light and dark themes of every page the car serves. Served at /ui/theme.js.

   Load it in <head>, before the page paints, so the theme applies with no
   flash: ?theme=light or ?theme=dark in the address, else the one picked
   earlier in this browser, else the device's own preference. Buttons with data-theme-pick="dark" or "light" switch it, and
   the picked one gets aria-pressed="true" (the stylesheet fills it red). */
(function () {
  'use strict';
  const KEY = 'nr-theme';
  const root = document.documentElement;

  function asked() {
    try { return new URLSearchParams(location.search).get('theme'); } catch (err) { return null; }
  }

  function saved() {
    try { return localStorage.getItem(KEY); } catch (err) { return null; }
  }

  function fallback() {
    return window.matchMedia && window.matchMedia('(prefers-color-scheme: light)').matches
      ? 'light' : 'dark';
  }

  function apply(theme) {
    root.dataset.theme = theme === 'light' ? 'light' : 'dark';
    for (const b of document.querySelectorAll('[data-theme-pick]')) {
      b.setAttribute('aria-pressed', String(b.dataset.themePick === root.dataset.theme));
    }
  }

  apply(asked() || saved() || fallback());
  document.addEventListener('DOMContentLoaded', () => {
    apply(root.dataset.theme);
    document.addEventListener('click', (ev) => {
      const b = ev.target.closest('[data-theme-pick]');
      if (!b) return;
      apply(b.dataset.themePick);
      try { localStorage.setItem(KEY, root.dataset.theme); } catch (err) { /* private window */ }
    });
  });
})();
