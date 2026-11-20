/* Closed-app screen for the lab dashboards. Served at /ui/lost.js.

   Every 2 s the page asks its own server for a small shared file. Two misses
   in a row (the app was stopped, for example by opening another driving app)
   cover the page with a screen that links back to the apps page on 8080. If
   the server answers again, the page reloads. */
(function () {
  'use strict';
  const PERIOD_MS = 2000;
  const MISSES = 2;
  const name = document.documentElement.dataset.app || 'This app';
  let misses = 0, shown = false;

  function show() {
    if (shown) return;
    shown = true;
    const back = `${location.protocol}//${location.hostname}:8080/`;
    const el = document.createElement('div');
    el.id = 'nr-lost';
    el.innerHTML = `<div class="box">
      <img class="nr-mark" src="/ui/brand/n-mark.svg" alt="NeoRacer">
      <h2></h2>
      <p class="nr-muted">Open it again from the apps page.</p>
      <a class="nr-btn primary" href="${back}">Back to apps</a></div>`;
    el.querySelector('h2').textContent = `${name} closed`;
    document.body.append(el);
  }

  async function check() {
    try {
      await fetch(`/ui/brand/n-mark.svg?t=${Date.now()}`, { cache: 'no-store' });
      if (shown) { location.reload(); return; }
      misses = 0;
    } catch (err) {
      if (++misses >= MISSES) show();
    }
  }
  setInterval(check, PERIOD_MS);
})();
