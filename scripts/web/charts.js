/* Time charts shared by the pages the car serves. Served at /ui/charts.js by
   scripts/webui.py; styles are .nr-chart in neoracer.css.

   NrCharts.timeChart(box, t0, t1, series, xTicks, opts) draws into `box`:
     series: [{name, style: 'a'|'b'|'c'|'d', axis: 'l'|'r', unit, digits, minSpan, pts: [[t, v], ...]}]
     xTicks(t0, t1) -> [[t, label], ...], with xTicks.label(t) for the readout
     opts: {gapSec, fixed: {l: [lo, hi], r: [lo, hi]}, marks: [[t, label], ...],
            bands: [[t, flag], ...] (a strip along the bottom, red where flag)}
   A right axis appears only when a series asks for it. Hover or tap shows a
   hairline and every series' value at the nearest sample. */
(function () {
  'use strict';
  const SVGNS = 'http://www.w3.org/2000/svg';
  // Series styles: color and dash, so identity never rests on color alone.
  // Red is the one accent (the first series); the others are the theme's inks.
  const COLOR = { a: 'var(--nr-red)', b: 'var(--nr-ink)', c: 'var(--nr-ink-2)', d: 'var(--nr-ink-3)' };
  const DASH = { a: '', b: '6 4', c: '1.5 4', d: '' };
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

  function el(tag, attrs, parent) {
    const e = document.createElementNS(SVGNS, tag);
    for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
    if (parent) parent.append(e);
    return e;
  }

  function niceTicks(lo, hi, count) {
    const span = hi - lo || 1, raw = span / count;
    const mag = 10 ** Math.floor(Math.log10(raw));
    const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw);
    const ticks = [];
    for (let v = Math.ceil(lo / step - 1e-9) * step; v <= hi + 1e-9; v += step) ticks.push(+v.toFixed(6));
    return ticks;
  }

  function legendHtml(series) {
    return series.map((s) => `<span><svg viewBox="0 0 22 8"><line x1="1" y1="4" x2="21" y2="4"
      stroke="${COLOR[s.style]}" stroke-width="2" stroke-dasharray="${DASH[s.style]}"/></svg>${esc(s.name)}</span>`).join('');
  }

  function timeChart(box, t0, t1, series, xTicks, opts = {}) {
    const gapSec = opts.gapSec ?? 1, fixed = opts.fixed || {};
    // Only samples inside the window are drawn or scaled.
    series = series.map((s) => ({ ...s, pts: s.pts.filter((p) => p[0] >= t0 && p[0] <= t1) }));
    box.replaceChildren();
    const w = box.clientWidth, h = box.clientHeight;
    if (!w) return;
    if (!series.some((s) => s.pts.length)) {
      const d = document.createElement('div');
      d.className = 'empty nr-small nr-muted'; d.textContent = 'No data.';
      box.append(d); return;
    }
    const hasR = series.some((s) => s.axis === 'r');
    const pad = { l: 40, r: hasR ? 40 : 8, t: 10, b: 28 };
    const scale = {};
    for (const side of ['l', 'r']) {
      const own = series.filter((s) => s.axis === side);
      if (!own.length) continue;
      let lo, hi;
      if (fixed[side]) {
        [lo, hi] = fixed[side];
      } else {
        const vals = own.flatMap((s) => s.pts.map((p) => p[1]));
        if (!vals.length) continue;
        lo = Math.min(...vals); hi = Math.max(...vals);
        const m = Math.max((hi - lo) * 0.15, own[0].minSpan || 0.1);
        lo -= m; hi += m;
      }
      const ticks = niceTicks(lo, hi, 4);
      scale[side] = { lo: Math.min(lo, ticks[0]), hi: Math.max(hi, ticks[ticks.length - 1]), ticks };
    }
    const x = (t) => pad.l + (t - t0) / (t1 - t0 || 1) * (w - pad.l - pad.r);
    const y = (side, v) => pad.t + (scale[side].hi - v) / (scale[side].hi - scale[side].lo) * (h - pad.t - pad.b);
    const svg = el('svg', { class: 'plot', viewBox: `0 0 ${w} ${h}`, role: 'img',
      'aria-label': series.map((s) => s.name).join(', ') }, box);
    for (const v of scale.l ? scale.l.ticks : []) {
      el('line', { class: 'grid', x1: pad.l, x2: w - pad.r, y1: y('l', v), y2: y('l', v) }, svg);
      el('text', { class: 'tick', x: pad.l - 6, y: y('l', v) + 4, 'text-anchor': 'end' }, svg).textContent = `${v}`;
    }
    for (const v of scale.r ? scale.r.ticks : []) {
      if (!scale.l) el('line', { class: 'grid', x1: pad.l, x2: w - pad.r, y1: y('r', v), y2: y('r', v) }, svg);
      el('text', { class: 'tick', x: w - pad.r + 6, y: y('r', v) + 4, 'text-anchor': 'start' }, svg).textContent = `${v}`;
    }
    for (const [t, label] of xTicks(t0, t1)) {
      // Labels at the plot's edges align inward so the clip never cuts them.
      const px = x(t), anchor = px > w - pad.r - 20 ? 'end' : (px < pad.l + 20 ? 'start' : 'middle');
      el('text', { class: 'tick', x: px, y: h - 3, 'text-anchor': anchor }, svg).textContent = label;
    }
    for (const [t, label] of opts.marks || []) {
      if (t < t0 || t > t1) continue;
      el('line', { class: 'mark', x1: x(t), x2: x(t), y1: pad.t, y2: h - pad.b }, svg);
      el('text', { class: 'tick', x: x(t) + 4, y: pad.t + 10 }, svg).textContent = label;
    }
    const bands = (opts.bands || []).filter((b) => b[0] >= t0 && b[0] <= t1);
    for (let i = 0; i < bands.length - 1; i++) {
      el('rect', { class: bands[i][1] ? 'band on' : 'band', x: x(bands[i][0]), y: h - pad.b - 5,
        width: Math.max(1, x(bands[i + 1][0]) - x(bands[i][0]) + 0.5), height: 5 }, svg);
    }
    for (const s of series) {
      if (!scale[s.axis]) continue;
      let d = '', prev = null;
      for (const p of s.pts) {
        d += `${prev === null || p[0] - prev > gapSec ? 'M' : 'L'}${x(p[0]).toFixed(1)},${y(s.axis, p[1]).toFixed(1)}`;
        prev = p[0];
      }
      el('path', { class: 'line', d, stroke: COLOR[s.style], 'stroke-dasharray': DASH[s.style] }, svg);
    }
    const hair = el('line', { class: 'hair', y1: pad.t, y2: h - pad.b, visibility: 'hidden' }, svg);
    const tip = document.createElement('div');
    tip.className = 'tipbox'; tip.hidden = true; box.append(tip);
    const ref = series.find((s) => s.pts.length).pts;
    const show = (px) => {
      const t = t0 + (px - pad.l) / (w - pad.l - pad.r) * (t1 - t0);
      let best = ref[0];
      for (const p of ref) if (Math.abs(p[0] - t) < Math.abs(best[0] - t)) best = p;
      const cx = x(best[0]);
      hair.setAttribute('x1', cx); hair.setAttribute('x2', cx); hair.setAttribute('visibility', 'visible');
      const lines = [xTicks.label(best[0])];
      for (const s of series) {
        const p = s.pts.find((q) => q[0] === best[0]);
        if (p) lines.push(`${s.name}: ${p[1].toFixed(s.digits)} ${s.unit}`);
      }
      tip.textContent = lines.join('\n');
      tip.hidden = false;
      tip.style.left = `${Math.min(Math.max(cx - tip.offsetWidth / 2, 0), w - tip.offsetWidth)}px`;
    };
    svg.addEventListener('pointermove', (e) => show(e.offsetX));
    svg.addEventListener('pointerdown', (e) => show(e.offsetX));
    svg.addEventListener('pointerleave', () => { hair.setAttribute('visibility', 'hidden'); tip.hidden = true; });
  }

  const clock = (t) => new Date(t * 1000).toTimeString().slice(0, 5);
  // Clock times every `step` s (30 min by default), for charts in wall-clock seconds.
  const clockTicksEvery = (step) => Object.assign((t0, t1) => {
    const out = [];
    for (let t = Math.ceil(t0 / step) * step; t <= t1; t += step) out.push([t, clock(t)]);
    return out;
  }, { label: clock });
  const clockTicks = clockTicksEvery(1800);
  // Seconds before `now`, every `step` s.
  const recentTicks = (now, step = 10) => Object.assign((t0, t1) => {
    const out = [];
    for (let s = 0; s <= t1 - t0 + 1e-6; s += step) out.push([t1 - s, s ? `-${s} s` : 'now']);
    return out;
  }, { label: (t) => `${(t - now).toFixed(1)} s` });

  window.NrCharts = { timeChart, legendHtml, niceTicks, clockTicks, clockTicksEvery, recentTicks, COLOR, DASH };
})();
