'use strict';

// ---- 定数 -------------------------------------------------------------
const W = 900, ML = 60, MR = 16, PW = W - ML - MR; // 推移グラフの幅・左右余白・描画幅
const PANEL_H = 110, PANEL_GAP = 16, TOP = 14, BOTTOM = 34;
const GAP_MS = 1000;      // これ以上時刻が飛んだら線をつながない
const MAX_BALL_POINTS = 3000;
const FONT = 'font-family="-apple-system,Helvetica,Arial,sans-serif"';
const COL = { pitch: '#d97706', roll: '#2563eb', comb: '#dc2626', fb: '#059669', lr: '#7c3aed' };

const $ = (id) => document.getElementById(id);
let D = null;                 // 読み込み済みログ { t(ms), pitch, roll, fb, lr, comb, name }
let range = [0, 1];           // 表示範囲（秒）
let tMin = 0, tMax = 1;       // ログ全体（秒）

// ---- CSV 読み込み -----------------------------------------------------
function parseCsv(text, name) {
  const lines = text.split('\n');
  let h = lines.findIndex((l) => l.startsWith('t_ms'));
  if (h < 0) throw new Error('t_ms のヘッダ行が見つかりません');
  const cols = lines[h].trim().split(',');
  const idx = (k) => {
    const i = cols.indexOf(k);
    if (i < 0) throw new Error(`列 ${k} がありません`);
    return i;
  };
  const ix = { t: idx('t_ms'), pitch: idx('pitch_deg'), roll: idx('roll_deg'), fb: idx('g_fb'), lr: idx('g_lr'), comb: idx('g_comb') };
  const cap = lines.length - h - 1;
  const d = { name, n: 0 };
  for (const k of Object.keys(ix)) d[k] = new Float64Array(cap);
  for (let i = h + 1; i < lines.length; i++) {
    const v = lines[i].trim().split(',');
    if (v.length < cols.length) continue; // 末尾の書きかけ行など
    const t = Number(v[ix.t]);
    if (!Number.isFinite(t)) continue;
    for (const k of Object.keys(ix)) d[k][d.n] = Number(v[ix[k]]);
    d.n++;
  }
  if (d.n < 2) throw new Error('データ行がありません');
  for (const k of Object.keys(ix)) d[k] = d[k].subarray(0, d.n);
  return d;
}

function load(text, name) {
  try {
    D = parseCsv(text, name);
  } catch (e) {
    alert(`${name}: ${e.message}`);
    return;
  }
  tMin = D.t[0] / 1000;
  tMax = D.t[D.n - 1] / 1000;
  $('fname').textContent = `${name} (${D.n} 行, ${(tMax - tMin).toFixed(1)} 秒)`;
  $('viewer').hidden = false;
  setRange(tMin, tMax);
}

// ---- 範囲 -------------------------------------------------------------
function setRange(a, b) {
  const minSpan = 0.05;
  a = Math.max(tMin, Math.min(a, tMax));
  b = Math.max(tMin, Math.min(b, tMax));
  if (b - a < minSpan) {
    if (a + minSpan <= tMax) b = a + minSpan; else a = b - minSpan;
  }
  range = [a, b];
  $('t0').value = a.toFixed(2);
  $('t1').value = b.toFixed(2);
  const span = tMax - tMin || 1;
  const s0 = Math.round(((a - tMin) / span) * 1000), s1 = Math.round(((b - tMin) / span) * 1000);
  $('s0').value = s0; $('s1').value = s1;
  const room = tMax - tMin - (b - a);
  $('pan').disabled = room <= 1e-9;
  $('pan').value = room > 1e-9 ? Math.round(((a - tMin) / room) * 1000) : 0;
  $('fill').style.left = s0 / 10 + '%';
  $('fill').style.width = (s1 - s0) / 10 + '%';
  render();
}

// t(秒) → 配列インデックス（最初の t >= sec）
function lowerBound(sec) {
  let lo = 0, hi = D.n;
  while (lo < hi) {
    const m = (lo + hi) >> 1;
    if (D.t[m] / 1000 < sec) lo = m + 1; else hi = m;
  }
  return lo;
}

// ---- 描画ヘルパ -------------------------------------------------------
const f1 = (v) => (Math.round(v * 100) / 100).toString();
const esc = (s) => String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));

function niceStep(span, target) {
  const raw = span / target, p = Math.pow(10, Math.floor(Math.log10(raw))), r = raw / p;
  return (r < 1.5 ? 1 : r < 3.5 ? 2 : r < 7.5 ? 5 : 10) * p;
}

function timeAxis(a, b, x, yTop, yBottom) {
  const step = niceStep(b - a, 8);
  let s = '';
  for (let t = Math.ceil(a / step) * step; t <= b + 1e-9; t += step) {
    const px = x(t);
    s += `<line x1="${px}" y1="${yTop}" x2="${px}" y2="${yBottom}" stroke="#e5e7eb"/>`;
    s += `<text x="${px}" y="${yBottom + 14}" font-size="11" fill="#555" text-anchor="middle" ${FONT}>${f1(t)}</text>`;
  }
  s += `<text x="${ML + PW}" y="${yBottom + 28}" font-size="11" fill="#555" text-anchor="end" ${FONT}>time [s]</text>`;
  return s;
}

// 1 系列を折れ線にする。点が多いときは画素列ごとに最小・最大だけ残す（ピークを落とさない）
function seriesPath(arr, i0, i1, x, y) {
  const n = i1 - i0, idxs = [];
  if (n <= PW * 2) {
    for (let i = i0; i < i1; i++) idxs.push(i);
  } else {
    for (let b = 0; b < PW; b++) {
      const s = i0 + Math.floor((b * n) / PW), e = i0 + Math.floor(((b + 1) * n) / PW);
      if (e <= s) continue;
      let lo = s, hi = s;
      for (let i = s; i < e; i++) { if (arr[i] < arr[lo]) lo = i; if (arr[i] > arr[hi]) hi = i; }
      if (lo === hi) idxs.push(lo); else if (lo < hi) idxs.push(lo, hi); else idxs.push(hi, lo);
    }
  }
  let d = '', prev = -1;
  for (const i of idxs) {
    const cmd = prev < 0 || D.t[i] - D.t[prev] > GAP_MS ? 'M' : 'L';
    d += `${cmd}${x(D.t[i] / 1000).toFixed(1)} ${y(arr[i]).toFixed(1)}`;
    prev = i;
  }
  return d;
}

// panels: [{ label, unit, mode:'sym'|'pos', base, step, series:[{key,color,name}] }]
function trendSvg(panels, title) {
  const [a, b] = range;
  const i0 = lowerBound(a), i1 = Math.max(i0 + 1, lowerBound(b + 1e-9));
  const x = (t) => ML + ((t - a) / (b - a)) * PW;
  const H = TOP + panels.length * PANEL_H + (panels.length - 1) * PANEL_GAP + BOTTOM;
  let s = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">`;
  s += `<rect width="${W}" height="${H}" fill="#fff"/>`;
  s += `<text x="${ML}" y="11" font-size="11" fill="#555" ${FONT}>${esc(title)}  ${f1(a)}–${f1(b)} s (${f1(b - a)} s)</text>`;
  panels.forEach((p, k) => {
    const y0 = TOP + k * (PANEL_H + PANEL_GAP), y1 = y0 + PANEL_H;
    let ext = p.base;
    for (const se of p.series) {
      const arr = D[se.key];
      for (let i = i0; i < i1; i++) {
        const v = p.mode === 'sym' ? Math.abs(arr[i]) : arr[i];
        if (v > ext) ext = v;
      }
    }
    ext = Math.ceil(ext / p.step - 1e-9) * p.step;
    const lo = p.mode === 'sym' ? -ext : 0, hi = ext;
    const y = (v) => y1 - ((v - lo) / (hi - lo)) * PANEL_H;
    s += timeAxis(a, b, x, y0, y1);
    const ticks = p.mode === 'sym' ? [-ext, -ext / 2, 0, ext / 2, ext] : [0, ext / 3, (2 * ext) / 3, ext];
    for (const tv of ticks) {
      s += `<line x1="${ML}" y1="${y(tv)}" x2="${ML + PW}" y2="${y(tv)}" stroke="${tv === 0 ? '#9ca3af' : '#e5e7eb'}"/>`;
      s += `<text x="${ML - 6}" y="${y(tv) + 4}" font-size="11" fill="#555" text-anchor="end" ${FONT}>${f1(tv)}</text>`;
    }
    if (p.refLines) for (const rv of p.refLines) {
      if (rv > hi || rv < lo) continue;
      s += `<line x1="${ML}" y1="${y(rv)}" x2="${ML + PW}" y2="${y(rv)}" stroke="#dc2626" stroke-dasharray="4 3" stroke-opacity=".6"/>`;
    }
    s += `<rect x="${ML}" y="${y0}" width="${PW}" height="${PANEL_H}" fill="none" stroke="#9ca3af"/>`;
    s += `<clipPath id="c${k}"><rect x="${ML}" y="${y0}" width="${PW}" height="${PANEL_H}"/></clipPath>`;
    for (const se of p.series) {
      s += `<path d="${seriesPath(D[se.key], i0, i1, x, y)}" fill="none" stroke="${se.color}" stroke-width="1.2" stroke-linejoin="round" clip-path="url(#c${k})"/>`;
    }
    s += `<text x="${ML + 6}" y="${y0 + 13}" font-size="12" font-weight="bold" fill="${p.series[0].color}" ${FONT}>${esc(p.label)} [${esc(p.unit)}]</text>`;
  });
  s += '</svg>';
  return s;
}

function ballSvg() {
  const [a, b] = range;
  const i0 = lowerBound(a), i1 = Math.max(i0 + 1, lowerBound(b + 1e-9));
  const n = i1 - i0, S = 560, cx = S / 2, cy = 290, R = 230, GMAX = 1.5, k = R / GMAX;
  let peak = i0, fMax = 0, bMax = 0, lMax = 0, rMax = 0;
  for (let i = i0; i < i1; i++) {
    if (D.comb[i] > D.comb[peak]) peak = i;
    fMax = Math.max(fMax, D.fb[i]); bMax = Math.max(bMax, -D.fb[i]);
    rMax = Math.max(rMax, D.lr[i]); lMax = Math.max(lMax, -D.lr[i]);
  }
  const H = 640;
  let s = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${S} ${H}" width="${S}" height="${H}">`;
  s += `<rect width="${S}" height="${H}" fill="#fff"/>`;
  s += `<text x="12" y="20" font-size="12" fill="#555" ${FONT}>G ball  ${f1(a)}–${f1(b)} s (${f1(b - a)} s, ${n} samples)</text>`;
  s += `<defs><linearGradient id="tg"><stop offset="0" stop-color="hsl(205,85%,78%)"/><stop offset="1" stop-color="hsl(225,85%,32%)"/></linearGradient></defs>`;
  for (const g of [0.5, 1, 1.5]) {
    s += `<circle cx="${cx}" cy="${cy}" r="${g * k}" fill="none" stroke="${g === 1 ? '#dc2626' : '#9ca3af'}" stroke-width="${g === 1.5 ? 1.5 : 1}" ${g === 1 ? 'stroke-opacity=".7"' : ''}/>`;
    s += `<text x="${cx + 4}" y="${cy - g * k - 3}" font-size="10" fill="#777" ${FONT}>${g}G</text>`;
  }
  s += `<line x1="${cx - R}" y1="${cy}" x2="${cx + R}" y2="${cy}" stroke="#9ca3af"/><line x1="${cx}" y1="${cy - R}" x2="${cx}" y2="${cy + R}" stroke="#9ca3af"/>`;
  s += `<g font-size="13" fill="#555" text-anchor="middle" ${FONT}><text x="${cx}" y="${cy - R - 8}">F</text><text x="${cx}" y="${cy + R + 18}">B</text><text x="${cx - R - 12}" y="${cy + 4}">L</text><text x="${cx + R + 12}" y="${cy + 4}">R</text></g>`;
  const stride = Math.max(1, Math.ceil(n / MAX_BALL_POINTS));
  for (let i = i0; i < i1; i += stride) {
    const f = n > 1 ? (i - i0) / (n - 1) : 1;
    const px = cx + D.lr[i] * k, py = cy - D.fb[i] * k;
    s += `<circle cx="${px.toFixed(1)}" cy="${py.toFixed(1)}" r="2" fill="hsl(${205 + 20 * f},85%,${78 - 46 * f}%)" fill-opacity=".75"/>`;
  }
  const pxk = cx + D.lr[peak] * k, pyk = cy - D.fb[peak] * k;
  s += `<circle cx="${pxk}" cy="${pyk}" r="7" fill="none" stroke="#dc2626" stroke-width="2"/>`;
  const lx = pxk > cx ? pxk - 12 : pxk + 12, anchor = pxk > cx ? 'end' : 'start';
  s += `<text x="${lx}" y="${pyk - 10}" font-size="12" font-weight="bold" fill="#dc2626" text-anchor="${anchor}" ${FONT}>PEAK ${D.comb[peak].toFixed(2)}G @${f1(D.t[peak] / 1000)}s</text>`;
  s += `<g font-size="12" fill="#333" ${FONT}><text x="12" y="${H - 30}">PEAK ${D.comb[peak].toFixed(2)}G   F ${fMax.toFixed(2)}  B ${bMax.toFixed(2)}  L ${lMax.toFixed(2)}  R ${rMax.toFixed(2)}</text></g>`;
  s += `<rect x="${S - 172}" y="${H - 34}" width="150" height="8" fill="url(#tg)"/>`;
  s += `<text x="${S - 172}" y="${H - 38}" font-size="10" fill="#555" ${FONT}>start</text><text x="${S - 22}" y="${H - 38}" font-size="10" fill="#555" text-anchor="end" ${FONT}>end</text>`;
  s += '</svg>';
  return s;
}

function statsHtml() {
  const [a, b] = range;
  const i0 = lowerBound(a), i1 = Math.max(i0 + 1, lowerBound(b + 1e-9));
  let peak = i0, pMin = Infinity, pMax = -Infinity, rMin = Infinity, rMax = -Infinity, over = 0;
  for (let i = i0; i < i1; i++) {
    if (D.comb[i] > D.comb[peak]) peak = i;
    pMin = Math.min(pMin, D.pitch[i]); pMax = Math.max(pMax, D.pitch[i]);
    rMin = Math.min(rMin, D.roll[i]); rMax = Math.max(rMax, D.roll[i]);
    if (D.comb[i] > 1) over++;
  }
  const items = [
    ['範囲', `${f1(b - a)} s`], ['PEAK G', `${D.comb[peak].toFixed(2)} G`], ['PEAK 時刻', `${f1(D.t[peak] / 1000)} s`],
    ['ピッチ min / max', `${pMin.toFixed(1)} / ${pMax.toFixed(1)}°`], ['ロール min / max', `${rMin.toFixed(1)} / ${rMax.toFixed(1)}°`],
    ['1G 超', `${over} 点`],
  ];
  return items.map(([k, v]) => `<div class="stat"><span>${k}</span><b>${v}</b></div>`).join('');
}

// ---- 画面更新 ---------------------------------------------------------
function render() {
  if (!D) return;
  $('trend-pr').innerHTML = trendSvg([
    { label: 'pitch', unit: '°', mode: 'sym', base: 10, step: 5, series: [{ key: 'pitch', color: COL.pitch }] },
    { label: 'roll', unit: '°', mode: 'sym', base: 10, step: 5, series: [{ key: 'roll', color: COL.roll }] },
  ], 'ピッチ・ロール推移');
  $('trend-g').innerHTML = trendSvg([
    { label: 'G (combined)', unit: 'G', mode: 'pos', base: 1.5, step: 0.5, refLines: [1], series: [{ key: 'comb', color: COL.comb }] },
    { label: 'F/B', unit: 'G', mode: 'sym', base: 1.5, step: 0.5, series: [{ key: 'fb', color: COL.fb }] },
    { label: 'L/R', unit: 'G', mode: 'sym', base: 1.5, step: 0.5, series: [{ key: 'lr', color: COL.lr }] },
  ], 'G 推移');
  $('ball').innerHTML = ballSvg();
  $('stats').innerHTML = statsHtml();
}

// ---- 入力 -------------------------------------------------------------
$('t0').addEventListener('change', () => setRange(Number($('t0').value), range[1]));
$('t1').addEventListener('change', () => setRange(range[0], Number($('t1').value)));
$('reset').addEventListener('click', () => setRange(tMin, tMax));
function fromSliders(which) {
  let v0 = Number($('s0').value), v1 = Number($('s1').value);
  if (which === 0 && v0 > v1 - 1) v0 = v1 - 1;
  if (which === 1 && v1 < v0 + 1) v1 = v0 + 1;
  const span = tMax - tMin;
  setRange(tMin + (v0 / 1000) * span, tMin + (v1 / 1000) * span);
}
$('s0').addEventListener('input', () => fromSliders(0));
$('s1').addEventListener('input', () => fromSliders(1));

// スクロール: 幅を保ったまま開始位置だけ動かす
function pan(dt) {
  const w = range[1] - range[0];
  const a = Math.max(tMin, Math.min(range[0] + dt, tMax - w));
  setRange(a, a + w);
}
$('pan').addEventListener('input', () => {
  const w = range[1] - range[0], a = tMin + (Number($('pan').value) / 1000) * (tMax - tMin - w);
  setRange(a, a + w);
});
const panStep = (m) => () => pan(m * (range[1] - range[0]));
$('pan-l2').addEventListener('click', panStep(-1));
$('pan-l1').addEventListener('click', panStep(-0.5));
$('pan-r1').addEventListener('click', panStep(0.5));
$('pan-r2').addEventListener('click', panStep(1));
for (const id of ['trend-pr', 'trend-g']) {
  $(id).addEventListener('wheel', (e) => {
    const d = Math.abs(e.deltaX) > Math.abs(e.deltaY) ? e.deltaX : e.shiftKey ? e.deltaY : 0;
    if (!d || !D) return;
    e.preventDefault();
    pan((d / $(id).clientWidth) * (range[1] - range[0]));
  }, { passive: false });
}
window.addEventListener('keydown', (e) => {
  if (!D || e.target.tagName === 'INPUT' && e.target.type === 'number') return;
  if (e.key === 'ArrowLeft') { pan(-0.1 * (range[1] - range[0])); e.preventDefault(); }
  if (e.key === 'ArrowRight') { pan(0.1 * (range[1] - range[0])); e.preventDefault(); }
});

// 推移グラフ上のドラッグで範囲指定
for (const id of ['trend-pr', 'trend-g']) {
  const box = $(id);
  box.addEventListener('mousedown', (e) => {
    const svg = box.querySelector('svg');
    if (!svg || e.button !== 0) return;
    if (e.shiftKey) { setRange(tMin, tMax); return; }
    const rect = svg.getBoundingClientRect(), [a, b] = range;
    const vbH = svg.viewBox.baseVal.height;
    const toX = (ev) => Math.max(ML, Math.min(ML + PW, ((ev.clientX - rect.left) * W) / rect.width));
    const toT = (px) => a + ((px - ML) / PW) * (b - a);
    const x0 = toX(e);
    const sel = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
    sel.setAttribute('fill', '#2563eb'); sel.setAttribute('fill-opacity', '.15');
    sel.setAttribute('y', 0); sel.setAttribute('height', vbH);
    svg.appendChild(sel);
    const move = (ev) => {
      const x1 = toX(ev);
      sel.setAttribute('x', Math.min(x0, x1)); sel.setAttribute('width', Math.abs(x1 - x0));
    };
    const up = (ev) => {
      window.removeEventListener('mousemove', move);
      window.removeEventListener('mouseup', up);
      const x1 = toX(ev);
      if (Math.abs(x1 - x0) > 3) setRange(toT(Math.min(x0, x1)), toT(Math.max(x0, x1))); else sel.remove();
    };
    window.addEventListener('mousemove', move);
    window.addEventListener('mouseup', up);
    e.preventDefault();
  });
}

// SVG ダウンロード
document.querySelectorAll('[data-dl]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const svg = $(btn.dataset.dl).querySelector('svg');
    if (!svg) return;
    const clone = svg.cloneNode(true);
    clone.querySelectorAll('rect[fill-opacity=".15"]').forEach((r) => r.remove()); // 選択中の枠
    const blob = new Blob(['<?xml version="1.0" encoding="UTF-8"?>\n', clone.outerHTML], { type: 'image/svg+xml' });
    const base = D.name.replace(/\.csv$/i, '');
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = `${base}_${btn.dataset.dl}_${f1(range[0])}-${f1(range[1])}s.svg`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  });
});

// ファイル読み込み
function readFile(file) {
  const r = new FileReader();
  r.onload = () => load(String(r.result), file.name);
  r.readAsText(file);
}
$('picker').addEventListener('change', (e) => e.target.files[0] && readFile(e.target.files[0]));
const drop = $('drop');
window.addEventListener('dragover', (e) => { e.preventDefault(); drop.classList.add('over'); });
window.addEventListener('dragleave', (e) => { if (!e.relatedTarget) drop.classList.remove('over'); });
window.addEventListener('drop', (e) => {
  e.preventDefault(); drop.classList.remove('over');
  if (e.dataTransfer.files[0]) readFile(e.dataTransfer.files[0]);
});

// サーバー側ログ一覧
fetch('/api/files').then((r) => r.json()).then((files) => {
  const sel = $('files');
  for (const f of files) {
    const o = document.createElement('option');
    o.value = f.name; o.textContent = `${f.name} (${(f.size / 1024).toFixed(0)} KB)`;
    sel.appendChild(o);
  }
}).catch(() => {});
$('files').addEventListener('change', (e) => {
  const name = e.target.value;
  if (!name) return;
  fetch('/api/file?name=' + encodeURIComponent(name)).then((r) => r.text()).then((t) => load(t, name));
});
