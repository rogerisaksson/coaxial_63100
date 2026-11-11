// The atlas page's script (tools/dev/atlas.py inlines it): the trunk drawn as a stack, its ties as wires,
// the layers' ties to each other as a table; a node at a time in the lens beside it.
(() => {
  'use strict';
  const $ = s => document.querySelector(s);
  const A = JSON.parse($('#atlas-data').textContent);
  const lens = $('#lens'), tower = $('#tower'), crumbs = $('#crumbs'), find = $('#find');
  if (!A || typeof A !== 'object') { lens.textContent = 'No atlas inlined: atlas.py --html lays it.'; return; }
  const N = A.id.length, LAYER = 0, FOLDER = 1, FILE = 2, SYMBOL = 3, L = A.layers.length, WIRES = 12;
  const kids = Array.from({length: N}, () => []);
  const own = [0, 1, 2].map(() => new Int32Array(N).fill(-1));
  for (let i = 0; i < N; i++) {
    const u = A.up[i], k = A.kind[i];
    if (u >= 0) { kids[u].push(i); for (let l = 0; l < 3; l++) own[l][i] = own[l][u]; }
    if (k < SYMBOL) own[k][i] = i;
  }
  const below = [0, 1, 2, 3].map(() => new Int32Array(N));
  for (let i = N - 1; i >= 0; i--) {
    const u = A.up[i];
    if (u < 0) continue;
    below[A.kind[i]][u]++;
    for (let k = 1; k < 4; k++) below[k][u] += below[k][i];
  }
  const byId = new Map(A.id.map((s, i) => [s, i])), byKey = new Map(A.layers.map((r, i) => [r[0], i]));
  const trunk = [], beside = [], pos = [];
  A.layers.forEach((r, i) => (r[1] === 'trunk' ? trunk : beside).push(i));
  trunk.forEach((l, p) => { pos[l] = p + 1; });
  const stack = trunk.slice().reverse().concat(beside);
  const between = Array.from({length: L}, () => new Int32Array(L));
  for (let t = 0; t < A.src.length; t++) between[own[0][A.src[t]]][own[0][A.dst[t]]]++;
  const count = k => A.kind.reduce((n, x) => n + (x === k), 0);
  const counted = (n, word) => n + ' ' + word + (n === 1 ? '' : 's');
  const esc = s => String(s).replace(/[&<>"']/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c]));
  const vscode = (path, line) => 'vscode://file/' + encodeURI(A.root + '/' + path) + ':' + (line || 1);
  const name = i => A.kind[i] === SYMBOL ? A.label[i] : A.id[i];
  const layerName = l => (pos[l] ? pos[l] + ' ' : '') + A.layers[l][0];
  const sw = c => '<span class="sw" style="--c:var(--' + c + ')"></span>';
  const kinds = set => [...set].map(t => A.ties[t]).join(' ');
  const totals = [counted(count(FILE), 'file'), counted(count(SYMBOL), 'symbol'), counted(A.src.length, 'tie')].join(' &middot; ');
  let cur = -1, query = '', pair = null, restore = -1, way = 'down', wired = [];

  function bump(m, k, t) { let e = m.get(k); if (!e) m.set(k, e = {n: 0, k: new Set()}); e.n++; e.k.add(t); }
  function ties(i) {
    if (A.kind[i] === SYMBOL) i = A.up[i];
    const lv = A.kind[i], out = new Map(), inn = new Map();
    for (let t = 0; t < A.src.length; t++) {
      const a = own[lv][A.src[t]], b = own[lv][A.dst[t]];
      if (a === b) continue;
      if (a === i) bump(out, b, A.tie[t]); else if (b === i) bump(inn, a, A.tie[t]);
    }
    return [out, inn];
  }
  const ranked = m => [...m.entries()].sort((x, y) => y[1].n - x[1].n);
  // The open node's ties by folder and by layer, each way: its wires and its tags.
  function lit(i) {
    const outF = new Map(), inF = new Map(), outL = new Map(), inL = new Map();
    if (i < 0) return {outF, inF, outL, inL};
    const node = A.kind[i] === SYMBOL ? A.up[i] : i, lv = A.kind[node];
    for (let t = 0; t < A.src.length; t++) {
      const s = A.src[t], d = A.dst[t], a = own[lv][s] === node, b = own[lv][d] === node;
      if (a && !b) { outF.set(own[1][d], (outF.get(own[1][d]) || 0) + 1); outL.set(own[0][d], (outL.get(own[0][d]) || 0) + 1); }
      else if (b && !a) { inF.set(own[1][s], (inF.get(own[1][s]) || 0) + 1); inL.set(own[0][s], (inL.get(own[0][s]) || 0) + 1); }
    }
    return {outF, inF, outL, inL};
  }
  function marked(s, q) {
    const low = String(s).toLowerCase();
    let out = '', at = 0, k;
    while (q && (k = low.indexOf(q, at)) >= 0) {
      out += esc(s.slice(at, k)) + '<mark>' + esc(s.slice(k, k + q.length)) + '</mark>';
      at = k + q.length;
    }
    return out + esc(String(s).slice(at));
  }

  function row(l, z) {
    const tiles = kids[l].map(f => '<span class="tile" data-go="' + f + '" data-f="' + f + '" style="--n:' +
      Math.max(1, below[FILE][f]) + '" title="' + esc(A.id[f] + ' (' + counted(below[FILE][f], 'file') + ')' +
      (A.brief[f] ? ' - ' + A.brief[f] : '')) + '"></span>').join('');
    const cap = l === trunk[trunk.length - 1] ? 'crown: up to the shunts' : l === trunk[0] ? 'foot: this terminal' : '';
    return '<div class="row' + (pos[l] ? '' : ' beside') + '" data-go="' + l + '" data-l="' + l + '" tabindex="0" role="button" ' +
      'aria-label="' + esc(layerName(l)) + '" style="z-index:' + z + '"><div class="slab">' + tiles + '</div><div class="tag"><b>' +
      esc(layerName(l)) + '</b><small>' + counted(kids[l].length, 'folder') + ', ' + counted(below[FILE][l], 'file') +
      '</small><i class="hits"></i>' + (cap ? '<em>' + cap + '</em>' : '') + '</div></div>';
  }
  tower.innerHTML = '<div class="inside"><svg id="wires" aria-hidden="true"></svg><div class="stack">' +
    trunk.slice().reverse().map((l, j) => row(l, 100 - j)).join('') + '</div><div class="legend"><span>' + sw('open') +
    'open</span><span>' + sw('out') + 'reaches</span><span>' + sw('in') + 'reached by</span><span>' + sw('hit') +
    'found</span></div><h4>beside the trunk</h4><div class="stack">' + beside.map((l, j) => row(l, 50 - j)).join('') + '</div></div>';
  $('#head').textContent = A.repo + ' @ ' + A.commit;

  function card(i, extra) {
    const k = A.kind[i];
    const n = k <= FOLDER ? counted(below[FILE][i], 'file') : k === FILE ? counted(below[SYMBOL][i], 'symbol') : ':' + A.line[i];
    const title = k === LAYER ? layerName(i) : k === FOLDER ? A.label[i] : A.label[i];
    return '<button class="card" data-go="' + i + '"><span class="top"><span class="t">' + esc(title) + '</span><span class="n">' +
      n + '</span></span>' + (A.brief[i] ? '<span class="b">' + esc(A.brief[i]) + '</span>' : '') +
      (extra ? '<span class="r">' + extra + '</span>' : '') + '</button>';
  }
  function layerCard(l) {
    const top = stack.filter(m => m !== l && between[l][m]).sort((x, y) => between[l][y] - between[l][x]).slice(0, 4);
    return card(l, top.length ? 'reaches ' + top.map(m => esc(A.layers[m][0]) + ' ' + between[l][m]).join(' &middot; ') : '');
  }
  function side(m, title) {
    const rows = ranked(m), shown = rows.slice(0, 60);
    return '<div><h3>' + title + ' <small>' + rows.length + '</small></h3>' + (shown.map(([j, e]) =>
      '<button class="tie" data-go="' + j + '"><span class="who">' + esc(name(j)) + '</span><span class="k">' + kinds(e.k) +
      '</span><span class="c">' + e.n + '</span></button>').join('') || '<p class="none">none</p>') +
      (rows.length > 60 ? '<p class="none">and ' + (rows.length - 60) + ' more</p>' : '') + '</div>';
  }
  function tiesHtml(i) {
    const [out, inn] = ties(i);
    return '<section class="ties">' + side(out, sw('out') + 'reaches') + side(inn, sw('in') + 'reached by') + '</section>';
  }
  function headHtml(i) {
    const k = A.kind[i], l = own[0][i], file = own[2][i];
    const where = k === LAYER ? (pos[i] ? 'layer ' + pos[i] + ' of ' + trunk.length + ' on the trunk' : 'beside the trunk') :
      'in ' + layerName(l) + (k === SYMBOL ? ', ' + A.id[file] : '');
    const held = [FOLDER, FILE, SYMBOL].filter(x => x > k && below[x][i]).map(x => counted(below[x][i], A.kinds[x])).join(' &middot; ');
    const link = file >= 0 ? '<a class="code" href="' + vscode(A.id[file], k === SYMBOL ? A.line[i] : 1) + '">VS Code</a>' : '';
    return '<div class="head"><span class="kind">' + A.kinds[k] + '</span><span class="where">' + esc(where) + '</span><h1>' +
      esc(k === FOLDER ? A.label[i] : name(i)) + '</h1>' + (A.brief[i] ? '<p class="brief">' + esc(A.brief[i]) + '</p>' : '') +
      '<p class="stats">' + [held, link].filter(Boolean).join(' &middot; ') + '</p></div>';
  }
  function symsHtml(i) {
    if (!kids[i].length) return '<p class="none">No symbols read from this kind of file.</p>';
    return '<div class="syms">' + kids[i].map(s => '<div class="sym"><a class="ln" href="' + vscode(A.id[i], A.line[s]) + '">:' +
      A.line[s] + '</a><button class="s" data-go="' + s + '">' + esc(A.label[s]) + '</button>' +
      (A.brief[s] ? '<span class="b">' + esc(A.brief[s]) + '</span>' : '') + '</div>').join('') + '</div>';
  }
  // Who ties to whom: a row's layer reaching a column's, one hue in five bins - its numbers its own table.
  const BINS = [[1, '1-4'], [5, '5-19'], [20, '20-79'], [80, '80-319'], [320, '320+']];
  const bin = n => BINS.reduce((b, [low], k) => (n >= low ? k + 1 : b), 0);
  function matrixHtml() {
    const head = stack.map(b => '<th scope="col"><span>' + esc(layerName(b)) + '</span></th>').join('');
    const body = stack.map(a => '<tr><th scope="row">' + esc(layerName(a)) + '</th>' + stack.map(b => {
      const n = between[a][b], tip = A.layers[a][0] + ' to ' + A.layers[b][0] + ': ' + counted(n, 'tie');
      return '<td class="b' + bin(n) + (a === b ? ' self' : '') + '"' + (n ? ' tabindex="0" data-pair="' + a + ',' + b + '"' : '') +
        ' title="' + esc(tip) + '">' + (n || '&middot;') + '</td>';
    }).join('') + '</tr>').join('');
    return '<h2>who ties to whom <small>a row\'s layer reaching a column\'s; a cell opens its ties</small></h2><div class="matrix">' +
      '<table class="heat"><thead><tr><th></th>' + head + '</tr></thead><tbody>' + body + '</tbody><caption>imports, includes, ' +
      'files named, the wire\'s commands; the diagonal a layer\'s ties within itself</caption></table></div><div class="scale">' +
      '<b>ties</b>' + BINS.map(([, s], k) => '<span class="b' + (k + 1) + '">' + s + '</span>').join('') + '</div>';
  }
  function rootHtml() {
    return '<div class="head"><span class="kind">trunk</span><h1>' + esc(A.repo) + '</h1><p class="brief">From the terminal at its ' +
      'foot up to the shunts at its crown, ' + trunk.length + ' layers; beside it ' + beside.map(l => esc(A.layers[l][0])).join(', ') +
      '. Open a slab, a tile, a card or a cell.</p><p class="stats">' + totals + ' &middot; ' + esc(A.commit) + '</p></div>' +
      matrixHtml() + '<h2>the trunk, crown first</h2><div class="grid wide">' + trunk.slice().reverse().map(layerCard).join('') +
      '</div><h2>beside the trunk</h2><div class="grid wide">' + beside.map(layerCard).join('') + '</div>';
  }
  function pairs(a, b) {
    const m = new Map();
    for (let t = 0; t < A.src.length; t++) {
      const s = A.src[t], d = A.dst[t];
      if (own[0][s] !== a || own[0][d] !== b) continue;
      const key = own[1][s] + ',' + own[1][d];
      let e = m.get(key);
      if (!e) m.set(key, e = {s: own[1][s], d: own[1][d], n: 0, k: new Set()});
      e.n++; e.k.add(A.tie[t]);
    }
    return [...m.values()].sort((x, y) => y.n - x.n);
  }
  function pairHtml(a, b) {
    const rows = pairs(a, b);
    return '<div class="head"><span class="kind">ties</span><span class="where">' + counted(between[a][b], 'tie') + ' between ' +
      counted(rows.length, 'pair') + ' of folders</span><h1>' + esc(layerName(a)) + ' &rarr; ' + esc(layerName(b)) + '</h1></div>' +
      '<section class="ties" style="grid-template-columns:1fr"><div>' + rows.map(e => '<button class="tie" data-go="' + e.s +
      '"><span class="who">' + esc(A.label[e.s]) + ' &rarr; ' + esc(A.label[e.d]) + '</span><span class="k">' + kinds(e.k) +
      '</span><span class="c">' + e.n + '</span></button>').join('') + '</div></section>';
  }
  function nodeHtml(i) {
    const k = A.kind[i];
    const body = k <= FOLDER ? '<div class="grid">' + kids[i].map(c => card(c)).join('') + '</div>' : k === FILE ? symsHtml(i) : '';
    return headHtml(i) + body + tiesHtml(i);
  }
  let hay = null;
  function findGroups(q) {
    hay = hay || A.id.map((s, i) => (s + '\n' + A.label[i] + '\n' + A.brief[i]).toLowerCase());
    const groups = new Map();
    for (let i = 0; i < N; i++) if (hay[i].includes(q)) { const l = own[0][i]; if (!groups.has(l)) groups.set(l, []); groups.get(l).push(i); }
    return groups;
  }
  function findHtml(q, groups) {
    const all = [...groups.values()].reduce((n, g) => n + g.length, 0), on = trunk.filter(l => groups.has(l)).length;
    return '<div class="head"><span class="kind">find</span><h1>' + esc(q) + '</h1><p class="brief">' + counted(all, 'node') +
      ' named, signed or briefed with it, on ' + on + ' of the trunk\'s ' + trunk.length + ' layers and ' + (groups.size - on) +
      ' beside it. atlas.py --find reads texts too.</p></div>' + stack.filter(l => groups.has(l)).map(l => {
        const g = groups.get(l);
        return '<h2>' + esc(layerName(l)) + ' <small>' + g.length + '</small></h2>' + g.slice(0, 80).map(i =>
          '<button class="hit" data-go="' + i + '"><span class="kind">' + A.kinds[A.kind[i]] + '</span><span class="who">' +
          marked(A.kind[i] === SYMBOL ? A.id[i] : name(i), q) + '</span><span class="b">' + marked(A.brief[i], q) + '</span></button>').join('') +
          (g.length > 80 ? '<p class="none">and ' + (g.length - 80) + ' more</p>' : '');
      }).join('');
  }

  // The wires: from the node open to the folders it reaches and from those reaching it, the heaviest
  // WIRES each way, a stroke's weight its ties; bowed out to the stack's left, clear of its names.
  function at(node) {
    const el = tower.querySelector(A.kind[node] === LAYER ? '.row[data-l="' + node + '"] .slab' : '.tile[data-f="' + node + '"]');
    if (!el) return null;
    const r = el.getBoundingClientRect(), base = $('#wires').getBoundingClientRect();
    return [r.left + r.width / 2 - base.left, r.top + r.height / 2 - base.top];
  }
  function drawWires() {
    const top = Math.max(1, ...wired.map(w => w[2]));
    $('#wires').innerHTML = wired.map(([a, b, n, c]) => {
      const p = at(a), q = at(b);
      if (!p || !q) return '';
      const x = Math.min(p[0], q[0]) - Math.min(52, 14 + Math.abs(q[1] - p[1]) * 0.22), f = v => v.toFixed(1);
      return '<path d="M' + f(p[0]) + ' ' + f(p[1]) + 'C' + f(x) + ' ' + f(p[1]) + ' ' + f(x) + ' ' + f(q[1]) + ' ' + f(q[0]) + ' ' +
        f(q[1]) + '" style="stroke:var(--' + c + ');stroke-width:' + (1.2 + 2.6 * n / top).toFixed(2) + '"/><circle cx="' +
        f(c === 'out' ? q[0] : p[0]) + '" cy="' + f(c === 'out' ? q[1] : p[1]) + '" r="2.6" style="fill:var(--' + c + ')"/>';
    }).join('');
  }
  function paint(groups) {
    const lt = groups || pair ? lit(-1) : lit(cur);
    const curL = cur >= 0 ? own[0][cur] : -1, curF = cur >= 0 ? own[1][cur] : -1, hitF = new Set();
    const src = pair ? new Set(pairs(pair[0], pair[1]).map(e => e.s)) : new Set();
    const dst = pair ? new Set(pairs(pair[0], pair[1]).map(e => e.d)) : new Set();
    if (groups) for (const g of groups.values()) for (const i of g) hitF.add(own[1][i]);
    tower.classList.toggle('root', cur < 0 && !groups && !pair);
    tower.querySelectorAll('.row').forEach(r => {
      const l = +r.dataset.l, n = groups && groups.has(l) ? groups.get(l).length : 0, o = lt.outL.get(l), m = lt.inL.get(l);
      r.classList.toggle('on', !groups && (pair ? l === pair[0] : l === curL));
      r.classList.toggle('dim', !!groups && !n);
      r.querySelector('.hits').innerHTML = n ? sw('hit') + n + ' found' : (o ? sw('out') + o + ' ' : '') + (m ? sw('in') + m : '');
    });
    tower.querySelectorAll('.tile').forEach(t => {
      const f = +t.dataset.f;
      t.className = 'tile' + (groups ? (hitF.has(f) ? ' hit' : '') : pair ? (src.has(f) ? ' on' : dst.has(f) ? ' out' : '') :
        f === curF ? ' on' : lt.outF.has(f) ? ' out' : lt.inF.has(f) ? ' in' : '');
    });
    const from = cur >= 0 ? (A.kind[cur] === LAYER ? cur : curF) : -1;
    wired = pair ? pairs(pair[0], pair[1]).filter(e => e.s !== e.d).slice(0, WIRES).map(e => [e.s, e.d, e.n, 'out']) :
      groups || from < 0 ? [] : [...[...lt.outF].filter(([f]) => f !== from).sort((x, y) => y[1] - x[1]).slice(0, WIRES).map(([f, n]) => [from, f, n, 'out']),
        ...[...lt.inF].filter(([f]) => f !== from).sort((x, y) => y[1] - x[1]).slice(0, WIRES).map(([f, n]) => [f, from, n, 'in'])];
    drawWires();
  }
  function render(focus) {
    const groups = query ? findGroups(query) : null;
    lens.dataset.way = way;
    lens.innerHTML = (groups ? findHtml(query, groups) : pair ? pairHtml(pair[0], pair[1]) : cur < 0 ? rootHtml() : nodeHtml(cur)) +
      '<footer><kbd>/</kbd> find &middot; <kbd>&uarr;</kbd> <kbd>&darr;</kbd> move &middot; <kbd>&rarr;</kbd> open &middot; ' +
      '<kbd>&larr;</kbd> up &middot; <kbd>Esc</kbd> the word, then up &middot; atlas.py --html at ' + esc(A.commit) + '</footer>';
    document.title = 'atlas' + (groups ? ' / ' + query : pair ? ' / ' + A.layers[pair[0]][0] + ' to ' + A.layers[pair[1]][0] :
      cur >= 0 ? ' / ' + name(cur) : '');
    const chain = [];
    for (let i = cur; i >= 0; i = A.up[i]) chain.unshift(i);
    crumbs.innerHTML = '<a data-go="-1">trunk</a>' + (pair ? '<span class="sep">/</span>' + esc(A.layers[pair[0]][0] + ' to ' +
      A.layers[pair[1]][0]) : chain.map(i => '<span class="sep">/</span><a data-go="' + i + '">' + esc(A.kind[i] === LAYER ?
      layerName(i) : A.kind[i] === SYMBOL ? A.label[i].split('(')[0] : A.label[i]) + '</a>').join(''));
    paint(groups);
    if (focus) {
      const el = (restore >= 0 && lens.querySelector('[data-go="' + restore + '"]')) || lens.querySelector('[data-go], [data-pair]');
      if (el) { el.focus({preventScroll: true}); el.scrollIntoView({block: 'nearest'}); }
    }
  }
  function go(i) {
    const chain = [];
    for (let k = cur; k >= 0; k = A.up[k]) chain.unshift(k);
    const k = i < 0 ? -1 : chain.indexOf(i), upward = !pair && cur >= 0 && i !== cur && (i < 0 || k >= 0);
    way = upward ? 'up' : 'down';
    restore = upward ? chain[k + 1] : -1;
    const h = i < 0 ? '' : '#' + encodeURIComponent(A.id[i]);
    if (location.hash !== h) location.hash = h; else { cur = i; pair = null; render(true); }
  }
  function route() {
    const key = decodeURIComponent(location.hash.slice(1));
    const ab = key.startsWith('ties=') ? key.slice(5).split(',').map(s => byKey.get(s)) : null;
    pair = ab && ab.length === 2 && ab.every(l => l !== undefined) ? ab : null;
    if (key.startsWith('find=')) { find.value = key.slice(5); query = find.value.trim().toLowerCase(); }
    cur = !pair && key && byId.has(key) ? byId.get(key) : -1;
    window.scrollTo(0, 0);
    render(true);
    restore = -1;
    way = 'down';
  }
  document.addEventListener('click', e => {
    const el = e.target.closest('[data-go], [data-pair]');
    if (!el || e.target.closest('a.ln, a.code')) return;
    e.preventDefault();
    if (query) { query = ''; find.value = ''; }
    if (el.dataset.pair) {
      const [a, b] = el.dataset.pair.split(',').map(Number);
      location.hash = '#ties=' + A.layers[a][0] + ',' + A.layers[b][0];
    } else go(+el.dataset.go);
  });
  let timer = 0;
  find.addEventListener('input', () => {
    clearTimeout(timer);
    timer = setTimeout(() => { query = find.value.trim().toLowerCase(); render(); }, 120);
  });
  const focusable = () => [...lens.querySelectorAll('[data-go], [data-pair]')];
  document.addEventListener('keydown', e => {
    const typing = e.target === find, list = focusable(), k = list.indexOf(document.activeElement);
    if (e.key === '/' && !typing) { e.preventDefault(); find.focus(); find.select(); }
    else if (e.key === 'Escape') {
      if (find.value) { find.value = ''; query = ''; render(true); } else if (pair) location.hash = ''; else if (cur >= 0) go(A.up[cur]);
      if (typing) find.blur();
    } else if (typing) {
      if ((e.key === 'ArrowDown' || e.key === 'Enter') && find.value.trim()) {
        e.preventDefault();
        clearTimeout(timer);
        query = find.value.trim().toLowerCase();
        render(false);
        const first = focusable()[0];
        if (first && e.key === 'Enter') first.click(); else if (first) first.focus();
      }
    } else if (e.target.classList && e.target.classList.contains('row') && e.key === 'Enter') { e.preventDefault(); e.target.click(); }
    else if (['ArrowDown', 'ArrowUp', 'Home', 'End'].includes(e.key) && list.length) {
      e.preventDefault();
      const next = e.key === 'Home' ? 0 : e.key === 'End' ? list.length - 1
        : Math.max(0, Math.min(list.length - 1, k + (e.key === 'ArrowDown' ? 1 : -1)));
      list[next].focus();
      list[next].scrollIntoView({block: 'nearest'});
    } else if ((e.key === 'ArrowRight' || e.key === 'Enter') && k >= 0) { e.preventDefault(); list[k].click(); }
    else if ((e.key === 'ArrowLeft' || e.key === 'Backspace') && (cur >= 0 || pair)) {
      e.preventDefault();
      if (pair) location.hash = ''; else go(A.up[cur]);
    }
  });
  tower.addEventListener('transitionend', e => { if (e.propertyName === 'transform') drawWires(); });
  window.addEventListener('resize', drawWires);
  window.addEventListener('hashchange', route);
  if (document.fonts) document.fonts.ready.then(drawWires);
  route();
})();
