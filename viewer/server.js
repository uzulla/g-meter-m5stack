#!/usr/bin/env node
// G メーターログビューワー。依存なし。
// 使い方: node viewer/server.js [ログのディレクトリ(既定: viewer/logs)] [PORT(既定: 3000)]
const http = require('http');
const fs = require('fs');
const path = require('path');

const logDir = path.resolve(process.argv[2] || path.join(__dirname, 'logs'));
const port = Number(process.argv[3] || process.env.PORT || 3000);
const publicDir = path.join(__dirname, 'public');

const types = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8' };

function send(res, code, body, type = 'text/plain; charset=utf-8') {
  res.writeHead(code, { 'Content-Type': type });
  res.end(body);
}

const server = http.createServer((req, res) => {
  const url = new URL(req.url, 'http://localhost');

  if (url.pathname === '/api/files') {
    let files = [];
    try {
      files = fs.readdirSync(logDir)
        .filter((f) => f.toLowerCase().endsWith('.csv'))
        .sort()
        .map((f) => ({ name: f, size: fs.statSync(path.join(logDir, f)).size }));
    } catch (e) { /* ディレクトリがなければ空 */ }
    return send(res, 200, JSON.stringify(files), 'application/json');
  }

  if (url.pathname === '/api/file') {
    const name = path.basename(url.searchParams.get('name') || '');
    const p = path.join(logDir, name);
    if (!name.toLowerCase().endsWith('.csv') || !fs.existsSync(p)) return send(res, 404, 'not found');
    return send(res, 200, fs.readFileSync(p), 'text/csv; charset=utf-8');
  }

  const rel = url.pathname === '/' ? 'index.html' : path.normalize(url.pathname).replace(/^(\.\.[/\\])+/, '');
  const p = path.join(publicDir, rel);
  if (!p.startsWith(publicDir) || !fs.existsSync(p) || !fs.statSync(p).isFile()) return send(res, 404, 'not found');
  send(res, 200, fs.readFileSync(p), types[path.extname(p)] || 'application/octet-stream');
});

server.listen(port, '127.0.0.1', () => {
  console.log(`G-meter log viewer: http://localhost:${port}/  (logs: ${logDir})`);
});
