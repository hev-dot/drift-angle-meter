// Minimal static file server for local development (ES modules need http://, and
// phone sensors need https://).
//   node tools/serve.js [port]            http on localhost
//   node tools/serve.js [port] --https    https for phones on the same Wi-Fi
//                                         (create the certificate first: tools/make-cert.ps1)
import { createServer as createHttp } from 'node:http';
import { createServer as createHttps } from 'node:https';
import { readFile } from 'node:fs/promises';
import { readFileSync, existsSync } from 'node:fs';
import { networkInterfaces } from 'node:os';
import { extname, join, normalize, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(fileURLToPath(new URL('..', import.meta.url)));
const args = process.argv.slice(2);
const https = args.includes('--https');
const port = Number(args.find((a) => /^\d+$/.test(a)) || (https ? 8443 : 8080));
const types = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.json': 'application/json', '.svg': 'image/svg+xml',
  '.png': 'image/png',
};

async function handler(req, res) {
  const path = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  const file = normalize(join(root, path.endsWith('/') ? path + 'index.html' : path));
  if (!file.startsWith(root) || file.endsWith('.pfx')) { res.writeHead(403).end(); return; }
  try {
    const body = await readFile(file);
    res.writeHead(200, { 'content-type': types[extname(file)] || 'application/octet-stream', 'cache-control': 'no-cache' }).end(body);
  } catch {
    res.writeHead(404).end('not found');
  }
}

let server;
if (https) {
  const pfx = join(root, 'tools', 'dev-cert.pfx');
  if (!existsSync(pfx)) {
    console.error('No tools/dev-cert.pfx. Create it with: powershell -ExecutionPolicy Bypass -File tools/make-cert.ps1');
    process.exit(1);
  }
  server = createHttps({ pfx: readFileSync(pfx), passphrase: 'drift-dev' }, handler);
} else {
  server = createHttp(handler);
}

server.listen(port, () => {
  const scheme = https ? 'https' : 'http';
  console.log(`serving ${root}`);
  console.log(`  ${scheme}://localhost:${port}/`);
  if (https) {
    for (const list of Object.values(networkInterfaces())) {
      for (const a of list) if (a.family === 'IPv4' && !a.internal) console.log(`  ${scheme}://${a.address}:${port}/   (phone on the same Wi-Fi)`);
    }
    console.log('The certificate is self-signed: accept the browser warning on the phone once.');
  }
});
