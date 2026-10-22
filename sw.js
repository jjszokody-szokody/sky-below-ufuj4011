/* Sky Below service worker: cache the whole app on first load, serve it offline. */
const CACHE = 'skybelow-pages-v5';
const ASSETS = ['./', './index.html', './app.js', './style.css', './manifest.webmanifest', './icons/icon.svg', './icons/icon-192.png', './icons/icon-512.png', './data/geo.js', './data/airports.js', './data/airlines.js', './data/ey13.js'];
// Reject tunnel interstitials / error pages masquerading as 200s (e.g. a free-tunnel warning screen).
async function valid(req, r) {
  if (!r || !r.ok || r.type === 'opaque') return false;
  const ct = r.headers.get('content-type') || '', p = new URL(req.url).pathname;
  if (/\.(js|css|json|webmanifest|png|svg)$/.test(p)) return !ct.includes('text/html');
  if (ct.includes('text/html')) { const t = await r.clone().text(); return t.includes('Sky Below') && !t.includes('pinggy.web.debugger'); }
  return true;
}
self.addEventListener('install', e => { e.waitUntil((async () => {
  const c = await caches.open(CACHE);
  for (const a of ASSETS) { const req = new Request(a, { cache: 'reload', credentials: 'same-origin' }); const r = await fetch(req); if (!(await valid(req, r))) throw new Error('bad asset ' + a); await c.put(req, r); }
  await self.skipWaiting();
})()); });
self.addEventListener('activate', e => { e.waitUntil(caches.keys().then(ks => Promise.all(ks.filter(k => k !== CACHE).map(k => caches.delete(k)))).then(() => self.clients.claim())); });
self.addEventListener('fetch', e => {
  const u = new URL(e.request.url);
  if (e.request.method !== 'GET' || u.origin !== location.origin || u.pathname.includes('/api/')) return; // live data: network only
  e.respondWith((async () => {
    const hit = await caches.match(e.request, { ignoreSearch: true }) || (e.request.mode === 'navigate' ? await caches.match('./index.html') : null);
    const net = fetch(e.request).then(async r => { if (await valid(e.request, r)) { const c = await caches.open(CACHE); await c.put(e.request, r.clone()); } return r; });
    if (hit) { e.waitUntil(net.catch(() => {})); return hit; } // cache-first, refresh in background
    try { return await net; } catch (err) { return Response.error(); }
  })());
});
