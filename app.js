/* Sky Below — offline-first in-flight companion. No external libraries. */
(() => {
'use strict';
const $ = s => document.querySelector(s), $$ = s => [...document.querySelectorAll(s)];
const R = 6371, D2R = Math.PI / 180, KT = 1.852;
const VERSION = '1.0.0';
const LS = { get(k, d) { try { const v = localStorage.getItem('sb.' + k); return v == null ? d : JSON.parse(v); } catch (e) { return d; } }, set(k, v) { try { localStorage.setItem('sb.' + k, JSON.stringify(v)); } catch (e) {} } };

/* ---------------- geometry ---------------- */
function hav(a, b, c, d) { const p1 = a * D2R, p2 = c * D2R, dp = p2 - p1, dl = (d - b) * D2R; const h = Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2; return 2 * R * Math.asin(Math.min(1, Math.sqrt(h))); }
function bearing(a, b, c, d) { const p1 = a * D2R, p2 = c * D2R, dl = (d - b) * D2R; const y = Math.sin(dl) * Math.cos(p2), x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl); return (Math.atan2(y, x) / D2R + 360) % 360; }
function interp(a, b, c, d, f) { const p1 = a * D2R, l1 = b * D2R, p2 = c * D2R, l2 = d * D2R, dd = hav(a, b, c, d) / R; if (dd < 1e-9) return [a, b]; const A = Math.sin((1 - f) * dd) / Math.sin(dd), B = Math.sin(f * dd) / Math.sin(dd); const x = A * Math.cos(p1) * Math.cos(l1) + B * Math.cos(p2) * Math.cos(l2), y = A * Math.cos(p1) * Math.sin(l1) + B * Math.cos(p2) * Math.sin(l2), z = A * Math.sin(p1) + B * Math.sin(p2); return [Math.atan2(z, Math.hypot(x, y)) / D2R, Math.atan2(y, x) / D2R]; }
function densify(wps, step = 25) { const out = [wps[0].slice(0, 2)]; for (let i = 0; i < wps.length - 1; i++) { const a = wps[i], b = wps[i + 1], n = Math.max(1, Math.round(hav(a[0], a[1], b[0], b[1]) / step)); for (let k = 1; k <= n; k++) out.push(interp(a[0], a[1], b[0], b[1], k / n)); } return out; }
function cumul(p) { const c = [0]; for (let i = 1; i < p.length; i++) c.push(c[i - 1] + hav(p[i - 1][0], p[i - 1][1], p[i][0], p[i][1])); return c; }
const wrap = x => ((x + 540) % 360) - 180;
function project(f, lat, lon) { // nearest point on path → {along, xd, i, side}
  const P = f.path; let best = 1e12, bi = 0;
  for (let i = 0; i < P.length; i += 4) { const d = hav(lat, lon, P[i][0], P[i][1]); if (d < best) { best = d; bi = i; } }
  for (let i = Math.max(0, bi - 4); i <= Math.min(P.length - 1, bi + 4); i++) { const d = hav(lat, lon, P[i][0], P[i][1]); if (d < best) { best = d; bi = i; } }
  let along = f.cum[bi];
  const j = Math.min(P.length - 1, bi + 1), i0 = j === bi ? bi - 1 : bi; let side = 0;
  if (i0 >= 0) { const segB = bearing(P[i0][0], P[i0][1], P[j][0], P[j][1]), toB = bearing(P[bi][0], P[bi][1], lat, lon); const rel = wrap(toB - segB); side = rel > 0 ? 1 : -1; const segLen = f.cum[j] - f.cum[i0]; if (bi < P.length - 1) along += Math.max(0, Math.min(segLen, best * Math.cos(rel * D2R))); }
  return { along, xd: best, i: bi, side };
}
function pointAt(f, along) { const c = f.cum, P = f.path; if (along <= 0) return { lat: P[0][0], lon: P[0][1], trk: bearing(P[0][0], P[0][1], P[1][0], P[1][1]) }; if (along >= f.total) { const n = P.length; return { lat: P[n - 1][0], lon: P[n - 1][1], trk: bearing(P[n - 2][0], P[n - 2][1], P[n - 1][0], P[n - 1][1]) }; } let lo = 0, hi = c.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (c[m] <= along) lo = m; else hi = m; } const fr = (along - c[lo]) / Math.max(1e-6, c[hi] - c[lo]); const [lat, lon] = interp(P[lo][0], P[lo][1], P[hi][0], P[hi][1], fr); return { lat, lon, trk: bearing(P[lo][0], P[lo][1], P[hi][0], P[hi][1]) }; }

/* ---------------- geo lookup (Natural Earth, embedded) ---------------- */
function pipFlat(r, x, y) { let ins = false; for (let i = 0, j = r.length - 2; i < r.length; j = i, i += 2) { const xi = r[i], yi = r[i + 1], xj = r[j], yj = r[j + 1]; if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) ins = !ins; } return ins; }
function findIn(list, lat, lon) { const x = Math.round(lon * 100), y = Math.round(lat * 100); for (const f of list) { const b = f.b; if (x < b[0] || x > b[2] || y < b[1] || y > b[3]) continue; for (const r of f.r) if (pipFlat(r, x, y)) return f; } return null; }
function nearestIn(list, lat, lon, maxDeg) { const x = lon * 100, y = lat * 100; let best = null, bd = maxDeg * 100; for (const f of list) { const b = f.b; if (x < b[0] - bd || x > b[2] + bd || y < b[1] - bd || y > b[3] + bd) continue; for (const r of f.r) for (let i = 0; i < r.length; i += 2) { const d = Math.hypot((r[i] - x) * Math.cos(lat * D2R), r[i + 1] - y); if (d < bd) { bd = d; best = f; } } } return best ? { f: best, km: bd / 100 * 111 } : null; }
function oceanName(lat, lon) { if (lat > 66) return 'Arctic Ocean'; if (lat < -60) return 'Southern Ocean'; if (lon > -100 && lon < 20 && lat > -60) return lat >= 0 ? 'North Atlantic Ocean' : 'South Atlantic Ocean'; if (lon >= 20 && lon < 147 && lat < 30) return 'Indian Ocean'; return lat >= 0 ? 'North Pacific Ocean' : 'South Pacific Ocean'; }
function whereAt(lat, lon) {
  const c = findIn(GEO.countries, lat, lon);
  if (c) { const lake = findIn(GEO.lakes, lat, lon); const a1 = findIn(GEO.admin1, lat, lon); return { land: true, country: c.n, a2: c.a2, admin1: a1 ? a1.n : null, lake: lake ? lake.n : null }; }
  const lake = findIn(GEO.lakes, lat, lon); if (lake) return { land: false, water: lake.n };
  const m = findIn(GEO.marine, lat, lon); const near = nearestIn(GEO.countries, lat, lon, 4);
  return { land: false, water: m ? m.n : oceanName(lat, lon), nearCountry: near ? near.f.n : null, nearKm: near ? near.km : null };
}
/* time zones: country → IANA zone (single-zone countries), US/Canada by state/province, else nautical (lon/15) */
const TZC = {AE:'Asia/Dubai',SA:'Asia/Riyadh',QA:'Asia/Qatar',BH:'Asia/Bahrain',KW:'Asia/Kuwait',IQ:'Asia/Baghdad',IR:'Asia/Tehran',SY:'Asia/Damascus',JO:'Asia/Amman',IL:'Asia/Jerusalem',PS:'Asia/Hebron',LB:'Asia/Beirut',TR:'Europe/Istanbul',GE:'Asia/Tbilisi',AM:'Asia/Yerevan',AZ:'Asia/Baku',OM:'Asia/Muscat',YE:'Asia/Aden',EG:'Africa/Cairo',CY:'Asia/Nicosia',GR:'Europe/Athens',BG:'Europe/Sofia',RO:'Europe/Bucharest',MD:'Europe/Chisinau',UA:'Europe/Kyiv',RS:'Europe/Belgrade',XK:'Europe/Belgrade',MK:'Europe/Skopje',AL:'Europe/Tirane',ME:'Europe/Podgorica',BA:'Europe/Sarajevo',HR:'Europe/Zagreb',SI:'Europe/Ljubljana',HU:'Europe/Budapest',SK:'Europe/Bratislava',CZ:'Europe/Prague',AT:'Europe/Vienna',PL:'Europe/Warsaw',DE:'Europe/Berlin',NL:'Europe/Amsterdam',BE:'Europe/Brussels',LU:'Europe/Luxembourg',FR:'Europe/Paris',CH:'Europe/Zurich',LI:'Europe/Vaduz',IT:'Europe/Rome',MT:'Europe/Malta',ES:'Europe/Madrid',PT:'Europe/Lisbon',GB:'Europe/London',IM:'Europe/Isle_of_Man',IE:'Europe/Dublin',IS:'Atlantic/Reykjavik',NO:'Europe/Oslo',SE:'Europe/Stockholm',DK:'Europe/Copenhagen',FI:'Europe/Helsinki',EE:'Europe/Tallinn',LV:'Europe/Riga',LT:'Europe/Vilnius',BY:'Europe/Minsk',FO:'Atlantic/Faroe',GL:'America/Nuuk',IN:'Asia/Kolkata',PK:'Asia/Karachi',AF:'Asia/Kabul',CN:'Asia/Shanghai',JP:'Asia/Tokyo',KR:'Asia/Seoul',KP:'Asia/Pyongyang',TH:'Asia/Bangkok',VN:'Asia/Ho_Chi_Minh',SG:'Asia/Singapore',MY:'Asia/Kuala_Lumpur',PH:'Asia/Manila',HK:'Asia/Hong_Kong',TW:'Asia/Taipei',NP:'Asia/Kathmandu',BD:'Asia/Dhaka',LK:'Asia/Colombo',UZ:'Asia/Tashkent',TM:'Asia/Ashgabat',TJ:'Asia/Dushanbe',KG:'Asia/Bishkek',CU:'America/Havana',BS:'America/Nassau',JM:'America/Jamaica',DO:'America/Santo_Domingo',HT:'America/Port-au-Prince',PR:'America/Puerto_Rico',CO:'America/Bogota',PE:'America/Lima',VE:'America/Caracas',PA:'America/Panama',CR:'America/Costa_Rica',GT:'America/Guatemala',HN:'America/Tegucigalpa',SV:'America/El_Salvador',NI:'America/Managua',BZ:'America/Belize',MA:'Africa/Casablanca',DZ:'Africa/Algiers',TN:'Africa/Tunis',LY:'Africa/Tripoli',SD:'Africa/Khartoum',ET:'Africa/Addis_Ababa',KE:'Africa/Nairobi',NG:'Africa/Lagos',ZA:'Africa/Johannesburg',NZ:'Pacific/Auckland',BM:'Atlantic/Bermuda'};
const TZUS = {E:'America/New_York',C:'America/Chicago',M:'America/Denver',P:'America/Los_Angeles'};
const USZ = {Connecticut:'E',Delaware:'E','District of Columbia':'E',Florida:'E',Georgia:'E',Indiana:'E',Kentucky:'E',Maine:'E',Maryland:'E',Massachusetts:'E',Michigan:'E','New Hampshire':'E','New Jersey':'E','New York':'E','North Carolina':'E',Ohio:'E',Pennsylvania:'E','Rhode Island':'E','South Carolina':'E',Vermont:'E',Virginia:'E','West Virginia':'E',Alabama:'C',Arkansas:'C',Illinois:'C',Iowa:'C',Kansas:'C',Louisiana:'C',Minnesota:'C',Mississippi:'C',Missouri:'C',Nebraska:'C','North Dakota':'C',Oklahoma:'C','South Dakota':'C',Texas:'C',Wisconsin:'C',Colorado:'M',Idaho:'M',Montana:'M','New Mexico':'M',Utah:'M',Wyoming:'M',California:'P',Nevada:'P',Oregon:'P',Washington:'P'};
const CAZ = {'Nova Scotia':'America/Halifax','New Brunswick':'America/Moncton','Prince Edward Island':'America/Halifax','Québec':'America/Toronto','Quebec':'America/Toronto',Ontario:'America/Toronto',Manitoba:'America/Winnipeg',Saskatchewan:'America/Regina',Alberta:'America/Edmonton','British Columbia':'America/Vancouver',Yukon:'America/Whitehorse','Northwest Territories':'America/Yellowknife'};
function tzAt(lat, lon, w) {
  w = w || whereAt(lat, lon);
  if (w.land) {
    if (w.a2 === 'US' && w.admin1) { if (w.admin1 === 'Arizona') return { tz: 'America/Phoenix' }; if (w.admin1 === 'Alaska') return { tz: 'America/Anchorage' }; if (w.admin1 === 'Hawaii') return { tz: 'Pacific/Honolulu' }; if (w.admin1 === 'Tennessee') return { tz: lon > -86.3 ? TZUS.E : TZUS.C }; if (USZ[w.admin1]) return { tz: TZUS[USZ[w.admin1]] }; }
    if (w.a2 === 'CA' && w.admin1) { if (/Newfoundland/.test(w.admin1)) return { tz: lat > 51.6 && lon < -57.1 ? 'America/Goose_Bay' : 'America/St_Johns' }; if (w.admin1 === 'Nunavut') return { tz: lon > -85 ? 'America/Iqaluit' : 'America/Rankin_Inlet' }; if (CAZ[w.admin1]) return { tz: CAZ[w.admin1] }; }
    if (TZC[w.a2]) return { tz: TZC[w.a2] };
  }
  return { off: Math.round(lon / 15), approx: true };
}
function fmtLocal(t, z, withDay) {
  try { if (z.tz) return new Intl.DateTimeFormat([], { hour: '2-digit', minute: '2-digit', weekday: withDay ? 'short' : undefined, timeZone: z.tz }).format(t) + ' ' + tzAbbr(t, z.tz); } catch (e) {}
  const off = z.off != null ? z.off : 0; const d = new Date(t + off * 3600e3);
  return String(d.getUTCHours()).padStart(2, '0') + ':' + String(d.getUTCMinutes()).padStart(2, '0') + ' (UTC' + (off >= 0 ? '+' : '') + off + ', approx)';
}
function tzAbbr(t, tz) { try { const p = new Intl.DateTimeFormat('en-US', { timeZone: tz, timeZoneName: 'short' }).formatToParts(t).find(x => x.type === 'timeZoneName'); return p ? p.value : ''; } catch (e) { return ''; } }

/* ---------------- data tables ---------------- */
const AP = { byI: new Map(), byC: new Map() };
for (const a of AIRPORTS) { const o = { iata: a[0], icao: a[1], name: a[2], city: a[3], cc: a[4], lat: a[5], lon: a[6] }; if (a[0]) AP.byI.set(a[0], o); if (a[1]) AP.byC.set(a[1], o); }
const airport = code => { code = String(code || '').toUpperCase().trim(); return AP.byI.get(code) || AP.byC.get(code) || null; };
const ICAO2IATA = {}; for (const k in AIRLINES) ICAO2IATA[AIRLINES[k][0]] = k;

const CAT = { city: '🏙️', town: '🏘️', capital: '🏛️', unesco: '🏛️', ruins: '🏺', battle: '⚔️', mountain: '⛰️', volcano: '🌋', river: '〰️', lake: '💧', sea: '🌊', glacier: '🧊', island: '🏝️', park: '🌲', religious: '🛕', castle: '🏰', region: '🗺️', airport: '✈️', desert: '🏜️', landmark: '📍', other: '📍' };
const CATNAME = { city: 'City', town: 'Town', capital: 'Capital city', unesco: 'UNESCO World Heritage', ruins: 'Ancient site', battle: 'Battle / historic event', mountain: 'Mountain', volcano: 'Volcano', river: 'River', lake: 'Lake', sea: 'Sea / ocean feature', glacier: 'Glacier / ice', island: 'Island', park: 'National park / nature', religious: 'Religious site', castle: 'Castle / palace', region: 'Region', airport: 'Airport', desert: 'Desert', landmark: 'Landmark', other: 'Place' };
function catOf(types, name) {
  const t = (types || '').toLowerCase() + ' ' + (name || '').toLowerCase();
  const has = (...w) => w.some(x => t.includes(x));
  if (has('battle', 'siege', 'massacre', 'war ', 'campaign', 'offensive')) return 'battle';
  if (has('archaeological', 'ancient city', 'tell ', 'ruins', 'ancient', 'necropolis', 'ziggurat', 'temple complex')) return 'ruins';
  if (has('volcano', 'stratovolcano')) return 'volcano';
  if (has('glacier', 'ice cap', 'ice sheet')) return 'glacier';
  if (has('mountain', 'massif', 'peak', 'summit', 'mountain range', 'hill')) return 'mountain';
  if (has('river', 'tributary', 'canal', 'waterfall', 'stream')) return 'river';
  if (has('lake', 'reservoir', 'lagoon')) return 'lake';
  if (has('sea', 'bay', 'gulf', 'strait', 'fjord', 'ocean', 'sound', 'ridge', 'trench', 'seamount', 'fracture zone', 'sea basin')) return 'sea';
  if (has('island', 'archipelago', 'peninsula', 'cape', 'headland')) return 'island';
  if (has('desert')) return 'desert';
  if (has('national park', 'nature reserve', 'protected area', 'forest', 'biosphere')) return 'park';
  if (has('castle', 'palace', 'fortress', 'fort', 'citadel', 'chateau')) return 'castle';
  if (has('cathedral', 'church', 'mosque', 'monastery', 'abbey', 'shrine', 'basilica', 'synagogue', 'temple')) return 'religious';
  if (/\b(Province|County|Governorate|Oblast|District|Region|Voivodeship)\b/.test(name || '')) return 'region';
  if (has('capital')) return 'capital';
  if (has('big city', 'metropolis', 'million city', 'largest city', 'city', 'megacity', 'municipality of')) return 'city';
  if (has('town', 'village', 'municipality', 'commune', 'borough', 'district')) return 'town';
  if (has('airport')) return 'airport';
  if (has('region', 'province', 'state of', 'governorate', 'county', 'oblast', 'historical country', 'geographic region')) return 'region';
  if (has('museum', 'monument', 'memorial', 'tower', 'bridge', 'dam', 'stadium', 'square', 'building', 'site')) return 'landmark';
  return 'other';
}

/* ---------------- IndexedDB (fallback: localStorage) ---------------- */
const IDB = (() => { let dbp; const open = () => dbp || (dbp = new Promise((res, rej) => { try { const r = indexedDB.open('skybelow', 1); r.onupgradeneeded = () => r.result.createObjectStore('kv'); r.onsuccess = () => res(r.result); r.onerror = () => rej(r.error); } catch (e) { rej(e); } }));
  return { async get(k) { try { const db = await open(); return await new Promise((res, rej) => { const q = db.transaction('kv').objectStore('kv').get(k); q.onsuccess = () => res(q.result); q.onerror = () => rej(q.error); }); } catch (e) { return LS.get('idb.' + k, undefined); } },
    async set(k, v) { try { const db = await open(); return await new Promise((res, rej) => { const tx = db.transaction('kv', 'readwrite'); tx.objectStore('kv').put(v, k); tx.oncomplete = () => res(); tx.onerror = () => rej(tx.error); }); } catch (e) { LS.set('idb.' + k, v); } },
    async clear() { try { const db = await open(); await new Promise(res => { const tx = db.transaction('kv', 'readwrite'); tx.objectStore('kv').clear(); tx.oncomplete = res; tx.onerror = res; }); } catch (e) {} } }; })();

/* ---------------- state ---------------- */
const S = {
  flight: null, pos: null, live: null, gps: null, lastFix: null, trail: [],
  opt: Object.assign({ big: false, low: false, miles: false, source: 'auto', proxy: '' }, LS.get('opt', {})),
  offsetMin: LS.get('offset', 0), view: 'follow', zoom: 1, pan: null, online: navigator.onLine, liveVia: null, liveMsg: '', liveWiki: [], liveWikiAt: null, gpsWatch: null,
};
const units = { dist: km => S.opt.miles ? Math.round(km / 1.609).toLocaleString() + ' mi' : Math.round(km).toLocaleString() + ' km', short: km => km < 10 ? (S.opt.miles ? (km / 1.609).toFixed(1) + ' mi' : km.toFixed(1) + ' km') : units.dist(km) };
function fmtDur(min) { if (!isFinite(min)) return '—'; min = Math.max(0, Math.round(min)); const h = Math.floor(min / 60), m = min % 60; return h ? h + 'h ' + String(m).padStart(2, '0') + 'm' : m + 'm'; }

/* ---------------- flights ---------------- */
function buildFlight(o) {
  const from = o.from, to = o.to; const wps = o.waypoints && o.waypoints.length > 1 ? o.waypoints : [[from.lat, from.lon], [to.lat, to.lon]];
  const path = densify(wps, 25); const cum = cumul(path);
  const f = { key: o.key, label: o.label, callsign: o.callsign, airline: o.airline || '', from, to, path, cum, total: cum[cum.length - 1], depMs: o.depMs, durMin: o.durMin, taxiOut: o.taxiOut ?? 20, taxiIn: o.taxiIn ?? 8, builtIn: !!o.builtIn, routeSrc: o.routeSrc || '', aircraft: o.aircraft || '', note: o.note || '', pois: [] };
  setPois(f, o.pois || []); return f;
}
function setPois(f, pois) {
  f.pois = pois.map(p => { const pr = project(f, p.lat, p.lon); return Object.assign({}, p, { c: p.c || catOf(p.ty, p.n), along: pr.along, xd: pr.xd, side: pr.side }); }).filter(p => p.xd <= 320);
}
function ey13Flight() {
  const E = EY13; return buildFlight({ key: 'ETD13:AUH-ATL', label: 'EY 13', callsign: 'ETD13', airline: 'Etihad Airways', from: airport('AUH'), to: airport('ATL'), waypoints: E.waypoints, depMs: Date.parse(LS.get('ey13dep', E.depUTC)), durMin: E.durMin, taxiOut: E.taxiOut, builtIn: true, routeSrc: E.routeNote, aircraft: E.aircraft, pois: E.pois, note: E.note });
}
function saveRecent(entry) { const r = LS.get('recent', []).filter(x => x.key !== entry.key); r.unshift(entry); LS.set('recent', r.slice(0, 8)); renderRecent(); }

/* ---------------- position: schedule estimate ---------------- */
function estimate(f, now) {
  const tOff = f.depMs + (f.taxiOut + S.offsetMin) * 60e3; const T = Math.max(60, f.durMin - f.taxiOut - f.taxiIn); const t = (now - tOff) / 60e3;
  const anchor = S.lastFix && now - S.lastFix.t < 4 * 3600e3 && S.lastFix.flightKey === f.key ? S.lastFix : null;
  const v = f.total / (T - 20.5); // km/min cruise, with climb/descent profile
  let along, alt, gsKmMin, phase;
  if (anchor) { along = Math.min(f.total, anchor.along + (anchor.gsKmMin || v) * (now - anchor.t) / 60e3); alt = anchor.alt || 35000; gsKmMin = anchor.gsKmMin || v; phase = 'dead-reckoning from last fix'; if (along >= f.total) phase = 'landed (est.)'; }
  else if (t <= 0) { along = 0; alt = 0; gsKmMin = 0; phase = t < -f.taxiOut ? 'not departed' : 'taxiing'; }
  else if (t >= T) { along = f.total; alt = 0; gsKmMin = 0; phase = 'landed (est.)'; }
  else if (t < 25) { along = 0.6 * v * t * (t / 25) / 1 * 0.5 + 0.6 * v * t * 0.5; alt = 36000 * t / 25; gsKmMin = v * (0.35 + 0.65 * t / 25); phase = 'climbing'; }
  else if (t > T - 30) { const td = t - (T - 30); along = f.total - 0.65 * v * (30 - td); alt = 36000 * (30 - td) / 30; gsKmMin = v * (0.45 + 0.55 * (30 - td) / 30); phase = 'descending'; }
  else { along = 0.6 * v * 25 + v * (t - 25); alt = 36000; gsKmMin = v; phase = 'cruise'; }
  along = Math.max(0, Math.min(f.total, along));
  const p = pointAt(f, along); const arr = anchor ? now + (f.total - along) / Math.max(1, gsKmMin) * 60e3 : tOff + T * 60e3;
  return { lat: p.lat, lon: p.lon, trk: p.trk, alt: Math.round(alt / 100) * 100, gs: gsKmMin * 60 / KT, along, src: 'est', phase, t: now, arrMs: arr, takeoffMs: tOff, airMin: T };
}

/* ---------------- position: live ADS-B ---------------- */
function withTimeout(p, ms) { return Promise.race([p, new Promise((_, rej) => setTimeout(() => rej(new Error('timeout')), ms))]); }
async function fetchJSON(u, ms = 9000) { const o = { cache: 'no-store' }; if (/\.loca\.lt\//.test(u)) o.headers = { 'bypass-tunnel-reminder': '1' }; const r = await withTimeout(fetch(u, o), ms); if (!r.ok) throw new Error('HTTP ' + r.status); return r.json(); }
const NO_RELAY = !!window.SKY_NO_RELAY || /\.github\.io$/.test(location.hostname);
const NO_RELAY_MSG = 'Live tracking unavailable on this host; using GPS/schedule';
function adsbUrls(cs) {
  const list = []; if (S.opt.proxy) list.push(['proxy', S.opt.proxy.replace(/\/$/, '') + '?cs=' + cs]);
  if (/^https?:/.test(location.protocol) && !NO_RELAY) list.push(['box proxy', 'api/adsb?cs=' + cs], ['box proxy (adsb.fi)', 'api/adsb?src=fi&cs=' + cs]);
  list.push(['adsb.lol', 'https://api.adsb.lol/v2/callsign/' + cs], ['adsb.fi', 'https://opendata.adsb.fi/api/v2/callsign/' + cs]);
  if (S.liveVia) list.sort((a, b) => (b[0] === S.liveVia) - (a[0] === S.liveVia)); return list;
}
async function lookupLive(cs) {
  let lastErr = null, sawEmpty = false;
  for (const [name, u] of adsbUrls(cs)) {
    try { const d = await fetchJSON(u); const arr = d.ac || d.aircraft || []; S.liveVia = name; markOnline(true);
      const a = arr.find(x => (x.flight || '').trim().toUpperCase() === cs) || arr[0];
      if (!a) { sawEmpty = true; return { none: true, via: name }; }
      const lp = a.lastPosition || {}; const lat = a.lat ?? lp.lat, lon = a.lon ?? lp.lon; if (lat == null) return { none: true, via: name, seenNoPos: true };
      const age = a.lat != null ? (a.seen_pos ?? a.seen ?? 0) : (lp.seen_pos ?? 9999);
      return { via: name, lat, lon, alt: a.alt_baro === 'ground' ? 0 : (a.alt_baro ?? a.alt_geom ?? null), onGround: a.alt_baro === 'ground', gs: a.gs ?? (a.tas ?? null), trk: a.track ?? a.true_heading ?? null, age, hex: a.hex, reg: a.r, type: a.t, desc: a.desc, t: Date.now() - age * 1000 };
    } catch (e) { lastErr = e; }
  }
  return { error: lastErr ? lastErr.message : 'unreachable', sawEmpty };
}
async function pollLive() {
  const f = S.flight; if (!f || S.opt.source === 'gps' || S.opt.source === 'est') return;
  if (!navigator.onLine) { S.liveMsg = 'No internet — live tracking paused'; markOnline(false); return; }
  const r = await lookupLive(f.callsign); if (S.flight !== f) return;
  if (r.error) { if (NO_RELAY && !S.opt.proxy) { S.liveMsg = NO_RELAY_MSG; S.noRelay = true; } else { S.liveMsg = 'Live data unreachable (' + r.error + ')'; S.noRelay = false; } markOnline(false); }
  else if (r.none) { S.liveMsg = r.seenNoPos ? f.callsign + ' seen by ADS-B, no position yet' : f.callsign + ' not currently broadcasting (not airborne yet, or out of receiver range)'; }
  else if (f.builtIn && !S.opt.anyDay && (Date.now() < f.depMs - 90 * 60e3 || Date.now() > f.depMs + (f.durMin + 180) * 60e3)) { S.live = null; S.liveMsg = f.callsign + ' is airborne now, but that is another day\'s EY 13 (yours departs ' + fmtLocal(f.depMs, tzAt(f.from.lat, f.from.lon), true) + ') — ignored'; S.otherDay = r; }
  else { S.live = r; const pr = project(f, r.lat, r.lon); const gsKmMin = r.gs ? r.gs * KT / 60 : null;
    S.lastFix = { t: r.t, lat: r.lat, lon: r.lon, along: pr.along, alt: r.alt, gsKmMin, flightKey: f.key, src: 'live' }; S.liveMsg = 'Live via ' + r.via + (r.age > 60 ? ' · position ' + fmtDur(r.age / 60) + ' old' : '') + (r.type ? ' · ' + (r.desc || r.type) + (r.reg ? ' ' + r.reg : '') : '');
    S.trail.push([r.lat, r.lon]); if (S.trail.length > 2000) S.trail.shift(); LS.set('trail.' + f.key, S.trail.slice(-600)); }
  tick(true);
}
function markOnline(v) { S.online = v; const c = $('#netChip'); const nr = !v && navigator.onLine && S.noRelay; c.textContent = v ? 'online' : (nr ? 'no live data' : (navigator.onLine ? 'no data' : 'offline')); c.title = nr ? NO_RELAY_MSG : ''; c.className = 'chip ' + (v ? 'on' : 'off'); const nb = document.getElementById('noRelayNote'); if (nb) { nb.textContent = NO_RELAY_MSG + ' (set a relay URL in Settings to enable live ADS-B)'; nb.hidden = !(NO_RELAY && !S.opt.proxy && S.noRelay); } }

/* ---------------- position: phone GPS ---------------- */
function startGps() {
  if (!('geolocation' in navigator)) { status('This browser has no GPS access'); return; }
  if (S.gpsWatch != null) navigator.geolocation.clearWatch(S.gpsWatch);
  S.gpsWatch = navigator.geolocation.watchPosition(p => { const c = p.coords; const prev = S.gps; let trk = c.heading; if ((trk == null || isNaN(trk)) && prev && hav(prev.lat, prev.lon, c.latitude, c.longitude) > 2) trk = bearing(prev.lat, prev.lon, c.latitude, c.longitude);
    let gs = c.speed != null && !isNaN(c.speed) ? c.speed * 3.6 / KT : null; if (gs == null && prev && p.timestamp > prev.t) gs = hav(prev.lat, prev.lon, c.latitude, c.longitude) / ((p.timestamp - prev.t) / 3600e3) / KT;
    S.gps = { lat: c.latitude, lon: c.longitude, alt: c.altitude != null ? c.altitude * 3.281 : null, gs, trk, acc: c.accuracy, t: p.timestamp || Date.now() };
    const f = S.flight; if (f) { const pr = project(f, c.latitude, c.longitude); S.lastFix = { t: S.gps.t, lat: c.latitude, lon: c.longitude, along: pr.along, alt: S.gps.alt, gsKmMin: gs ? gs * KT / 60 : (S.lastFix && S.lastFix.gsKmMin), flightKey: f.key, src: 'gps' }; S.trail.push([c.latitude, c.longitude]); }
    LS.set('gpsOn', true); tick(true);
  }, e => { status('GPS: ' + e.message + (e.code === 1 ? ' (permission denied)' : '')); }, { enableHighAccuracy: !S.opt.low, maximumAge: S.opt.low ? 120000 : 15000, timeout: 120000 });
  $('#gpsBtn').textContent = 'GPS is on (waiting for fix…)'; status('GPS requested — hold phone near a window');
}

/* ---------------- choose position ---------------- */
function currentPos(now) {
  const f = S.flight, src = S.opt.source; const est = estimate(f, now);
  const liveOk = S.live && now - S.live.t < 15 * 60e3, gpsOk = S.gps && now - S.gps.t < 3 * 60e3;
  if ((src === 'auto' || src === 'live') && liveOk) { const L = S.live; const pr = project(f, L.lat, L.lon); let lat = L.lat, lon = L.lon;
    const dt = (now - L.t) / 3600e3; if (L.gs && L.trk != null && dt > 0 && dt < 0.25 && !L.onGround) { const d = L.gs * KT * dt; const p2 = destPoint(lat, lon, L.trk, d); lat = p2[0]; lon = p2[1]; }
    return { lat, lon, trk: L.trk ?? pointAt(f, pr.along).trk, alt: L.alt, gs: L.gs, along: project(f, lat, lon).along, xd: pr.xd, src: 'live', stale: now - L.t > 5 * 60e3, phase: L.onGround ? 'on ground' : 'airborne', t: now };
  }
  if ((src === 'auto' || src === 'gps') && gpsOk) { const G = S.gps; const pr = project(f, G.lat, G.lon); return { lat: G.lat, lon: G.lon, trk: G.trk ?? pointAt(f, pr.along).trk, alt: G.alt, gs: G.gs, along: pr.along, xd: pr.xd, src: 'gps', acc: G.acc, phase: 'GPS fix', t: now }; }
  return est;
}
function destPoint(lat, lon, brg, km) { const d = km / R, b = brg * D2R, p1 = lat * D2R, l1 = lon * D2R; const p2 = Math.asin(Math.sin(p1) * Math.cos(d) + Math.cos(p1) * Math.sin(d) * Math.cos(b)); const l2 = l1 + Math.atan2(Math.sin(b) * Math.sin(d) * Math.cos(p1), Math.cos(d) - Math.sin(p1) * Math.sin(p2)); return [p2 / D2R, wrap(l2 / D2R)]; }

/* ---------------- UI update ---------------- */
let lastLists = 0, lastWhere = null;
function tick(force) {
  const f = S.flight; if (!f) return; const now = Date.now(); const p = currentPos(now); S.pos = p;
  const chip = $('#srcChip'); const lbl = { live: 'LIVE ADS-B', gps: 'PHONE GPS', est: S.lastFix && p.phase && p.phase.startsWith('dead') ? 'EST · LAST FIX' : 'SCHEDULE EST' }[p.src];
  chip.textContent = lbl + (p.stale ? ' (stale)' : ''); chip.className = 'chip ' + (p.stale ? 'stale' : p.src);
  const offRoute = p.xd != null && p.xd > 150; const remKm = offRoute ? hav(p.lat, p.lon, f.to.lat, f.to.lon) : Math.max(0, f.total - p.along);
  $('#sAlt').textContent = p.alt == null ? '—' : (p.src === 'est' ? '~' : '') + Math.round(p.alt).toLocaleString() + ' ft';
  $('#sGs').textContent = p.gs ? (p.src === 'est' ? '~' : '') + Math.round(p.gs) + ' kt · ' + (S.opt.miles ? Math.round(p.gs * 1.15078) + ' mph' : Math.round(p.gs * KT) + ' km/h') : '—';
  let leftMin; if (p.src === 'est') leftMin = (p.arrMs - now) / 60e3; else leftMin = p.gs > 80 ? remKm / (p.gs * KT) * 60 + 10 : NaN;
  const arrMs = now + leftMin * 60e3;
  $('#sLeft').textContent = p.phase === 'not departed' ? 'dep in ' + fmtDur((p.takeoffMs - now) / 60e3) : (remKm < 3 ? 'arrived' : fmtDur(leftMin));
  $('#sDistLbl').textContent = 'To ' + (f.to.iata || f.to.icao); $('#sDist').textContent = units.dist(remKm);
  const w = whereAt(p.lat, p.lon); const z = tzAt(p.lat, p.lon, w); $('#sLocal').textContent = fmtLocal(now, z);
  const prog = Math.max(0, Math.min(1, 1 - remKm / f.total)); $('#sProg').textContent = Math.round(prog * 100) + '%' + (isFinite(arrMs) && remKm > 3 ? ' · ETA ' + fmtLocal(arrMs, tzAt(f.to.lat, f.to.lon)) : ''); $('#progBar').style.width = (prog * 100).toFixed(1) + '%';
  const main = w.land ? (w.lake ? w.lake + ', ' : '') + (w.admin1 ? w.admin1 + ', ' : '') + w.country : w.water;
  const near = nearestCity(p); const sub = [w.land ? '' : (w.nearCountry ? units.dist(w.nearKm) + ' from ' + w.nearCountry : 'open water'), near ? 'nearest city: ' + near.n + ' (' + units.short(near.d) + ')' : '', p.phase && p.src === 'est' ? p.phase : '', p.src === 'gps' && p.acc ? '±' + Math.round(p.acc) + ' m' : ''].filter(Boolean).join(' · ');
  $('#belowMain').textContent = main; $('#belowSub').textContent = sub;
  if (main !== lastWhere) { lastWhere = main; }
  const estInfo = $('#estInfo'); const e = estimate(f, now);
  estInfo.textContent = 'Takeoff ' + fmtLocal(e.takeoffMs, tzAt(f.from.lat, f.from.lon), true) + ' · land ~' + fmtLocal(e.takeoffMs + e.airMin * 60e3, tzAt(f.to.lat, f.to.lon), true) + (S.liveMsg ? ' · ' + S.liveMsg : '');
  if (force || now - lastLists > (S.opt.low ? 30000 : 8000)) { lastLists = now; renderLists(p); }
  drawMap();
}
function nearestCity(p) { let best = null; for (const q of S.flight.pois) { if (!['city', 'capital', 'town'].includes(q.c)) continue; const d = hav(p.lat, p.lon, q.lat, q.lon); if (!best || d < best.d) best = { n: q.n, d }; } if (best && best.d > 600) return null; return best; }
function relSide(p, q) { if (p.trk == null) return ''; const rb = wrap(bearing(p.lat, p.lon, q.lat, q.lon) - p.trk); const a = Math.abs(rb); if (a < 25) return 'ahead'; if (a > 155) return 'behind'; return rb > 0 ? 'right side' : 'left side'; }
function poiLi(q, right, sub) { const li = document.createElement('li'); li.innerHTML = '<span class="ic"></span><span class="tx"><div class="nm"></div><div class="sb"></div></span><span class="rt"></span>'; li.querySelector('.ic').textContent = CAT[q.c] || '📍'; const nm = li.querySelector('.nm'); nm.textContent = q.n; if (q.u) { const b = document.createElement('span'); b.className = 'badge'; b.textContent = 'UNESCO'; nm.appendChild(b); } li.querySelector('.sb').textContent = sub; li.querySelector('.rt').innerHTML = right; li.onclick = () => openPoi(q); return li; }
function renderLists(p) {
  const f = S.flight; const near = f.pois.map(q => ({ q, d: hav(p.lat, p.lon, q.lat, q.lon) })).filter(x => x.d < 450).sort((a, b) => a.d - b.d).slice(0, 15);
  const nl = $('#nearList'); nl.textContent = '';
  if (!near.length) nl.innerHTML = '<li class="empty"><span class="tx"><div class="nm">Nothing catalogued within 450 km</div><div class="sb">Open water or remote terrain — check “Coming up”.</div></span></li>';
  for (const { q, d } of near) nl.appendChild(poiLi(q, '<b>' + units.short(d) + '</b>' + relSide(p, q), (CATNAME[q.c] || '') + (q.b ? ' · ' + q.b.slice(0, 90) : '')));
  const speedKmMin = p.gs > 80 ? p.gs * KT / 60 : f.total / Math.max(60, f.durMin - 50);
  const up = f.pois.filter(q => q.along > p.along + 10 && q.xd < 260).sort((a, b) => a.along - b.along);
  // keep the list interesting: up to 25 items, preferring notable ones in dense stretches
  const picked = []; let lastA = -1e9; for (const q of up) { if (picked.length >= 25) break; if (q.along - lastA < 15 && (q.s || 0) < 60) continue; picked.push(q); lastA = q.along; }
  const xl = $('#nextList'); xl.textContent = '';
  if (!picked.length) xl.innerHTML = '<li class="empty"><span class="tx"><div class="nm">No more catalogued places ahead</div></span></li>';
  for (const q of picked) { const mins = (q.along - p.along) / speedKmMin; xl.appendChild(poiLi(q, '<b>' + fmtDur(mins) + '</b>' + (q.xd < 20 ? 'under you' : units.short(q.xd) + ' ' + (q.side > 0 ? 'right' : 'left')), (CATNAME[q.c] || '') + (q.b ? ' · ' + q.b.slice(0, 90) : ''))); }
}
function openPoi(q) { $('#poiTitle').textContent = (CAT[q.c] || '📍') + ' ' + q.n; const p = S.pos; const d = p ? hav(p.lat, p.lon, q.lat, q.lon) : null;
  $('#poiMeta').textContent = [CATNAME[q.c], q.u ? 'UNESCO World Heritage Site' : '', d != null ? units.short(d) + ' away' + (p ? ', ' + relSide(p, q) : '') : '', q.lat.toFixed(2) + '°, ' + q.lon.toFixed(2) + '°'].filter(Boolean).join(' · ');
  $('#poiBlurb').textContent = q.b || 'No offline description saved for this place.'; const a = $('#poiLink'); if (q.w) { a.href = 'https://en.wikipedia.org/wiki/' + encodeURIComponent(q.w.replace(/ /g, '_')); a.style.display = ''; } else a.style.display = 'none'; show('#poiSheet'); }

/* ---------------- map (canvas, offline) ---------------- */
const cv = $('#map'), ctx = cv.getContext('2d'); let W = 0, H = 0, DPR = 1, view = { lat: 0, lon: 0, k: 10 };
function resize() { DPR = Math.min(2, window.devicePixelRatio || 1); const r = cv.getBoundingClientRect(); W = r.width; H = r.height; cv.width = Math.round(W * DPR); cv.height = Math.round(H * DPR); drawMap(); }
function computeView() {
  const f = S.flight, p = S.pos; if (!f || !p) return;
  if (S.view === 'route') { let mnla = 90, mxla = -90; const ref = f.path[Math.floor(f.path.length / 2)][1]; let mnlo = 1e9, mxlo = -1e9; for (const q of f.path) { mnla = Math.min(mnla, q[0]); mxla = Math.max(mxla, q[0]); const lo = ref + wrap(q[1] - ref); mnlo = Math.min(mnlo, lo); mxlo = Math.max(mxlo, lo); } const clat = (mnla + mxla) / 2, cl = Math.cos(clat * D2R); const k = Math.min((W - 30) / Math.max(1, (mxlo - mnlo) * cl), (H - 70) / Math.max(1, mxla - mnla)) * S.zoom; view = { lat: clat + 3 / S.zoom, lon: wrap((mnlo + mxlo) / 2), k }; }
  else if (S.view === 'free' && S.pan) view = Object.assign({}, S.pan);
  else { const k = Math.min(W, H) / 16 * S.zoom; view = { lat: p.lat, lon: p.lon, k }; }
}
function xy(lat, lon) { const cl = Math.cos(view.lat * D2R); return [W / 2 + wrap(lon - view.lon) * cl * view.k, H / 2 - (lat - view.lat) * view.k]; }
function drawRings(list, fill, stroke, lw) {
  const cl = Math.cos(view.lat * D2R); const halfLon = W / 2 / (cl * view.k) + 2, halfLat = H / 2 / view.k + 2;
  ctx.beginPath();
  for (const f of list) { const b = f.b; if (b[3] / 100 < view.lat - halfLat || b[1] / 100 > view.lat + halfLat) continue;
    for (const r of f.r) { const shift = wrap(r[0] / 100 - view.lon) - (r[0] / 100 - view.lon); // consistent per ring
      const x0 = (r[0] / 100 - view.lon + shift); if (Math.abs(x0) > 200) continue;
      for (let i = 0; i < r.length; i += 2) { const x = W / 2 + (r[i] / 100 - view.lon + shift) * cl * view.k, y = H / 2 - (r[i + 1] / 100 - view.lat) * view.k; if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); } ctx.closePath(); } }
  if (fill) { ctx.fillStyle = fill; ctx.fill('evenodd'); } if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); }
}
function polyline(pts, color, lw, dash) { ctx.beginPath(); let prev = null; for (const q of pts) { const [x, y] = xy(q[0], q[1]); if (prev && Math.abs(x - prev[0]) > W) ctx.moveTo(x, y); else if (!prev) ctx.moveTo(x, y); else ctx.lineTo(x, y); prev = [x, y]; } ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.setLineDash(dash || []); ctx.stroke(); ctx.setLineDash([]); }
let drawPending = false;
function drawMap() { if (drawPending) return; drawPending = true; (S.opt.low ? (fn => setTimeout(fn, 0)) : requestAnimationFrame)(() => { drawPending = false; reallyDraw(); }); }
function reallyDraw() {
  const f = S.flight, p = S.pos; if (!f || !p || !W) return; computeView(); ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  ctx.fillStyle = '#081426'; ctx.fillRect(0, 0, W, H);
  // graticule
  ctx.strokeStyle = 'rgba(56,189,248,.07)'; ctx.lineWidth = 1; ctx.beginPath(); const g = view.k > 40 ? 5 : view.k > 12 ? 10 : 15;
  for (let la = -90; la <= 90; la += g) { const [, y] = xy(la, view.lon); ctx.moveTo(0, y); ctx.lineTo(W, y); }
  for (let lo = -180; lo < 180; lo += g) { const [x] = xy(view.lat, lo); ctx.moveTo(x, 0); ctx.lineTo(x, H); } ctx.stroke();
  drawRings(GEO.countries, '#14243d', '#2d4a72', 0.8);
  if (view.k > 20) drawRings(GEO.admin1, null, 'rgba(120,150,200,.18)', 0.6);
  drawRings(GEO.lakes, '#0b1a31', null, 0);
  // path
  const cur = pointAt(f, p.along); const flown = [], rem = [];
  for (let i = 0; i < f.path.length; i++) (f.cum[i] <= p.along ? flown : rem).push(f.path[i]);
  flown.push([cur.lat, cur.lon]); rem.unshift([cur.lat, cur.lon]);
  polyline(rem, 'rgba(56,189,248,.7)', 2, [6, 6]); polyline(flown, '#f59e0b', 2.5);
  if (S.trail.length > 1) polyline(S.trail, '#22c55e', 2);
  if (p.xd > 150) polyline([[p.lat, p.lon], [f.to.lat, f.to.lon]], 'rgba(34,197,94,.6)', 1.5, [3, 5]);
  // POIs
  const shown = f.pois.filter(q => { const [x, y] = xy(q.lat, q.lon); return x > -10 && x < W + 10 && y > -10 && y < H + 10; }).sort((a, b) => (b.s || 0) - (a.s || 0)).slice(0, view.k > 25 ? 160 : 70);
  ctx.font = '600 11px system-ui, sans-serif'; ctx.textBaseline = 'middle'; const labels = []; const placed = [];
  for (const q of shown) { const [x, y] = xy(q.lat, q.lon); ctx.fillStyle = q.u ? '#fbbf24' : q.c === 'city' || q.c === 'capital' ? '#e2e8f0' : '#7dd3fc'; ctx.beginPath(); ctx.arc(x, y, q.u ? 3.2 : 2.4, 0, 7); ctx.fill(); labels.push([q, x, y]); }
  if (view.k > 9) for (const [q, x, y] of labels.slice(0, view.k > 25 ? 40 : 18)) { const w = ctx.measureText(q.n).width; if (placed.some(b => x + 6 < b[0] + b[2] && x + 6 + w > b[0] && Math.abs(y - b[1]) < 13)) continue; placed.push([x + 6, y, w]); ctx.fillStyle = 'rgba(7,11,20,.7)'; ctx.fillRect(x + 4, y - 7, w + 4, 14); ctx.fillStyle = '#cbd5e1'; ctx.fillText(q.n, x + 6, y); }
  // airports
  for (const a of [f.from, f.to]) { const [x, y] = xy(a.lat, a.lon); ctx.fillStyle = '#f8fafc'; ctx.beginPath(); ctx.arc(x, y, 4, 0, 7); ctx.fill(); ctx.font = '700 12px system-ui'; ctx.fillText(a.iata || a.icao, x + 7, y - 9); }
  // plane
  const [px, py] = xy(p.lat, p.lon); ctx.save(); ctx.translate(px, py); ctx.rotate(((p.trk || 0)) * D2R); const s = 1.25;
  ctx.fillStyle = p.src === 'live' ? '#22c55e' : p.src === 'gps' ? '#38bdf8' : '#f59e0b'; ctx.strokeStyle = '#020617'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(0, -12 * s); ctx.lineTo(2 * s, -4 * s); ctx.lineTo(11 * s, 2 * s); ctx.lineTo(11 * s, 4 * s); ctx.lineTo(2 * s, 1.5 * s); ctx.lineTo(1.5 * s, 8 * s); ctx.lineTo(4.5 * s, 10.5 * s); ctx.lineTo(4.5 * s, 12 * s); ctx.lineTo(0, 10.5 * s); ctx.lineTo(-4.5 * s, 12 * s); ctx.lineTo(-4.5 * s, 10.5 * s); ctx.lineTo(-1.5 * s, 8 * s); ctx.lineTo(-2 * s, 1.5 * s); ctx.lineTo(-11 * s, 4 * s); ctx.lineTo(-11 * s, 2 * s); ctx.lineTo(-2 * s, -4 * s); ctx.closePath(); ctx.stroke(); ctx.fill(); ctx.restore();
}
// map interaction: drag to pan, buttons to zoom
(() => { let drag = null; cv.addEventListener('pointerdown', e => { computeView(); drag = { x: e.clientX, y: e.clientY, v: Object.assign({}, view), moved: false }; cv.setPointerCapture(e.pointerId); });
  cv.addEventListener('pointermove', e => { if (!drag) return; const dx = e.clientX - drag.x, dy = e.clientY - drag.y; if (Math.hypot(dx, dy) > 6) drag.moved = true; if (!drag.moved) return; const cl = Math.cos(drag.v.lat * D2R); S.pan = { lat: Math.max(-85, Math.min(85, drag.v.lat + dy / drag.v.k)), lon: wrap(drag.v.lon - dx / (drag.v.k * cl)), k: drag.v.k }; S.view = 'free'; $('#viewToggle').textContent = 'Plane'; drawMap(); });
  cv.addEventListener('pointerup', e => { if (drag && !drag.moved) tapMap(e); drag = null; });
  function tapMap(e) { const r = cv.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top; let best = null, bd = 22; for (const q of S.flight.pois) { const [qx, qy] = xy(q.lat, q.lon); const d = Math.hypot(qx - x, qy - y); if (d < bd) { bd = d; best = q; } } if (best) openPoi(best); }
  $('#zoomIn').onclick = () => { if (S.view === 'free' && S.pan) S.pan.k *= 1.6; else S.zoom *= 1.6; drawMap(); };
  $('#zoomOut').onclick = () => { if (S.view === 'free' && S.pan) S.pan.k /= 1.6; else S.zoom /= 1.6; drawMap(); };
  $('#viewToggle').onclick = () => { S.view = S.view === 'follow' ? 'route' : 'follow'; S.zoom = 1; S.pan = null; $('#viewToggle').textContent = S.view === 'follow' ? 'Route' : 'Plane'; drawMap(); };
})();

/* ---------------- live Wikipedia (bonus, online only) ---------------- */
async function liveWiki(manual) {
  const p = S.pos; if (!p) return; const hint = $('#liveHint');
  if (!navigator.onLine) { hint.textContent = 'Offline — live Wikipedia needs internet. Bundled places are still in “What’s below”.'; return; }
  if (!manual && S.liveWikiAt && hav(S.liveWikiAt.lat, S.liveWikiAt.lon, p.lat, p.lon) < 40) return;
  hint.textContent = 'Asking Wikipedia…';
  try { const u = 'https://en.wikipedia.org/w/api.php?action=query&format=json&origin=*&generator=geosearch&ggscoord=' + p.lat.toFixed(4) + '|' + p.lon.toFixed(4) + '&ggsradius=10000&ggslimit=20&prop=extracts|coordinates|description&exintro=1&explaintext=1&exsentences=2&exlimit=20';
    const d = await fetchJSON(u, 12000); const pages = Object.values((d.query || {}).pages || {}); S.liveWikiAt = { lat: p.lat, lon: p.lon };
    const items = pages.filter(x => x.coordinates).map(x => ({ n: x.title, w: x.title, lat: x.coordinates[0].lat, lon: x.coordinates[0].lon, b: (x.extract || '').trim(), c: catOf(x.description, x.title), desc: x.description || '' }));
    items.sort((a, b) => hav(p.lat, p.lon, a.lat, a.lon) - hav(p.lat, p.lon, b.lat, b.lon));
    const ul = $('#liveList'); ul.textContent = ''; hint.textContent = items.length ? 'Wikipedia articles within 10 km of the point below the plane (fetched ' + new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }) + ')' : 'Wikipedia has no geotagged articles within 10 km of this spot (common over oceans and remote land).';
    for (const q of items) ul.appendChild(poiLi(q, '<b>' + units.short(hav(p.lat, p.lon, q.lat, q.lon)) + '</b>', q.desc || q.b.slice(0, 80)));
    markOnline(true);
  } catch (e) { hint.textContent = 'Could not reach Wikipedia (' + e.message + ').'; }
}

/* ---------------- corridor POIs for any route (online, cached offline) ---------------- */
const QLEVER = 'https://qlever.dev/api/wikidata';
function sparqlBox(th, s, n, w, e) { return `PREFIX wd: <http://www.wikidata.org/entity/>
PREFIX wdt: <http://www.wikidata.org/prop/direct/>
PREFIX wikibase: <http://wikiba.se/ontology#>
PREFIX schema: <http://schema.org/>
PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
PREFIX geof: <http://www.opengis.net/def/function/geosparql/>
SELECT ?item ?itemLabel ?coord ?sl ?article (GROUP_CONCAT(DISTINCT ?typeLabel;separator="|") AS ?types) (SAMPLE(?unesco) AS ?u) WHERE {
 ?item wdt:P625 ?coord . ?item wikibase:sitelinks ?sl . FILTER(?sl >= ${th})
 BIND(geof:latitude(?coord) AS ?lat) BIND(geof:longitude(?coord) AS ?lon)
 FILTER(?lat > ${s} && ?lat < ${n} && ?lon > ${w} && ?lon < ${e})
 ?article schema:about ?item . ?article schema:isPartOf <https://en.wikipedia.org/> .
 ?item rdfs:label ?itemLabel . FILTER(LANG(?itemLabel)="en")
 MINUS { ?item wdt:P31 wd:Q6256 } MINUS { ?item wdt:P31 wd:Q3624078 }
 OPTIONAL { ?item wdt:P31 ?type . ?type rdfs:label ?typeLabel . FILTER(LANG(?typeLabel)="en") }
 OPTIONAL { ?item wdt:P1435 wd:Q9259 . BIND(1 AS ?unesco) }
} GROUP BY ?item ?itemLabel ?coord ?sl ?article ORDER BY DESC(?sl) LIMIT 120`; }
async function qlever(q) { const r = await withTimeout(fetch(QLEVER, { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded', Accept: 'application/sparql-results+json' }, body: 'query=' + encodeURIComponent(q) }), 60000); if (!r.ok) throw new Error('Wikidata HTTP ' + r.status); return (await r.json()).results.bindings; }
async function wikiExtracts(titles) { const out = {}; for (let i = 0; i < titles.length; i += 20) { const chunk = titles.slice(i, i + 20); const u = 'https://en.wikipedia.org/w/api.php?action=query&format=json&origin=*&redirects=1&prop=extracts&exintro=1&explaintext=1&exsentences=3&exlimit=20&titles=' + encodeURIComponent(chunk.join('|')); try { const d = await fetchJSON(u, 15000); const map = {}; for (const r of (d.query.redirects || []).concat(d.query.normalized || [])) map[r.to] = r.from; for (const pg of Object.values(d.query.pages || {})) { if (pg.extract) { let t = pg.title; out[t] = pg.extract; let back = map[t]; while (back) { out[back] = pg.extract; back = map[back]; } } } } catch (e) {} } return out; }
function trimBlurb(t) { if (!t) return ''; t = t.replace(/\s*\[[^\]]*\]/g, '');
  for (let k = 0; k < 2; k++) t = t.replace(/\s*\(([^()]*)\)/g, (m, g) => (m.length > 60 || /[ˈˌəɪʊʃʒθðŋɛɔæʌːɑɒɜ]|pronunc|listen|^\s*[,;]|^\s*$|:\s*$/.test(g)) ? '' : m);
  t = t.replace(/\s+([,.;])/g, '$1').replace(/\s+/g, ' ').trim(); const sents = t.split(/(?<=[.!?])\s+(?=[A-Z0-9"'(])/); let o = ''; for (const s of sents) { if (o && o.length + s.length > 380) break; o += (o ? ' ' : '') + s; } return o.trim(); }
async function downloadCorridor(f, onProg) {
  const centers = []; let last = -1e9; for (let i = 0; i < f.path.length; i++) if (f.cum[i] - last >= 280) { centers.push(f.path[i]); last = f.cum[i]; } centers.push(f.path[f.path.length - 1]);
  const items = new Map(); let done = 0, fails = 0; const CORR = 250;
  const work = centers.map(c => async () => { const dlat = CORR / 111, dlon = Math.min(60, CORR / (111 * Math.cos(c[0] * D2R)));
      let got = [];
      for (const th of [30, 8]) { try { const rows = await qlever(sparqlBox(th, c[0] - dlat, c[0] + dlat, c[1] - dlon, c[1] + dlon)); got = [];
          for (const r of rows) { const m = /POINT\(([-\d.eE]+) ([-\d.eE]+)\)/i.exec(r.coord.value); if (!m) continue; const lon = +m[1], lat = +m[2]; const pr = project(f, lat, lon); if (pr.xd > CORR) continue; got.push({ q: r.item.value.split('/').pop(), n: r.itemLabel.value, lat: +lat.toFixed(4), lon: +lon.toFixed(4), s: +r.sl.value, w: decodeURIComponent(r.article.value.split('/wiki/')[1]).replace(/_/g, ' '), ty: r.types ? r.types.value : '', u: r.u ? 1 : 0 }); }
          if (got.length >= 12) break; } catch (e) { fails++; break; } }
      got.sort((a, b) => b.s - a.s).slice(0, 35).forEach(g => { if (!items.has(g.q)) items.set(g.q, g); }); done++; onProg && onProg('Finding places… ' + done + '/' + centers.length + ' sections, ' + items.size + ' places'); });
  const pool = async () => { while (work.length) await work.shift()(); }; await Promise.all([pool(), pool(), pool()]);
  // Fallback / supplement: Wikipedia geosearch along the line (every ~150 km)
  if (items.size < 40) { let lastG = -1e9; for (let i = 0; i < f.path.length; i++) { if (f.cum[i] - lastG < 150) continue; lastG = f.cum[i]; const c = f.path[i]; try { const d = await fetchJSON('https://en.wikipedia.org/w/api.php?action=query&format=json&origin=*&list=geosearch&gscoord=' + c[0].toFixed(3) + '|' + c[1].toFixed(3) + '&gsradius=10000&gslimit=4', 10000); for (const g of d.query.geosearch || []) items.set('wp' + g.pageid, { q: 'wp' + g.pageid, n: g.title, w: g.title, lat: g.lat, lon: g.lon, s: 0, ty: '' }); } catch (e) {} onProg && onProg('Wikipedia geosearch… ' + items.size + ' places'); } }
  const list = [...items.values()]; onProg && onProg('Downloading descriptions for ' + list.length + ' places…');
  const ex = await wikiExtracts(list.map(x => x.w)); for (const x of list) { x.b = trimBlurb(ex[x.w] || ''); x.c = catOf(x.ty, x.n); delete x.ty; }
  if (!list.length) throw new Error(fails ? 'place servers unreachable' : 'no places found');
  return list;
}

/* ---------------- flight search ---------------- */
function parseFlight(s) {
  s = String(s || '').toUpperCase().replace(/[\s-]/g, ''); let m;
  if ((m = /^([A-Z]{3})0*(\d{1,4}[A-Z]?)$/.exec(s))) { const ia = ICAO2IATA[m[1]]; return { callsign: m[1] + m[2], icao: m[1], iata: ia || null, num: m[2], label: (ia ? ia + ' ' : m[1] + ' ') + m[2], airline: ia && AIRLINES[ia] ? AIRLINES[ia][1] : '' }; }
  if ((m = /^([A-Z0-9]{2})0*(\d{1,4}[A-Z]?)$/.exec(s)) && !/^\d\d$/.test(m[1])) { const al = AIRLINES[m[1]]; if (!al) return { error: 'Unknown airline code “' + m[1] + '”. Try the 3-letter ICAO callsign instead (e.g. ETD13).' }; return { callsign: al[0] + m[2], icao: al[0], iata: m[1], num: m[2], label: m[1] + ' ' + m[2], airline: al[1] }; }
  return { error: 'Enter a flight like EY13, DL 1553 or a callsign like ETD13.' };
}
let pending = null;
async function doSearch(q) {
  const msg = $('#searchMsg'); const pf = parseFlight(q); $('#routeEdit').classList.add('hidden');
  if (pf.error) { msg.className = 'msg err'; msg.textContent = pf.error; return; }
  if (pf.callsign === 'ETD13') { loadFlight(ey13Flight()); hide('#flightSheet'); return; }
  msg.className = 'msg'; msg.textContent = 'Looking up ' + pf.label + ' (' + pf.callsign + ')…';
  let route = null, live = null, routeErr = '';
  const [rr, lr] = await Promise.allSettled([fetchJSON('https://api.adsbdb.com/v0/callsign/' + pf.callsign, 12000), lookupLive(pf.callsign)]);
  if (rr.status === 'fulfilled') { const fr = rr.value && rr.value.response && rr.value.response.flightroute; if (fr && fr.origin && fr.destination) route = { from: airport(fr.origin.iata_code) || airport(fr.origin.icao_code) || { iata: fr.origin.iata_code, icao: fr.origin.icao_code, name: fr.origin.name, city: fr.origin.municipality, lat: fr.origin.latitude, lon: fr.origin.longitude }, to: airport(fr.destination.iata_code) || airport(fr.destination.icao_code) || { iata: fr.destination.iata_code, icao: fr.destination.icao_code, name: fr.destination.name, city: fr.destination.municipality, lat: fr.destination.latitude, lon: fr.destination.longitude }, airline: fr.airline && fr.airline.name }; else routeErr = 'route not in adsbdb'; }
  else routeErr = navigator.onLine ? 'route lookup failed (' + rr.reason.message + ')' : 'you are offline';
  if (lr.status === 'fulfilled' && lr.value && lr.value.lat != null) live = lr.value;
  pending = { pf, live };
  const ed = $('#routeEdit'); ed.classList.remove('hidden'); $('#reFrom').value = route ? (route.from.iata || route.from.icao) : ''; $('#reTo').value = route ? (route.to.iata || route.to.icao) : '';
  const dflt = new Date(Date.now() + 3600e3); dflt.setMinutes(0, 0, 0); $('#reDep').value = toLocalInput(live ? Date.now() - 3600e3 : dflt.getTime()); $('#reDur').value = '';
  msg.className = route ? 'msg ok' : 'msg err';
  msg.textContent = (route ? pf.label + ' (' + (route.airline || pf.airline || pf.callsign) + '): ' + (route.from.city || route.from.name) + ' → ' + (route.to.city || route.to.name) + ' — route from adsbdb (crowd-sourced; check it).' : 'Couldn’t find the route for ' + pf.label + ' (' + routeErr + '). Enter the airports below.') + (live ? ' ✈︎ Live now via ' + live.via + '.' : (lr.value && lr.value.error ? ' Live lookup unavailable (' + lr.value.error + ').' : ' Not broadcasting right now — set the departure time for an estimate; I’ll keep checking for a live position.'));
  if (route) applyRoute();
}
function toLocalInput(ms) { const d = new Date(ms - new Date(ms).getTimezoneOffset() * 60e3); return d.toISOString().slice(0, 16); }
function applyRoute() {
  const pf = pending && pending.pf; if (!pf) return; const msg = $('#searchMsg'); const from = airport($('#reFrom').value), to = airport($('#reTo').value);
  if (!from || !to) { msg.className = 'msg err'; msg.textContent = 'Unknown airport code — use IATA (ATL) or ICAO (KATL).'; return; }
  const dist = hav(from.lat, from.lon, to.lat, to.lon); let dur = /^(\d+):(\d{1,2})$/.exec($('#reDur').value.trim()); const durMin = dur ? +dur[1] * 60 + +dur[2] : Math.round(dist / 820 * 60 + 35);
  const depMs = $('#reDep').value ? new Date($('#reDep').value).getTime() : Date.now();
  const key = pf.callsign + ':' + (from.iata || from.icao) + '-' + (to.iata || to.icao);
  const spec = { key, label: pf.label, callsign: pf.callsign, airline: pf.airline, fromC: from.iata || from.icao, toC: to.iata || to.icao, depMs, durMin };
  saveRecent(spec); openSpec(spec, pending.live); hide('#flightSheet');
}
async function openSpec(spec, live) {
  if (spec.key === 'ETD13:AUH-ATL') { loadFlight(ey13Flight()); return; }
  const from = airport(spec.fromC), to = airport(spec.toC); const f = buildFlight({ key: spec.key, label: spec.label, callsign: spec.callsign, airline: spec.airline, from, to, depMs: spec.depMs, durMin: spec.durMin, routeSrc: 'great-circle ' + (from.iata || from.icao) + '→' + (to.iata || to.icao) });
  const cached = await IDB.get('pois:' + (from.iata || from.icao) + '-' + (to.iata || to.icao)); if (cached && cached.pois) setPois(f, cached.pois);
  loadFlight(f); if (live) { S.live = live; }
  status(cached ? 'Places for this route are saved offline (' + f.pois.length + ')' : 'No places saved for this route yet — tap ✈︎ then “Download places for offline” while online');
  if (!cached && navigator.onLine) startDownload(true);
}
async function startDownload(auto) {
  const f = S.flight; const pr = $('#dlProg'); if (!f) return;
  if (f.builtIn) { pr.className = 'msg ok'; pr.textContent = 'EY 13 places are built in (' + f.pois.length + '). The app itself is cached for offline use' + (navigator.serviceWorker && navigator.serviceWorker.controller ? ' ✓' : ' after this page finishes loading') + '.'; return; }
  if (!navigator.onLine) { pr.className = 'msg err'; pr.textContent = 'You are offline — connect to download places.'; return; }
  pr.className = 'msg'; $('#dlBtn').disabled = true;
  try { const list = await downloadCorridor(f, t => { pr.textContent = t; status(t); }); const rk = (f.from.iata || f.from.icao) + '-' + (f.to.iata || f.to.icao); await IDB.set('pois:' + rk, { pois: list, savedAt: Date.now() }); if (S.flight === f) { setPois(f, list); tick(true); } pr.className = 'msg ok'; pr.textContent = '✓ Saved ' + list.length + ' places for ' + rk + ' — available offline.'; status(pr.textContent); }
  catch (e) { pr.className = 'msg err'; pr.textContent = 'Download failed: ' + e.message + (auto ? '' : ' — try again on better Wi-Fi.'); }
  $('#dlBtn').disabled = false;
}
function renderRecent() { const ul = $('#recentList'); ul.textContent = ''; const items = [{ key: 'ETD13:AUH-ATL', label: 'EY 13', fromC: 'AUH', toC: 'ATL' }].concat(LS.get('recent', []).filter(x => x.key !== 'ETD13:AUH-ATL'));
  for (const r of items) { const li = document.createElement('li'); li.textContent = r.label + ' · ' + r.fromC + '→' + r.toC; li.onclick = () => { hide('#flightSheet'); openSpec(r, null); }; ul.appendChild(li); } }

/* ---------------- app wiring ---------------- */
function loadFlight(f) {
  S.flight = f; S.live = null; S.liveVia = S.liveVia; S.trail = LS.get('trail.' + f.key, []); S.lastFix = null; S.liveMsg = ''; S.view = 'follow'; S.zoom = 1; S.pan = null; $('#viewToggle').textContent = 'Route';
  $('#flightTitle').textContent = f.label; $('#flightRoute').textContent = (f.from.iata || f.from.icao) + ' → ' + (f.to.iata || f.to.icao) + (f.airline ? ' · ' + f.airline : '');
  LS.set('current', f.builtIn ? { key: f.key } : { key: f.key, label: f.label, callsign: f.callsign, airline: f.airline, fromC: f.from.iata || f.from.icao, toC: f.to.iata || f.to.icao, depMs: f.depMs, durMin: f.durMin });
  $('#aboutLine').textContent = 'Sky Below v' + VERSION + ' · ' + f.label + ': ' + f.pois.length + ' places on board · route: ' + (f.routeSrc || 'great circle') + (f.aircraft ? ' · ' + f.aircraft : '');
  tick(true); pollLive();
}
function status(t) { $('#statusLine').textContent = t; }
function show(s) { $(s).classList.remove('hidden'); } function hide(s) { $(s).classList.add('hidden'); }
function applyOpts() { document.body.classList.toggle('big', S.opt.big); document.body.classList.toggle('low', S.opt.low); $('#optBig').checked = S.opt.big; $('#optLow').checked = S.opt.low; $('#optMiles').checked = S.opt.miles; $('#optSource').value = S.opt.source; $('#optProxy').value = S.opt.proxy; LS.set('opt', S.opt); restartTimers(); }
let tTick, tLive, tWiki;
function restartTimers() { clearInterval(tTick); clearInterval(tLive); clearInterval(tWiki); tTick = setInterval(() => tick(false), S.opt.low ? 15000 : 2000); tLive = setInterval(pollLive, S.opt.low ? 180000 : 45000); tWiki = setInterval(() => { if (!document.hidden && $('#pane-live').classList.contains('active')) liveWiki(false); }, 300000); }
function init() {
  $$('.tab').forEach(b => b.onclick = () => { $$('.tab').forEach(x => x.classList.toggle('active', x === b)); $$('.pane').forEach(x => x.classList.toggle('active', x.id === 'pane-' + b.dataset.tab)); if (b.dataset.tab === 'live') liveWiki(false); });
  $$('[data-close]').forEach(b => b.onclick = () => b.closest('.sheet').classList.add('hidden')); $$('.sheet').forEach(s => s.addEventListener('click', e => { if (e.target === s) s.classList.add('hidden'); }));
  $('#flightBtn').onclick = () => { renderRecent(); $('#dlProg').textContent = ''; show('#flightSheet'); setTimeout(() => $('#searchInput').focus(), 50); };
  $('#settingsBtn').onclick = () => show('#settingsSheet');
  $('#searchForm').onsubmit = e => { e.preventDefault(); doSearch($('#searchInput').value); };
  $('#reApply').onclick = applyRoute; $('#dlBtn').onclick = () => startDownload(false);
  $('#liveWikiBtn').onclick = () => liveWiki(true);
  $('#optBig').onchange = e => { S.opt.big = e.target.checked; applyOpts(); resize(); }; $('#optLow').onchange = e => { S.opt.low = e.target.checked; applyOpts(); if (S.gpsWatch != null) startGps(); }; $('#optMiles').onchange = e => { S.opt.miles = e.target.checked; applyOpts(); tick(true); };
  $('#optSource').onchange = e => { S.opt.source = e.target.value; applyOpts(); tick(true); if (e.target.value === 'gps') startGps(); pollLive(); }; $('#optProxy').onchange = e => { S.opt.proxy = e.target.value.trim(); applyOpts(); pollLive(); };
  $('#gpsBtn').onclick = startGps; $('#resetBtn').onclick = async () => { await IDB.clear(); LS.set('recent', []); status('Cleared saved flights'); renderRecent(); };
  const off = $('#offset'); off.value = S.offsetMin; const showOff = () => $('#offsetVal').textContent = (S.offsetMin > 0 ? '+' : '') + S.offsetMin + ' min'; showOff();
  off.oninput = () => { S.offsetMin = +off.value; LS.set('offset', S.offsetMin); showOff(); tick(true); }; $('#offsetReset').onclick = () => { off.value = 0; off.oninput(); };
  window.addEventListener('online', () => { markOnline(true); pollLive(); }); window.addEventListener('offline', () => markOnline(false)); markOnline(navigator.onLine);
  window.addEventListener('resize', resize); document.addEventListener('visibilitychange', () => { if (!document.hidden) { tick(true); pollLive(); } });
  applyOpts(); resize();
  const cur = LS.get('current', null); if (cur && cur.key !== 'ETD13:AUH-ATL' && cur.fromC && airport(cur.fromC) && airport(cur.toC)) openSpec(cur, null); else loadFlight(ey13Flight());
  if (LS.get('gpsOn', false) && S.opt.source !== 'est' && S.opt.source !== 'live') startGps();
  if ('serviceWorker' in navigator && /^https?:/.test(location.protocol) && !window.SINGLE_FILE) { status('Saving app for offline use…'); navigator.serviceWorker.register('sw.js').catch(e => status('Offline cache not available: ' + e.message)); navigator.serviceWorker.ready.then(() => { S.offlineReady = true; status('✓ Offline-ready: app saved on this phone'); }); }
  else if (window.SINGLE_FILE) status('Single-file version — works without internet');
}
// test hook (used by automated tests; harmless in normal use)
window.__sky = { S, pollLive, whereAt, tzAt, estimate, project, pointAt, parseFlight, airport, tick, currentPos, hav, setFix(lat, lon, alt, gs, trk) { S.live = { lat, lon, alt, gs, trk, age: 0, t: Date.now(), via: 'test' }; S.opt.source = 'auto'; tick(true); return S.pos; }, downloadCorridor, loadFlight, buildFlight, openSpec, doSearch, lookupLive };
init();
})();
