"""
stream_server.py — Low-latency MJPEG live stream server with click-to-track web UI.

Serves annotated camera frames over HTTP as a multipart/x-mixed-replace MJPEG
stream. Any standard client (browser, VLC, ffplay, OpenCV) can consume it.

Endpoints:
    /         — interactive HTML page (click a person to lock tracking)
    /stream   — raw MJPEG  (VLC / ffplay / <img src>)
    /click    — GET ?x=0.42&y=0.61  → lock / unlock person at that point
    /unlock   — GET  → release any lock, revert to auto mode
    /status   — GET  → JSON {lock_id, ids}
    /zoom_in  — GET  → zoom in
    /zoom_out — GET  → zoom out

Tailscale setup (one-time):
    Jetson: sudo tailscale up; tailscale ip -4   (note the IP)
    Browser: http://<tailscale-ip>:8080/
"""

import json
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2

import config as cfg
from config.settings import Settings, load_settings

# Endpoints that mutate state — require token when STREAM_TOKEN is set
_CONTROL_PATHS = {'/click', '/unlock', '/mode', '/gimbal', '/zoom_in', '/zoom_out'}


# Stream resolutions
_STREAM_W_HI = 1280
_STREAM_H_HI = 720
_STREAM_W_LO = 854
_STREAM_H_LO = 480


class _ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    """HTTPServer that spawns one daemon thread per client connection."""
    daemon_threads      = True
    allow_reuse_address = True


class StreamServer:
    """Low-latency 720p MJPEG stream server with click-to-track web UI.

    A dedicated stream thread (tracker._stream_loop) pushes annotated
    720p JPEG frames via push_frame().  Client threads block on a
    Condition and are woken the instant a new frame arrives — zero
    polling, minimum latency regardless of YOLO speed.
    """

    _BOUNDARY = b"--f"

    def __init__(self, port: int | None = None, settings: Settings | None = None) -> None:
        self._s        = settings or load_settings()
        self._port     = port or self._s.stream_port
        self._jpg_hi   = b""
        self._jpg_lo   = b""
        self._frame_id = 0
        self._cond     = threading.Condition()
        self._server   = None
        self._thread   = None
        self._active   = False
        self._click_cb  = None   # callable(nx, ny) → dict
        self._zoom_cb   = None   # callable(direction: str) → None
        self._mode_cb   = None   # callable(mode_str_or_None) → dict
        self._gimbal_cb = None   # callable(direction: str) → dict

    # ------------------------------------------------------------------
    #  Frame push  (called by dedicated stream thread at STREAM_MAX_FPS)
    # ------------------------------------------------------------------

    def push_frame(self, frame_hi, frame_lo=None) -> None:
        """Encode JPEG buffers and wake all MJPEG client threads."""
        ok_hi, buf_hi = cv2.imencode(
            ".jpg", frame_hi,
            [cv2.IMWRITE_JPEG_QUALITY, cfg.STREAM_QUALITY_HI])
        jpg_lo = self._jpg_lo
        if frame_lo is not None:
            ok_lo, buf_lo = cv2.imencode(
                ".jpg", frame_lo,
                [cv2.IMWRITE_JPEG_QUALITY, cfg.STREAM_QUALITY_LO])
            if ok_lo:
                jpg_lo = buf_lo.tobytes()
        with self._cond:
            if ok_hi:
                self._jpg_hi = buf_hi.tobytes()
            self._jpg_lo = jpg_lo
            self._frame_id += 1
            self._cond.notify_all()

    def _get_latest_frame(self, last_id: int, quality: str = 'hi'):
        """Return (jpg_bytes, frame_id) for the given quality level.

        Returns immediately if a newer frame is already available — prevents
        lag accumulation after a slow TCP write.
        """
        with self._cond:
            if self._frame_id == last_id:
                self._cond.wait(timeout=0.05)
            jpg = self._jpg_lo if quality == 'lo' else self._jpg_hi
            return jpg, self._frame_id

    # ------------------------------------------------------------------
    #  Callback wiring
    # ------------------------------------------------------------------

    def set_click_callback(self, cb) -> None:
        self._click_cb = cb

    def set_zoom_callback(self, cb) -> None:
        self._zoom_cb = cb

    def set_mode_callback(self, cb) -> None:
        self._mode_cb = cb

    def set_gimbal_callback(self, cb) -> None:
        self._gimbal_cb = cb

    # ------------------------------------------------------------------
    #  HTML UI
    # ------------------------------------------------------------------

    def _build_html(self) -> bytes:
        # Canvas + JS MJPEG parser instead of <img src="/stream">.
        # Parses JPEG boundaries from the raw stream and drops frames when
        # busy rendering — lag can NEVER accumulate.
        auth_token = json.dumps(self._s.stream_token)
        return (f"""<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drone Tracker</title>
<style>
*{{box-sizing:border-box;margin:0;padding:0}}
html,body{{width:100%;height:100%;overflow:hidden;background:#000}}
body{{display:flex;flex-direction:column;font-family:monospace;color:#ddd}}
#hdr{{background:#111;padding:5px 12px;display:flex;align-items:center;
      gap:10px;border-bottom:1px solid #222;flex-shrink:0}}
#hdr h1{{font-size:12px;color:#0cf;letter-spacing:1px;white-space:nowrap}}
#stat{{font-size:11px;color:#0f0;min-width:220px}}
#badge{{font-size:11px;background:#004400;color:#7fff7f;border:1px solid #0a0;
        padding:1px 8px;border-radius:8px;display:none;white-space:nowrap}}
.zbtn{{background:#003366;color:#aaddff;border:1px solid #06f;
       padding:2px 10px;border-radius:4px;cursor:pointer;font-size:13px;font-weight:bold}}
.zbtn:hover{{background:#004488}}
#qbtn{{background:#223300;color:#aaff88;border:1px solid #4a0;
       padding:2px 8px;border-radius:4px;cursor:pointer;font-size:11px;font-weight:bold}}
#qbtn:hover{{background:#334400}}
#ubtn{{margin-left:auto;background:#550000;color:#ffaaaa;border:1px solid #a00;
       padding:2px 10px;border-radius:4px;cursor:pointer;font-size:11px}}
#ubtn:hover{{background:#880000}}
#wrap{{flex:1;position:relative;display:flex;
       justify-content:center;align-items:center;background:#000}}
canvas{{max-width:100%;max-height:100%;display:block;cursor:crosshair}}
#ring{{position:absolute;pointer-events:none;display:none;
       width:40px;height:40px;transform:translate(-50%,-50%);
       border:2px solid #0f0;border-radius:50%;
       box-shadow:0 0 10px #0f0;animation:pop .5s ease-out forwards}}
@keyframes pop{{
  0%  {{opacity:1;transform:translate(-50%,-50%) scale(.4)}}
  100%{{opacity:0;transform:translate(-50%,-50%) scale(1.6)}}
}}
#toast{{position:absolute;bottom:18px;left:50%;transform:translateX(-50%);
        background:rgba(0,160,80,.95);color:#000;padding:5px 16px;
        border-radius:14px;font-size:13px;font-weight:bold;
        pointer-events:none;opacity:0;transition:opacity .15s;white-space:nowrap}}
#toast.e{{background:rgba(180,30,30,.95);color:#fff}}
#toast.on{{opacity:1}}
#mbtn{{background:#003300;color:#88ff88;border:1px solid #0a0;
       padding:2px 10px;border-radius:4px;cursor:pointer;
       font-size:12px;font-weight:bold;min-width:80px}}
#mbtn:hover{{opacity:.85}}
#dpad{{display:none;position:absolute;bottom:40px;right:12px;z-index:10;
       background:rgba(0,0,0,.75);border-radius:8px;padding:6px;user-select:none;
       flex-direction:column;align-items:center;gap:4px}}
.dp{{background:#1a2a3a;color:#cce;border:1px solid #446;width:38px;height:38px;
     border-radius:4px;cursor:pointer;font-size:18px;display:flex;
     align-items:center;justify-content:center;touch-action:none}}
.dp:active{{background:#2a4a6a}}
#dpad-lbl{{font-size:10px;color:#888;margin-top:3px}}
</style></head>
<body>
<div id="hdr">
  <h1>&#9654; DRONE TRACKER</h1>
  <span id="stat">connecting…</span>
  <span id="badge">LOCK ID ?</span>
  <button id="mbtn" data-mode="AUTO" onclick="toggleMode()" title="Toggle Manual/Auto">AUTO</button>
  <button class="zbtn" onclick="doZoom('in')"  title="Zoom In">I</button>
  <button class="zbtn" onclick="doZoom('out')" title="Zoom Out">O</button>
  <button id="qbtn"  onclick="toggleQuality()" title="Switch HD/SD quality">HD</button>
  <button id="ubtn" onclick="doUnlock()">UNLOCK</button>
</div>
<div id="wrap">
  <canvas id="c" width="{_STREAM_W_HI}" height="{_STREAM_H_HI}"></canvas>
  <div id="ring"></div>
  <div id="toast"></div>
  <div id="dpad">
    <button class="dp" onpointerdown="gDir('up')"    onpointerup="gStop()"
            onpointerleave="gStop()" onpointercancel="gStop()">&#8593;</button>
    <div style="display:flex;gap:4px">
      <button class="dp" onpointerdown="gDir('left')"  onpointerup="gStop()"
              onpointerleave="gStop()" onpointercancel="gStop()">&#8592;</button>
      <button class="dp" onclick="gStop()" title="Stop gimbal">&#9632;</button>
      <button class="dp" onpointerdown="gDir('right')" onpointerup="gStop()"
              onpointerleave="gStop()" onpointercancel="gStop()">&#8594;</button>
    </div>
    <button class="dp" onpointerdown="gDir('down')"   onpointerup="gStop()"
            onpointerleave="gStop()" onpointercancel="gStop()">&#8595;</button>
    <div id="dpad-lbl">GIMBAL</div>
  </div>
</div>
<script>
const cvs   = document.getElementById('c');
const ctx   = cvs.getContext('2d', {{alpha:false}});
const stat  = document.getElementById('stat');
const badge = document.getElementById('badge');
const ring  = document.getElementById('ring');
const toast = document.getElementById('toast');
const AUTH_TOKEN = {auth_token};
let rendering = false;
let fcount = 0, ft = Date.now();
let tt = null;
let streamQuality = 'hi';
let streamReader  = null;

function ctlUrl(path) {{
  if (!AUTH_TOKEN) return path;
  const sep = path.includes('?') ? '&' : '?';
  return path + sep + 'token=' + encodeURIComponent(AUTH_TOKEN);
}}

function tickFPS() {{
  const el = (Date.now() - ft) / 1000;
  if (el >= 1) {{
    const d = (streamQuality === 'hi') ? '{_STREAM_W_HI}\u00d7{_STREAM_H_HI}' : '{_STREAM_W_LO}\u00d7{_STREAM_H_LO}';
    stat.textContent = 'LIVE ' + d + '  ' + (fcount / el).toFixed(1) + ' fps';
    fcount = 0; ft = Date.now();
  }}
}}

function showToast(msg, err) {{
  clearTimeout(tt);
  toast.textContent = msg;
  toast.className = 'on' + (err ? ' e' : '');
  tt = setTimeout(() => {{ toast.className = ''; }}, 2400);
}}

function syncBadge(d) {{
  if (d && d.lock_id != null) {{
    badge.style.display = 'inline';
    badge.textContent   = 'LOCK ID ' + d.lock_id;
  }} else {{
    badge.style.display = 'none';
  }}
}}

cvs.addEventListener('click', e => {{
  const r  = cvs.getBoundingClientRect();
  const sx = cvs.width  / r.width;
  const sy = cvs.height / r.height;
  const nx = ((e.clientX - r.left) * sx) / cvs.width;
  const ny = ((e.clientY - r.top)  * sy) / cvs.height;
  if (nx < 0 || nx > 1 || ny < 0 || ny > 1) return;
  ring.style.left    = (e.clientX - r.left) + 'px';
  ring.style.top     = (e.clientY - r.top)  + 'px';
  ring.style.display = 'block';
  setTimeout(() => {{ ring.style.display = 'none'; }}, 560);
  fetch(ctlUrl('/click?x=' + nx.toFixed(4) + '&y=' + ny.toFixed(4)))
    .then(r => r.json())
    .then(d => {{
      if      (d.status === 'locked')   {{ showToast('LOCKED ID ' + d.id, false); syncBadge(d); }}
      else if (d.status === 'unlocked') {{ showToast('UNLOCKED', false); syncBadge(null); }}
      else                              {{ showToast(d.msg || 'No person here', true); }}
    }})
    .catch(() => showToast('Request error', true));
}});

function doUnlock() {{
  fetch(ctlUrl('/unlock')).then(r => r.json())
    .then(d => {{ showToast('UNLOCKED', false); syncBadge(null); }});
}}

function doZoom(dir) {{
  fetch(ctlUrl('/zoom_' + dir))
    .then(r => r.json())
    .then(d => {{ showToast(dir === 'in' ? 'Zoom In' : 'Zoom Out', false); }})
    .catch(() => showToast('Zoom error', true));
}}

function toggleQuality() {{
  streamQuality = (streamQuality === 'hi') ? 'lo' : 'hi';
  document.getElementById('qbtn').textContent = (streamQuality === 'hi') ? 'HD' : 'SD';
  document.getElementById('qbtn').style.color  = (streamQuality === 'hi') ? '#aaff88' : '#ffcc44';
  if (streamReader) {{ try {{ streamReader.cancel(); }} catch(e) {{}} streamReader = null; }}
  showToast(streamQuality === 'hi' ? '720p HD' : '480p SD — lower bandwidth', false);
}}

setInterval(() => {{
  fetch('/status').then(r => r.json()).then(syncBadge).catch(() => {{}});
}}, 2000);

function toggleMode() {{
  const btn  = document.getElementById('mbtn');
  const next = (btn.dataset.mode === 'AUTO') ? 'manual' : 'auto';
  fetch(ctlUrl('/mode?set=' + next))
    .then(r => r.json())
    .then(d => applyMode(d.mode))
    .catch(() => showToast('Mode switch error', true));
}}

function applyMode(mode) {{
  const btn  = document.getElementById('mbtn');
  const dpad = document.getElementById('dpad');
  btn.dataset.mode     = mode;
  btn.textContent      = mode;
  btn.style.background = (mode === 'AUTO') ? '#003300' : '#553300';
  btn.style.color      = (mode === 'AUTO') ? '#88ff88' : '#ffcc44';
  btn.style.border     = (mode === 'AUTO') ? '1px solid #0a0' : '1px solid #a80';
  dpad.style.display   = (mode === 'MANUAL') ? 'flex' : 'none';
  showToast(mode === 'AUTO' ? 'AUTO mode — autonomous tracking' : 'MANUAL mode — use D-pad or arrow keys', false);
}}

function gDir(dir) {{ fetch(ctlUrl('/gimbal?dir=' + dir)).catch(() => {{}}); }}
function gStop()   {{ fetch(ctlUrl('/gimbal?dir=stop')).catch(() => {{}}); }}

setInterval(() => {{
  fetch(ctlUrl('/mode')).then(r => r.json()).then(d => {{
    const btn = document.getElementById('mbtn');
    if (btn.dataset.mode !== d.mode) applyMode(d.mode);
  }}).catch(() => {{}});
}}, 3000);

async function runStream() {{
  stat.textContent = 'connecting…';
  const resp = await fetch('/stream?q=' + streamQuality, {{cache: 'no-store'}});
  if (!resp.ok) throw new Error('HTTP ' + resp.status);
  const reader = resp.body.getReader();
  streamReader = reader;
  let buf = new Uint8Array(0);
  const dim = (streamQuality === 'hi') ? '{_STREAM_W_HI}\u00d7{_STREAM_H_HI}' : '{_STREAM_W_LO}\u00d7{_STREAM_H_LO}';
  stat.textContent = dim + ' buffering…';
  for (;;) {{
    const {{ value, done }} = await reader.read();
    if (done) throw new Error('stream closed');
    const tmp = new Uint8Array(buf.length + value.length);
    tmp.set(buf);
    tmp.set(value, buf.length);
    buf = tmp;
    for (;;) {{
      let si = -1;
      for (let i = 0; i < buf.length - 1; i++) {{
        if (buf[i] === 0xFF && buf[i + 1] === 0xD8) {{ si = i; break; }}
      }}
      if (si < 0) {{ buf = new Uint8Array(0); break; }}
      let ei = -1;
      for (let i = si + 2; i < buf.length - 1; i++) {{
        if (buf[i] === 0xFF && buf[i + 1] === 0xD9) {{ ei = i + 2; break; }}
      }}
      if (ei < 0) break;
      const jpeg = buf.slice(si, ei);
      buf = buf.slice(ei);
      if (!rendering) {{
        rendering = true;
        createImageBitmap(new Blob([jpeg], {{ type: 'image/jpeg' }}))
          .then(bmp => {{
            ctx.drawImage(bmp, 0, 0, cvs.width, cvs.height);
            bmp.close();
            fcount++;
            tickFPS();
            rendering = false;
          }})
          .catch(() => {{ rendering = false; }});
      }}
    }}
    if (buf.length > 2_000_000) {{ buf = new Uint8Array(0); }}
  }}
}}

(async () => {{
  for (;;) {{
    try {{
      await runStream();
    }} catch (e) {{
      stat.textContent = 'reconnecting…';
    }}
    await new Promise(r => setTimeout(r, 1500));
  }}
}})();
</script></body></html>
""").encode()

    # ------------------------------------------------------------------
    #  HTTP server
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start multi-client HTTP server in a background daemon thread."""
        import json as _json
        server_ref  = self
        html_bytes  = self._build_html()
        stream_token = self._s.stream_token   # capture once so closure is cfg-free

        class _Handler(BaseHTTPRequestHandler):
            def log_message(self, _fmt, *_args): pass

            def _check_token(self) -> bool:
                """Allow request if route is read-only, token matches, or loopback-only."""
                route = urllib.parse.urlparse(self.path).path
                if route not in _CONTROL_PATHS:
                    return True
                if stream_token:
                    params = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
                    return params.get('token', [''])[0] == stream_token
                # No token configured: only allow from loopback
                return self.client_address[0] in ('127.0.0.1', '::1')

            def do_GET(self):
                if not self._check_token():
                    self.send_response(403)
                    self.send_header('Content-Type', 'text/plain')
                    self.send_header('Content-Length', '9')
                    self.end_headers()
                    self.wfile.write(b'Forbidden')
                    return

                parts = self.path.split('?', 1)
                route, qs = parts[0], (parts[1] if len(parts) > 1 else '')

                if route == '/stream':
                    self._mjpeg(qs)
                elif route in ('/', '/index.html'):
                    self._bytes(200, 'text/html; charset=utf-8', html_bytes)
                elif route == '/click':
                    self._click(qs)
                elif route == '/unlock':
                    cb = server_ref._click_cb
                    if cb:
                        cb(-1.0, -1.0)
                    self._json({'status': 'unlocked', 'lock_id': None})
                elif route == '/status':
                    cb = server_ref._click_cb
                    data = cb(None, None) if cb else {'lock_id': None, 'ids': []}
                    self._json(data)
                elif route == '/zoom_in':
                    zb = server_ref._zoom_cb
                    if zb: zb('in')
                    self._json({'status': 'ok', 'zoom': 'in'})
                elif route == '/zoom_out':
                    zb = server_ref._zoom_cb
                    if zb: zb('out')
                    self._json({'status': 'ok', 'zoom': 'out'})
                elif route == '/mode':
                    kv = urllib.parse.parse_qs(qs, keep_blank_values=False)
                    mode_str = kv.get('set', [None])[0]
                    cb = server_ref._mode_cb
                    data = cb(mode_str) if cb else {'mode': 'UNKNOWN'}
                    self._json(data)
                elif route == '/gimbal':
                    kv = urllib.parse.parse_qs(qs, keep_blank_values=False)
                    direction = kv.get('dir', ['stop'])[0]
                    cb = server_ref._gimbal_cb
                    data = cb(direction) if cb else {'status': 'error', 'msg': 'not ready'}
                    code = 400 if isinstance(data, dict) and data.get('status') == 'error' else 200
                    self._json(data, code)
                else:
                    self.send_response(404)
                    self.end_headers()

            def _click(self, qs):
                try:
                    kv = urllib.parse.parse_qs(qs, keep_blank_values=False)
                    nx = float(kv.get('x', [''])[0])
                    ny = float(kv.get('y', [''])[0])
                    if not (0.0 <= nx <= 1.0 and 0.0 <= ny <= 1.0):
                        raise ValueError('coords out of [0,1]')
                except (KeyError, ValueError) as e:
                    self._json({'status': 'error', 'msg': str(e)}, 400)
                    return
                cb = server_ref._click_cb
                result = cb(nx, ny) if cb else {'status': 'error', 'msg': 'tracker not ready'}
                self._json(result)

            def _mjpeg(self, qs=''):
                kv = urllib.parse.parse_qs(qs, keep_blank_values=False)
                quality = 'lo' if kv.get('q', ['hi'])[0].lower() == 'lo' else 'hi'

                import socket as _sock
                raw = self.request
                raw.setsockopt(_sock.IPPROTO_TCP, _sock.TCP_NODELAY, 1)
                raw.setsockopt(_sock.SOL_SOCKET, _sock.SO_SNDBUF, 256 * 1024)

                self.send_response(200)
                self.send_header('Content-Type',
                    f'multipart/x-mixed-replace; boundary={StreamServer._BOUNDARY.decode()}')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Connection', 'close')
                self.end_headers()

                last_id = -1
                try:
                    while server_ref._active:
                        jpg, last_id = server_ref._get_latest_frame(last_id, quality)
                        if not jpg:
                            continue
                        part = (StreamServer._BOUNDARY + b'\r\n'
                                b'Content-Type: image/jpeg\r\n'
                                + f'Content-Length: {len(jpg)}\r\n\r\n'.encode()
                                + jpg + b'\r\n')
                        self.wfile.write(part)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass

            def _json(self, data, code=200):
                body = _json.dumps(data).encode()
                self._bytes(code, 'application/json', body)

            def _bytes(self, code, ctype, body):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body)

        if self._active:
            return
        self._active = True
        stream_host = self._s.stream_host
        self._server = _ThreadingHTTPServer((stream_host, self._port), _Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True, name='StreamSrv')
        self._thread.start()
        print(f"[STREAM] 720p live stream ready  host={stream_host}  port={self._port}")
        print(f"  Browser   http://{stream_host}:{self._port}/  (click to track)")
        print(f"  Raw MJPEG http://{stream_host}:{self._port}/stream")
        if stream_host != '127.0.0.1' and not stream_token:
            print("[STREAM] WARNING: server exposed on network without STREAM_TOKEN — set it!")

    def stop(self) -> None:
        if not self._active:
            return
        self._active = False
        with self._cond:
            self._cond.notify_all()
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3.0)
            self._thread = None

    @property
    def is_active(self) -> bool:
        return self._active
