from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import logging
import json
import threading
import time

from .interfaces import ICameraSource


class MjpegServer:
    def __init__(self, camera: ICameraSource, host: str, port: int, max_fps: float = 30.0, diagnostics=None, actions=None, configuration=None, recent_logs=None) -> None:
        self._camera = camera
        self._interval = 1.0 / max(1.0, max_fps)
        self._log = logging.getLogger(__name__)
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                if self.path != "/arm/action" or actions is None:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                try:
                    length=int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 4096: raise ValueError("Invalid body size")
                    payload=json.loads(self.rfile.read(length))
                    result=actions(payload,self.client_address[0])
                    status=HTTPStatus.OK
                except Exception as exc:
                    result={"ok":False,"error":str(exc)}
                    status=HTTPStatus.CONFLICT
                body=json.dumps(result,ensure_ascii=False,allow_nan=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type","application/json; charset=utf-8")
                self.send_header("Cache-Control","no-store")
                self.send_header("Content-Length",str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
                read = diagnostics if self.path == "/diagnostics" else configuration if self.path == "/configuration" else recent_logs if self.path == "/logs/recent" else None
                if read is not None:
                    try:
                        body = json.dumps(read(), ensure_ascii=False, allow_nan=False).encode("utf-8")
                    except Exception as exc:
                        body = json.dumps({"error": str(exc)}).encode("utf-8")
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path == "/healthz":
                    body = b"ok\n"
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path != "/stream.mjpg":
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        started = time.monotonic()
                        jpeg = outer._camera.get_jpeg_frame()
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        time.sleep(max(0.0, outer._interval - (time.monotonic() - started)))
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
                except Exception as exc:
                    outer._log.warning("MJPEG client stopped: %s", exc)

            def log_message(self, fmt: str, *args: object) -> None:
                outer._log.info("http %s - %s", self.client_address[0], fmt % args)

        self._server = ThreadingHTTPServer((host, port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="mjpeg-http", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

