"""Isolate acquisition/encoding from control. Latest JPEG only; no frame queue."""
import multiprocessing as mp
from queue import Empty, Full
from ..interfaces import ICameraSource


def capture(out, stop, options):
    from .d435 import D435Camera
    camera=D435Camera(**options)
    try:
        while not stop.wait(.03):
            jpeg=camera.get_jpeg_frame()
            if jpeg:
                try: out.put_nowait(jpeg)
                except Full:
                    try: out.get_nowait()
                    except Empty: pass
    finally: camera.close()


class ProcessCamera(ICameraSource):
    def __init__(self, **options):
        ctx=mp.get_context('spawn')
        self._queue=ctx.Queue(1); self._stop=ctx.Event()
        self._process=ctx.Process(target=capture,args=(self._queue,self._stop,options),daemon=True)
        self._process.start(); self._jpeg=None
    def get_jpeg_frame(self):
        if not self._process.is_alive(): return None
        try: self._jpeg=self._queue.get_nowait()
        except Empty: pass
        return self._jpeg
    def get_depth_frame(self): return None
    def close(self):
        self._stop.set(); self._process.join(2)
        if self._process.is_alive(): self._process.terminate(); self._process.join(1)


class NoCamera(ICameraSource):
    def get_jpeg_frame(self): return None
    def get_depth_frame(self): return None
    def close(self): pass
