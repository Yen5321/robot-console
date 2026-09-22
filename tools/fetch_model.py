"""Restore pinned model files against bundled SHA256 manifest; no branches."""
from pathlib import Path
import concurrent.futures
import hashlib
import json
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
DEST=ROOT/'ros2_ws/src/robot_console_servo/model'


def main():
    manifest=json.loads((DEST/'manifest.json').read_text())
    entries=manifest['files']
    def fetch(e):
        p=DEST/e['path']; p.parent.mkdir(parents=True,exist_ok=True)
        def valid(b): return hashlib.sha256(b).hexdigest()==e['sha256']
        if p.exists() and valid(p.read_bytes()): return
        name=e['path']
        if name in ('official_piper.srdf','LICENSE.piper_ros'):
            repo='piper_ros';rev=manifest['acm_commit']
            remote='src/piper_moveit/piper_no_gripper_moveit/config/piper.srdf' if name.endswith('.srdf') else 'LICENSE'
        else:repo='agx_arm_urdf';rev=manifest['commit'];remote=name
        b=urllib.request.urlopen(f'https://raw.githubusercontent.com/agilexrobotics/{repo}/{rev}/{remote}',timeout=120).read()
        if not valid(b): raise ValueError('Model checksum mismatch: '+e['path'])
        p.write_bytes(b)
    with concurrent.futures.ThreadPoolExecutor(4) as pool: list(pool.map(fetch,entries))
    print('Pinned model verified:', DEST)

if __name__=='__main__': main()
