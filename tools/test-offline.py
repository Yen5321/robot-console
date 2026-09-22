"""One entry point for hardware-free Python tests; exit code is authoritative."""
import os,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
for cwd,args in [(root/'bridge',['-m','unittest','discover','-s','tests','-q']),(root,['-m','unittest','discover','-s','tests','-v'])]:
    subprocess.run([sys.executable,*args],cwd=cwd,check=True)
