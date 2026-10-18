"""Test-process-only workaround for PyBullet paths on Windows Unicode folders."""
import os
from pathlib import Path
import runpy
import sys

sys.stdout.reconfigure(encoding='utf-8')
sys.stderr.reconfigure(encoding='utf-8')
ROOT=Path(__file__).resolve().parents[3]
os.chdir(str(ROOT))
target=ROOT/sys.argv[1]
sys.argv=[str(target)]+sys.argv[2:]
sys.path.insert(0,str(target.parent))
import pybullet
original=pybullet.loadURDF

def relative_load(path,*args,**kwargs):
    if isinstance(path,str) and os.path.isabs(path):
        try:
            relative=os.path.relpath(path,str(ROOT))
            if not relative.startswith('..'):
                path=relative
        except ValueError:
            pass
    return original(path,*args,**kwargs)

pybullet.loadURDF=relative_load
runpy.run_path(str(target),run_name='__main__')
