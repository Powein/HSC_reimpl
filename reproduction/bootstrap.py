"""Prefer the project-local packages; never modify the system environment."""
from pathlib import Path
import os
import sys

ROOT = Path(__file__).resolve().parent
DEPS = ROOT / "_deps"
if DEPS.is_dir():
    sys.path.insert(0, str(DEPS))
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "_cache" / "matplotlib"))
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
