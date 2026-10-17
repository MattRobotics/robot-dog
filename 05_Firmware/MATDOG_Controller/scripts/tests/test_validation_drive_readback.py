#!/usr/bin/env python3
"""Real ServoBus + installed SCServo, replacing the old extracted-body mock."""
import subprocess,sys
from pathlib import Path
subprocess.run([sys.executable,str(Path(__file__).with_name('run_prime_diagnostic_tests.py')),'--bus-only'],check=True)
