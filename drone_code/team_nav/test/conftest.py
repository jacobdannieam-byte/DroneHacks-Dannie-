"""Lets the tests import the team_nav package from this folder without installing it, and the
test helpers (fake_drone.py, fake_ros.py) from this test folder."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
