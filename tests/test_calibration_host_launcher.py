from pathlib import Path
import os
import shutil
import subprocess
import pytest


def run(*extra):
    shell = 'C:/Program Files/Git/bin/bash.exe' if os.name == 'nt' else shutil.which('bash')
    if not shell or not Path(shell).exists():
        pytest.skip('bash unavailable')
    return subprocess.run([shell, 'simulator/gazebo_trial/run_hybrid.sh', '--check',
                           '--world', 'scenarios/calibration/straight_lane_v1.json',
                           '--profile', 'scenarios/calibration/c1_stop_020.json', *extra],
                          cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True, timeout=15)


def test_disarmed_calibration_host_mode_has_no_launch():
    result = run('--mode', 'calibration_check')
    assert result.returncode == 0, result.stderr
    assert 'launch=false' in result.stdout


def test_moving_calibration_without_approval_fails_before_launch():
    result = run('--mode', 'calibration_trial')
    assert result.returncode != 0
    assert 'team-approved' in result.stderr


def test_calibration_cannot_silently_use_phone_course():
    assert run('--mode', 'calibration_check', '--world',
               'drone_code/team_nav/course_maps/phone_scan_2026-11-06.json').returncode != 0
