"""Offline Gazebo startup guard tests; no live processes are signalled."""
import os
from pathlib import Path
import subprocess

import pytest

GUARD = Path(__file__).resolve().parents[1] / "scripts/lib/gazebo_guard.sh"


def check(tmp_path, code, uri="http://localhost:11345"):
    return subprocess.run(
        ["bash", "-c", 'source "$1"; LOG="$2"; ' + code,
         "test", str(GUARD), str(tmp_path)],
        env=dict(os.environ, GAZEBO_MASTER_URI=uri),
        text=True, capture_output=True, timeout=5,
    )


def test_free_port(tmp_path):
    result = check(tmp_path, 'ss() { return 0; }; gazebo_check_port')
    assert result.returncode == 0


def test_occupied_port_reports_owner_without_killing(tmp_path):
    result = check(tmp_path, """
ss() { echo 'LISTEN users:(("gzserver",pid=1234,fd=7))'; }
kill() { echo UNEXPECTED_KILL; }
gazebo_check_port
""")
    assert result.returncode == 1
    assert "pid=1234" in result.stderr
    assert "UNEXPECTED_KILL" not in result.stdout


def test_custom_port_used(tmp_path):
    result = check(tmp_path, """
ss() { [[ "$3" == "sport = :11400" ]] || return 2; }
gazebo_check_port
""", uri="http://localhost:11400")
    assert result.returncode == 0


def test_invalid_uri_fails(tmp_path):
    result = check(tmp_path, "gazebo_check_port", uri="http://localhost:bad")
    assert result.returncode == 1
    assert "Invalid GAZEBO_MASTER_URI" in result.stderr


@pytest.mark.parametrize("line", [
    "[ERROR] [gzserver-3]: process has died [pid 1234, exit code 255]",
    "[INFO] [gzserver-3]: process has finished cleanly [pid 1234]",
])
def test_server_exit_detected_even_with_live_launch(tmp_path, line):
    (tmp_path / "gazebo.log").write_text(line)
    result = check(tmp_path, 'GAZEBO_LAUNCH_PID=$$; gazebo_check_alive')
    assert result.returncode == 1
    assert "aborting startup" in result.stderr


def test_live_launch_without_failure(tmp_path):
    (tmp_path / "gazebo.log").write_text("[INFO] [gzserver-3]: process started")
    result = check(tmp_path, 'GAZEBO_LAUNCH_PID=$$; gazebo_check_alive')
    assert result.returncode == 0


def test_launch_exit_detected(tmp_path):
    (tmp_path / "gazebo.log").write_text("")
    result = check(tmp_path, 'kill() { return 1; }; GAZEBO_LAUNCH_PID=1234; gazebo_check_alive')
    assert result.returncode == 1


def test_server_must_belong_to_this_launch(tmp_path):
    result = check(tmp_path, """
GAZEBO_LAUNCH_PID=1234
pgrep() { [[ "$*" == "-P 1234 -x gzserver" ]]; }
gazebo_has_server
""")
    assert result.returncode == 0


def test_failure_cleanup_children_before_parent(tmp_path):
    result = check(tmp_path, """
pgrep() { if [[ "$2" == 1234 ]]; then echo 5678; fi; }
kill() { echo "$*"; }
gazebo_stop_launch_tree 1234
""")
    assert result.returncode == 0
    assert result.stdout.splitlines() == ["-TERM 5678", "-TERM 1234"]
