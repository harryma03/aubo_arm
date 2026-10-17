"""Offline regression tests: never launch ROS nodes or signal live processes."""
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


def run(*args, cwd=None, env=None):
    return subprocess.run(
        args, cwd=cwd, env=env, text=True, capture_output=True, timeout=20,
    )


def cli(*args, **kwargs):
    return run("bash", str(ROOT / "aubo.sh"), *args, **kwargs)


def test_help_from_another_directory(tmp_path):
    result = cli("help", cwd=tmp_path)
    assert result.returncode == 0
    assert "including hardware driver" in result.stdout


@pytest.mark.parametrize("args", [
    ("pick", "0"), ("spawn", "7"), ("pick", "abc"),
    ("spawn", "1", "2"), ("start", "invalid"), ("unknown",),
])
def test_invalid_arguments_rejected_before_runtime(args):
    result = cli(*args)
    assert result.returncode == 2


def test_stop_help_is_non_mutating():
    result = cli("stop", "--help")
    assert result.returncode == 0
    assert "Usage:" in result.stdout
    assert "Cleanup command finished" not in result.stdout


def test_stop_bounds_ros2_daemon_refresh():
    source = (ROOT / "scripts/stop.sh").read_text()
    assert "timeout --signal=TERM --kill-after=1s 5s" in source
    assert "--ros-domain-id $domain_id" in source
    assert 'kill -KILL "$pid"' in source


def make_stub(tmp_path, name, content):
    path = tmp_path / "scripts" / name
    path.parent.mkdir(exist_ok=True)
    path.write_text(content)
    return dict(os.environ, AUBO_GRASP_WS=str(tmp_path))


def test_pick_forwards_count(tmp_path):
    env = make_stub(tmp_path, "pick.sh", 'printf "%s" "$AUBO_RANDOM_CUBE_COUNT"')
    result = cli("pick", "4", env=env)
    assert result.returncode == 0
    assert result.stdout == "4"


def test_gui_flags(tmp_path):
    env = make_stub(
        tmp_path, "start.sh",
        'printf "%s %s %s" "$AUBO_GAZEBO_GUI" "$AUBO_MOVEIT_RVIZ" "$AUBO_GAZEBO_RVIZ"',
    )
    fake_display = tmp_path / "xdpyinfo"
    fake_display.write_text("#!/bin/sh\nexit 0\n")
    fake_display.chmod(0o755)
    env.update(DISPLAY=":stub", PATH=str(tmp_path) + os.pathsep + env["PATH"])
    result = cli("start", "gui", env=env)
    assert result.returncode == 0
    assert result.stdout == "true true false"


def test_config_loader_preserves_nounset(tmp_path):
    config = tmp_path / "custom.env"
    config.write_text("CLI_TEST_VALUE=custom\n")
    dds = tmp_path / "dds.xml"
    dds.write_text("<CycloneDDS/>\n")
    env = dict(
        os.environ, AUBO_CONFIG_FILE=str(config),
        AUBO_CYCLONEDDS_CONFIG=str(dds),
    )
    env.pop("CYCLONEDDS_URI", None)
    result = run(
        "bash", "-uc",
        'source "$1"; aubo_load_config; [[ "$-" == *u* ]] || exit 3; '
        '[[ "$-" != *a* ]] || exit 4; '
        'bash -c \'printf "%s|%s" "$CLI_TEST_VALUE" "$CYCLONEDDS_URI"\'',
        "test", str(ROOT / "scripts/lib/env.sh"), env=env,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout == f"custom|file://{dds}"


@pytest.mark.parametrize("signal", ["TERM", "KILL"])
def test_cleanup_tree_skips_self_and_signals_children_first(signal):
    source = (ROOT / "scripts/stop.sh").read_text()
    # Extract only function definitions, never the top-level cleanup workflow.
    functions = source.split("collect_descendants() {", 1)[1]
    functions = "collect_descendants() {" + functions.split(
        "# Gazebo plugins", 1,
    )[0]
    mocks = r'''
pgrep() {
  if [[ "$1" == "-f" ]]; then
    printf '%s\n' "$$" 910001
  elif [[ "$2" == 910001 ]]; then
    printf '%s\n' "$$" 910002
  fi
}
kill() {
  if [[ "$1" != "-0" ]]; then printf 'SIGNAL %s %s\n' "$1" "$2"; fi
  return 0
}
'''
    result = run(
        "bash", "-euc", functions + mocks + '\nsignal_tree_pattern "$1" stub test',
        "test", signal,
    )
    assert result.returncode == 0, result.stderr
    assert [line for line in result.stdout.splitlines() if line.startswith("SIGNAL")] == [
        f"SIGNAL -{signal} 910002", f"SIGNAL -{signal} 910001",
    ]


def test_gitignore_keeps_sources_and_hardware(tmp_path):
    # Isolated repository: the user's workspace is not initialized or modified.
    assert run("git", "init", "-q", str(tmp_path)).returncode == 0
    (tmp_path / ".gitignore").write_text((ROOT / ".gitignore").read_text())
    ignored = [
        "build/pkg/result", "install/lib/libaubo.so", "run_logs/gazebo.log",
        ".vscode/browse.vc.db", "archive/backup.tar.gz",
        "src/aubo_ros2_driver/aubo_description/urdf/aubo_i10.urdf.xacro",
    ]
    retained = [
        "src/aubo_ros2_driver/aubo_ros2_driver/CMakeLists.txt",
        "src/aubo_ros2_driver/aubo_description/meshes/aubo_i10/base.STL",
        "src/aubo_ros2_driver/aubo_description/urdf/aubo_i10.urdf",
        "src/aubo_ros2_driver/aubo_description/urdf/xacro/aubo.urdf.xacro",
        "src/aubo_workcell_description/config/workcell_layout.env",
        "src/aubo_moveit_config/config/controllers.yaml",
    ]
    for path in ignored:
        assert run("git", "check-ignore", "-q", path, cwd=tmp_path).returncode == 0
    for path in retained:
        assert run("git", "check-ignore", "-q", path, cwd=tmp_path).returncode == 1

