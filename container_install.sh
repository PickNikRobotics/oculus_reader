#!/usr/bin/env bash
#
# One-time setup for running oculus_reader inside the MoveIt Pro dev container.
#
# Inside a container, system Python is the right place for this package's deps
# (the container is already the isolation boundary; a venv on top is redundant
# and would also fight ros2 launch / ros2 run). After this script, any terminal
# in the container that sources the workspace overlay can launch the teleop
# directly:
#
#     source install/setup.bash
#     ros2 launch oculus_reader teleoperate.launch.py
#
# Usage:
#     bash container_install.sh

set -euo pipefail

# --- pretty output ---------------------------------------------------------
bold()   { printf '\033[1m%s\033[0m\n' "$*"; }
green()  { printf '\033[1;32m%s\033[0m\n' "$*"; }
yellow() { printf '\033[1;33m%s\033[0m\n' "$*"; }
red()    { printf '\033[1;31m%s\033[0m\n' "$*"; }

# Auto-prefix sudo when we are not root.
SUDO=""
if [[ "$(id -u)" -ne 0 ]]; then
    if command -v sudo >/dev/null 2>&1; then
        SUDO="sudo"
    else
        red "Not running as root and sudo is not installed. Re-run as root."
        exit 1
    fi
fi

# --- adb (apt) -------------------------------------------------------------
bold "==> Installing adb (android-tools-adb)"
if command -v adb >/dev/null 2>&1; then
    echo "adb already installed: $(adb --version | head -1)"
else
    $SUDO apt-get update -qq
    $SUDO apt-get install -y --no-install-recommends android-tools-adb
    echo "adb installed: $(adb --version | head -1)"
fi

# --- pip deps (system-wide) -----------------------------------------------
bold "==> Installing Python dependencies system-wide"
# We deliberately install system-wide (no --user). In the container, system
# Python is already where ROS 2 and rclpy live; matching that path avoids
# surprises when ros2 launch / ros2 run start the script under whatever user
# the operator happens to be.
#
# --ignore-installed forces pip to write to system-site (/usr/local/lib/...)
# even if a stale '--user' copy exists in /root/.local or ~/.local from a
# previous run -- those would otherwise shadow the install and pip would
# helpfully report "already satisfied" without actually fixing anything for
# the user who'll run the node (typically not root).
$SUDO -H pip3 install --upgrade --no-cache-dir --ignore-installed \
    pure-python-adb numpy pyyaml

# --- smoke test ------------------------------------------------------------
bold "==> Verifying imports"
# Verify under the *current* user, who is the one that will run the node.
# This catches the "installed-but-only-in-/root/.local" case.
python3 - <<'PY'
import sys
try:
    from ppadb.client import Client  # noqa: F401
    import numpy, yaml
except ImportError as e:
    sys.stderr.write(
        f"\033[1;31mverification failed:\033[0m {e}\n"
        f"sys.path = {sys.path}\n"
    )
    sys.exit(1)
print(f"  numpy {numpy.__version__}, pyyaml {yaml.__version__}")
print(f"  ppadb importable from {Client.__module__}")
PY

# --- next steps ------------------------------------------------------------
green ""
green "Container setup complete."
echo ""
echo "Next steps:"
echo "  1. Build the package (if you haven't already):"
echo "       colcon build --packages-select oculus_reader"
echo "  2. Source the workspace overlay in each new terminal:"
echo "       source install/setup.bash"
echo "  3. Launch teleop:"
echo "       ros2 launch oculus_reader teleoperate.launch.py"
echo ""
echo "Re-run this script after a container rebuild, or fold its contents into"
echo "the dev container Dockerfile to make the deps permanent."
