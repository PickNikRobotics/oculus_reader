#!/usr/bin/env bash
#
# One-time setup for the Quest teleop pipeline inside the MoveIt Pro dev
# container. Installs:
#   - adb (apt), for talking to the Quest over USB.
#   - The pure-python-adb / numpy / pyyaml pip packages.
#   - A .pth file that exposes this repo's 'oculus_reader' Python module on
#     the system Python path, so the data_collection ROS package can import it.
#
# After this script + a colcon build, you can launch the teleop from any
# terminal that sources the workspace overlay:
#
#     source install/setup.bash
#     ros2 launch data_collection data_collection.launch.py
#
# Inside a container, system Python is the right place for these deps -- the
# container is the isolation boundary, and a venv on top would just fight
# 'ros2 launch' / 'ros2 run'.
#
# Usage:
#     bash container_install.sh

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

# --- oculus_reader (this repo's plain Python module) ----------------------
bold "==> Exposing the oculus_reader Python module on sys.path"
# The 'oculus_reader' Python module lives at ${REPO_DIR}/oculus_reader/
# (upstream layout). The data_collection ROS package needs to be able to
# 'from oculus_reader.reader import OculusReader'.
#
# We don't pip-install it: a setup.py at the repo root would make colcon see
# the whole repo as a Python package and stop descending, hiding the
# data_collection ament_python package one level deeper. Instead we drop a
# .pth file that adds the repo root to system Python's sys.path. Then
# 'import oculus_reader' resolves to ${REPO_DIR}/oculus_reader/ directly,
# tracking source changes with no editable-install indirection.
SITE_PACKAGES=$(python3 -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
PTH_FILE="${SITE_PACKAGES}/oculus_reader.pth"
echo "writing ${PTH_FILE} -> ${REPO_DIR}"
echo "${REPO_DIR}" | $SUDO tee "${PTH_FILE}" >/dev/null

# --- smoke test ------------------------------------------------------------
bold "==> Verifying imports"
# Verify under the *current* user, who is the one that will run the node.
# This catches the "installed-but-only-in-/root/.local" case.
python3 - <<'PY'
import sys
try:
    from ppadb.client import Client  # noqa: F401
    from oculus_reader.reader import OculusReader  # noqa: F401
    import numpy, yaml
except ImportError as e:
    sys.stderr.write(
        f"\033[1;31mverification failed:\033[0m {e}\n"
        f"sys.path = {sys.path}\n"
    )
    sys.exit(1)
print(f"  numpy {numpy.__version__}, pyyaml {yaml.__version__}")
print(f"  ppadb importable from {Client.__module__}")
print(f"  oculus_reader importable from {OculusReader.__module__}")
PY

# --- next steps ------------------------------------------------------------
green ""
green "Container setup complete."
echo ""
echo "Next steps:"
echo "  1. Build the ROS package (if you haven't already):"
echo "       colcon build --packages-select data_collection"
echo "  2. Source the workspace overlay in each new terminal:"
echo "       source install/setup.bash"
echo "  3. Launch teleop:"
echo "       ros2 launch data_collection data_collection.launch.py"
echo ""
echo "Re-run this script after a container rebuild, or fold its contents into"
echo "the dev container Dockerfile to make the deps permanent."
