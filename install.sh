#!/usr/bin/env bash
#
# Install oculus_reader into a local virtualenv.
#
# Usage:
#   ./install.sh                  # create .venv and install
#   ./install.sh --venv <path>    # use a custom venv path
#   ./install.sh --no-venv        # install into the current Python environment
#   ./install.sh --recreate       # delete and recreate the venv
#
# After install, activate with:
#   source .venv/bin/activate
# then run:
#   python oculus_reader/reader.py

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${REPO_DIR}/.venv"
USE_VENV=1
RECREATE=0

# --- argument parsing -------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --venv)       VENV_DIR="$2"; shift 2 ;;
        --no-venv)    USE_VENV=0;    shift   ;;
        --recreate)   RECREATE=1;    shift   ;;
        -h|--help)
            sed -n '3,12p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 2 ;;
    esac
done

# --- pretty output helpers --------------------------------------------------
bold()  { printf '\033[1m%s\033[0m\n' "$*"; }
green() { printf '\033[1;32m%s\033[0m\n' "$*"; }
yellow(){ printf '\033[1;33m%s\033[0m\n' "$*"; }
red()   { printf '\033[1;31m%s\033[0m\n' "$*"; }

# --- preflight --------------------------------------------------------------
bold "==> Checking prerequisites"

if ! command -v python3 >/dev/null 2>&1; then
    red "python3 not found. Install Python 3.8 or newer first."
    exit 1
fi
PY_VERSION="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
echo "Python: $(command -v python3) (${PY_VERSION})"

# Warn if the APK looks like an unfetched git-lfs pointer rather than a real file.
APK="${REPO_DIR}/oculus_reader/APK/teleop-debug.apk"
if [[ -f "$APK" ]]; then
    APK_SIZE=$(stat -c '%s' "$APK" 2>/dev/null || stat -f '%z' "$APK")
    if [[ "$APK_SIZE" -lt 100000 ]]; then
        yellow "APK at ${APK} looks suspiciously small (${APK_SIZE} bytes)."
        yellow "It may be a git-lfs pointer. Install git-lfs and re-pull:"
        yellow "    sudo apt install git-lfs && git lfs install && git lfs pull"
    fi
else
    yellow "APK not found at ${APK}. The APK install step will fail later."
fi

# --- venv ------------------------------------------------------------------
PIP=("python3" "-m" "pip")
if [[ $USE_VENV -eq 1 ]]; then
    if [[ $RECREATE -eq 1 && -d "$VENV_DIR" ]]; then
        bold "==> Removing existing venv at ${VENV_DIR}"
        rm -rf "$VENV_DIR"
    fi
    if [[ ! -d "$VENV_DIR" ]]; then
        bold "==> Creating venv at ${VENV_DIR}"
        # --system-site-packages lets the venv see ROS 2's rclpy / tf2_ros / etc.
        # when /opt/ros/<distro>/setup.bash has been sourced. pip installs still
        # go into the venv and shadow system packages as needed.
        python3 -m venv --system-site-packages "$VENV_DIR"
    else
        bold "==> Reusing existing venv at ${VENV_DIR}"
    fi
    PIP=("${VENV_DIR}/bin/pip")
else
    bold "==> Installing into current Python environment (no venv)"
fi

# --- install ---------------------------------------------------------------
bold "==> Upgrading pip"
"${PIP[@]}" install --upgrade pip >/dev/null

bold "==> Installing Python dependencies"
"${PIP[@]}" install -r "${REPO_DIR}/requirements.txt"

bold "==> Installing oculus_reader (editable)"
"${PIP[@]}" install -e "${REPO_DIR}"

# --- smoke test ------------------------------------------------------------
bold "==> Verifying imports"
if [[ $USE_VENV -eq 1 ]]; then
    "${VENV_DIR}/bin/python" - <<'PY'
from oculus_reader.reader import OculusReader   # noqa: F401
from oculus_reader.buttons_parser import parse_buttons  # noqa: F401
from ppadb.client import Client  # noqa: F401
import numpy, yaml
print(f"  numpy {numpy.__version__}, pyyaml {yaml.__version__}")
print("  oculus_reader imports OK")
PY
else
    python3 - <<'PY'
from oculus_reader.reader import OculusReader   # noqa: F401
from oculus_reader.buttons_parser import parse_buttons  # noqa: F401
from ppadb.client import Client  # noqa: F401
import numpy, yaml
print(f"  numpy {numpy.__version__}, pyyaml {yaml.__version__}")
print("  oculus_reader imports OK")
PY
fi

# --- adb sanity check ------------------------------------------------------
bold "==> Checking for adb"
if command -v adb >/dev/null 2>&1; then
    echo "adb: $(command -v adb) ($(adb --version | head -1))"
else
    yellow "adb not found. Install it (Ubuntu/Debian):"
    yellow "    sudo apt install android-tools-adb"
    yellow "Required to talk to the Quest. Continuing without."
fi

# --- next steps ------------------------------------------------------------
green ""
green "Install complete."
echo ""
echo "Next steps:"
echo "  1. Put the Quest in developer mode, connect it via USB-C,"
echo "     and approve the 'Allow USB Debugging' prompt in the headset."
echo "  2. Verify the device is visible:"
echo "       adb devices"
echo ""
echo "  Stream controller poses (no ROS needed):"
if [[ $USE_VENV -eq 1 ]]; then
    echo "       source ${VENV_DIR}/bin/activate"
fi
echo "       python ${REPO_DIR}/oculus_reader/reader.py"
echo ""
echo "  Publish poses as TF in ROS 2 (source ROS *first*, then activate venv):"
echo "       source /opt/ros/<distro>/setup.bash"
if [[ $USE_VENV -eq 1 ]]; then
    echo "       source ${VENV_DIR}/bin/activate"
fi
echo "       python ${REPO_DIR}/oculus_reader/visualize_oculus_transforms_ros2.py"
