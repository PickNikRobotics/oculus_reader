# shellcheck shell=bash
#
# Activate the oculus_reader venv in the *current* shell.
#
# IMPORTANT: This script must be sourced, not executed.
#
# Usage:
#   source enter_venv.sh           # just the venv
#   source enter_venv.sh --ros     # also source /opt/ros/${ROS_DISTRO}/setup.bash
#                                  # (defaults to humble; override with ROS_DISTRO=...)
#
# Exit the venv later with: deactivate

# --- sourced vs executed detection ---------------------------------------
if ! (return 0 2>/dev/null); then
    printf '\033[1;31merror:\033[0m enter_venv.sh must be sourced, not executed.\n'
    printf 'Run:  source %s %s\n' "$0" "$*"
    exit 1
fi

# --- locate the venv next to this script ---------------------------------
# ${BASH_SOURCE[0]} is the path of *this* file, even when sourced.
_OCULUS_REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_OCULUS_VENV="${_OCULUS_REPO_DIR}/.venv"

# --- arg parsing ---------------------------------------------------------
_OCULUS_WITH_ROS=0
for _arg in "$@"; do
    case "$_arg" in
        --ros)  _OCULUS_WITH_ROS=1 ;;
        -h|--help)
            sed -n '3,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            unset _OCULUS_REPO_DIR _OCULUS_VENV _OCULUS_WITH_ROS _arg
            return 0
            ;;
        *)
            printf '\033[1;31merror:\033[0m unknown argument: %s\n' "$_arg"
            unset _OCULUS_REPO_DIR _OCULUS_VENV _OCULUS_WITH_ROS _arg
            return 2
            ;;
    esac
done
unset _arg

# --- optionally source ROS first (must be before venv activation) --------
if [[ ${_OCULUS_WITH_ROS} -eq 1 ]]; then
    _OCULUS_DISTRO="${ROS_DISTRO:-humble}"
    _OCULUS_ROS_SETUP="/opt/ros/${_OCULUS_DISTRO}/setup.bash"
    if [[ -f "${_OCULUS_ROS_SETUP}" ]]; then
        # shellcheck disable=SC1090
        source "${_OCULUS_ROS_SETUP}"
        printf 'sourced ROS 2 %s (%s)\n' "${_OCULUS_DISTRO}" "${_OCULUS_ROS_SETUP}"
    else
        printf '\033[1;33mwarning:\033[0m %s not found; skipping ROS source.\n' "${_OCULUS_ROS_SETUP}"
    fi
    unset _OCULUS_DISTRO _OCULUS_ROS_SETUP
fi

# --- activate the venv ---------------------------------------------------
if [[ ! -f "${_OCULUS_VENV}/bin/activate" ]]; then
    printf '\033[1;31merror:\033[0m venv not found at %s\n' "${_OCULUS_VENV}"
    printf 'Run ./install.sh first.\n'
    unset _OCULUS_REPO_DIR _OCULUS_VENV _OCULUS_WITH_ROS
    return 1
fi

# shellcheck disable=SC1091
source "${_OCULUS_VENV}/bin/activate"
printf 'activated venv at %s\n' "${_OCULUS_VENV}"
printf 'leave with: \033[1mdeactivate\033[0m\n'

unset _OCULUS_REPO_DIR _OCULUS_VENV _OCULUS_WITH_ROS
