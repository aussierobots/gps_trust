#!/usr/bin/env bash
#
# install_gpstrust_service.sh
#
# Set up a dedicated gpstrust user, environment file, and systemd service
# for running the gps_trust stack on a station.
#
# Usage: sudo -E ./setup/install_gpstrust_service.sh

set -euo pipefail

# --- Helpers ---------------------------------------------------------------

prompt_default() {
    local prompt="$1"
    local default="$2"
    local var
    read -r -p "$prompt [$default]: " var
    if [ -z "$var" ]; then
        echo "$default"
    else
        echo "$var"
    fi
}

prompt_required() {
    local prompt="$1"
    local var=""
    while [ -z "$var" ]; do
        read -r -p "$prompt: " var
        if [ -z "$var" ]; then
            echo "Value is required."
        fi
    done
    echo "$var"
}

prompt_secret() {
    local prompt="$1"
    local var=""
    while [ -z "$var" ]; do
        read -r -s -p "$prompt: " var
        echo
        if [ -z "$var" ]; then
            echo "Value is required."
        fi
    done
    echo "$var"
}

# --- Sanity checks ---------------------------------------------------------

if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root (use sudo)." >&2
    exit 1
fi

# Determine repo root (directory above this script)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Detected gps_trust repo root (this script): $REPO_ROOT"
echo

# --- Service user setup ----------------------------------------------------

# Allow override via GPSTRUST_USER in the environment
SERVICE_USER_DEFAULT="${GPSTRUST_USER:-gpstrust}"
SERVICE_USER="$(prompt_default 'Service user name' "$SERVICE_USER_DEFAULT")"

if id "$SERVICE_USER" >/dev/null 2>&1; then
    echo "User '$SERVICE_USER' already exists – reusing."
else
    echo "Creating user '$SERVICE_USER'..."
    useradd -m -s /bin/bash "$SERVICE_USER"
fi

echo "Adding '$SERVICE_USER' to dialout and plugdev groups (for USB/serial access)..."
usermod -aG dialout,plugdev "$SERVICE_USER" || true

# --- Paths & workspace -----------------------------------------------------

SERVICE_HOME="$(getent passwd "$SERVICE_USER" | cut -d: -f6)"

# ROS base install prefix, e.g. /opt/ros/rolling
ROS_INSTALL_PREFIX_DEFAULT="${ROS_INSTALL_PREFIX:-/opt/ros/rolling}"
ROS_INSTALL_PREFIX="$(prompt_default 'ROS install prefix (ROS_INSTALL_PREFIX)' "$ROS_INSTALL_PREFIX_DEFAULT")"

# gps_trust workspace (GT)
ROS2_GT_DEFAULT="${ROS2_GT:-$SERVICE_HOME/gps_trust}"
ROS2_GT="$(prompt_default 'gps_trust workspace path (ROS2_GT)' "$ROS2_GT_DEFAULT")"

GT_SETUP_BASH_DEFAULT="${GT_SETUP_BASH:-$ROS2_GT/install/setup.bash}"
GT_SETUP_BASH="$(prompt_default 'gps_trust setup.bash (GT_SETUP_BASH)' "$GT_SETUP_BASH_DEFAULT")"

# ublox_dgnss workspace (UD)
ROS2_UD_DEFAULT="${ROS2_UD:-$SERVICE_HOME/ublox_dgnss}"
ROS2_UD="$(prompt_default 'ublox_dgnss workspace path (ROS2_UD)' "$ROS2_UD_DEFAULT")"

SERVICE_UD_REPO_DIR="$(prompt_default 'ublox_dgnss repo path for service user' "$ROS2_UD")"

UD_SETUP_BASH_DEFAULT="${UD_SETUP_BASH:-$ROS2_UD/install/setup.bash}"
UD_SETUP_BASH="$(prompt_default 'ublox_dgnss setup.bash (UD_SETUP_BASH)' "$UD_SETUP_BASH_DEFAULT")"

# gps_trust repo dir (within GT workspace)
SERVICE_REPO_DIR="$(prompt_default 'gps_trust repo path for service user' "$ROS2_GT")"



echo
echo "Service user home:       $SERVICE_HOME"
echo "ublox_dgnss repo path:   $SERVICE_UD_REPO_DIR"
echo "gps_trust repo path:     $SERVICE_REPO_DIR"
echo

# --- Repo management: ublox_dgnss git clone / git pull ----------------------

UD_GIT_URL_DEFAULT="${UBLOX_DGNSS_GIT_URL:-https://github.com/aussierobots/ublox_dgnss.git}"
UD_GIT_URL="$(prompt_default 'Git URL for ublox_dgnss repo' "$UD_GIT_URL_DEFAULT")"

# Ensure src directory exists
mkdir -p "$(dirname "$SERVICE_UD_REPO_DIR")"

if [ ! -d "$SERVICE_UD_REPO_DIR/.git" ]; then
    echo "No ublox_dgnss git repository found at '$SERVICE_UD_REPO_DIR'."
    echo "Cloning $UD_GIT_URL into '$SERVICE_UD_REPO_DIR' as $SERVICE_USER..."
    sudo -u "$SERVICE_USER" git clone "$UD_GIT_URL" "$SERVICE_UD_REPO_DIR"
else
    echo "Existing ublox_dgnss git repo detected at '$SERVICE_UD_REPO_DIR'."
    read -r -p "Run 'git pull' in ublox_dgnss repo as $SERVICE_USER? [Y/n]: " UD_PULL_CHOICE
    if [[ ! "$UD_PULL_CHOICE" =~ ^[Nn]$ ]]; then
        sudo -u "$SERVICE_USER" bash -lc "
          cd '$SERVICE_UD_REPO_DIR'
          if ! git diff --quiet --ignore-submodules HEAD; then
              echo '⚠ Local changes detected in ublox_dgnss — resetting to origin/main to avoid merge errors.'
              git fetch origin main
              git reset --hard origin/main
          else
              git pull --ff-only
          fi
        "
    else
        echo "Skipping git pull for ublox_dgnss; using existing checkout."
    fi
fi

# Ensure ownership
chown -R "$SERVICE_USER:$SERVICE_USER" "$SERVICE_UD_REPO_DIR" || true

# --- Repo management: git clone / git pull ---------------------------------

GIT_URL_DEFAULT="${GPSTRUST_GIT_URL:-https://github.com/aussierobots/gps_trust.git}"
GIT_URL="$(prompt_default 'Git URL for gps_trust repo' "$GIT_URL_DEFAULT")"

if [ ! -d "$SERVICE_REPO_DIR/.git" ]; then
    echo "No git repository found at '$SERVICE_REPO_DIR'."

    # If we're running inside a git repo, offer clone vs copy
    if git -C "$REPO_ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        echo "Current script is running from a git repo at: $REPO_ROOT"
        read -r -p "Clone from '$GIT_URL' as $SERVICE_USER instead of copying local tree? [Y/n]: " CLONE_CHOICE
        if [[ ! "$CLONE_CHOICE" =~ ^[Nn]$ ]]; then
            echo "Cloning $GIT_URL into '$SERVICE_REPO_DIR' as $SERVICE_USER..."
            sudo -u "$SERVICE_USER" git clone "$GIT_URL" "$SERVICE_REPO_DIR"
        else
            echo "Copying current repo tree to '$SERVICE_REPO_DIR'..."
            mkdir -p "$SERVICE_REPO_DIR"
            rsync -a --delete --exclude ".git" "$REPO_ROOT/" "$SERVICE_REPO_DIR/"
        fi
    else
        # Not running from a git repo at all → must clone
        echo "Cloning $GIT_URL into '$SERVICE_REPO_DIR' as $SERVICE_USER..."
        sudo -u "$SERVICE_USER" git clone "$GIT_URL" "$SERVICE_REPO_DIR"
    fi
else
    echo "Existing git repo detected at '$SERVICE_REPO_DIR'."
    read -r -p "Run 'git pull' in that repo as $SERVICE_USER? [Y/n]: " PULL_CHOICE
    if [[ ! "$PULL_CHOICE" =~ ^[Nn]$ ]]; then
        sudo -u "$SERVICE_USER" bash -lc "
          cd '$SERVICE_REPO_DIR'
          if ! git diff --quiet --ignore-submodules HEAD; then
              echo '⚠ Local changes detected — resetting to origin/main to avoid merge errors.'
              git fetch origin main
              git reset --hard origin/main
          else
              git pull --ff-only
          fi
        "
    else
        echo "Skipping git pull; using existing checkout."
    fi
fi

# Ensure ownership is correct
chown -R "$SERVICE_USER:$SERVICE_USER" "$SERVICE_REPO_DIR" || true

# --- Build the workspace (colcon) -----------------------------------------

echo
echo "Running colcon build for ublox_dgnss in '$ROS2_UD' as $SERVICE_USER..."

sudo -u "$SERVICE_USER" bash -lc "
  if [ -r '$ROS_INSTALL_PREFIX/setup.bash' ]; then
      source '$ROS_INSTALL_PREFIX/setup.bash'
  else
      echo 'WARNING: ROS base setup $ROS_INSTALL_PREFIX/setup.bash not found; proceeding without it.' >&2
  fi

  mkdir -p '$ROS2_UD/src'
  cd '$ROS2_UD'
  colcon build --symlink-install
"

echo
echo "Checking for workspace setup files..."

if [ ! -r "$UD_SETUP_BASH" ]; then
    echo "ERROR: Expected ublox_dgnss workspace setup '$UD_SETUP_BASH' not found." >&2
    exit 1
fi

echo
echo "Running colcon build for gps_trust in '$ROS2_GT' as $SERVICE_USER..."

sudo -u "$SERVICE_USER" bash -lc "
  if [ -r '$ROS_INSTALL_PREFIX/setup.bash' ]; then
      source '$ROS_INSTALL_PREFIX/setup.bash'
  else
      echo 'WARNING: ROS base setup $ROS_INSTALL_PREFIX/setup.bash not found; proceeding without it.' >&2
  fi

  if [ -r '$UD_SETUP_BASH' ]; then
      source '$UD_SETUP_BASH'
  else
      echo 'WARNING: ublox_dgnss workspace setup $UD_SETUP_BASH not found; proceeding without it.' >&2
  fi

  cd '$ROS2_GT'
  colcon build --symlink-install
"

echo
echo "Checking for workspace setup files..."

if [ ! -r "$UD_SETUP_BASH" ]; then
    echo "ERROR: Expected ublox_dgnss workspace setup '$UD_SETUP_BASH' not found." >&2
    exit 1
fi

if [ ! -r "$GT_SETUP_BASH" ]; then
    echo "ERROR: Expected gps_trust workspace setup '$GT_SETUP_BASH' not found." >&2
    exit 1
fi

# Ensure gpstrust.sh is executable (we assume scripts/gpstrust.sh in the repo)
if [ -f "$SERVICE_REPO_DIR/scripts/gpstrust.sh" ]; then
    chmod +x "$SERVICE_REPO_DIR/scripts/gpstrust.sh"
else
    echo "WARNING: $SERVICE_REPO_DIR/scripts/gpstrust.sh not found."
    echo "The systemd service will fail until that script exists."
fi

# --- Environment variables -------------------------------------------------

echo
echo "Configure environment (values will be written to /etc/gpstrust.env):"
echo

# API key: allow API_KEY from environment as default
GPS_TRUST_DEVICE_API_KEY_DEFAULT="${GPS_TRUST_DEVICE_API_KEY:-}"
if [ -n "$GPS_TRUST_DEVICE_API_KEY_DEFAULT" ]; then
    GPS_TRUST_DEVICE_API_KEY="$(prompt_default 'API key for gps_trust_device (GPS_TRUST_DEVICE_API_KEY)' "$GPS_TRUST_DEVICE_API_KEY_DEFAULT")"
else
    GPS_TRUST_DEVICE_API_KEY="$(prompt_required 'API key for gps_trust_device (GPS_TRUST_DEVICE_API_KEY)')"
fi

# Defaults derived from current env if set, otherwise sane station defaults
NTRIP_HOST_DEFAULT="${NTRIP_HOST:-ntrip.data.gnss.ga.gov.au}"
NTRIP_PORT_DEFAULT="${NTRIP_PORT:-443}"
NTRIP_MOUNTPOINT_DEFAULT="${NTRIP_MOUNTPOINT:-MBCH00AUS0}"
USE_HTTPS_DEFAULT="${USE_HTTPS:-true}"

NTRIP_HOST="$(prompt_default 'NTRIP host (NTRIP_HOST)' "$NTRIP_HOST_DEFAULT")"
NTRIP_PORT="$(prompt_default 'NTRIP port (NTRIP_PORT)' "$NTRIP_PORT_DEFAULT")"

# Username: default from env if present, otherwise required
if [ -n "${NTRIP_USERNAME-}" ]; then
    NTRIP_USERNAME="$(prompt_default 'NTRIP username (NTRIP_USERNAME)' "$NTRIP_USERNAME")"
else
    NTRIP_USERNAME="$(prompt_required 'NTRIP username (NTRIP_USERNAME)')"
fi

# Password: if NTRIP_PASSWORD already set, allow pressing Enter to reuse it
if [ -n "${NTRIP_PASSWORD-}" ]; then
    echo "NTRIP password (NTRIP_PASSWORD): [press Enter to reuse existing value]"
    read -r -s NTRIP_PASSWORD_INPUT
    echo
    if [ -n "$NTRIP_PASSWORD_INPUT" ]; then
        NTRIP_PASSWORD="$NTRIP_PASSWORD_INPUT"
    fi
else
    NTRIP_PASSWORD="$(prompt_secret 'NTRIP password (NTRIP_PASSWORD)')"
fi

NTRIP_MOUNTPOINT="$(prompt_default 'NTRIP mountpoint (NTRIP_MOUNTPOINT)' "$NTRIP_MOUNTPOINT_DEFAULT")"
USE_HTTPS="$(prompt_default 'Use HTTPS? (USE_HTTPS true/false)' "$USE_HTTPS_DEFAULT")"

# Device type: F9P family vs X20P
GPS_TRUST_DEVICE_TYPE_DEFAULT="${GPS_TRUST_DEVICE_TYPE:-F9P}"

read -r -p "u-blox device family [F9P/X20P] (GPS_TRUST_DEVICE_TYPE) [$GPS_TRUST_DEVICE_TYPE_DEFAULT]: " GPS_TRUST_DEVICE_TYPE_INPUT
if [ -z "$GPS_TRUST_DEVICE_TYPE_INPUT" ]; then
    GPS_TRUST_DEVICE_TYPE="$GPS_TRUST_DEVICE_TYPE_DEFAULT"
else
    GPS_TRUST_DEVICE_TYPE="$GPS_TRUST_DEVICE_TYPE_INPUT"
fi

# Normalise a bit
GPS_TRUST_DEVICE_TYPE="$(echo "$GPS_TRUST_DEVICE_TYPE" | tr '[:lower:]' '[:upper:]')"

# Log and PID dirs: default from env if set, otherwise standard locations
LOG_DIR_DEFAULT="${LOG_DIR:-/var/log/gpstrust}"
PID_DIR_DEFAULT="${PID_DIR:-/run/gpstrust}"

LOG_DIR="$(prompt_default 'Log directory (LOG_DIR)' "$LOG_DIR_DEFAULT")"
PID_DIR="$(prompt_default 'PID directory (PID_DIR)' "$PID_DIR_DEFAULT")"

# --- Write /etc/gpstrust.env ----------------------------------------------

ENV_FILE="/etc/gpstrust.env"
echo
echo "Writing environment file to $ENV_FILE..."

cat > "$ENV_FILE" <<EOF
# Autogenerated by install_gpstrust_service.sh

ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ROS_INSTALL_PREFIX="$ROS_INSTALL_PREFIX"

ROS2_GT="$ROS2_GT"
GT_SETUP_BASH="$GT_SETUP_BASH"

ROS2_UD="$ROS2_UD"
UD_SETUP_BASH="$UD_SETUP_BASH"

GPS_TRUST_DEVICE_API_KEY="$GPS_TRUST_DEVICE_API_KEY"
GPS_TRUST_DEVICE_TYPE="$GPS_TRUST_DEVICE_TYPE"

NTRIP_HOST="$NTRIP_HOST"
NTRIP_PORT="$NTRIP_PORT"
NTRIP_USERNAME="$NTRIP_USERNAME"
NTRIP_PASSWORD="$NTRIP_PASSWORD"
NTRIP_MOUNTPOINT="$NTRIP_MOUNTPOINT"
USE_HTTPS="$USE_HTTPS"

LOG_DIR="$LOG_DIR"
PID_DIR="$PID_DIR"
EOF

chown root:"$SERVICE_USER" "$ENV_FILE"
chmod 640 "$ENV_FILE"

echo "Environment file created."

# --- Create log and runtime directories ------------------------------------

echo
echo "Creating log and PID directories..."

mkdir -p "$LOG_DIR"
chown "$SERVICE_USER:$SERVICE_USER" "$LOG_DIR"

# PID_DIR will usually be managed by systemd via RuntimeDirectory, but we can
# create it now in case the script is run manually.
mkdir -p "$PID_DIR"
chown "$SERVICE_USER:$SERVICE_USER" "$PID_DIR" || true

echo "Log dir: $LOG_DIR"
echo "PID dir: $PID_DIR"

# --- Write systemd service --------------------------------------------------

SERVICE_FILE="/etc/systemd/system/gpstrust.service"
echo
echo "Writing systemd unit to $SERVICE_FILE..."

cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=GPS Trust ROS 2 Stack
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes

User=$SERVICE_USER
Group=$SERVICE_USER

RuntimeDirectory=gpstrust

EnvironmentFile=/etc/gpstrust.env

WorkingDirectory=$SERVICE_REPO_DIR
ExecStart=$SERVICE_REPO_DIR/scripts/gpstrust.sh start
ExecStop=$SERVICE_REPO_DIR/scripts/gpstrust.sh stop
ExecReload=$SERVICE_REPO_DIR/scripts/gpstrust.sh restart

StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "Systemd unit created."

# --- Reload systemd and optionally enable/start -----------------------------

echo
echo "Reloading systemd daemon..."
systemctl daemon-reload

read -r -p "Enable gpstrust.service to start at boot? [Y/n]: " ENABLE_CHOICE
if [[ ! "$ENABLE_CHOICE" =~ ^[Nn]$ ]]; then
    systemctl enable gpstrust.service
    echo "Service enabled."
else
    echo "Service not enabled."
fi

read -r -p "Start gpstrust.service now? [Y/n]: " START_CHOICE
if [[ ! "$START_CHOICE" =~ ^[Nn]$ ]]; then
    systemctl start gpstrust.service
    systemctl status gpstrust.service --no-pager || true
else
    echo "Service not started. You can start it later with:"
    echo "  sudo systemctl start gpstrust.service"
fi

echo
echo "Setup complete."
echo "You can check logs with:"
echo "  journalctl -u gpstrust.service -f"
