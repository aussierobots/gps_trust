#!/usr/bin/env bash

# GPS Trust System Management Script
# Usage: gpstrust.sh {start|stop|status|restart}
#
# This script:
#   - starts/stops the gps_trust ROS 2 stack
#   - manages PIDs in a PID directory
#   - writes logs per component
#
# Configuration is loaded from an environment file (default: /etc/gpstrust.env)
# which should define at least:
#   ROS2_WS, SETUP_BASH, API_KEY,
#   NTRIP_HOST, NTRIP_PORT, NTRIP_USERNAME, NTRIP_PASSWORD, MOUNTPOINT, USE_HTTPS
#   LOG_DIR (optional), PID_DIR (optional)

set -eo pipefail

# ---------------------------------------------------------------------------
# Environment / configuration
# ---------------------------------------------------------------------------

# Where to read configuration from (overridable via GPSTRUST_ENV_FILE for testing)
GPSTRUST_ENV_FILE="${GPSTRUST_ENV_FILE:-/etc/gpstrust.env}"
if [ -f "$GPSTRUST_ENV_FILE" ]; then
    # shellcheck disable=SC1090
    . "$GPSTRUST_ENV_FILE"
fi

# After sourcing GPSTRUST_ENV_FILE (or letting systemd inject env)
if [ ! -r "$SETUP_BASH" ]; then
    echo "ERROR: SETUP_BASH '$SETUP_BASH' not found or not readable" >&2
    exit 1
fi

# Colours for output (harmless in systemd logs, helpful when run interactively)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Required environment variables
: "${ROS2_WS:?ROS2_WS must be set (e.g. /home/gpstrust/gps_trust)}"
: "${ROS_INSTALL_PREFIX:?ROS_INSTALL_PREFIX must be set (e.g. /opt/ros/rolling)}"
: "${SETUP_BASH:?SETUP_BASH must be set (e.g. /home/gpstrust/ros2_ws/install/setup.bash)}"
: "${GPS_TRUST_DEVICE_API_KEY:?GPS_TRUST_DEVICE_API_KEY must be set}"
: "${NTRIP_HOST:?NTRIP_HOST must be set}"
: "${NTRIP_PORT:?NTRIP_PORT must be set}"
: "${NTRIP_USERNAME:?NTRIP_USERNAME must be set}"
: "${NTRIP_PASSWORD:?NTRIP_PASSWORD must be set}"
: "${NTRIP_MOUNTPOINT:?NTRIP_MOUNTPOINT must be set}"
: "${USE_HTTPS:?USE_HTTPS must be set to true/false}"


ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-$ROS_INSTALL_PREFIX/setup.bash}"

if [ ! -r "$ROS_DISTRO_SETUP" ]; then
    echo "ERROR: ROS base setup '$ROS_DISTRO_SETUP' not found or not readable" >&2
    exit 1
fi

if [ ! -r "$SETUP_BASH" ]; then
    echo "ERROR: workspace setup '$SETUP_BASH' not found or not readable" >&2
    exit 1
fi

# Optional, with sensible defaults for service usage
LOG_DIR="${LOG_DIR:-/var/log/gpstrust}"
PID_DIR="${PID_DIR:-/run/gpstrust}"

mkdir -p "$LOG_DIR" "$PID_DIR"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Check if a process (by PID file) is running
is_running() {
    local pid_file="$1"
    if [ -f "$pid_file" ]; then
        local pid
        pid="$(cat "$pid_file")"
        if ps -p "$pid" > /dev/null 2>&1; then
            return 0
        else
            rm -f "$pid_file"
            return 1
        fi
    fi
    return 1
}

# Start a component: name + command
start_component() {
    local name="$1"
    local cmd="$2"
    local pid_file="$PID_DIR/${name}.pid"
    local log_file="$LOG_DIR/${name}_$(date +%Y%m%d_%H%M%S).log"

    if is_running "$pid_file"; then
        echo -e "${YELLOW}⚠ ${name} is already running (PID: $(cat "$pid_file"))${NC}"
        return 1
    fi

    echo -e "${BLUE}Starting ${name}...${NC}"

    (
        # Run under ROS 2 environment
        # shellcheck disable=SC1090
        set +u
        source "$SETUP_BASH"
        eval "$cmd" > "$log_file" 2>&1
    ) &

    local pid=$!
    echo "$pid" > "$pid_file"

    # Give it a moment to start
    sleep 2

    if is_running "$pid_file"; then
        echo -e "${GREEN}✓ ${name} started successfully (PID: $pid)${NC}"
        return 0
    else
        echo -e "${RED}✗ ${name} failed to start (see log: $log_file)${NC}"
        return 1
    fi
}

# Stop a component by name
stop_component() {
    local name="$1"
    local pid_file="$PID_DIR/${name}.pid"

    if is_running "$pid_file"; then
        local pid
        pid="$(cat "$pid_file")"
        echo -e "${BLUE}Stopping ${name} (PID: $pid)...${NC}"

        # Graceful shutdown
        kill -TERM "$pid" 2>/dev/null || true

        local count=0
        while [ $count -lt 10 ] && kill -0 "$pid" 2>/dev/null; do
            sleep 1
            count=$((count + 1))
        done

        # Force kill if still running
        if kill -0 "$pid" 2>/dev/null; then
            echo -e "${YELLOW}Force stopping ${name}...${NC}"
            kill -KILL "$pid" 2>/dev/null || true
        fi

        rm -f "$pid_file"
        echo -e "${GREEN}✓ ${name} stopped${NC}"
    else
        echo -e "${YELLOW}⚠ ${name} is not running${NC}"
    fi
}

# ---------------------------------------------------------------------------
# GPS stack orchestration
# ---------------------------------------------------------------------------

start_gps() {
    echo -e "${GREEN}=== Starting GPS Trust System ===${NC}"
    echo -e "${BLUE}Timestamp: $(date)${NC}\n"

    # The gps_trust satellite launch already includes the ublox driver
    start_component "gps_trust_satellite" \
        "ros2 launch gps_trust ublox_gt_hpposllh_satellite.launch.py"
    sleep 5

    start_component "ntrip_client" \
        "ros2 launch ublox_dgnss ntrip_client.launch.py use_https:=$USE_HTTPS host:=$NTRIP_HOST port:=$NTRIP_PORT username:=$NTRIP_USERNAME password:=$NTRIP_PASSWORD mountpoint:=$NTRIP_MOUNTPOINT"
    sleep 3

    start_component "gps_trust_main" \
        "ros2 launch gps_trust gps_trust.launch.py GPS_TRUST_DEVICE_API_KEY:=$GPS_TRUST_DEVICE_API_KEY"

    echo -e "\n${GREEN}GPS Trust System startup complete${NC}"

    # Quick status check
    sleep 3
    echo -e "\n${BLUE}Verifying components...${NC}"
    show_status
}

stop_gps() {
    echo -e "${YELLOW}=== Stopping GPS Trust System ===${NC}"
    echo -e "${BLUE}Timestamp: $(date)${NC}\n"

    # Stop in reverse order
    stop_component "gps_trust_main"
    stop_component "ntrip_client"
    stop_component "gps_trust_satellite"

    # Clean up any orphaned ROS 2 processes
    echo -e "\n${BLUE}Cleaning up orphaned processes...${NC}"
    pkill -f "ros2 launch ublox_dgnss" 2>/dev/null || true
    pkill -f "ros2 launch gps_trust" 2>/dev/null || true
    killall -q component_container component_container_mt 2>/dev/null || true

    echo -e "\n${GREEN}GPS Trust System stopped${NC}"
}

restart_gps() {
    echo -e "${YELLOW}=== Restarting GPS Trust System ===${NC}\n"
    stop_gps
    echo -e "\n${BLUE}Waiting 5 seconds before restart...${NC}"
    sleep 5
    start_gps
}

# ---------------------------------------------------------------------------
# Status and diagnostics
# ---------------------------------------------------------------------------

show_status() {
    echo -e "${BLUE}=== GPS Trust System Status ===${NC}"
    echo -e "${BLUE}Timestamp: $(date)${NC}\n"

    local all_running=true
    local pid_file pid

    echo -e "${BLUE}Component Status:${NC}"

    # gps_trust_satellite
    pid_file="$PID_DIR/gps_trust_satellite.pid"
    if is_running "$pid_file"; then
        pid="$(cat "$pid_file")"
        echo -e "${GREEN}✓ gps_trust_satellite: Running (managed, PID: $pid)${NC}"
    elif pgrep -f "ros2 launch gps_trust ublox_gt_hpposllh_satellite" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ gps_trust_satellite: Running (unmanaged)${NC}"
    else
        echo -e "${RED}✗ gps_trust_satellite: Not running${NC}"
        all_running=false
    fi

    # ntrip_client
    pid_file="$PID_DIR/ntrip_client.pid"
    if is_running "$pid_file"; then
        pid="$(cat "$pid_file")"
        echo -e "${GREEN}✓ ntrip_client: Running (managed, PID: $pid)${NC}"
    elif pgrep -f "ros2 launch ublox_dgnss ntrip_client" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ ntrip_client: Running (unmanaged)${NC}"
    else
        echo -e "${RED}✗ ntrip_client: Not running${NC}"
        all_running=false
    fi

    # gps_trust_main
    pid_file="$PID_DIR/gps_trust_main.pid"
    if is_running "$pid_file"; then
        pid="$(cat "$pid_file")"
        echo -e "${GREEN}✓ gps_trust_main: Running (managed, PID: $pid)${NC}"
    elif pgrep -f "ros2 launch gps_trust gps_trust.launch" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ gps_trust_main: Running (unmanaged)${NC}"
    else
        echo -e "${RED}✗ gps_trust_main: Not running${NC}"
        all_running=false
    fi

    echo -e "\n${BLUE}ROS2 Nodes:${NC}"
    set +u
    if source "$SETUP_BASH" 2>/dev/null; then
        local nodes
        nodes="$(ros2 node list 2>/dev/null | grep -E 'ublox|ntrip|gps_trust' | wc -l || echo 0)"
        if [ "$nodes" -gt 0 ]; then
            echo -e "${GREEN}✓ $nodes GPS-related nodes detected${NC}"
            ros2 node list 2>/dev/null | grep -E 'ublox|ntrip|gps_trust' | sed 's/^/  /'

            if [ "$all_running" = false ] && [ "$nodes" -ge 3 ]; then
                echo -e "${YELLOW}  Note: Components started outside of this script${NC}"
                all_running=true
            fi
        else
            echo -e "${RED}✗ No GPS nodes detected${NC}"
        fi
    fi

    echo -e "\n${BLUE}GPS Topics:${NC}"
    set +u
    if source "$SETUP_BASH" 2>/dev/null; then
        local topics
        topics="$(ros2 topic list 2>/dev/null | grep -E 'gps|rtcm|ubx_nav' | wc -l || echo 0)"
        if [ "$topics" -gt 0 ]; then
            echo -e "${GREEN}✓ $topics GPS topics available${NC}"

            echo -e "\n${BLUE}Checking data flow...${NC}"

            # gps_trust_indicator
            local trust_data
            trust_data="$(timeout 3 ros2 topic echo /gps_trust_indicator --once 2>/dev/null || true)"
            if echo "$trust_data" | grep -q "trust_level"; then
                local trust_level trust_status
                trust_level="$(echo "$trust_data" | grep "trust_level:" | awk '{print $2}')"
                trust_status="$(echo "$trust_data" | grep "status:" | awk '{print $2}')"
                echo -e "${GREEN}✓ GPS trust indicator: Level ${trust_level}/10 - Status: ${trust_status}${NC}"
            else
                echo -e "${RED}✗ GPS trust indicator: No data${NC}"
            fi

            # RTCM corrections
            if timeout 5 ros2 topic echo /ntrip_client/rtcm --once 2>/dev/null | grep -q "message"; then
                echo -e "${GREEN}✓ RTCM corrections: Receiving from NTRIP${NC}"
            else
                echo -e "${YELLOW}⚠ RTCM corrections: No data (check NTRIP connection)${NC}"
            fi

            # Navigation data and accuracy
            local nav_data
            nav_data="$(timeout 5 ros2 topic echo /ubx_nav_hp_pos_llh --once 2>/dev/null || true)"
            if echo "$nav_data" | grep -q "lat"; then
                local h_acc v_acc
                h_acc="$(echo "$nav_data" | grep "h_acc:" | awk '{print $2}')"
                v_acc="$(echo "$nav_data" | grep "v_acc:" | awk '{print $2}')"

                # Convert from mm to cm
                local h_acc_cm v_acc_cm
                h_acc_cm="$(echo "scale=1; $h_acc / 10" | bc 2>/dev/null || echo "$h_acc mm")"
                v_acc_cm="$(echo "scale=1; $v_acc / 10" | bc 2>/dev/null || echo "$v_acc mm")"
                echo -e "${GREEN}✓ Navigation: H.Acc: ${h_acc_cm}cm, V.Acc: ${v_acc_cm}cm${NC}"

                local lat lon
                lat="$(echo "$nav_data" | grep "^lat:" | awk '{print $2}')"
                lon="$(echo "$nav_data" | grep "^lon:" | awk '{print $2}')"
                if [ -n "${lat:-}" ] && [ -n "${lon:-}" ]; then
                    local lat_deg lon_deg
                    lat_deg="$(echo "scale=7; $lat / 10000000" | bc 2>/dev/null || echo "N/A")"
                    lon_deg="$(echo "scale=7; $lon / 10000000" | bc 2>/dev/null || echo "N/A")"
                    echo -e "${BLUE}  Position: ${lat_deg}°, ${lon_deg}°${NC}"
                fi
            else
                echo -e "${RED}✗ Navigation data: No position data${NC}"
            fi
        else
            echo -e "${RED}✗ No GPS topics found${NC}"
        fi
    fi

    echo -e "\n${BLUE}Overall Status:${NC}"
    if [ "$all_running" = true ]; then
        echo -e "${GREEN}✓ All components running${NC}"
    else
        echo -e "${YELLOW}⚠ Some components are not running${NC}"
    fi
}

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

show_usage() {
    echo "GPS Trust System Management Script"
    echo "Usage: $0 {start|stop|status|restart}"
    echo
    echo "Commands:"
    echo "  start   - Start all GPS components"
    echo "  stop    - Stop all GPS components"
    echo "  status  - Show status of GPS components"
    echo "  restart - Restart all GPS components"
    echo
    echo "Environment is loaded from: ${GPSTRUST_ENV_FILE}"
    echo "  ROS2_WS:        $ROS2_WS"
    echo "  NTRIP Host:     $NTRIP_HOST:$NTRIP_PORT"
    echo "  NTRIP Mount:    $NTRIP_MOUNTPOINT"
    echo "  Log Directory:  $LOG_DIR"
    echo "  PID Directory:  $PID_DIR"
}

cmd="${1:-}"

case "${cmd,,}" in
    start|-start|--start)
        start_gps
        ;;
    stop|-stop|--stop)
        stop_gps
        ;;
    status|-status|--status)
        show_status
        ;;
    restart|-restart|--restart)
        restart_gps
        ;;
    *)
        show_usage
        exit 1
        ;;
esac

exit 0
