#!/usr/bin/env bash
# Unified launcher: boots the whole RC car stack with one command.
#
#   ./run.sh
#
# Starts (in the background) bridge / icp_slam / voxel_mapper / frontier_planner /
# motion_controller / rerun_viz, then runs the control console in the FOREGROUND of
# this terminal -- it needs a real TTY for its keystrokes, so it cannot be launched
# as a ROS node. Quitting the console (q) or Ctrl-C tears the whole graph down.
#
# Run inside WSL2 with ROS2 Humble installed. See ROS2_SETUP.md.

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HERE/ros2_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
RERUN_PORT="${RERUN_PORT:-9876}"

DO_BUILD=0
WANT_VIZ=1
WANT_CONSOLE=1
FORCE_SPAWN=0
VERBOSE=0
GRAPH_LOG="${GRAPH_LOG:-/tmp/rc_car_graph.log}"
CAR_HOST="${CAR_HOST:-rccar.local}"
CAR_PORT="${CAR_PORT:-9001}"
CONNECT_ADDR=""
CONTINUOUS=false
START_ENABLED=false

usage() {
    cat <<EOF
Usage: ./run.sh [options]

Boots the full ROS2 graph + Rerun visualizer, then the control console.
Console keys once running:
  c = calibrate   p = plan   g = GO   h/SPACE = HOLD/abort   q = quit

Options:
  --build            colcon build before launching (also automatic if the
                     workspace has never been built)
  --car HOST[:PORT]  ESP32 address for car_driver_node.
                     Default: $CAR_HOST:$CAR_PORT. The sketch prints its IP
                     on serial at boot; use that if mDNS does not resolve.
  --connect ADDR     Rerun viewer to connect to, e.g. 192.168.1.50:9876.
                     Default: auto-detect the Windows host.
  --spawn            Run the Rerun viewer inside WSL via WSLg instead of
                     connecting to one on Windows (slower).
  --no-viz           Do not start rerun_viz_node at all.
  --no-console       Graph only; do not start the control console.
  --continuous       Replan on every /map instead of only when you press p.
  --go               Arm driving at boot (default is HOLD -- the safe default).
  --verbose          Keep the node logs on screen. Off by default because the
                     nodes log at 10 Hz and would overwrite the console line;
                     they go to $GRAPH_LOG instead.
  -h, --help         This message.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --build)      DO_BUILD=1 ;;
        --connect)    CONNECT_ADDR="${2:-}"; shift
                      [ -n "$CONNECT_ADDR" ] || { echo "--connect needs an address" >&2; exit 2; } ;;
        --car)        car_arg="${2:-}"; shift
                      [ -n "$car_arg" ] || { echo "--car needs a host" >&2; exit 2; }
                      CAR_HOST="${car_arg%%:*}"
                      case "$car_arg" in *:*) CAR_PORT="${car_arg##*:}" ;; esac ;;
        --spawn)      FORCE_SPAWN=1 ;;
        --no-viz)     WANT_VIZ=0 ;;
        --no-console) WANT_CONSOLE=0 ;;
        --continuous) CONTINUOUS=true ;;
        --go)         START_ENABLED=true ;;
        --verbose)    VERBOSE=1 ;;
        -h|--help)    usage; exit 0 ;;
        *)            echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

# --- sanity: this only runs under Linux/WSL2 -------------------------------
if [ ! -f "$ROS_SETUP" ]; then
    echo "ERROR: $ROS_SETUP not found." >&2
    echo "This launcher runs inside WSL2 with ROS2 Humble. See ROS2_SETUP.md." >&2
    exit 1
fi

# --- Windows host detection (for the Rerun viewer) -------------------------
windows_host() {
    local mode gw
    mode="$(wslinfo --networking-mode 2>/dev/null || true)"
    if [ "$mode" = "mirrored" ]; then
        # WSL shares the host's network stack: the viewer is on localhost.
        echo "127.0.0.1"
        return
    fi
    # NAT: the default gateway is the Windows host.
    gw="$(ip route show default 2>/dev/null | awk '{print $3; exit}')"
    if [ -z "$gw" ]; then
        gw="$(awk '/^nameserver/ {print $2; exit}' /etc/resolv.conf 2>/dev/null)"
    fi
    echo "$gw"
}

reachable() {   # reachable HOST PORT -- 1s TCP connect probe
    timeout 1 bash -c "exec 3<>/dev/tcp/$1/$2" 2>/dev/null
}

VIZ=false
if [ "$WANT_VIZ" -eq 1 ]; then
    VIZ=true
    if [ "$FORCE_SPAWN" -eq 1 ]; then
        CONNECT_ADDR=""
        echo "Rerun: spawning the viewer in WSL (WSLg)."
    elif [ -n "$CONNECT_ADDR" ]; then
        echo "Rerun: connecting to $CONNECT_ADDR"
    else
        host="$(windows_host)"
        if [ -n "$host" ] && reachable "$host" "$RERUN_PORT"; then
            CONNECT_ADDR="$host:$RERUN_PORT"
            echo "Rerun: connecting to the Windows viewer at $CONNECT_ADDR"
        else
            CONNECT_ADDR=""
            echo "Rerun: no viewer answering on ${host:-<unknown>}:$RERUN_PORT --" \
                 "falling back to a WSLg viewer."
            echo "       (For the faster path: run 'rerun' on Windows first," \
                 "then re-run this script.)"
        fi
    fi
fi

# --- build if asked, or if the workspace has never been built --------------
# ROS's own setup scripts read unset variables (AMENT_TRACE_SETUP_FILES et al),
# so nounset has to come off around every source of them.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

if [ "$DO_BUILD" -eq 1 ] || [ ! -f "$WS/install/setup.bash" ]; then
    echo "Building autonomous_rc_car_ros..."
    ( cd "$WS" && colcon build --packages-select autonomous_rc_car_ros ) || {
        echo "ERROR: colcon build failed -- not launching." >&2
        exit 1
    }
fi

set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# --- launch the graph in its own process group so we can kill all of it ----
LAUNCH_PID=""
cleanup() {
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo
        echo "Shutting the graph down..."
        kill -TERM "-$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 1 20); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.25
        done
        kill -KILL "-$LAUNCH_PID" 2>/dev/null
    fi
}
trap cleanup EXIT INT TERM

# The nodes log at 10 Hz. With the console sharing this terminal that spam
# overwrites its status line, so send it to a file unless --verbose.
QUIET_LOG=0
if [ "$WANT_CONSOLE" -eq 1 ] && [ "$VERBOSE" -eq 0 ]; then
    QUIET_LOG=1
fi

if [ "$QUIET_LOG" -eq 1 ]; then
    setsid ros2 launch autonomous_rc_car_ros bringup.launch.py \
        viz:="$VIZ" \
        connect_addr:="$CONNECT_ADDR" \
        continuous:="$CONTINUOUS" \
        start_enabled:="$START_ENABLED" \
        car_host:="$CAR_HOST" \
        car_port:="$CAR_PORT" > "$GRAPH_LOG" 2>&1 &
else
    setsid ros2 launch autonomous_rc_car_ros bringup.launch.py \
        viz:="$VIZ" \
        connect_addr:="$CONNECT_ADDR" \
        continuous:="$CONTINUOUS" \
        start_enabled:="$START_ENABLED" \
        car_host:="$CAR_HOST" \
        car_port:="$CAR_PORT" &
fi
LAUNCH_PID=$!

echo "Graph starting (pid $LAUNCH_PID)."
echo "Point the iPhone app at this laptop, port 9000."
echo "Car: $CAR_HOST:$CAR_PORT   (console shows the link state)"

if [ "$WANT_CONSOLE" -eq 0 ]; then
    echo "No console (--no-console). Ctrl-C to stop."
    wait "$LAUNCH_PID"
    exit $?
fi

[ "$QUIET_LOG" -eq 1 ] && echo "Node logs -> $GRAPH_LOG   (tail -f $GRAPH_LOG)"

# Give the nodes a moment so their startup logs do not scribble over the console.
sleep 3
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: the graph exited during startup." >&2
    if [ "$QUIET_LOG" -eq 1 ]; then
        echo "--- last 20 lines of $GRAPH_LOG ---" >&2
        tail -20 "$GRAPH_LOG" >&2
    fi
    exit 1
fi

echo "Starting the control console (p=plan  g=go  h/SPACE=hold  q=quit)"
ros2 run autonomous_rc_car_ros motion_enable_node
