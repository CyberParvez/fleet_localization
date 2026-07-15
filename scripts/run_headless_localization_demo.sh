#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then echo "usage: $0 ABSOLUTE_FLEET_YAML ROBOT [ROBOT ...]" >&2; exit 2; fi
manifest=$1; shift
robots=("$@")
if [[ $manifest != /* || ! -f $manifest ]]; then echo "invalid absolute manifest: $manifest" >&2; exit 2; fi

run_dir=$(mktemp -d /tmp/fleet-localization-smoke.XXXXXX)
groups=()
localization_groups=()
captures=()
cleanup() {
  status=$?
  set +e
  for pgid in "${groups[@]}"; do kill -INT -- "-$pgid" 2>/dev/null; done
  for pid in "${captures[@]}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  for pgid in "${groups[@]}"; do kill -TERM -- "-$pgid" 2>/dev/null; done
  for pgid in "${groups[@]}"; do wait "$pgid" 2>/dev/null; done
  if [[ $status -eq 0 ]]; then rm -rf "$run_dir"; else
    echo "harness failed; logs retained at $run_dir" >&2
    for log in "$run_dir"/*.log; do echo "--- $log" >&2; tail -n 30 "$log" >&2; done
  fi
  return "$status"
}
trap cleanup EXIT INT TERM

wait_log() { local file=$1 pattern=$2; timeout 120 bash -c "until grep -q -- '$pattern' '$file' 2>/dev/null; do sleep 1; done"; }
wait_topic() { timeout 120 ros2 topic echo --once "$1" >/dev/null; }
active() { timeout 30 bash -c "until ros2 lifecycle get '$1' 2>/dev/null | grep -q active; do sleep 1; done"; }

# No world override: the manifest is the sole simulator/localization provenance.
setsid ros2 launch turtlebot_fleet_sim fleet_sim.launch.py fleet_config:="$manifest" gui:=false \
  >"$run_dir/sim.log" 2>&1 &
groups+=("$!")
wait_log "$run_dir/sim.log" FLEET_READY

for robot in "$@"; do
  setsid ros2 launch fleet_localization localization.launch.py \
    fleet_config:="$manifest" robot:="$robot" rviz:=false \
    >"$run_dir/$robot.log" 2>&1 &
  groups+=("$!")
  localization_groups+=("$!")
done

for robot in "$@"; do
  active "/$robot/map_server"
  active "/$robot/amcl"
  timeout 120 ros2 topic echo --once "/$robot/amcl_pose" >"$run_dir/$robot-amcl-pose.txt" &
  capture=$!
  captures+=("$capture")
  timeout 120 ros2 topic echo "/$robot/localization/health" >"$run_dir/$robot-health.txt" &
  health_capture=$!
  captures+=("$health_capture")
  timeout 15 bash -c "until ros2 topic info '/$robot/amcl_pose' 2>/dev/null | grep -Eq 'Subscription count: [1-9]'; do sleep 0.2; done"
  timeout 15 bash -c "until ros2 topic info '/$robot/localization/health' 2>/dev/null | grep -Eq 'Subscription count: [1-9]'; do sleep 0.2; done"
  python3 "$(dirname "$0")/publish_initialpose.py" --fleet-config "$manifest" --robot "$robot"
  # 0.08 m/s for 5 s exceeds the configured AMCL update_min_d=0.25 m.
  python3 "$(dirname "$0")/publish_motion_pulse.py" --robot "$robot" --seconds 5.0
  wait "$capture"
  health_marker=$(wc -c <"$run_dir/$robot-health.txt")
  timeout 120 bash -c "until tail -c +$((health_marker + 1)) '$run_dir/$robot-health.txt' | grep -q 'message: localized'; do sleep 0.2; done"
  kill "$health_capture" 2>/dev/null || true
  timeout 30 ros2 topic echo --once "/$robot/map" | grep -q "frame_id: $robot/map"
  wait_topic "/$robot/odometry/filtered"
  timeout 30 bash -c "until timeout 3 ros2 run tf2_ros tf2_echo '$robot/odom' '$robot/base_footprint' 2>&1 | grep -q 'Translation:'; do sleep 0.2; done"
done

if [[ ${#robots[@]} -ge 2 ]]; then
  stopped_robot=${robots[0]}
  survivor=${robots[1]}
  stopped_group=${localization_groups[0]}
  kill -INT -- "-$stopped_group"
  timeout 20 bash -c "while kill -0 '$stopped_group' 2>/dev/null; do sleep 0.2; done" || kill -TERM -- "-$stopped_group" 2>/dev/null || true
  wait "$stopped_group" 2>/dev/null || true
  timeout 30 bash -c "until ! ros2 lifecycle get '/$stopped_robot/amcl' >/dev/null 2>&1; do sleep 0.2; done"
  active "/$survivor/map_server"
  active "/$survivor/amcl"
  timeout 30 ros2 topic echo --once "/$survivor/map" | grep -q "frame_id: $survivor/map"
  wait_topic "/$survivor/odometry/filtered"
  timeout 30 ros2 topic echo "/$survivor/localization/health" >"$run_dir/$survivor-survivor-health.txt" &
  survivor_health=$!
  captures+=("$survivor_health")
  timeout 15 bash -c "until ros2 topic info '/$survivor/localization/health' 2>/dev/null | grep -Eq 'Subscription count: [1-9]'; do sleep 0.2; done"
  survivor_marker=$(wc -c <"$run_dir/$survivor-survivor-health.txt")
  timeout 30 bash -c "until tail -c +$((survivor_marker + 1)) '$run_dir/$survivor-survivor-health.txt' | grep -Eq 'message: (localizing|localized)'; do sleep 0.2; done"
  kill "$survivor_health" 2>/dev/null || true
  timeout 30 bash -c "until timeout 3 ros2 run tf2_ros tf2_echo '$survivor/odom' '$survivor/base_footprint' 2>&1 | grep -q 'Translation:'; do sleep 0.2; done"
  setsid ros2 launch fleet_localization localization.launch.py \
    fleet_config:="$manifest" robot:="$stopped_robot" rviz:=false \
    >"$run_dir/$stopped_robot-restart.log" 2>&1 &
  groups+=("$!")
  active "/$stopped_robot/map_server"
  active "/$stopped_robot/amcl"
  timeout 30 ros2 topic echo --once "/$stopped_robot/map" | grep -q "frame_id: $stopped_robot/map"
  echo "FAILURE_ISOLATION_READY stopped=$stopped_robot survivor=$survivor restarted=$stopped_robot"
fi

echo "HEADLESS_LOCALIZATION_READY manifest=$manifest robots=$*"
