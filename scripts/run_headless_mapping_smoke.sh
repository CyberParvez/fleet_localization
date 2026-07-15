#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 3 ]]; then echo "usage: $0 ABSOLUTE_FLEET_YAML ROBOT NEW_MAP_ID" >&2; exit 2; fi
source_manifest=$1; robot=$2; map_id=$3
if [[ $source_manifest != /* || ! -f $source_manifest ]]; then echo "invalid absolute manifest: $source_manifest" >&2; exit 2; fi

run_dir=$(mktemp -d /tmp/fleet-mapping-smoke.XXXXXX)
groups=()
cleanup() {
  status=$?
  set +e
  for pgid in "${groups[@]}"; do kill -INT -- "-$pgid" 2>/dev/null; done
  sleep 2
  for pgid in "${groups[@]}"; do kill -TERM -- "-$pgid" 2>/dev/null; done
  for pgid in "${groups[@]}"; do wait "$pgid" 2>/dev/null; done
  if [[ $status -eq 0 ]]; then rm -rf "$run_dir"; else
    echo "harness failed; disposable fixture retained at $run_dir" >&2
    for log in "$run_dir"/*.log; do echo "--- $log" >&2; tail -n 30 "$log" >&2; done
  fi
  return "$status"
}
trap cleanup EXIT INT TERM

# Copy the selected immutable store and rewrite only the disposable manifest.
manifest="$run_dir/fleet.yaml"
python3 - "$source_manifest" "$manifest" "$run_dir/maps" <<'PY'
from pathlib import Path
import shutil, sys, yaml
source, target, store = map(Path, sys.argv[1:])
doc = yaml.safe_load(source.read_text())
origin = Path(doc['fleet']['map_store'])
if not origin.is_absolute(): origin = (source.parent / origin).resolve()
shutil.copytree(origin, store)
doc['fleet']['map_store'] = str(store)
target.write_text(yaml.safe_dump(doc))
PY

wait_log() { local file=$1 pattern=$2; timeout 120 bash -c "until grep -q -- '$pattern' '$file' 2>/dev/null; do sleep 1; done"; }
active() { timeout 30 bash -c "until ros2 lifecycle get '$1' 2>/dev/null | grep -q active; do sleep 1; done"; }
stop_group() { local pgid=$1; kill -INT -- "-$pgid" 2>/dev/null || true; timeout 15 bash -c "while kill -0 '$pgid' 2>/dev/null; do sleep 1; done" || kill -TERM -- "-$pgid" 2>/dev/null || true; }

setsid ros2 launch turtlebot_fleet_sim fleet_sim.launch.py fleet_config:="$manifest" gui:=false >"$run_dir/sim.log" 2>&1 &
groups+=("$!"); wait_log "$run_dir/sim.log" FLEET_READY
setsid ros2 launch fleet_localization localization.launch.py fleet_config:="$manifest" robot:="$robot" mode:=mapping rviz:=false >"$run_dir/mapping.log" 2>&1 &
mapping_pgid=$!; groups+=("$mapping_pgid")
active "/$robot/slam_toolbox"
timeout 120 ros2 topic echo --once "/$robot/map" >/dev/null

python3 "$(dirname "$0")/publish_motion_pulse.py" --robot "$robot" --seconds 5.0
ros2 run fleet_localization map_save --fleet-config "$manifest" --robot "$robot" --map-id "$map_id"
test -f "$run_dir/maps/$map_id/map.yaml" -a -f "$run_dir/maps/$map_id/world.yaml"
if ros2 run fleet_localization map_save --fleet-config "$manifest" --robot "$robot" --map-id "$map_id"; then
  echo "duplicate immutable map save unexpectedly succeeded" >&2; exit 1
fi
stop_group "$mapping_pgid"

setsid ros2 launch fleet_localization localization.launch.py fleet_config:="$manifest" robot:="$robot" map_id:="$map_id" rviz:=false >"$run_dir/load.log" 2>&1 &
groups+=("$!")
active "/$robot/map_server"; active "/$robot/amcl"
timeout 120 ros2 topic echo --once "/$robot/map" | grep -q "frame_id: $robot/map"
echo "HEADLESS_MAPPING_SAVE_LOAD_READY robot=$robot map_id=$map_id disposable_store=$run_dir/maps"
