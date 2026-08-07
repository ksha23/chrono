#!/bin/bash
# Run the scenario suite serially in the VSG viewer, one window at a time.
#
# No frame capture: SAVE_FRAMES writes a PNG per rendered frame and costs about 13x real time,
# so it is left off here. These run at roughly 0.9x real time.
#
# Each entry covers a different actor kind or road network, so the set doubles as a visual check
# that vehicles, pedestrians, cyclists and misc objects all import correctly.
#
# Usage:  ./demos_live/run_all_scenarios.sh [esmini_root]
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
E="${1:-${ESMINI_ROOT:-$ROOT/../esmini}}"
X="$E/resources/xosc"
A="$E/test/OSC-ALKS-scenarios/Variations"

# name | binary | scenario | seconds | note
RUNS=(
  "cut-in            |scenario_vsg    |$X/cut-in_external.xosc                                             |24|overtake, cut-in at 7.5 s, then hard braking"
  "acc-test          |scenario_vsg    |$X/acc-test.xosc                                                    |20|closes on a lead car, lane change out and back"
  "pedestrian        |scenario_vsg    |$X/alks_pedestrian.xosc                                             |12|pedestrian mesh in lane, ego passes at 2.4 m"
  "drop-bike         |scenario_vsg    |$X/drop-bike.xosc                                                   |10|car + cyclist + misc box, all three actor kinds"
  "ALKS 4.3_2 brake  |scenario_vsg    |$A/ALKS_Scenario_4.3_2_FollowLeadVehicleEmergencyBrake_Variation.xosc|20|UN R157 lead-vehicle emergency brake, 2.0 m/s"
  "ALKS 4.4_1 cut-in |scenario_vsg    |$A/ALKS_Scenario_4.4_1_CutInNoCollision_Variation.xosc               |20|UN R157 cut-in"
  "junction: pedestrian|intersection_vsg|$X/pedestrian_collision.xosc                                      |14|fabriksgatan junction, VRU crossing, 0.8 m"
  "junction: LTAP    |intersection_vsg|$X/ltap-od.xosc                                                     |12|fabriksgatan junction, left turn across path"
)

i=0
total=${#RUNS[@]}
for entry in "${RUNS[@]}"; do
  IFS='|' read -r name bin scen secs note <<< "$entry"
  i=$((i + 1))
  name="$(echo "$name" | xargs)"; bin="$(echo "$bin" | xargs)"; scen="$(echo "$scen" | xargs)"; secs="$(echo "$secs" | xargs)"
  printf '\n========================================================================\n'
  printf '[%d/%d] %s\n        %s\n        %s\n' "$i" "$total" "$name" "$note" "$(basename "$scen")"
  printf '========================================================================\n'
  if [ ! -f "$scen" ]; then
    echo "  missing scenario file, skipping"
    continue
  fi
  "$DIR/$bin" "$scen" "$secs" 2>&1 |
    grep -vE "VK_|Vulkan|extensions|Desired section|Loaded JSON|^\t|^\[|^esmini " || true
done

printf '\nall %d scenarios done\n' "$total"
