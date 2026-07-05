#!/usr/bin/env bash
# engage 按鈕：左手 Y（left_secondary_click）切換 pink 接手 / 放開。預設 disengage。
source /tmp/teleop_env.sh
exec "$TELEOP_PY" /tmp/engage_button.py --ros-args \
  -p button_field:=left_secondary_click \
  -p start_engaged:=false