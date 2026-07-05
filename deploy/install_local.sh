#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 原生 Ubuntu 單機安裝：把本套件的節點與腳本複製到 /tmp（免 base64 / CRLF 處理）。
#   用法：bash deploy/install_local.sh
# 注意：/tmp 重開機不留，重開機後重跑本腳本即可。
# ---------------------------------------------------------------------------
set -e
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # teleop_share/
DST=/tmp
echo "installing from $SRC -> $DST"

cp "$SRC"/nodes/*.py    "$DST"/
cp "$SRC"/scripts/*.sh  "$DST"/
cp "$SRC"/diag/*        "$DST"/
# 真機堆疊範本（若不需要真機可忽略）
for f in run_real_robot.sh run_real_stop.sh real_prep.sh; do
  [ -f "$SRC/robot/$f" ] && cp "$SRC/robot/$f" "$DST"/ || true
done

# 若檔案曾在 Windows 編輯過，去掉 CR 保險
for f in "$DST"/*.sh; do sed -i 's/\r$//' "$f"; done
chmod +x "$DST"/*.sh

echo "done."
echo "下一步（確認機器人堆疊已在跑後）："
echo "  bash /tmp/teleop_up.sh real headpose      # 真機 + 頭顯朝向"
echo "  bash /tmp/teleop_up.sh isaac headpose     # Isaac 模擬 + 頭顯朝向"