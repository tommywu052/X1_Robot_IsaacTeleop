#!/usr/bin/env bash
# 一鍵重啟雙臂 IK：停節點 -> 重擺 ready -> 重啟節點。
# 本腳本自身 cmdline 不含 'pink_arm_ik.py'，故底下的 pkill 不會誤殺自己。
set +e
echo "== stop node =="
pkill -f pink_arm_ik.py
sleep 1.5
echo "== re-pose to ready =="
bash /tmp/ready_run.sh
echo "== relaunch node =="
nohup bash /tmp/pink_run.sh > /tmp/pink.log 2>&1 &
disown
sleep 3
echo "== node log =="
tail -3 /tmp/pink.log
echo "== proc =="
pgrep -af pink_arm_ik.py