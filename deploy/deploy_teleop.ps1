# ---------------------------------------------------------------------------
# 一鍵把遙操 bridge 節點 + 腳本佈署到 cam 的 /tmp。
# 在【本機 Windows PowerShell】執行（不是 cam 上）：
#     pwsh deploy\deploy_teleop.ps1
# 需求：本機能 `ssh cam`（cam 上進的是 Windows PowerShell，內部用 wsl 執行 bash）。
#
# 佈署方式沿用專案慣例：base64 經 stdin 餵入，再 `sed 's/\r$//'` 去 CR，
# 徹底避開引號地獄與 CRLF 問題。.sh 會 chmod +x。
# 注意：/tmp 的檔案重開機不留，重開機後需重跑本腳本。
# ---------------------------------------------------------------------------
param(
  [string]$CamHost = 'cam'   # ssh 目標（~/.ssh/config 的別名或 user@host）
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot   # teleop_share/

$sets = @(
  @{ dir = 'nodes';   files = @('pink_arm_ik.py','ready_pose.py','pink_engage.py','gripper_bridge.py','engage_button.py','head_bridge.py') },
  @{ dir = 'scripts'; files = @('teleop_env.sh','ready_run.sh','pink_run.sh','gripper_run.sh','engage_run.sh','head_run.sh','restart_pink.sh','restart_head.sh','teleop_up.sh','teleop_down.sh') },
  @{ dir = 'diag';    files = @('live_ctrl.py','head_live.py','headpose_live.py','rate_probe.py','real_check.sh','list_ctrl.py','head_probe.py','probe_topics.py','btn_detect.py') },
  # 真機堆疊啟動/停止/準備範本（cam 專屬，路徑/busid/使用者請先對照環境調整）
  @{ dir = 'robot';   files = @('run_real_robot.sh','run_real_stop.sh','real_prep.sh') }
)

foreach ($s in $sets) {
  foreach ($f in $s.files) {
    $src = Join-Path (Join-Path $root $s.dir) $f
    if (-not (Test-Path $src)) { Write-Output "MISS: $src"; continue }
    $dst = "/tmp/$f"
    $b64 = [Convert]::ToBase64String([IO.File]::ReadAllBytes($src))
    $chmod = ''
    if ($f -like '*.sh') { $chmod = " && chmod +x $dst" }
    $remote = "wsl bash -lc `"echo $b64 | base64 -d > $dst && sed -i 's/\r`$//' $dst$chmod`""
    ssh $CamHost $remote
    Write-Output "sent $f -> $dst"
  }
}
Write-Output ''
Write-Output '=== 佈署完成。接著在 cam WSL 執行： ==='
Write-Output '  bash /tmp/teleop_up.sh real headpose      # 真機 + 頭顯朝向'
Write-Output '  bash /tmp/teleop_up.sh isaac headpose     # Isaac 模擬 + 頭顯朝向'