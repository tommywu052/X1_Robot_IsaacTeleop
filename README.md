# X1 Quest3 遙操套件（Windows + WSL2 版）

用 Quest3（或桌面 IWER 模擬）遙控 X1 機器人：**雙臂 + 夾爪 + 頭部**。
本 README 針對「後端 bridge 與機器人堆疊跑在 **WSL2（代號 cam）**、從 **Windows** 部署」的情境。
若你想把整套裝在單一 Ubuntu，改看 `README_ubuntu.md`。

- 後端/機器人堆疊：WSL2（Ubuntu + ROS 2 Jazzy）。部署動作在 **Windows PowerShell** 執行（`ssh cam`）。
- 前端可在 **同一台 cam** 或 **另一台主機**；跨機時靠 DDS `ROS_STATIC_PEERS=<前端IP>` 打通（見 `scripts/teleop_env.sh`）。

---

## 1. 三層式架構

```
 ① 前端 XR 服務            ② 後端 bridge              ③ 機器人控制堆疊
 (NVIDIA Isaac Teleop)     (★本包 / 純 Python)        (★本包已附 ros2_ws_src)
 Quest3/IWER ──CloudXR──▶  訂閱 /xr_teleop/*   ──▶    /*_controller、/head_controller
   發 /xr_teleop/*         轉成關節/軌跡指令           ros2_control + MoveIt 驅動真機
```

| | Code 來源 | 一次性安裝（只做一次） | 每次啟動 |
|---|---|---|---|
| **① 前端** | NVIDIA 參考實作 | `docker build`（§2.3） | `docker run`（§3.1）→ 發 `/xr_teleop/*` |
| **③ 機器人堆疊** | 本包已附 `ros2_ws_src/` | 在 cam WSL `colcon build`（§2.2） | `usbipd attach` + `run_real_robot.sh`（§3.2） |
| **② 後端 bridge** | ★本包 | 從 Windows `deploy_teleop.ps1` 佈署到 `cam:/tmp`（§3.3） | `bash /tmp/teleop_up.sh`（§3.3）→ 訂閱①、驅動③ |

> 前端發的 XR topic：`/xr_teleop/ee_poses`（雙手位姿）、`/xr_teleop/head_pose`（頭顯位姿）、
> `/xr_teleop/controller_data`（搖桿/按鈕/扳機）、`/xr_teleop/finger_joints`（手指彎曲）。

---

## 2. 一次性安裝（每台機器只做一次）

### 2.1 cam WSL2 相依（ROS 2 + Python venv）

在 cam 的 WSL2（`ssh cam` 後進 `wsl`，或直接開 WSL 終端）：

```bash
# ROS 2（Jazzy 範例）
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-moveit \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  python3-colcon-common-extensions python3-venv build-essential

# 後端 bridge 用的 Python venv（含 pinocchio / pink / msgpack）
python3 -m venv ~/pink_venv
~/pink_venv/bin/pip install --upgrade pip
~/pink_venv/bin/pip install pin pink msgpack numpy
```

### 2.2 ③ 機器人控制堆疊 — 在 cam 編譯本包附的 `ros2_ws_src`

本包已附三個核心套件（`pkg_robot_model` / `pkg_robot_model_moveit_config` / `xiaobei_hardware`，
**且已整合頭部控制器 `head_controller`（j_101 yaw / j_102 pitch）**）。把 `ros2_ws_src` 複製進 cam
（可先用 §3.3 的部署，或手動 scp / cp 到 WSL），再編譯一次：

```bash
mkdir -p ~/xiaobei_X1_ws/src
cp -r ros2_ws_src/* ~/xiaobei_X1_ws/src/

cd ~/xiaobei_X1_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash        # 之後每個新終端都要 source 這行
```

> 真機透過 `/dev/ttyACM0`（Feetech 匯流排伺服，1000000 baud，CH343 USB-serial 經 usbipd 掛進 WSL）連接。

### 2.3 ① 前端 XR 服務（NVIDIA Isaac Teleop 的 `teleop_ros2`）

裝 **NVIDIA 的前端**，把 Quest3 手/頭/按鈕資料經 CloudXR 收進來、以 `teleop_ros2_node`
發成 `/xr_teleop/*`。用官方 Docker 建置一次（容器自帶 CloudXR Runtime）：

- Repo：<https://github.com/NVIDIA/IsaacTeleop> ／ 文件：<https://nvidia.github.io/IsaacTeleop/>
- 我們要的發佈端是 `examples/teleop_ros2/`（不是 Quick Start 那個只印夾爪數值、不發 ROS topic 的 DEMO）。

```bash
git clone https://github.com/NVIDIA/IsaacTeleop.git
cd IsaacTeleop
# 建置一次（Jazzy；Humble 去掉 build-arg 即可）
docker build -f examples/teleop_ros2/Dockerfile \
  --build-arg ROS_DISTRO=jazzy --build-arg PYTHON_VERSION=3.12 \
  -t teleop_ros2_ref:jazzy .
```

---

## 3. 每次啟動（依 ① → ③ → ② 順序）

### 3.1 ① 啟動前端 `teleop_ros2_ref` Docker（發 `/xr_teleop/*`）

在前端主機（同 cam 或另一台）：

```bash
docker run --rm --gpus all --net=host --ipc=host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_LOCALHOST_ONLY=1 \
  -v $HOME/.cloudxr:/root/.cloudxr \
  --name teleop_ros2_ref teleop_ros2_ref:jazzy \
  --ros-args -p cloudxr_accept_eula:=true
```

開防火牆並連頭顯 / 桌面模擬：
```bash
sudo ufw allow 47998/udp; sudo ufw allow 49100,48322/tcp
# 瀏覽器（桌面會自動載入 IWER 模擬 Quest3，或用頭顯瀏覽器）開：
#   https://nvidia.github.io/IsaacTeleop/client
#   輸入前端主機 IP -> 接受自簽憑證 -> Connect
```

### 3.2 ③ 啟動機器人控制堆疊（在 cam WSL）

真機需先從 **Windows（系統管理員 PowerShell）** 把機器人 USB 掛進 WSL：

```powershell
# busid 依你的裝置調整（usbipd list 查）
usbipd attach --wsl --busid <BUSID>
```

接著在 cam WSL：

```bash
source ~/xiaobei_X1_ws/install/setup.bash
bash /tmp/run_real_robot.sh          # 真機：ros2_control + spawn 6 個控制器 + MoveIt/RViz
```
> ⚠ 真機會**上電、執行軌跡**，先淨空工作區、把電源/急停開關放在手邊。首次修 `/dev/ttyACM0`
> 權限會提示輸入 sudo 密碼（互動式）。只想跑模擬：改用 Isaac Sim + `ros2 launch pkg_robot_model moveit_isaac.launch.py`。

啟動後控制器應包含：`joint_state_broadcaster / right_arm / right_gripper / left_arm / left_gripper / head`。

### 3.3 ② 佈署並啟動後端 bridge（本包）

在 **Windows PowerShell**（能 `ssh cam`）：

```powershell
# 一鍵把 bridge 節點 + 腳本佈署到 cam:/tmp（base64 傳輸 + 去 CR + chmod）
pwsh deploy\deploy_teleop.ps1
#   若 ssh 目標不叫 cam： pwsh deploy\deploy_teleop.ps1 -CamHost user@<CAM_IP>
```

回到 cam WSL：

```bash
# 若前端在「另一台」機器：改 /tmp/teleop_env.sh 的 HORDE_IP 為前端主機 IP；
# 前端與後端同一台 cam 則維持預設 127.0.0.1。

# 啟動四個 bridge（雙臂 + 夾爪 + engage + 頭部）
bash /tmp/teleop_up.sh real headpose     # 真機 + 頭顯朝向驅動頭部
#   bash /tmp/teleop_up.sh isaac headpose # 改跑 Isaac 模擬
#   第二參數 stick = 用搖桿控頭部（預設 headpose）
```

> `/tmp` 內的檔案**重開機不留**；重開機後重跑 `deploy_teleop.ps1` 與 `teleop_up.sh` 即可。
> 停止後端：`bash /tmp/teleop_down.sh`；真機收 torque：`bash /tmp/run_real_stop.sh`。

---

## 4. 操作

1. 戴上 Quest3 / 桌面瀏覽器連上 IWER（<https://nvidia.github.io/IsaacTeleop/client>）→ 確認前端已連
   （後端可 `python3 /tmp/probe_topics.py` 看到 `/xr_teleop/*`）。
2. **按左手 Y 鍵 engage**：手臂接手，開始跟隨控制器姿態（真機先回 ready pose 安定約 3.5s）。
3. **轉頭**：驅動機器人頭部（yaw/pitch）。
4. **擠壓扳機**：控制對應側夾爪開合（免 engage，隨時可用）。
5. 再按左手 Y 可 disengage（手臂停止跟隨）。

`/xr_teleop/*` 與本包 bridge 對應：`ee_poses`→雙臂、`finger_joints`→夾爪、`head_pose`→頭部、`controller_data`→按鈕/搖桿與 engage。

---

## 5. 內容清單

```
teleop_share/
├─ README.md                   # 本檔（Windows + WSL2 版）
├─ README_ubuntu.md            # Ubuntu 單機版
├─ nodes/                      # ② 後端 bridge（純 Python，部署到 cam:/tmp）
│   ├─ pink_arm_ik.py          #   雙臂差分 IK（Pinocchio + Pink）
│   ├─ gripper_bridge.py       #   夾爪
│   ├─ head_bridge.py          #   頭部（headpose / stick）
│   ├─ engage_button.py        #   engage 按鈕（左手 Y）
│   ├─ ready_pose.py / pink_engage.py
├─ scripts/                    # ② 啟動腳本（部署到 cam:/tmp）
│   ├─ teleop_env.sh           #   共用環境（ROS/venv 路徑、HORDE_IP、DDS）
│   ├─ teleop_up.sh / teleop_down.sh
│   ├─ ready_run.sh / pink_run.sh / gripper_run.sh / engage_run.sh / head_run.sh
│   └─ restart_pink.sh / restart_head.sh
├─ deploy/
│   ├─ deploy_teleop.ps1       #   Windows→cam:/tmp 一鍵部署（本版主用）
│   ├─ install_local.sh        #   原生 Ubuntu 安裝（供 README_ubuntu）
│   └─ clean_sh.py             #   CRLF 清理小工具
├─ robot/                      # ③ 機器人堆疊啟停腳本（部署到 cam:/tmp）
│   ├─ run_real_robot.sh / run_real_stop.sh / real_prep.sh
├─ ros2_ws_src/                # ③ 機器人控制堆疊（cam colcon build 用；已整合 head_controller）
│   ├─ pkg_robot_model/                 # URDF、my_controllers.yaml、moveit_real/isaac 等 launch
│   ├─ pkg_robot_model_moveit_config/   # MoveIt 設定
│   └─ xiaobei_hardware/                # Feetech 伺服 ros2_control 硬體介面（C++）
└─ diag/                       # 診斷小工具（probe_topics / head_probe / rate_probe / ...）
```

---

## 6. 疑難排解（Windows + WSL2 常見狀況）

- **一連上手臂就亂甩**：本包預設 disengage，連上不會動；engage 是跨連線保留的 toggle，先按左手 **Y** 放開再重來。
- **頭不動、按 Y 沒反應但手臂能動**：多半是手把休眠/切到 hand-tracking（頭與按鈕靠 `controller_data`）。
  把手把拿起來按鍵喚醒即自動恢復。判別：`python3 /tmp/rate_probe.py`（壞況＝`controller_data 0Hz`）。
- **headpose 頭部零反應**：head_bridge 內建 `resub_timeout_s`(4s) 自我修復；手動救援
  `bash /tmp/restart_head.sh real headpose`。
- **headpose 上下/左右相反**：改 `head_run.sh` 傳入的 `-p yaw_sign:=` / `-p pitch_sign:=`（第 3 參數可覆寫 pitch_sign）。
- **CloudXR 串流斷線**（`0xC0F22221`、ee_poses 凍結）：先重整瀏覽器重新 Connect；無效則在前端主機
  `docker restart teleop_ros2_ref`。cam 端 bridge 會自動經 DDS 重連，不用重啟。
- **找不到 `/dev/ttyACM0`**：從 Windows 系統管理員 PowerShell `usbipd attach --wsl --busid <BUSID>`。
- **健檢**：`python3 /tmp/probe_topics.py`（列 topic）、`bash /tmp/real_check.sh`（真機控制器/硬體）、
  `python3 /tmp/list_ctrl.py`（控制器 active）、`python3 /tmp/btn_detect.py`（找對的 button_field）。