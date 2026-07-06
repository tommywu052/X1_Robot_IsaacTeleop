# X1 Quest3 遙操套件（Ubuntu 原生單機版）

用 Quest3（或桌面 IWER 模擬）遙控 X1 機器人：**雙臂 + 夾爪 + 頭部**。
本 README 假設你是「乾淨的 Ubuntu、什麼都還沒裝」，一步步帶你裝完並跑起來。

- 系統：Ubuntu 22.04 / 24.04、ROS 2（Humble/Jazzy）、NVIDIA GPU（前端 CloudXR 需要）。
- 以下所有指令預設「前端、機器人堆疊、後端」都跑在**同一台 Ubuntu**。

---

## 變更紀錄 (Change History)

### 2026-07-06 — Quest3 方向校正 + 左夾爪修復
- 頭部（headpose）與手腕旋轉方向校正。
- 修復左夾爪（j_7）不動：`joint_map["j_7"]` direction 改 `-1`。
- 新增 `diag/servo_tool.cpp`（Feetech 伺服狀態 / torque 診斷）。

---

## 1. 三層式架構

```
 ① 前端 XR 服務            ② 後端 bridge              ③ 機器人控制堆疊
 (NVIDIA Isaac Teleop)     (★本包 / 純 Python)        (★本包已附 ros2_ws_src)
 Quest3/IWER ──CloudXR──▶  訂閱 /xr_teleop/*   ──▶    /*_controller、/head_controller
   發 /xr_teleop/*         轉成關節/軌跡指令           ros2_control + MoveIt 驅動真機
```

| | Code來源 | 一次性安裝（只做一次） | 每次啟動 |
|---|---|---|---|
| **① 前端** | NVIDIA 參考實作 | `docker build`（§2.3） | `docker run`（§3.1）→ 發 `/xr_teleop/*` |
| **③ 機器人堆疊** | 本包已附 `ros2_ws_src/` | 放進工作區 `colcon build`（§2.2） | `bash /tmp/run_real_robot.sh`（§3.2） |
| **② 後端 bridge** | ★本包 | `bash deploy/install_local.sh`（§3.3） | `bash /tmp/teleop_up.sh`（§3.3）→ 訂閱①、驅動③ |


---

## 2. 一次性安裝（每台機器只做一次）

### 2.1 系統相依（ROS 2 + Python venv）

```bash
# ROS 2（Jazzy 範例；Humble 換成 humble）
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-moveit \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  python3-colcon-common-extensions python3-venv build-essential

# 後端 bridge 用的 Python venv（含 pinocchio / pink / msgpack）
python3 -m venv ~/pink_venv
~/pink_venv/bin/pip install --upgrade pip
~/pink_venv/bin/pip install pin pink msgpack numpy
```

### 2.2 ③ 機器人控制堆疊 — 編譯本包附的 `ros2_ws_src`

本包已附三個核心套件（`pkg_robot_model` / `pkg_robot_model_moveit_config` / `xiaobei_hardware`，
**且已整合頭部控制器 `head_controller`（j_101 yaw / j_102 pitch）**）。放進工作區編譯一次即可：

```bash
mkdir -p ~/xiaobei_X1_ws/src
cp -r ros2_ws_src/* ~/xiaobei_X1_ws/src/

cd ~/xiaobei_X1_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash        # 之後每個新終端都要 source 這行
```

> 真機透過 `/dev/ttyACM0`（Feetech 匯流排伺服，1000000 baud）連接；接上後確認裝置存在、可寫。

### 2.3 ① 前端 XR 服務（NVIDIA Isaac Teleop 的 `teleop_ros2`）

這步是裝 **NVIDIA Repo的前端**，它把 Quest3 手/頭/按鈕資料經 CloudXR 收進來、以 `teleop_ros2_node`
發成 `/xr_teleop/*`。用官方 Docker 建置一次（容器自帶 CloudXR Runtime）：

- Repo：<https://github.com/NVIDIA/IsaacTeleop> ／ 文件：<https://nvidia.github.io/IsaacTeleop/>
- 建置 `teleop_ros2_ref` Docker Container 發佈端

```bash
git clone https://github.com/NVIDIA/IsaacTeleop.git
cd IsaacTeleop
# 建置一次（Jazzy；Humble 去掉 build-arg 即可）
docker build -f examples/teleop_ros2/Dockerfile \
  --build-arg ROS_DISTRO=jazzy --build-arg PYTHON_VERSION=3.12 \
  -t teleop_ros2_ref:jazzy .
```

---

## 3. 每次啟動（依 ① → ③ → ② 順序，各開一個終端）

### 3.1 ① 啟動`teleop_ros2_ref` Docker 前端（發 `/xr_teleop/*`）

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
#   輸入本機 IP -> 接受自簽憑證 -> Connect
```

### 3.2 ③ 啟動機器人控制堆疊

```bash
source ~/xiaobei_X1_ws/install/setup.bash
bash /tmp/run_real_robot.sh          # 真機：ros2_control + spawn 6 個控制器 + MoveIt/RViz
```
> ⚠ 真機會**上電、執行軌跡**，先淨空工作區、把電源/急停開關放在手邊。
> 只想跑模擬：改用 Isaac Sim + `ros2 launch pkg_robot_model moveit_isaac.launch.py`。

啟動後控制器應包含：`joint_state_broadcaster / right_arm / right_gripper / left_arm / left_gripper / head`。

### 3.3 ② 安裝並啟動後端 bridge（本包）

```bash
# 安裝一次：把節點/腳本複製到 /tmp（重開機後 /tmp 會清空，重跑本行即可）
bash deploy/install_local.sh

# 若前端在「另一台」機器：改 /tmp/teleop_env.sh 的 HORDE_IP 為那台 IP；
# 同一台單機可維持預設（走 localhost）。

# 啟動四個 bridge（雙臂 + 夾爪 + engage + 頭部）
bash /tmp/teleop_up.sh real headpose     # 真機 + 頭顯朝向驅動頭部
#   bash /tmp/teleop_up.sh isaac headpose # 改跑 Isaac 模擬
#   第二參數 stick = 用搖桿控頭部（預設 headpose）
```

停止後端：`bash /tmp/teleop_down.sh`

---

## 4. 操作

1. 戴上Quest3瀏覽器連上`IWER` (或桌面瀏覽器打開 `IWER`: <https://nvidia.github.io/IsaacTeleop/client>) → 確認前端已連（後端可 `python3 /tmp/probe_topics.py` 看到 `/xr_teleop/*`）。
2. **按左手 Y 鍵 engage**：手臂接手，開始跟隨控制器姿態。
3. **轉頭**：驅動機器人頭部（yaw/pitch）。
4. **擠壓扳機**：控制對應側夾爪開合。
5. 再按左手 Y 可 disengage（手臂停止跟隨）。

`/xr_teleop/*` 與本包 bridge 對應：`ee_poses`→雙臂、`finger_joints`→夾爪、`head_pose`→頭部、`controller_data`→按鈕/搖桿與 engage。

---

## 5. 內容清單

```
teleop_share/
├─ README_ubuntu.md            # 本檔（Ubuntu 單機版）
├─ README.md                   # Windows + WSL2 版
├─ nodes/                      # ② 後端 bridge（純 Python）
│   ├─ pink_arm_ik.py          #   雙臂差分 IK（Pinocchio + Pink）
│   ├─ gripper_bridge.py       #   夾爪
│   ├─ head_bridge.py          #   頭部（headpose / stick）
│   ├─ engage_button.py        #   engage 按鈕
│   ├─ ready_pose.py / pink_engage.py
├─ scripts/                    # ② 啟動腳本
│   ├─ teleop_env.sh           #   共用環境（ROS/venv 路徑、HORDE_IP、DDS）
│   ├─ teleop_up.sh / teleop_down.sh
│   ├─ ready_run.sh / pink_run.sh / gripper_run.sh / engage_run.sh / head_run.sh
│   └─ restart_pink.sh / restart_head.sh
├─ deploy/
│   ├─ install_local.sh        #   原生 Ubuntu 安裝（複製到 /tmp）
│   └─ deploy_teleop.ps1       #   Windows+WSL2 遠端部署
├─ robot/                      # ③ 機器人堆疊啟停腳本
│   ├─ run_real_robot.sh / run_real_stop.sh / real_prep.sh
├─ ros2_ws_src/                # ③ 機器人控制堆疊（colcon build 用；已整合 head_controller）
│   ├─ pkg_robot_model/                 # URDF、my_controllers.yaml、moveit_real/isaac 等 launch
│   ├─ pkg_robot_model_moveit_config/   # MoveIt 設定
│   └─ xiaobei_hardware/                # Feetech 伺服 ros2_control 硬體介面（C++）
└─ diag/                       # 診斷小工具（probe_topics / head_probe / ...）
```