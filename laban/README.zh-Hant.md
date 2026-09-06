# X1 Laban 手勢播放器（中文摘要）

把 [Microsoft LabanotationSuite](https://github.com/microsoft/LabanotationSuite)
的上半身手勢 JSON 播到 X1 上，可從瀏覽器頁面或命令列觸發。同一個節點同時驅動
Isaac Sim 分身與真機，走的是和本 repo 其他部分相同的 `JointTrajectoryController` 話題。

這份是摘要；完整說明請見 [README.md](README.md)（英文）。

```
Laban JSON  ->  decoder.py  ->  x1_ik_mapper.py  ->  /left_arm_controller/joint_trajectory
                                                     /right_arm_controller/joint_trajectory
                                                     /head_controller/joint_trajectory
```

## 先決條件

這是疊在本 repo 機器人堆疊之上的一層，不是獨立套件。在它能動手臂之前，需要：

1. **ROS 2 堆疊已建置並執行中** —— 依 [README](../README.md) §2.2 編好 `ros2_ws_src`，
   然後啟動真機（§3.2）或 Isaac Sim。手臂與 `head_controller`（`j_101` yaw／`j_102` pitch）
   必須是 active。
2. **`~/pink_venv` 存在**且裝有 `pin` 與 `pink` —— 即 [README](../README.md) §2.1 的 venv。
   IK 映射器會讀 `~/xiaobei_X1_ws/src/pkg_robot_model/urdf/pkg_robot_model.urdf`，
   所以步驟 1 的 `colcon build` 正是把 URDF 放到映射器會找的位置的動作。
3. **`/tmp/teleop_env.sh` 已就位**（任何碰到真機的操作都需要）—— 由
   `pwsh deploy/deploy_teleop.ps1` 產生。少了它，播放器會用錯的 DDS 設定發到虛空裡，
   controller 永遠收不到。

> 播放手勢會讓手臂以全速移動。清空工作區並讓緊急停止按鈕在手邊。先停掉 Quest 3 teleop
> 橋接（`bash /tmp/teleop_down.sh`）—— 它們命令同一組 controller，兩個發布者會搶手臂。

## 網頁介面

```bash
python3 -m pip install -r laban/ui/requirements.txt   # 一次就好，裝進 ~/pink_venv
bash laban/ui/run_ui.sh                               # http://<host>:9200
```

頁面列出手勢庫、點一下就播，並回報手臂何時開始動、手勢何時結束。速度滑桿與「頭部跟著動」
開關套用到下一個手勢；「停止」會取消正在進行的那一個。

`X1_DDS` 決定目標：`real`（預設）、`isaac`（只有分身）、`both`（真機與分身共用一條時間軸）。

```bash
X1_DDS=isaac bash laban/ui/run_ui.sh    # 先在這裡驗證
X1_DDS=both  bash laban/ui/run_ui.sh
```

## 命令列

一個手勢一個行程：

```bash
X1_DDS=isaac bash laban/run_player.sh laban/gestures/library/hello.json --mapper ik
X1_DDS=real  bash laban/run_player.sh laban/gestures/library/hello.json \
  --mapper ik --allow-ik-real --speed 0.5
```

| 旗標 | 意義 |
|------|------|
| `--dry-run` | 印出關鍵幀與關節角度，不發布 |
| `--check` | 回報 controller 的訂閱者數量，不發布 |
| `--speed 1.5` | 加快播放 |
| `--no-head` | 頭部不動 |
| `--approach 3.0` | 從當前姿勢進入第一個關鍵幀的斜坡上限 |
| `--no-return` | 停在收尾姿勢，不回到 ready |
| `--loop` | 重複 |
| `--mapper ik` | 基於 URDF 的 Pink 手肘＋手腕 IK（網頁介面用的就是這個） |
| `--mapper lut` | 舊的粗略符號對關節查表，保留當後備 |
| `--ik-gain 1.0` | IK 幅度：1.0 完全依循 Laban 方向 |
| `--allow-ik-real` | 在真機上跑 IK 映射器必須加這個 |

## 常駐播放器

每個手勢都另開一個播放器，手臂能動之前要花約 2.3 秒（其中 1.0 秒是 bash、venv python
和 IK，其餘是 `rclpy` 初始化與 controller 探索）。常駐 daemon 會在手勢之間保住 `rclpy`、
已配對的 controller 和 Pinocchio 模型，並快取解好的關鍵幀，把時間壓到快取命中 0.4 秒、
第一次求解 0.8 秒。

```bash
X1_DDS=real bash laban/run_player.sh --daemon --mapper ik

# 任何 python3 都行，不需要 rclpy
python3 laban/laban_ctl.py status
python3 laban/laban_ctl.py play laban/gestures/library/hello.json --head
python3 laban/laban_ctl.py stop        # 發布空軌跡，是真正的取消
python3 laban/laban_ctl.py quit
```

請求是 `/tmp/x1_laban_<mode>.sock` 上一行一個 JSON 物件。網頁介面有 socket 就用、沒有就
另開播放器，所以 daemon 是選用的——頁面上的狀態標示會說它走了哪條路。在同一個 socket 上
啟動第二個 daemon 會被拒絕：兩個都會各自發布一整條軌跡。

## 手勢庫

`gestures/library/` 收了 MSRAbotChatSimulation 的 92 個 LabanotationLibrary 樣本；
`gestures/*.total.json` 是 LabanotationSuite 原始的示範手勢。

全幅度下 92 個裡有 87 個能乾淨重定向，5 個（`hello`、`goodbye`、`confuse r`、
`robot d move`、`interesting`）會自動以降低的幅度重試——`Place/High` 要求手臂直伸過頭，
會把某個肩關節推到約 2.8 rad，使下一個關鍵幀把肩膀翻到另一個 IK 分支。播放器會以
0.85、0.70、0.55、0.40 的 gain 重試整個手勢，並記錄最後落在哪裡。

**13 個手勢被直接封鎖。** 全幅度下重定向在它們身上會命令出真實的自我碰撞——碰撞網格
互相穿透、最小間隙 0.000 m，而 `thanks` 在它 8 個關鍵幀中的 6 個都會這樣。它們只在真機上
「看起來沒事」，因為重力把肩膀拉開了，而這不是硬體可以被信任會保持的間隙。
`gesture_policy.py` 在建立目錄的地方就強制執行，所以網頁介面和 daemon 都碰不到它們：

```bash
python3 laban/gesture_policy.py --profile chat    # 什麼可播、不可播的原因
```

量測到的間隙放在 `gesture_allowlist.yaml`；封鎖清單本身是編譯進程式的常數，因為一個
少了設定檔就會消失的安全性質，不算安全性質。

## 重定向怎麼運作

Laban 關鍵幀帶的是離散的方向符號，所以符號對關節的查表是階梯函數，且**只在關鍵幀上求值**。
播放器接著在關節空間以 smoothstep 內插、附上有限差分速度，並把 `MAX_JOINT_VEL` 認為
太快的區段拉長。少了這一步，軌跡就是一道階梯——平平的，然後在 0.1 秒內跳 0.6 rad——
軀幹會晃。

IK 映射器把每個符號化關鍵幀變成每隻手臂兩個笛卡兒目標：

```
肩 + 上臂方向 * URDF 上臂長度 -> 肘
肘 + 前臂方向 * URDF 前臂長度 -> 腕
```

Pink 以連續性與 ready 姿勢正規化求解 L4/L6 與 R54/R56 的位置任務，並從兩個種子
（前一個關鍵幀、ready 姿勢）出發，讓舉起的手臂不會把肩膀留在旋轉過的分支上。
殘差 0.05–0.20 m 是正常而非錯誤：目標來自人體比例的方向、X1 的 ready 姿勢並非直垂而下，
而肘與腕加起來對 6 自由度手臂是過約束。

安全檢查會擋掉 IK 求解失敗、關節超出 ±2.9 rad，以及相鄰關鍵幀相差超過 2.2 rad
（那是 IK 分支翻轉；正常的休息到舉起大約 1.5 rad，會改用時間拉長處理）。

## 頭部

Labanotation 沒有頭部軸，所以頭是由另外兩處驅動：`head` 肢體方向，以及 `rotation` 欄位
（原本記的是軀幹旋轉），後者以 `HEAD_ROTATION_SCALE` = 0.5 映到頭部 yaw，好留在自然範圍內。

量測到的軸向**在兩個軸上都和 Laban 字面意思相反**，所以 `x1_mapper.py` 把兩者都反轉。
先在分身、再在真機上確認過：`j_101` 為正時頭轉向它的右邊，`j_102` 為正時頭往上仰。

URDF 把兩個頭部關節都宣告為 `continuous`、本身沒有限位，所以限位與速度上限是在這裡用
軟體強制的：`HEAD_LIMITS` 限制行程，`MAX_HEAD_VEL` 是 1.5 rad/s，對比手臂的 1.0。
頭部慣量小得多，用手臂的上限會讓每個點頭都顯得吃力——但若沒有自己的上限，也就不會有人
抓到 `shake head` 要求在 500 ms 內轉 0.7 rad。

手勢庫裡有 19 個是純頭部手勢：手臂整段維持一個姿勢，所有動作都在頭上。`nod`、
`shake head`、`laugh head` 是最明顯的幾個。**在頭部關閉的情況下它們是無聲的空操作**，
看起來就像播放器壞了，所以如果某個手勢好像什麼都沒做，先檢查「頭部跟著動」開關
（或 `--no-head`）。

## 回到 ready 姿勢

手勢的最後一個關鍵幀是為了表達而選的，不是為了休息，而 controller 會維持最後收到的東西。
有幾個雙手手勢收尾時兩個夾爪幾乎相碰，所以機器人以前就停在那裡——而接下來每個手勢都從
那裡開始。`thanks.json` 留下的姿勢兩手之間只有 4 mm 間隙，對比 ready 姿勢的 231 mm。

因此播放器會補上一段與前導斜坡對稱的收尾：先維持收尾姿勢 `X1_RETURN_HOLD` 秒
（預設 0.6，讓手勢要表達的姿勢還讀得出來），再以 `MAX_JOINT_VEL` 所需的時間緩回 ready
姿勢，並和時間軸其餘部分一樣取樣，好讓 Isaac 分身也跟著走。`--no-return` 可以關掉它，
`--loop` 也會（在重複之間回 ready 會破壞循環）。

改動後量測：controller 參考值、真機與分身都停在 ready 姿勢 0.02 rad 以內，手部間隙
0.230 m，對比名目值 0.231 m。

## 同時驅動真機與分身

`X1_DDS=both` 保留真機堆疊的 DDS 設定並加上一個鏡像。分身不吃軌跡——它的 OmniGraph
監聽 `/isaac_joint_commands`（`sensor_msgs/JointState`）——所以同一批取樣點以軌跡送給
controller、以 50 Hz 內插後的 `JointState` 送給分身，共用一條時間軸。

**不要**另開第二個 `X1_DDS=isaac` 的播放器去接分身。真機的 controller 跑在同一個 WSL 裡，
所以一個 LOCALHOST 範圍的播放器會再次探索到真機的 controller，結果是命令機器人兩次、
而分身不動。

對每個播放器行程來說 Isaac 都是遲到者（測試中 1–5 秒），所以播放器會先等
`X1_ISAAC_WAIT` 秒（預設 3）再開始，若分身始終沒出現就發警告，而不是拖住機器人。

## 什麼都不動 —— 去哪裡找原因

| 症狀 | 原因 |
|---|---|
| 介面說播了、log 說 ok，機器人不動 | DDS 錯了。真機需要 `/tmp/teleop_env.sh`；`run_ui.sh` 啟動時若缺少會警告。 |
| 手勢是空操作、也沒有錯誤 | 純頭部手勢但頭部被關閉。打開「頭部跟著動」。 |
| 手勢在介面上是灰的 | 因自我碰撞被封鎖。`python3 laban/gesture_policy.py` 會印出原因。 |
| 手臂跳一下就停 | 同一組 controller 上有另一個發布者——Quest 3 teleop 還開著。`bash /tmp/teleop_down.sh`。 |
| `--mapper ik` 在真機上被拒 | 設計如此。先在 Isaac 驗證，再加 `--allow-ik-real`。 |
| 每個手勢都要 2.3 秒才開始 | 沒有 daemon。用 `run_player.sh --daemon` 起一個。 |
| `gesture_policy` 警告沒有 allow-list | 少了 `gesture_allowlist.yaml`；編譯進去的 13 個封鎖仍然有效，分層則失效。 |

## 出處

手勢 JSON 與 Labanotation 編碼來自 Microsoft
[LabanotationSuite](https://github.com/microsoft/LabanotationSuite)（MIT），其中
MSRAbotChatSimulation 的手勢庫原樣沿用。