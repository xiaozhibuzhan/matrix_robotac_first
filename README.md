# matrix_robotac_first：操作与 Ubuntu 迁移指南

更新日期：2026-10-02。请按“准备文件 → 校验与备份 → 安装 → 环境检查 → 预览 → 行走 → 停止”的顺序操作。**最新现场日志已有 4 次 ARRIVED；本次加速和长路线恢复修复通过 96 项离线测试，更新后的 Ubuntu 运动效果待复测。** 已安装用户先看 [长路线修复与更新步骤](navigation/LONG_ROUTE_FIX_20261002.md)。

第一开发目标：在 RViz 点一个普通可达位置，机器狗自主规划、到达并可靠停住。任务一建图已通过 Ubuntu 验收；宝箱接触、消失确认与比赛计时计分属于后续阶段。

## 0. 先认清目录

| 名称 | 当前路径或示例 | 用途 |
| --- | --- | --- |
| Windows 原始工作区 | `E:\各种比赛\robortac\matrix_robotac_first` | 开发原件，包含官方运行时与大型资料 |
| Windows Git 仓库副本 | `E:\各种比赛\robortac\matrix_robotac_first\matrix_robotac_first` | 提交 GitHub 的精简副本，和上一级不是同一目录 |
| Ubuntu 已跑通的工程 | 本文示例 `~/robotac/matrix_robotac_first` | 导航实际安装位置，已有官方仿真、SDK 与任务一程序 |
| Ubuntu 下载位置 | 本文示例 `~/Downloads/task2-update` | 存放本次 zip 和校验文件，不在这里启动仿真 |

**本次只增量替换 Ubuntu 工程中的 `navigation/`。** 不覆盖 `config/`、`src/`、`deps/`、`rviz/`、`mapping/`、`run_sim.sh` 与原始 `maps/`。不要删除官方 `src/robot_mc/build/export/` 和 `src/robot_mujoco/simulate/build/`，其中可能就是运行程序。

GitHub 仓库保存开发扩展、文档、配置快照和必要地图，**不是完整仿真安装包**。不含 UE 运行程序、官方运控/物理仿真构建产物、SDK 共享库/安装包、机器人网格资源、原始 rosbag 和运行日志。刚克隆的精简目录不能直接替代已安装好的官方工程。

下文 `TASK2_PROJECT` 一律指 **Ubuntu 已经跑通的那个工程**；示例路径不同于你的实际位置时，先改路径再执行。

## 1. 准备迁移文件：Windows

### 1.1 使用现成增量包

在原始工作区找到以下两个文件，通过 U 盘、共享文件夹或已有 SSH/SCP 传到 Ubuntu 的下载位置：

```text
navigation/updates/task2_point_navigation_20260926.zip
navigation/updates/task2_point_navigation_20260926.zip.sha256
```

必须同时复制来自**同一次打包**的两个文件。文件名保留首版日期，内容可能更新；以 SHA256 和包内 manifest 判断版本，不能只看名称。

Git 仓库副本也有 `navigation/updates/`。只有确认提交已推送，才能从 GitHub 取到最新包；本地 commit 不代表远端已更新。私有仓库需要登录有权限的账号才能下载。

**完成标志：**Ubuntu 下载目录下同时有上述 zip 与 `.zip.sha256`，不要只传文件夹或单独一个 zip。

### 1.2 修改扩展后才需要重新打包

在 **Windows PowerShell** 中运行，目录是原始工作区：

```powershell
Set-Location 'E:\各种比赛\robortac\matrix_robotac_first'
python -B -m navigation.preflight --offline
if ($LASTEXITCODE -ne 0) { throw '预检查失败，停止打包' }
python -B -m unittest discover -s navigation/tests -v
if ($LASTEXITCODE -ne 0) { throw '测试失败，停止打包' }
python -B -m navigation.package_extension
if ($LASTEXITCODE -ne 0) { throw '打包失败，不传输旧包' }
Get-FileHash '.\navigation\updates\task2_point_navigation_20260926.zip' -Algorithm SHA256
Get-Content '.\navigation\updates\task2_point_navigation_20260926.zip.sha256'
```

当前测试预期为 `Ran 96 tests` 与 `OK`；检查和测试通过后再打包。最后两条显示的 ZIP SHA256 应一致，大小写不影响比较。随后重新传输这两个文件。

打包器会收集 `navigation/` 中允许的文件类型。打包前确认没有准备留在现场的私密配置，特别是自行新增的 `config/local.yaml`。若提示 CRLF，只把报错的扩展文本文件改为 UTF-8/LF，不批量修改官方文件。

## 2. Ubuntu 校验、备份与安装

先在旧导航终端 Ctrl+C 结束导航，确认机器狗停止。已有官方仿真与 Zenoh 可保持运行；不要为了复制文件重复启动仿真。

### 2.1 找对路径

打开 **Ubuntu Bash 终端**，修改下面两个路径：

```bash
TASK2_PROJECT="$HOME/robotac/matrix_robotac_first"   # 已有可运行工程
TASK2_DOWNLOAD="$HOME/Downloads/task2-update"       # zip 和 sha256 所在目录
printf '工程：%s\n下载：%s\n' "$TASK2_PROJECT" "$TASK2_DOWNLOAD"
ls -ld "$TASK2_PROJECT/src" "$TASK2_PROJECT/deps"
ls -l "$TASK2_PROJECT/run_sim.sh"
ls -lh "$TASK2_DOWNLOAD/task2_point_navigation_20260926.zip" \
       "$TASK2_DOWNLOAD/task2_point_navigation_20260926.zip.sha256"
```

**预期：**全部存在。遇 `No such file or directory` 先改路径，不继续覆盖。保留双引号以支持空格。目录存在只是检查位置，仿真可运行性仍以此前现场验证为准。

### 2.2 先检查暂存包，再替换扩展

继续在**同一个终端**完整执行以下代码；其中任一步失败，本段自动停止：

```bash
export TASK2_PROJECT TASK2_DOWNLOAD
bash <<'BASH'
set -euo pipefail
test -f "$TASK2_PROJECT/run_sim.sh"
test -d "$TASK2_PROJECT/src"
test -d "$TASK2_PROJECT/deps"
cd "$TASK2_DOWNLOAD"
sha256sum -c task2_point_navigation_20260926.zip.sha256

TASK2_STAGE=$(mktemp -d)
unzip -q task2_point_navigation_20260926.zip -d "$TASK2_STAGE"
cd "$TASK2_STAGE"
/usr/bin/python3 -B -m navigation.check_install

TASK2_BACKUP=$(mktemp -d "$TASK2_PROJECT/task2_backup_$(date +%Y%m%d_%H%M%S)_XXXXXX")
printf '暂存：%s\n备份：%s\n' "$TASK2_STAGE" "$TASK2_BACKUP"
if [ -e "$TASK2_PROJECT/navigation" ]; then
  mv -- "$TASK2_PROJECT/navigation" "$TASK2_BACKUP/navigation"
fi
cp -a -- "$TASK2_STAGE/navigation" "$TASK2_PROJECT/navigation"
cd "$TASK2_PROJECT"
/usr/bin/python3 -B -m navigation.check_install
printf '安装完成，请记下备份目录：%s\n' "$TASK2_BACKUP"
BASH
```

**成功标志：**

1. 校验显示 `task2_point_navigation_20260926.zip: OK`。
2. 两次 manifest 检查的 `missing_or_changed` 都是 `[]`。
3. 最后打印“安装完成”和备份目录。请保存这个路径，旧参数与日志都在其中。

校验不一致就重新传输同版的两个文件；缺少 unzip 时只需 `sudo apt install unzip`。安装中途失败不要删除备份，按第 8 节回退。

若是直接从 Git 拷贝源码，`check_install` 可能提示 `No package manifest: source checkout`。这不是通过校验，而是当前目录没有打包清单；本页推荐使用 zip 安装流程。

### 2.3 创建现场配置

在原包完整性核对通过后运行：

```bash
cd "$TASK2_PROJECT"
cp -n navigation/config/default.yaml navigation/config/local.yaml
nano navigation/config/local.yaml
```

`cp -n` 不覆盖已有文件。首次试运行可沿用默认参数；旧备份有调好的配置时，对照新默认值逐项恢复，不把旧文件整份盲目覆盖。Nano 按 **Ctrl+O → Enter** 保存，**Ctrl+X** 退出。

后面所有预检查与启动都显式使用 `--config navigation/config/local.yaml`，避免“改了文件却没加载”。不要靠缩小足迹或增大超时来掩盖问题。

首先核对以下配置。修改现有同名项，不在 YAML 末尾重复添加；保持冒号后的空格，不用 Tab 缩进：

```yaml
map: navigation/maps/task1_20260911/field_map.yaml
frame: world
odom_topic: /odom/mujoco_odom
cloud_topic: /front_lidar
client_ip: 127.0.0.1
robot_ip: 127.0.0.1
client_port: 43988
```

地图路径相对工程根目录；前四项应与所用地图和现场 topic 一致。后三项沿用官方示例的同机仿真地址，跨机器时按现场已验证的 SDK 网络配置修改。**导航 SDK 只读取这个配置文件的地址和端口，不会把 `SDK_CLIENT_IP` 环境变量自动填入其中。** 实际加载值会写入本轮日志的 `config.yaml`。

**本次加速必须核对 `local.yaml`：旧文件里的 0.20 / 0.35 会覆盖新默认值。** 将下面同名项改成以下数值；`replan_progress_distance` 缺失时补一行。保留现场地图、地址和传感器外参等已验证设置：

```yaml
max_speed: 0.40
max_yaw_rate: 0.60
acceleration: 0.20
max_tracking_replans: 3
replan_progress_distance: 0.75
```

保持 `goal_timeout: 180.0` 和 `progress_timeout: 15.0`。本次半路停止发生在约 59 秒，原因是恢复额度耗尽；增大超时不能解决它。新的恢复额度按连续失败计算，已恢复行走并取得足够进展后复位。

## 3. 终端分工与环境

| 终端 | 用途 | 注意 |
| --- | --- | --- |
| A | 官方仿真与运控 | 已经正常运行就保留，不重复启动 |
| B | Zenoh 路由器 | 同样只保留一个已验证实例 |
| C | 预检查、预览、实际导航 | 启动后保持该终端运行 |
| D | 查看 topic、状态、取消与日志 | 结束 echo 只停止查看，不停止机器人 |

**每新开 C/D 终端，都重新设置路径并加载环境。** 以下变量是历史验收值；若现场工作配置不同，用现场值，不要只为了与文档一致而改变正在工作的通信环境。`SDK_CLIENT_IP` 是原官方环境记录，导航连接地址仍按第 2.3 节配置：

```bash
TASK2_PROJECT="$HOME/robotac/matrix_robotac_first"   # 改成实际路径
cd "$TASK2_PROJECT"
source /opt/ros/humble/setup.bash
# 如原来还需 source Zenoh/项目 overlay，此处按原顺序执行原命令。
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1
printf 'ROS=%s  RMW=%s  DOMAIN=%s  SDK_IP=%s\n' \
  "$ROS_DISTRO" "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID" "$SDK_CLIENT_IP"
/usr/bin/python3 --version
command -v ros2
command -v rviz2
```

**预期：**ROS 为 Humble，系统 Python 为 **3.10.x**，ros2/rviz2 能找到。不要用 Conda、Windows Python 或不同小版本替代官方 SDK 所需的系统 Python。

冷启动且没有已有路由器时，终端 B 加载同样环境后执行：

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

终端 A 使用**目标 Ubuntu 以前成功的官方命令与参数**。本地 `run_sim.sh` 第 5 个参数决定是否启用 MuJoCo，省略时默认 `0`，而且脚本会改配置和清理旧进程；不能把 Windows 快照的无参命令当作已验证的运动启动方式。本次增量安装不需要重启已工作的仿真。若忘记现场命令，可在原 Bash 终端用 `history | grep 'run_sim.sh' | tail -n 5` 找出成功记录并核对。

## 4. 预检查与传感器检查

### 4.1 终端 C：先不连接 SDK

以下命令逐条执行。任何一步报错或测试失败，就停在这一步，处理完再继续，不直接进入预览或控制。

```bash
cd "$TASK2_PROJECT"
/usr/bin/python3 -c "import numpy, scipy, yaml, rclpy; print('imports OK')"
/usr/bin/python3 -B -m navigation.preflight --offline --config navigation/config/local.yaml
/usr/bin/python3 -B -m unittest discover -s navigation/tests -v
/usr/bin/python3 -B -m navigation.preflight --drive --config navigation/config/local.yaml
```

预期依次为 `imports OK`、地图摘要、当前 96 项测试 `OK`、完整预检查 JSON。核对 JSON 的 `control_settings` 中 `max_speed` 为 `0.4`、`max_yaw_rate` 为 `0.6`。这里 `preflight --drive` **仅检查文件与导入，不连接机器人，不发送速度**；不代表 SDK 动态加载、连接和制动已通过。

仅缺少相应依赖时补装：

```bash
sudo apt install python3-numpy python3-scipy python3-yaml
# 以下仅用于 ROS imports 或 rviz2 缺失：
sudo apt install ros-humble-rclpy ros-humble-geometry-msgs ros-humble-nav-msgs \
  ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-std-srvs \
  ros-humble-visualization-msgs ros-humble-rviz2
```

软件源太旧导致找不到包时，先 `sudo apt update` 再重试；不要重新运行整套官方安装脚本，不需要 colcon build。

离线测试需要原工程已有 `maps/run_20260911_223510_ujmvehqs/field_map.yaml` 与对应 PGM；地图重生成另需 PCD 和 `mapping/clean_pcd.py`。**navigation 增量包不含任务一源文件。** 缺失时先核对验收基准，只补回缺失的同一轮文件，不覆盖目标机其他正确地图。日常导航使用包内派生图，不需每次重建地图。

### 4.2 终端 D：确认有真实消息

先加载第 3 节环境，然后运行：

```bash
ros2 topic list
ros2 topic info /odom/mujoco_odom -v
ros2 topic info /front_lidar -v
ros2 topic echo /odom/mujoco_odom --once --qos-reliability best_effort
ros2 topic echo /front_lidar --field header --once --qos-reliability best_effort
```

预期两个 topic 都有发布者且能收到新数据。雷达仅打印 header，避免整帧点云刷屏。若一直没有输出，Ctrl+C 结束该查看命令，先查仿真、路由器与终端环境，不继续开控制。需要检查频率时运行 `ros2 topic hz /front_lidar`，观察数秒后 Ctrl+C。

**选择时间模式：**默认使用普通 ROS 系统时间。仅当现场传感器确定使用仿真时间时，用 `ros2 topic info /clock -v` 和 `ros2 topic echo /clock` 核对时钟发布；观察连续多条 `sec/nanosec` 非零且递增，然后 Ctrl+C 结束查看。只有 topic 名称或单条消息不够。此时预览和控制启动命令都追加 `--use-sim-time`，不能只给其中一个加。遇时钟错误先查基准，不靠增大超时绕过。

## 5. RViz 预览：只看地图和路径

终端 C 执行：

```bash
bash navigation/run_navigation.sh --config navigation/config/local.yaml
```

终端应打印 `PREVIEW (no SDK motion)` 与 logs 路径，随后打开专用 RViz。让 C 保持运行；节点详细输出主要写日志，不要因终端没有持续刷新就再启动一次。

终端 D 执行 `ros2 topic echo /task2/status`，然后按顺序操作：

1. RViz 左侧 Global Options 的 Fixed Frame 应为 `world`，Navigation Map 应出现地图。
2. 保持机器狗静止，点击顶部 **2D Pose Estimate**。
3. 在地图中**机器人此刻实际所在的位置**按住左键，沿实际朝向拖动箭头再松开。它只建立定位，不会传送机器狗，也不是设置目的地。
4. 绿色位置/方向和红色实时障碍应与地图、仿真画面的墙体结构一致；状态应进入 `IDLE`。不确定出生点对应地图哪里时，先对照墙角/通道确定，不照抄离线示例或目标坐标当起点。
5. 点击 **Publish Point**，在地图地面上的开阔区选距当前位置约 1 米的普通点，应出现目标；状态经 `STOPPING_FOR_PLAN → PLANNING`，规划完成出现青色路径并进入 `TRACKING`。
6. **预览中机器人不动是正常的。** 不连接 SDK，约 15 秒无路径进展后会取消目标；这里只确认输入、坐标和路径，不验收真实到达。
7. 点被拒绝时看状态的 `reason`，可打开 `Footprint Clearance (optional)` 查看足迹可规划区域。不要点机器人/障碍标记或缩小足迹强行通过。

本版不用 `2D Goal Pose`。预览完成后在 C 按 Ctrl+C，等命令行提示符返回；预览与控制不能同时保留。

## 6. 实际导航：先 1 米，再 2 米

**最新 18:00:42 开始的现场日志：**5 个接受目标中 4 个到达；第 4 个约 59 秒后因累计恢复次数耗尽停止。本次将前进上限改为 0.40 m/s、转向上限改为 0.60 rad/s，并改善路径余量、连续恢复和转向进展判断。先按第 2.3 节核对现场配置，详见 [长路线修复说明](navigation/LONG_ROUTE_FIX_20261002.md)。

此前 SDK 拒绝低于 0.05 m/s 的前向指令、ROS 退出竞态已修复，历史诊断见 [低速修复说明](navigation/VELOCITY_FIX_20261002.md)。不用另开 highlevel_demo。

2026-10-02 回传日志中，旧版带 `--stand-up` 仍立即退出的问题已修复：零速度 move 不能先于 standUp。遇到 `SDK returned 12295` 请先更新本次增量包，具体证据和重试见 [启动修复说明](navigation/STARTUP_FIX_20261002.md)。

关闭官方键盘 demo 和其他运动命令发送者。扩展的进程锁只能阻止它自己重复启动，不能阻止其他 SDK 程序同时控制。

终端 C **下面两条只选一条**：

官方 SDK 已允许进入 move 状态（不能仅靠画面判断已站立）：

```bash
bash navigation/run_navigation.sh --config navigation/config/local.yaml --drive
```

需要扩展调用一次官方 standUp：

```bash
bash navigation/run_navigation.sh --config navigation/config/local.yaml --drive --stand-up
```

第 4 节确认使用仿真时钟时，在所选命令末尾追加 `--use-sim-time`。`--stand-up` 会先请求站立，等待 4 秒后才发第一条零速度 move；不能用于只预览模式。成功就绪后 sdk.log 应有 `TASK2_READY`，node.log 应有 `DRIVE enabled`。

应看到 `DRIVE`、`Loaded limits: max_speed=0.40 m/s, max_yaw_rate=0.60 rad/s` 和本轮日志目录。若仍显示 0.20 / 0.35，先 Ctrl+C 退出，检查本次 `--config` 指向的文件。**每次重启导航都重新操作 2D Pose Estimate**，上次预览的定位不会沿用。按第 5 节对齐后，用 Publish Point 点击约 1 米开阔点，同时观察仿真画面和 `/task2/status`。

| 阶段 | 默认判据/预期 |
| --- | --- |
| 行走 | 向原点击点沿规划路径行走，左右转向正确 |
| 接近目标 | 进入 `SETTLING`，持续请求零速度 |
| 到达 | 距原点击点 ≤0.10 m，反馈线速度 ≤0.025 m/s、角速度 ≤0.06 rad/s，连续 ≥0.8 s 后报告 `ARRIVED` |
| 到达后 | 再观察至少 3 秒，画面停稳，没有恢复旧目标 |

正常顺序为 `WAIT_INITIAL_POSE → IDLE → STOPPING_FOR_PLAN → PLANNING → TRACKING → SETTLING → ARRIVED`。`PLANNING` 期间保持停车，后台计算路线并继续接收传感器；`STOPPED` 是异常停止，不是到达。先通过短距离，再做 2 米、不同朝向、静态绕障与连续目标，建议累计 10 次并记录。

短程到点、转弯和制动都正常后，再选上轮长路线的同一目标复测；不要把旧日志坐标当成本轮初始位姿。转弯前减速停稳、短暂停车重规划仍是预期行为。`tracking_replans` 记录整段总恢复数，`tracking_replan_streak` 记录连续恢复数；后者最多 3 次，恢复后剩余路线缩短至少 0.75 m 且实际位移至少 0.375 m 才复位。实时障碍、持续无进展或同一处反复失败仍会取消目标。

本阶段不测试碰宝箱。新局部障碍会导致停车取消目标，需要人检查后重选，尚未实现动态障碍自动绕行。

## 7. 取消、退出与日志

### 7.1 只取消当前目标

终端 D 若在 echo，先 Ctrl+C 结束查看，再执行：

```bash
ros2 service call /task2/cancel std_srvs/srv/Trigger '{}'
```

`success: true` 只表示目标已清除、已请求零速度，仍须看实际反馈/画面停稳。随后可重新点选。直接点击新目标也会撤销旧目标，停稳后重新规划。

### 7.2 结束导航

在 C 按 Ctrl+C 并等退出，或关闭本轮专用 RViz。启动器回收本轮节点和 SDK 子进程，请求停车，不关闭 A/B 的官方仿真和 Zenoh。最后确认画面上确实停止。**在 D 结束 echo 不等于停止机器人。**

### 7.3 找到日志并保留

启动时打印的目录最准确；下面查看最近一轮：

```bash
cd "$TASK2_PROJECT"
TASK2_RUN=$(ls -dt navigation/runs/run_* 2>/dev/null | head -n 1)
if [ -n "$TASK2_RUN" ]; then
  printf '最近一轮：%s\n' "$TASK2_RUN"
  tail -n 60 "$TASK2_RUN/node.log"
  test ! -f "$TASK2_RUN/sdk.log" || tail -n 60 "$TASK2_RUN/sdk.log"
fi
```

没有 run 目录通常是预检查阶段已失败，直接读 C 终端错误。预览没有 sdk.log 正常。反馈问题时保留 `run.json`、`config.yaml`、`events.jsonl`、`node.log`、`sdk.log`（如有）、`rviz.log` 和点击位置/现场画面，不只截最后一行。新版 `events.jsonl` 增加 `planned_path` 事件；状态含当前路点、剩余路径长度及连续恢复数，可区分正常转弯、重新规划和真正停止。

## 8. 失败时恢复旧扩展

先退出导航、确认停稳。找到第 2 节保存的备份路径；首次安装没有旧 navigation，就没有可恢复的旧版本。以下保留失败版本，只处理自己的扩展，不触碰官方目录：

```bash
TASK2_PROJECT="$HOME/robotac/matrix_robotac_first"   # 改为实际工程
TASK2_BACKUP="$TASK2_PROJECT/task2_backup_实际备份目录名"
export TASK2_PROJECT TASK2_BACKUP
bash <<'BASH'
set -euo pipefail
test -f "$TASK2_PROJECT/run_sim.sh"
test -d "$TASK2_BACKUP/navigation"
TASK2_FAILED=$(mktemp -d "$TASK2_PROJECT/task2_failed_$(date +%Y%m%d_%H%M%S)_XXXXXX")
if [ -e "$TASK2_PROJECT/navigation" ]; then
  mv -- "$TASK2_PROJECT/navigation" "$TASK2_FAILED/navigation"
fi
cp -a -- "$TASK2_BACKUP/navigation" "$TASK2_PROJECT/navigation"
printf '旧扩展已恢复；失败版本位于：%s\n' "$TASK2_FAILED"
BASH
```

备份仍保留。恢复后从环境检查和预览重新开始，不直接进入控制。

## 9. 常见现象与处理

| 现象 | 首先检查 |
| --- | --- |
| `No module named navigation` | `pwd` 确认是工程根目录，`ls navigation/run_navigation.sh`；不能在 navigation 子目录里执行 -m |
| 找不到 rclpy | 系统 `/usr/bin/python3` 与 Humble/overlay 是否加载，不用 Conda 替代 |
| Python 版本或 SDK .so 错误 | 要求 Linux CPython 3.10 与对应 `uname -m` 架构；GitHub 精简仓库没有 SDK 二进制 |
| RViz 无地图 | C/日志有无启动错误、`/task2/map` 是否有发布者、Fixed Frame 是否 world |
| 不能初始定位 | 机器人是否停稳、里程计是否新鲜；随后重做 2D Pose Estimate，Publish Point 不能代替初始化 |
| 预览不走 | 正常，退出预览后用 --drive 才会控制 |
| Odometry / LiDAR stale | 仿真、Zenoh、各终端 domain/RMW、消息时效；不要只放宽超时 |
| wrong clock / 旧包 / 未来包 | 系统时间与 /clock 是否选对，预览/控制参数是否一致 |
| No path / 余量不足 | 起点和目标是否同一足迹可通行区域，先选开阔短程点 |
| Live obstacle | 已保护停车，对照实际障碍和外参叠加后再点选 |
| 更新后仍然慢 | 检查启动 `Loaded limits` 和日志 `config.yaml`，旧 local.yaml 会覆盖新速度上限；拐点附近仍会主动降速 |
| Tracking error crosses blocked cells | 核对已安装长路线修复包；检查初始定位、路径与障碍叠加，保留新日志，不靠增加超时或无限重试解决 |
| SDK worker 失败/锁定 | 查 sdk.log 的具体错误；0x3013 表示速度范围不合法，不能一概认为未连接。更新修复包后重启并重新定位 |
| 已有 session / 锁占用 | 回到旧导航终端正常 Ctrl+C，不用全局 pkill python |
| bad interpreter / CRLF | 重新复制校验通过的 LF 增量包，不批量转换官方脚本 |
| 退出后仍运动 | 采用现场已验证的官方停车方式，保留日志；软件零指令不代替运控失联保护 |

## 10. 文档入口与后续同步

原始工作区与 Git 副本是两份文件。修改原件后同步对应 README、扩展文件和新包，再在 Git 副本查看 git status、提交、推送；不要把整个官方运行工程直接加入仓库。是否上传成功，以远端对应 commit 为准。

| 入口 | 内容 |
| --- | --- |
| [navigation/README.md](navigation/README.md) | 实现范围、参数、派生地图、状态与控制边界 |
| [navigation/OPERATIONS_UBUNTU.md](navigation/OPERATIONS_UBUNTU.md) | 随增量包携带的完整操作流程，Ubuntu 上可离线查阅 |
| [navigation/LONG_ROUTE_FIX_20261002.md](navigation/LONG_ROUTE_FIX_20261002.md) | 本次加速、半路停车证据、现场参数与复测步骤 |
| [mapping/README.md](mapping/README.md) | 任务一采集和离线建图 |
| [README_GPT.md](README_GPT.md) | 项目历史及已验证环境背景 |
| [TASK2_PLAN.md](TASK2_PLAN.md) | 任务二规则、目标与后续宝箱阶段 |
| [MIGRATION_UBUNTU.md](MIGRATION_UBUNTU.md) | 迁移原则索引 |

本文命令已对照当前接口；Ubuntu 图形、通信、SDK 动态加载和实际运动必须现场按步骤验证，不能把 Windows 离线通过当成现场验收。
