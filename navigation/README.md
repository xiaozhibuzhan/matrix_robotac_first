# 任务二第一开发目标：RViz 点选 → 自主到达 → 停稳

更新：2026-09-26。目标按用户确认执行：**在 RViz 点一个可达位置，机器狗自主到达并可靠停住。** 本目录是独立扩展，不修改官方代码，不改变已验收的任务一建图程序与原始地图。

**当前状态：首版代码与离线验证已完成，目标 Ubuntu 的 ROS2、SDK、仿真运动闭环尚未现场验收。** 本地 36 项测试全部通过，128 个选定官方/任务一文件 SHA256 未变化。结果见 [验证报告](validation/local_validation.json) 和 [测试日志](validation/offline_tests.txt)。Windows 测试不能代替真实 SDK、步态外形、制动距离和现场重复测试。

## 1. 已实现与当前边界

- 加载 PGM/YAML；未知区、地图外部、障碍及足迹空间不足区域不可通行。
- `2D Pose Estimate` 输入**机器人当前实际位置及朝向**，建立地图与里程计变换；不会移动或重置仿真中的机器人。
- `Publish Point` 输入一个当前目的地。先停稳再 A* 规划，检查对角切角和简化路径穿障；无效点会被拒绝，不偷偷吸附到另一个位置。
- 用里程计闭环跟踪路径，只发送前向速度和偏航角速度；在拐点先减速停稳再转向。跟踪偏差使旧线段不可行时，先停稳，最多做 3 次静态图恢复重规划；最终目的地和总时限不变。
- 只有同时满足位置、实际速度和持续时间条件，才报告 `ARRIVED`。发送零速度不等于已经停稳。
- 新点击撤销旧目标；支持取消、退出，以及传感器断流、时间戳倒退、位姿跳变、异常倾斜、无进展时停车。
- 独立 SDK 子进程检查命令生成时刻和序号。父节点退出/管道断开会停车；移动命令超过 0.30 秒未更新则停车并锁定，必须重启导航。
- 保存参数、地图/代码哈希、初始位姿、点击、轨迹、状态和异常日志。

静态障碍由全局路径绕开；**新出现的局部障碍目前导致停车、取消目标，由人检查后重新点选**，尚未实现动态障碍自动绕行。也未实现宝箱接触、识别/消失确认、比赛计时计分、多目标队列、自动寻宝和在线地图匹配定位。这些不属于本次第一开发目标。

## 2. 不修改官方代码与 Ubuntu 增量迁移

所有新增执行文件、配置、派生地图、RViz、测试与日志都在 `navigation/`。根目录 `README_GPT.md` 和 `TASK2_PLAN.md` 只更新进度入口。`run_sim.sh`、`config/`、原有 `rviz/`、`src/`、`deps/`、`mapping/`、任务一 `maps/` 保持原样。128 项校验覆盖选定源文件/配置，不表示扫描了全部 UE 二进制。

SDK 只加载已有官方库；无需编译 SDK、执行 `colcon build` 或重建整个仿真。专用 RViz 是 `navigation/rviz/point_navigation.rviz`；运行时再复制一份到本轮日志目录，不覆盖官方配置。

增量包：`navigation/updates/task2_point_navigation_20260926.zip`，附同名 `.sha256`；包内仅有 `navigation/`，排除日志、缓存和旧更新包。**不要用整个 Windows 项目覆盖 Ubuntu，不要删除目标机 build、运行库或已跑通环境。**

在 Ubuntu 先关闭旧导航，将 zip 和 sha256 放在同一目录。以下项目路径按实际修改：

```bash
sha256sum -c task2_point_navigation_20260926.zip.sha256
TASK2_STAGE=$(mktemp -d)
unzip task2_point_navigation_20260926.zip -d "$TASK2_STAGE"
TASK2_PROJECT="$HOME/robotac/matrix_robotac_first"
# 已有 navigation 时先备份；不搬动任何官方目录。
if [ -d "$TASK2_PROJECT/navigation" ]; then
  mv -- "$TASK2_PROJECT/navigation" "$TASK2_PROJECT/navigation.backup.$(date +%Y%m%d_%H%M%S)"
fi
cp -a -- "$TASK2_STAGE/navigation" "$TASK2_PROJECT/navigation"
cd "$TASK2_PROJECT"
/usr/bin/python3 -B -m navigation.check_install
```

完整性校验后，对照备份，将现场调好的参数逐项放入 `navigation/config/local.yaml`。不要用默认值覆盖现场标定；有意修改原包配置后，`check_install` 报出变化属于正常现象。开发用 `protected_baseline.json` 是本机防误改快照，不要求 Ubuntu 所有配置与 Windows 相同。

## 3. 启动顺序

### 3.1 沿用现有环境

使用已有 Ubuntu 22.04 / ROS2 Humble / **系统 CPython 3.10**。官方扩展文件名是 `cpython-310-...-linux-gnu.so`，不能迁移 Windows Python/.pyc 或直接换用不同 Python 小版本。支持官方已有的 x86_64 / aarch64 库。新增源码采用 UTF-8/LF，以项目相对路径定位资源。

按任务一已经跑通的顺序启动官方仿真、运控和 Zenoh。导航脚本不会重启或杀死它们，也不修改环境脚本、官方 JSON、`ROS_DOMAIN_ID` 或 `RMW_IMPLEMENTATION`。历史验收环境是 `rmw_zenoh_cpp`、domain `89`、`SDK_CLIENT_IP=127.0.0.1`；**以目标机当前已验证值为准**，各终端保持一致，并沿用原 source/overlay 顺序。

```bash
source /opt/ros/humble/setup.bash
# 然后沿用原来的 Zenoh/项目环境加载方式与已验证变量。
cd ~/robotac/matrix_robotac_first
/usr/bin/python3 --version
/usr/bin/python3 -c "import numpy, scipy, yaml, rclpy; print('imports OK')"
/usr/bin/python3 -B -m navigation.preflight --offline
/usr/bin/python3 -B -m unittest discover -s navigation/tests -v
/usr/bin/python3 -B -m navigation.preflight --drive
```

数值依赖仅在缺少时安装：`sudo apt install python3-numpy python3-scipy python3-yaml`。ROS 使用现有 rclpy、geometry/nav/sensor/std/visualization messages、std_srvs、rviz2；按预检查补齐缺失的 Humble 包，不重建仿真。测试会读取已有任务一基准图；缺失时先核对基准，不能称作通过。`preflight --drive` **只检查、不连接 SDK**；实际运动由下面启动器的 `--drive` 开启。

### 3.2 先预览

```bash
bash navigation/run_navigation.sh
```

默认打开 RViz 和导航节点，**不加载 SDK、不发送机器人运动指令**。预览显示路径和候选速度；机器人不动时，随后按无进展超时取消目标是预期行为。

1. 传感器数据稳定、机器人静止后，在固定坐标系 `world` 下，用 `2D Pose Estimate` 指定机器人**当前实际位置和实际朝向**。不要把目的地当初始位置，也不要照抄离线示例坐标。
2. 对照仿真画面，检查绿色机器人标记、方向及红色实时障碍与地图固定结构对齐。整体偏移/旋转时先修正初始位姿或外参。
3. 用 `Publish Point` 在地图地面上选视野内、开阔区、约 1～2 米的普通可达点，确认青色路径与终点正确。首轮暂不选宝箱或复杂窄缝。
4. 可打开 `Footprint Clearance (optional)` 检查实际允许机器人中心通过的区域。灰色未知区、障碍和足迹余量不足位置会拒绝。

本版本没有接入 `2D Goal Pose`；输入约定就是 `2D Pose Estimate` 与 `Publish Point`。

### 3.3 开启实际控制

退出预览；关闭键盘控制 demo 和其他运动命令发送者。每次重启导航均重新设置初始位姿。以下命令二选一：

```bash
# 机器狗已站立：
bash navigation/run_navigation.sh --drive
# 本轮需要扩展调用一次官方 standUp 时：
bash navigation/run_navigation.sh --drive --stand-up
```

默认 SDK 地址来自项目官方示例：local IP `127.0.0.1`、端口 `43988`、robot IP `127.0.0.1`。目标机不同则修改独立配置，以 `--config navigation/config/local.yaml` 启动。程序不会自动把环境变量覆盖进这些配置，实际地址保存在本轮 `config.yaml`。

同一 domain 的本扩展启动器、同一端口的本扩展 SDK worker 分别加锁，防止重复启动；**这些锁不能阻止其他 demo 发命令**。没有 `--drive` 时拒绝 `--stand-up`。

仅当传感器使用仿真时间且有 `/clock` 发布者时加 `--use-sim-time`。时间基准不一致会拒绝旧包/未来包，不要靠增大超时掩盖。控制定时器和 SDK 看门狗采用 steady/单调时间，暂停仿真时间不会暂停过期停车。

### 3.4 取消、退出与日志

```bash
ros2 topic echo /task2/status
ros2 service call /task2/cancel std_srvs/srv/Trigger '{}'
```

服务成功仅表示目标已取消且零速度已请求；仍须核对反馈与实际停止。启动终端 Ctrl+C 或关闭本轮 RViz 会回收本轮导航，退出前反复发送零速度；不会使用全局 pkill，不停止官方仿真和 Zenoh。

正常状态：`WAIT_INITIAL_POSE → IDLE → STOPPING_FOR_PLAN → TRACKING → SETTLING → ARRIVED`。异常停止后不会自动恢复旧目标。里程计重连/重置/跳变/换 frame 后须重新初始化；SDK worker 超时或退出后须重启导航。

日志位于 `navigation/runs/run_日期_时间_随机后缀/`：`run.json` 记录环境、命令、代码/地图哈希与退出信息；`config.yaml` 为实际参数；`events.jsonl` 含点击、状态与约 2 Hz 轨迹；`node.log`、`sdk.log`、`rviz.log` 用于排错。预览不创建 SDK 日志。

## 4. 导航派生图

任务一基准为 `maps/run_20260911_223510_ujmvehqs/`。原 PGM 597×482、0.05 m/格，自由格 30,986；按初始圆形半径 0.35 m + 余量 0.05 m 膨胀后没有可规划单元。

本次从已验收 PCD 的实测地面点派生独立导航图：选取 `|z| ≤0.06 m` 点，仅在 XY 边长全部 ≤0.30 m、顶点高度差 ≤0.04 m、坡度 ≤0.18 rad 的三角形内部推断支持；不外推、不全图填白，保持原障碍和原自由格。膨胀同时考虑障碍格面积和机器人中心在格内的位置。未知格参与障碍膨胀。

输出在 [maps/task1_20260911](maps/task1_20260911/)：自由格 91,713，其中插值推断 60,727；足迹可规划格 49,147，3 个连通区域，最大区域 47,859 格。**插值是推断，不是新测量或已验证的地面承载/越障能力**；仍须结合场景、实时雷达与现场通行核对。

`report.json` 记录源文件 SHA256、参数、连通性及约 39.8 米离线路径示例。示例不是机器人当前位姿/首次试车目的地，不保证在默认 180 秒时限内走完。`inferred_support.pgm` 标出推断区域；`robot_clearance.pgm` 对应默认足迹，实际运行按当前配置重新膨胀。

调整生成参数时写入**新目录**，不覆盖任务一地图或既有派生图：

```bash
/usr/bin/python3 -B -m navigation.prepare_map --output navigation/maps/task1_trial_02 --max-edge 0.30
bash navigation/run_navigation.sh --map navigation/maps/task1_trial_02/field_map.yaml
```

地图生成仅复用任务一 PCD 读取函数，不改任务一程序。不能为了让目标可达而随意减小足迹或扩大插值跨度。

## 5. 定位与停止标准

采用人工初始位姿 + `/odom/mujoco_odom` 相对运动。地图与里程计即使同名 `world`，也经初始位姿建立变换；本扩展不改官方 TF，不包含 AMCL/SLAM 或漂移校正。运行较长距离后需检查定位误差。

使用 `/front_lidar` 局部点，剥除 `intensity=111` 的仿真绝对位姿元数据，**不拿它作导航定位**。采集包近邻样本表明雷达比里程计参考点高约 0.300 m，证据见 [sensor_reference_sample.json](validation/sensor_reference_sample.json)；默认外参 `[0,0,0.30]` 来自配置和该样本，目标机配置变化后必须复核。

**仿真里程计的比赛使用许可仍待规则核对**；去掉内嵌绝对位姿不表示当前定位来源已经获得赛规确认。

以下集中于 [config/default.yaml](config/default.yaml)，都是工程起始参数，尚非实测性能或比赛条款：

| 参数 | 默认值 | 验证含义 |
| --- | --- | --- |
| `robot_radius + safety_margin` | 0.35 + 0.05 m | 覆盖步态脚部外形和转身扫掠 |
| `max_speed / max_yaw_rate` | 0.20 m/s / 0.35 rad/s | 核对 SDK 符号、单位与响应 |
| `acceleration` | 0.20 m/s² | 加速上限与制动模型假设，须实测 |
| `goal_tolerance` | 0.10 m | 距离原点击点的误差 |
| `stop_speed / stop_yaw_rate` | 0.025 m/s / 0.06 rad/s | 由约 0.15 秒里程计差分估计实际速度 |
| `settle_seconds` | 0.8 s | 位置和速度条件同时成立的持续时间 |
| `odom_timeout / cloud_timeout` | 0.35 / 0.45 s | 有效数据过期取消目标，重复时间戳不刷新 |
| `command_timeout` | 0.30 s | SDK 独立进程移动命令期限，超时锁定 |
| `max_header_age` | 0.50 s | 检查传感器 header 与节点时钟 |
| `progress_timeout / goal_timeout` | 15 / 180 s | 无路径进展/总时限超出时停止 |
| `max_tracking_replans` | 3 次 | 跟踪偏差的有限恢复；不代表动态障碍自动绕行 |
| 障碍高度带 | 0.15～1.20 m | 不是台阶、沟沿、落差等全地形感知 |

实时障碍保护考虑预测弧线、圆形足迹、反应时间、假定制动和缓冲；依赖视野、外参、地面高度。软件看门狗不是硬件急停：SDK 自身被强杀/挂死、网络完全断开、官方运控不接受零指令时，不能由本程序保证停车，须核对官方运控失联行为。

## 6. Ubuntu 现场验收

本地 36 项覆盖坐标、未知区/足迹、对角切角、绕墙、180° 转向、连续目标、0.25 秒响应滞后模型、实际派生图短程到达、取消替换、传感器故障、元数据、看门狗过期/乱序/超限。响应滞后仅为测试模型，不是机器狗动力学标定。

以下现场项目目前尚未验证：

1. **预览与对齐**：包/环境/ROS 数据正常，地图、机器人、实时障碍一致；未知点、错误 frame、障碍附近目标被拒绝。
2. **低速短程**：先约 1 m，再 2 m，确认前向与左右转向方向。全程观察仿真画面，异常取消并按已有官方方式停车。
3. **到点与停稳**：原点击点误差 ≤0.10 m，线速度 ≤0.025 m/s、角速度 ≤0.06 rad/s 持续 ≥0.8 s；之后观察至少 3 秒，没有漂移或旧目标恢复，画面与状态一致。
4. **重复性**：建议 10 次短程，覆盖不同朝向、一次静态绕障和连续点选，记录误差、耗时和失败原因。次数是工程建议，不是赛规。
5. **替换/取消/退出/重启**：移动中换目标、服务取消、Ctrl+C、关闭 RViz；旧目标失效。重启仿真后须重新初始定位。
6. **异常停车**：受控仿真中验证雷达/里程计断流，导航父节点暂停/退出，观察独立 SDK 零指令与真实停止。只对本轮确认的 PID 操作，不用全局 pkill。SDK 本身被强杀时属于官方失联机制验证。
7. **保留证据**：保存完整 runs 目录、实际参数和必要视频；普通点选闭环通过后，再开发宝箱受控接触与人工消失确认。

先读 `node.log` / `sdk.log`：初始位姿拒绝查静止与里程计；时钟错误查环境与 /clock；No path 查足迹与连通性；Live obstacle 查实际障碍/外参；SDK 返回错误查官方实际返回与连接。不要通过关闭检查、忽略 SDK 错误或填白未知区来强行运行。

## 7. 开发入口

`run_navigation.sh / launcher.py`：启动与退出；`node.py / sensors.py`：ROS 与 RViz；`core.py / geometry.py`：定位、跟踪与停止；`grid.py / prepare_map.py`：地图与 A*；`sdk_bridge.py / sdk_worker.py`：控制与看门狗；`preflight.py / check_install.py`：环境/包核对；`validate.py / tests/`：离线验证；`package_extension.py`：只打包本扩展。

总体任务规则及后续阶段见 [TASK2_PLAN.md](../TASK2_PLAN.md)。本次验收界限仅为普通点选目标自主到达和停止，不把宝箱能力写成已经实现。
