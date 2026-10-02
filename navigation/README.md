# 任务二第一开发目标：RViz 点选 → 自主到达 → 停稳

更新：2026-10-02。目标按用户确认执行：**在 RViz 点一个可达位置，机器狗自主到达并可靠停住。** 本目录是独立扩展，不修改官方代码，不改变已验收的任务一建图程序与原始地图。

**当前状态：最新现场日志已有 4 次 ARRIVED；本次加速和长路线恢复修复待 Ubuntu 复测。** 本地 96 项测试全部通过，128 个选定官方/任务一文件 SHA256 未变化。结果见 [验证报告](validation/local_validation.json) 和 [测试日志](validation/offline_tests.txt)。Windows 测试不能代替真实 SDK、步态外形、制动距离和现场重复测试。

**最新修复：**18:00:42 开始的日志中，5 个目标有 4 个到达；第 4 个因跟踪偏移耗尽累计 3 次恢复额度而停止。本次默认速度改为 0.40 m/s、转向改为 0.60 rad/s，路径优先保留额外余量，恢复额度按连续失败计算。**旧 local.yaml 会覆盖新默认值，必须检查。** 见 [本次诊断、更新与复测步骤](LONG_ROUTE_FIX_20261002.md)。

历史修复已保留：先 standUp、等待 4 秒再 move；SDK 非零速度下限适配；Humble 幂等关闭。历史证据见 [启动修复](STARTUP_FIX_20261002.md) 和 [低速修复](VELOCITY_FIX_20261002.md)，不要把这些较早日志当成本轮连接状态。

## 1. 已实现与当前边界

- 加载 PGM/YAML；未知区、地图外部、障碍及足迹空间不足区域不可通行。
- `2D Pose Estimate` 输入**机器人当前实际位置及朝向**，建立地图与里程计变换；不会移动或重置仿真中的机器人。
- `Publish Point` 输入一个当前目的地。先停稳再由后台线程 A* 规划，等待期间持续输出零速度并接收传感器；取消或替换后丢弃旧结果。检查对角切角和简化路径穿障；无效点会被拒绝，不偷偷吸附到另一个位置。
- A* 和路径简化优先保留额外约 0.10 m 余量，减少贴边转弯；这是软偏好，不拒绝原有合法窄路，不改变足迹或目标位置。
- 用里程计闭环跟踪路径，只发送前向速度和偏航角速度；在拐点先减速停稳再转向。跟踪偏差使旧线段不可行时，先停稳，最多连续做 3 次静态图恢复重规划；剩余路线缩短至少 0.75 m 且实际位移至少 0.375 m 后复位连续计数。总恢复数仍记录，最终目的地和总时限不变。
- 只有同时满足位置、实际速度和持续时间条件，才报告 `ARRIVED`。发送零速度不等于已经停稳。
- 新点击撤销旧目标；支持取消、退出，以及传感器断流、时间戳倒退、位姿跳变、异常倾斜、无进展时停车。
- 独立 SDK 子进程检查命令生成时刻和序号。父节点退出/管道断开会停车；移动命令超过 0.30 秒未更新则停车并锁定，必须重启导航。
- 保存参数、地图/代码哈希、初始位姿、点击、轨迹、状态和异常日志。

静态障碍由全局路径绕开；**新出现的局部障碍目前导致停车、取消目标，由人检查后重新点选**，尚未实现动态障碍自动绕行。也未实现宝箱接触、识别/消失确认、比赛计时计分、多目标队列、自动寻宝和在线地图匹配定位。这些不属于本次第一开发目标。

## 2. 安装与操作入口

**首次操作请按 [OPERATIONS_UBUNTU.md](OPERATIONS_UBUNTU.md) 逐步执行。** 它随增量包一起提供，包含 Windows 打包、Ubuntu 路径检查、校验、备份、安装、配置、四个终端的分工、RViz 点选、停止、日志和回退；只拿到 ZIP 也能离线查阅。项目根 README 的第 0～9 节与该操作指南同步维护。

所有新增执行文件、配置、派生地图、RViz、测试与日志都在 `navigation/`。官方 `run_sim.sh`、`config/`、原有 `rviz/`、`src/`、`deps/`、已验收的 `mapping/` 和任务一 `maps/` 保持原样。128 项校验覆盖选定源文件/配置，不表示扫描了全部 UE 二进制。`protected_baseline.json` 是开发机的防误改记录，不要求目标 Ubuntu 配置与 Windows 完全一致。

增量包为 `navigation/updates/task2_point_navigation_20260926.zip` 及同名 `.sha256`。包内只含 `navigation/`，不含官方运行库、任务一源地图或 ROS 环境。**不要用整个 Windows 工程覆盖 Ubuntu，不要删除官方 build 或已跑通的运行库。** SDK 只加载已有官方库，不需要编译 SDK、执行 `colcon build` 或重建整个仿真。

## 3. 启动接口与运行约定

完整操作顺序以 [操作指南](OPERATIONS_UBUNTU.md) 为准；以下供已经完成安装检查的开发者查接口：

| 操作 | 工程根目录下的命令 |
| --- | --- |
| 离线地图/配置检查 | `/usr/bin/python3 -B -m navigation.preflight --offline --config navigation/config/local.yaml` |
| ROS 与 SDK 文件预检查，不连接机器人 | `/usr/bin/python3 -B -m navigation.preflight --drive --config navigation/config/local.yaml` |
| 预览，不连接 SDK | `bash navigation/run_navigation.sh --config navigation/config/local.yaml` |
| SDK 已允许 move 状态时的实际控制 | `bash navigation/run_navigation.sh --config navigation/config/local.yaml --drive` |
| 本轮需要请求一次 standUp | 在上一行命令末尾追加 `--stand-up` |
| 查看状态 | `ros2 topic echo /task2/status`；Ctrl+C 只结束查看 |
| 取消目标 | `ros2 service call /task2/cancel std_srvs/srv/Trigger '{}'` |

- 使用目标机原有 Ubuntu 22.04 / ROS2 Humble / 系统 CPython 3.10，保留原 source/overlay 顺序和已验证的通信变量；不要用 Conda 解释器代替。官方 SDK 为 Linux x86_64 / aarch64 二进制。
- `local.yaml` 需按操作指南创建。SDK 地址取其中 `client_ip / robot_ip / client_port`，默认 `127.0.0.1 / 127.0.0.1 / 43988`，不会被 `SDK_CLIENT_IP` 自动覆盖。
- 每次启动重新 `2D Pose Estimate` 定位，再 `Publish Point` 选目标；本版本不用 `2D Goal Pose`。预览不动时，约 15 秒无进展取消目标是预期行为。
- 只保留一个导航实例，并关闭其他运动命令发送者。同 domain 的启动器和同端口的 SDK worker 有锁，但不能阻止官方 demo 同时控制。
- 仅当现场传感器确定使用仿真时间且 `/clock` 非零、持续推进时，预览和控制都追加 `--use-sim-time`。控制定时器和 SDK 看门狗采用单调时间，仿真时钟暂停不会暂停过期停车。
- 正常状态为 `WAIT_INITIAL_POSE → IDLE → STOPPING_FOR_PLAN → PLANNING → TRACKING → SETTLING → ARRIVED`。后台规划期间保持停车，不阻塞传感器回调。异常停止不自动恢复旧目标；里程计重连、重置、跳变或换 frame 后须重新初始化，SDK worker 超时或退出后须重启导航。
- 取消服务成功表示已清除目标并请求零速度，仍须核对实际停止。导航终端 Ctrl+C 或关闭本轮 RViz 会回收本轮导航进程，不关闭官方仿真和 Zenoh。
- 日志在 `navigation/runs/run_日期_时间_随机后缀/`。`run.json` 保存环境、命令和哈希，`config.yaml` 保存本轮参数，`events.jsonl` 含点击、规划路径、状态与约 2 Hz 轨迹；状态记录当前路点、剩余路径长度、总恢复数及连续恢复数。排错看 `node.log / sdk.log / rviz.log`，预览无 SDK 日志。

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
| `max_speed / max_yaw_rate` | 0.40 m/s / 0.60 rad/s | 开阔路段上限，转弯与接近目标仍降速；旧 local.yaml 须同步修改 |
| `acceleration` | 0.20 m/s² | 期望起步累计与制动模型参数，SDK 的 0→0.05 命令阶跃使它不是实际加速度保证，须实测 |
| `goal_tolerance` | 0.10 m | 距离原点击点的误差 |
| `stop_speed / stop_yaw_rate` | 0.025 m/s / 0.06 rad/s | 由约 0.15 秒里程计差分估计实际速度 |
| `settle_seconds` | 0.8 s | 位置和速度条件同时成立的持续时间 |
| `odom_timeout / cloud_timeout` | 0.35 / 0.45 s | 有效数据过期取消目标，重复时间戳不刷新 |
| `command_timeout` | 0.30 s | SDK 独立进程移动命令期限，超时锁定 |
| `max_header_age` | 0.50 s | 检查传感器 header 与节点时钟 |
| `progress_timeout / goal_timeout` | 15 / 180 s | 无路径或实际转向进展/总时限超出时停止；重规划不重置总时限 |
| `max_tracking_replans` | 连续 3 次 | 同一处持续失败仍停止；分散在长路线上的恢复不共用终身 3 次额度 |
| `replan_progress_distance` | 0.75 m | 剩余路线减少达到该值、且实际位移达到其一半，才复位连续恢复计数 |
| 障碍高度带 | 0.15～1.20 m | 不是台阶、沟沿、落差等全地形感知 |

SDK 前向输出为 0 或至少 0.05 m/s，偏航为 0 或绝对值至少 0.02 rad/s；默认最大值为 0.40 m/s 与 0.60 rad/s。最低前进速度也先通过安全检查。预测制动距离取实际速度和命令速度中的较大者；临近拐点时不再无条件把直线预测延伸到拐点之后，但始终保留完整制动距离。

实时障碍保护考虑预测弧线、圆形足迹、反应时间、假定制动和缓冲；依赖视野、外参、地面高度。软件看门狗不是硬件急停：SDK 自身被强杀/挂死、网络完全断开、官方运控不接受零指令时，不能由本程序保证停车，须核对官方运控失联行为。

## 6. Ubuntu 现场验收

本地 96 项覆盖坐标、未知区/足迹、对角切角、绕墙、180° 转向、连续目标、0.25 秒响应滞后模型、实际派生图短程到达、取消替换、传感器故障、元数据、看门狗过期/乱序/超限，以及 SDK 启动和速度域、ROS 退出与打包规则。本次新增日志长路线、分散/连续恢复、实际转向进展、0.40 m/s 障碍停车、路径余量及后台规划等待/取消/过期结果回归。响应滞后仅为测试模型，不是机器狗动力学标定。长路线比较见 [离线报告](validation/long_route_validation.json)。

上个版本的现场日志已有 4 次到达，尚不构成全部边界与重复性验收。以下是本次加速版本的现场复测清单：

1. **预览与对齐**：包/环境/ROS 数据正常，地图、机器人、实时障碍一致；未知点、错误 frame、障碍附近目标被拒绝。
2. **低速短程**：先约 1 m，再 2 m，确认前向与左右转向方向。全程观察仿真画面，异常取消并按已有官方方式停车。
3. **到点与停稳**：原点击点误差 ≤0.10 m，线速度 ≤0.025 m/s、角速度 ≤0.06 rad/s 持续 ≥0.8 s；之后观察至少 3 秒，没有漂移或旧目标恢复，画面与状态一致。
4. **重复性**：建议 10 次短程，覆盖不同朝向、一次静态绕障和连续点选，记录误差、耗时和失败原因。次数是工程建议，不是赛规。
5. **替换/取消/退出/重启**：移动中换目标、服务取消、Ctrl+C、关闭 RViz；旧目标失效。重启仿真后须重新初始定位。
6. **异常停车**：受控仿真中验证雷达/里程计断流，导航父节点暂停/退出，观察独立 SDK 零指令与真实停止。只对本轮确认的 PID 操作，不用全局 pkill。SDK 本身被强杀时属于官方失联机制验证。
7. **保留证据**：保存完整 runs 目录、实际参数和必要视频；普通点选闭环通过后，再开发宝箱受控接触与人工消失确认。

先读 `node.log` / `sdk.log`：初始位姿拒绝查静止与里程计；时钟错误查环境与 /clock；No path 查足迹与连通性；Live obstacle 查实际障碍/外参；SDK 返回错误查官方实际返回与连接。不要通过关闭检查、忽略 SDK 错误或填白未知区来强行运行。

## 7. 开发入口

`run_navigation.sh / launcher.py`：启动与退出；`node.py / sensors.py`：ROS 与 RViz；`core.py / geometry.py`：定位、跟踪与停止；`grid.py / prepare_map.py / planning.py`：地图、A* 与后台规划；`sdk_bridge.py / sdk_worker.py`：控制与看门狗；`preflight.py / check_install.py`：环境/包核对；`validate.py / tests/`：离线验证；`package_extension.py`：只打包本扩展。

完整源码工程中的总体任务规则及后续阶段见 [TASK2_PLAN.md](../TASK2_PLAN.md)；该文件不在 navigation 增量包内。本次验收界限仅为普通点选目标自主到达和停止，不把宝箱能力写成已经实现。
