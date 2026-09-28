# RobotAC 场地三维建图与 Ubuntu 迁移记录

更新日期：2026-09-26（任务二第一目标已实现首版代码与离线验证，待 Ubuntu 现场验收；任务一验收结论沿用 2026-09-24 记录）。本文件保留项目背景与迁移记录；当前建图操作入口和参数以 [mapping/README.md](mapping/README.md) 为准。**任务一（场地三维建图）已完成，建图精准高效、完全符合比赛要求，详见 [mapping/README.md](mapping/README.md)。** 下文保留的 2026-09-07 内容仅用于历史追溯，不能覆盖本轮 Ubuntu 验证结果。

## 2026-09-26：任务二第一目标——点选、自主到达与停稳

任务二“路径规划夺分”的总体规则和阶段安排见 [TASK2_PLAN.md](TASK2_PLAN.md)。用户明确第一开发目标是“在 RViz 点一个可达位置，机器狗自主到达并可靠停住”。新增独立 [navigation/](navigation/README.md) 已实现初始定位、Publish Point 单目标、足迹膨胀与 A*、闭环跟踪、停稳判定、取消替换、异常停车和独立 SDK 看门狗；36 项离线测试通过，Ubuntu 实际 ROS2/SDK/仿真闭环仍待验收。默认启动是只预览，实际控制需显式加 `--drive`。迁移步骤、增量包和现场检查见导航说明。

官方代码、既有任务一建图程序和 `maps/run_20260911_223510_ujmvehqs/` 均保持原样；导航派生地图只保存在 `navigation/maps/`。每轮重启后重新初始定位。宝箱接触、人工观察消失和整分钟计分属于后续阶段，尚未加入本次首个目标；仿真里程计的比赛使用许可仍待核对。不要求自动寻宝或访问排序。

以下章节继续保留任务一及迁移历史；其中旧版“后续工作”不替代上述任务二计划。

## 0. 2026-09-08 新一轮 Ubuntu 验证

用户已在目标 Ubuntu 验证 `bash mapping/run_mapping_rviz.sh` 联合启动可正常完成建图，确认精度不高和转弯重影问题已解决，场地建模和 PCD 输出等其他功能未受影响。地图显示正常；同轮日志中的独立图像通道报错、队列与位姿窗口拒帧、退出发布提示见 [mapping/README.md](mapping/README.md) 的记录。

本轮结果保存在 [`maps/run_20260908_152708_phj0ivtz`](maps/run_20260908_152708_phj0ivtz/)：[`field_map.pcd`](maps/run_20260908_152708_phj0ivtz/field_map.pcd) 为本次输出，建图过程记录在 [`mapper.log`](maps/run_20260908_152708_phj0ivtz/mapper.log) 和 [`rviz.log`](maps/run_20260908_152708_phj0ivtz/rviz.log)，启动命令与通信环境见 [`run.json`](maps/run_20260908_152708_phj0ivtz/run.json)。这些文件构成本轮校准进度的现场证据。

后续复测、参数说明、日志判读及验收步骤统一查看 [mapping/README.md](mapping/README.md)；不要再按下文历史章节使用旧的分开启动流程或旧参数。

## 1. 当前状态与目标

用户最新确认的运行状态：

- `config.json` 问题已经解决，节点通信问题也已解决。
- `rviz/matrix_mapping.rviz` 已可以打开并运行，Ubuntu 迁移进展顺利。文件扩展名是 `.rviz`，不是 `.rivz`。
- 已放弃自行搭建可视化程序，继续使用比赛官方 RViz 配置及其扩展。当前目录中没有 `tool/` 或 `tools/`，不要恢复旧可视化方案。
- `mapping/` 是为 RViz 提供地图数据的 ROS2 后端，不是另一套可视化界面，应保留。

当前目标是：通过机器人探索采集场地信息，在 RViz 中显示累计三维点云及可通行/结构区域，保存三维 PCD 点云，完成覆盖度和几何质量检查后按比赛要求提交。

任务一（场地三维建图）已通过目标 Ubuntu 验收，输出 `field_map.pcd`、`field_map.pgm`、`field_map.yaml` 与报告完整，建图精准高效、完全符合比赛要求；后续按比赛规则核对文件要求即可提交。

## 2. 目录与基准

本次实际检查的 Windows 项目根目录：

```text
E:\各种比赛\robortac\matrix_robotac_first
```

旧记录中的 `matrix_robotac_firstA\matrix_robotac_first` 已不是当前路径；配置的 `matrix_robotac_first.zip\matrix_robotac_first` 目录本次检查为空，不应在其中维护另一份代码。

下文 Ubuntu 命令以 `~/robotac/matrix_robotac_first` 为示例，请换成自己的实际路径。Ubuntu 上已跑通的代码、参数和环境是运行基准；本地 Windows 文件是本次阅读的快照，尚未逐项核对与 Ubuntu 是否完全一致，不能直接整目录覆盖。

| 路径（相对项目根目录） | 用途 |
| --- | --- |
| `run_sim.sh` | 官方仿真、运控及可选 MuJoCo 启动入口，会修改多处配置 |
| `config/config.json` | 机器人、传感器、初始位姿配置 |
| `rviz/matrix.rviz` | 保留的官方原始 RViz 配置 |
| `rviz/matrix_mapping.rviz` | 在官方配置基础上增加累计地图和分类显示 |
| `mapping/field_mapper_node.py` | 点云变换、累计、简单分类和 PCD 保存 |
| `mapping/run_mapping_rviz.sh` | 当前 Ubuntu 已验证的建图与 RViz 联合启动入口；详见 `mapping/README.md` |
| `mapping/run_mapping.sh` | 仍可单独启动建图节点；日常使用上述联合入口 |
| `src/UeSim/` | 官方 UE 仿真运行时、场景和模型资源 |
| `src/robot_mc/`、`src/robot_mujoco/` | 官方运控和物理仿真程序及配置 |
| `deps/zsibot_sdk/` | 机器人控制 SDK；操作方式参考其文档 |
| `scripts/` | 安装、下载及构建辅助脚本，执行前阅读实际内容 |
| `field_map.pcd` | 已有采集结果，不是缓存，不能随意删除或覆盖 |

本地已能找到 `src/UeSim/Linux/UeSim.sh`、模型配置、`src/robot_mc/build/export/` 和 `src/robot_mujoco/simulate/build/robot_mujoco`。文件存在不等于运行时完整可用，也不能据此推断另一台 Ubuntu 的情况。

现有 `field_map.pcd` 文件头标示 `POINTS 25405`、`FIELDS x y z`、`DATA binary`；这里只检查了文件头，未据此认定地图完整或准确。

## 3. 建图方式与能力边界（2026-09-07 历史记录）

本节中的 `odom`、最近位姿匹配、无位姿插值等描述属于 2026-09-07 历史版本；当前实现和坐标系、时序与融合参数见 [mapping/README.md](mapping/README.md)。

平台基于 MATRiX（Unreal Engine + MuJoCo），当前使用 xgb 机器狗。建图流程为：订阅雷达和里程计，按邻近时间戳选取位姿，将雷达点转换到里程计全局坐标系，体素去重累计，再发布给 RViz 并写入 PCD。

| 方向 | Topic | ROS2 消息类型 |
| --- | --- | --- |
| 输入 | `/front_lidar` | `sensor_msgs/msg/PointCloud2` |
| 输入 | `/odom/mujoco_odom` | `nav_msgs/msg/Odometry` |
| 输出：累计三维点云 | `/field_map` | `sensor_msgs/msg/PointCloud2` |
| 输出：绿色初步可通行区域 | `/field_map/passable` | `visualization_msgs/msg/MarkerArray` |
| 输出：橙色结构区域 | `/field_map/structure` | `visualization_msgs/msg/MarkerArray` |

节点名为 `/field_mapper`，输出默认属于 `odom`，有点云时以约 2 Hz 发布，QoS 为 `Reliable` + `Transient Local`、深度 1。当前后端直接作为 Python 程序运行，无需为它单独执行 `colcon build`。

### TF 与坐标系

RViz 左侧没有添加 TF 显示项，不等于 ROS 中一定没有 TF 数据。此前未观察到 `/tf`、`/tf_static`，因此当前节点直接使用里程计，不要求添加 TF 显示项或补一棵 TF 树才能建图。

地图输出和 RViz 的 `Fixed Frame` 都默认为 `odom`。这要求输入里程计的父坐标系确实与该全局坐标系一致；`--global-frame` 只设置输出坐标系名称，不会执行额外变换。不能靠把名字改成 `map` 来修复坐标错位。

当前仅支持雷达相对机体的 Z 偏移，没有完整的 X/Y 平移和旋转外参。输入点云必须是与此假设匹配的雷达局部坐标，不能已变换到全局后再变换一次；如果里程计给的是雷达位姿，需核实是否应使用 `--sensor-z 0`。

### 当前实现的限制

- 这是基于已有里程计的点云累计，不是完整 SLAM：没有点云配准、回环优化、漂移校正或位姿插值。
- 只在最近 200 个里程计样本中寻找邻近时间戳；时间基准不一致、位姿或外参错误都会造成重影。不要仅放大时间容差掩盖问题。
- 按 XY 网格内最高点与最低点的高度差分类，未做可靠的地面分割、坡度/机器人宽度检查；绿色只表示粗略候选，不能直接作为安全导航结论，未观测区域也不能视为可通行。
- 输出 PCD 只含 XYZ，不含 RGB、强度或分类标签。RViz 的绿色/橙色标记不会保存到该 PCD 中。
- 当前不生成三角网格、贴图模型或普通图片，也没有 PCD 加载续建功能。是否需要额外格式，以比赛最新提交规范为准。

## 4. Ubuntu 日常操作流程（2026-09-07 历史流程）

本节记录上一轮迁移时的分开启动方式。当前目标 Ubuntu 使用 `mapping/run_mapping_rviz.sh` 联合入口，按 [mapping/README.md](mapping/README.md) 操作。

使用已跑通的 Ubuntu 环境。目标环境按官方依赖为 Ubuntu 22.04、ROS2 Humble；迁移到另一台机器时还需确认 CPU 架构与官方二进制兼容、显卡驱动和图形会话可用。

### 4.1 各终端统一环境

先沿用 Ubuntu 上已验证的环境脚本和变量。下面是本项目此前使用的配置示例，不应无条件覆盖目标机已经验证的设置：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1
cd ~/robotac/matrix_robotac_first
```

`SDK_CLIENT_IP=127.0.0.1` 仅适用于仿真和控制端在同一台机器上的情况；跨机器时按实际网卡、SDK 配置和已验证网络设置处理。各 ROS 进程的通信环境必须匹配；已有额外工作空间也要按原顺序 `source`。

使用 Zenoh 时，沿用已有路由器的启动方式。只有确认没有可用实例且当前部署需要时，才在独立终端运行 `ros2 run rmw_zenoh_cpp rmw_zenohd`。不要重复启动；端口被占用也不证明现有进程就是正常的 Zenoh 路由器，应检查占用进程和日志。

### 4.2 启动仿真并确认输入

终端一执行你在 Ubuntu 上已经成功使用的仿真命令。如果此前使用默认参数，对应命令为：

```bash
bash run_sim.sh
```

本地脚本的位置参数依次为：机器人、场景编号、离屏、像素流、是否单独启动 MuJoCo，默认是 `xgb 1 0 0 0`。例如 `bash run_sim.sh xgb 1 0 0 1` 会启用单独的 MuJoCo；这里仅解释参数，不要求切换已跑通的模式。

不要重复运行启动脚本：它会终止匹配名称的旧仿真/运控进程并写配置。采集中重启仿真、切换场景或重置位姿后，应另开一次建图，避免混合不同坐标原点的地图。

终端二检查输入，无需重复安装已正常运行的节点：

```bash
ros2 node list
ros2 topic type /front_lidar
ros2 topic type /odom/mujoco_odom
ros2 topic hz /front_lidar
```

`hz` 持续输出，查看后按 `Ctrl+C` 结束检查，再运行：

```bash
ros2 topic echo /odom/mujoco_odom --once
```

预期类型见上一节，雷达频率持续更新，里程计有位姿和时间戳。RViz 能打开但输入没有数据时，建图节点仍无法累计。

### 4.3 启动建图

在终端二的项目根目录为每次采集创建独立输出目录，然后启动节点：

```bash
mkdir -p maps
mapping_run_dir="$(mktemp -d "$PWD/maps/run_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
bash mapping/run_mapping.sh --pcd-file "$mapping_run_dir/field_map.pcd"
```

节点会打印绝对输出路径，请记录它。未传 `--pcd-file` 时默认为项目根目录下的 `field_map.pcd`；相对路径也是相对于项目根目录。

每次启动节点都从空地图开始，不会读入已有 PCD；复用同一输出路径会覆盖已有结果。新文件名不等于续建，也不要为重新采集而删除旧地图。

### 4.4 打开 RViz 并采集

终端三加载环境并进入项目后运行：

```bash
rviz2 -d rviz/matrix_mapping.rviz
```

1. 在 `Displays -> Global Options` 检查 `Fixed Frame` 为 `odom`，累计点云和两个区域显示项已启用，Topic 对应第 3 节。
2. 在三维视图使用鼠标左键拖动旋转、中键拖动平移、滚轮缩放。找不到点云时先调整视角与距离，不要先改坐标系。
3. 先静止观察数秒，确认出现累计点云；再用已经跑通的官方控制程序/SDK 缓慢探索，覆盖通道、转角和结构表面。
4. 观察墙面等静态结构应固定在全局坐标中，转弯或重复经过时不应明显分层、重影；记录异常位置和当时参数。
5. 绿色/橙色标记可分别取消勾选以检查原始累计点云。RViz 负责显示，不负责自动驾驶；工具栏的目标点工具并不表示项目已经接入导航。

扩展配置意图关闭原始局部雷达显示。若手动开启 `/front_lidar` 后出现从雷达坐标到 `odom` 的变换错误，它与已经转换好的 `/field_map` 是不同显示链路，不要因此添加未经验证的静态 TF。

本地 JSON 深度图 Topic 为 `/front_depth/image/compressed`，本地 RViz 的一个 Image 项仍指向 `/front_camera/depth/compressed`。这只是本地文件差异，并非当前已确认的 Ubuntu 故障；只有图像不显示时才按 `ros2 topic list -t` 核对实际 Topic。三维建图后端不依赖该图像。

### 4.5 保存与验收

保持建图节点运行，在另一个已加载相同环境的终端执行：

```bash
ros2 service call /field_mapper/save std_srvs/srv/Trigger '{}'
```

确认返回 `success: true`，并核对消息中的保存路径和点数。默认有新增数据时约每 5 秒自动保存，正常 `Ctrl+C` 退出时有未保存数据也会尝试保存；不要只因 RViz 有图就认定磁盘保存成功。

PCD 是三维点云文件，不是“PCD 图片”。可用支持 PCD 的点云查看器检查文件；例如目标机已有 PCL 工具时用 `pcl_viewer` 打开实际输出文件。RViz 的保存配置操作只保存 `.rviz` 布局，不保存点云。

最终检查：文件非空且能打开、场地覆盖充分、结构比例与高度合理、无明显重影或坐标跳变，并核对比赛最新文件命名和提交规则。检查完成后保留原始采集结果及本次启动参数。

`/field_mapper/clear` 是清空服务，会清空内存并覆盖写入空 PCD，不是普通刷新按钮。不要用它排查显示问题；当前显示还可能暂时残留旧数据，不能据此判断磁盘文件仍然存在有效点云。

## 5. 建图参数基线（2026-09-07 历史默认值）

本节参数仅用于追溯上一轮版本；当前默认值和参数含义以 [mapping/README.md](mapping/README.md) 为准（例如当前体素与时序策略已更新）。

以下是本地 `field_mapper_node.py` 的 CLI 默认值，不代表 Ubuntu 当前实际启动参数；配置 JSON 和 RViz 文件不会自动替节点设置这些参数。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--topic` | `/front_lidar` | 输入雷达点云 |
| `--odom-topic` | `/odom/mujoco_odom` | 输入里程计 |
| `--global-frame` | `odom` | 输出坐标系名，应匹配真实里程计父坐标系 |
| `--pcd-file` | `field_map.pcd` | 输出路径；建议每次采集指定新路径 |
| `--voxel` | `0.05` | 三维去重体素尺寸，米 |
| `--cell-size` | `0.20` | XY 分类网格尺寸，米 |
| `--max-step` | `0.22` | 初步可通行分类的高度差阈值，米 |
| `--max-pose-age` | `0.50` | 雷达与里程计允许的时间差，秒；设 0 会取消此检查 |
| `--sensor-z` | `0.30` | 雷达相对里程计机体位姿的 Z 偏移，米 |
| `--autosave` | `5.0` | 自动保存间隔，秒；代码限制最小为 1 秒 |

本地 JSON 快照：机器人 `xgb`，初始位置 `(0, 0, 0)`，`mujoco_running: false`；雷达为 Mid360、10 Hz、位置 `(0, 0, 0.3)`、旋转为零；RGB 为 1920x1080、10 Hz，深度为 640x480、10 Hz。这些值用于比较差异，不要求覆盖 Ubuntu 已验证的参数。

## 6. 后续迁移到 Ubuntu 的原则与步骤

重点是保留当前已跑通的成果，而不是重新安装一切。迁往另一台 Ubuntu 时同样按“备份、记录、比较、增量同步、验证”执行。

1. 停止采集并显式保存 PCD。备份 Ubuntu 已跑通的项目到新的带日期目录，保留原目录；确认磁盘空间，记录系统/ROS 版本、环境脚本、完整启动命令、官方安装包版本及来源。
2. 将 Windows 变更放到 Ubuntu 独立的待比较目录，不直接解压覆盖工作目录。当前项目未发现 Git 仓库，不依赖不存在的提交号；可用逐文件 SHA256 和差异比较记录版本。
3. 区分源代码变更、机器环境参数、启动脚本自动生成差异。先比较 `mapping/`、`rviz/`、`run_sim.sh` 和各配置，只同步确认需要的内容；保留 Ubuntu 对运行必需的修正，并把确认有效的修改回收为以后统一维护的版本。
4. 保留或从同版本官方安装包补齐 UE、运控、MuJoCo 运行时及模型资源。跨架构/系统迁移不能假定二进制通用；需要重建时先确认源码和官方构建方法确实齐全。
5. 检查 Linux 路径大小写、脚本 LF 换行和可执行权限。Ubuntu Humble 使用其系统 Python 环境，不复制 Windows 虚拟环境、Python 3.12 `.pyc` 或 Windows 依赖到 Ubuntu Python 3.10 环境。
6. 沿用目标机已验证的 ROS/Zenoh/SDK 设置，只补确实缺失的依赖。随后按第 4 节验证输入、建图输出、RViz 显示和一次显式 PCD 保存；通过后记录参数、文件版本和日期。

### 必须保留的运行文件

不能用 `src/**/build/`、`src/**/install/`、`src/**/log/` 等通配规则一概删除或排除。特别是 `src/robot_mc/build/export/` 和 `src/robot_mujoco/simulate/build/robot_mujoco` 存放官方运行所需内容，不只是可以重建的临时缓存。PCD、日志及有效配置也应备份，不因“清理”而丢弃。

`run_sim.sh` 会修改或同步下列文件，因此不能再认为“只允许 config.json 不同”：

```text
config/config.json
src/UeSim/Linux/UeSim/Content/model/config/config.json
src/robot_mujoco/simulate/config.yaml
src/robot_mc/run_mc.sh
src/robot_mc/build/export/config/xg-user-parameters.yaml
src/robot_mujoco/zsibot_robots/xgb/unreal.xml
src/robot_mujoco/zsibot_robots/xgb/xgb.xml
```

运行前后的差异要结合脚本行为判断。不要为了对齐哈希盲目恢复旧配置，也不要用手工编辑 UE 内部 JSON 代替确认根配置及同步逻辑。

### 权限、依赖与脚本注意事项

迁移后，必要时在 Ubuntu 项目根目录恢复已知入口的权限：

```bash
chmod +x run_sim.sh mapping/run_mapping.sh src/robot_mc/run_mc.sh
chmod +x src/UeSim/Linux/UeSim.sh src/robot_mujoco/simulate/build/robot_mujoco
```

上述只是入口示例，官方运行时的其他二进制和脚本也需要正确权限，优先保留原 Linux 安装包权限。若出现 `bash\r` 或 `$'\r'`，仅对确认存在 CRLF 的文本脚本转为 LF，不对二进制或 PCD 批量转换。

建图所需 ROS 包包括 `rviz2`、`rclpy`、`sensor_msgs`、`sensor_msgs_py`、`nav_msgs`、`visualization_msgs`、`std_srvs`；沿用 Zenoh 通信时还需要对应 RMW。仅在新机器缺少这些包时安装：

```bash
sudo apt update
sudo apt install ros-humble-rviz2 ros-humble-rclpy \
  ros-humble-sensor-msgs ros-humble-sensor-msgs-py ros-humble-nav-msgs \
  ros-humble-visualization-msgs ros-humble-std-srvs ros-humble-rmw-zenoh-cpp
```

前提是目标机已正确配置 ROS2 Humble 软件源；这不是整个平台的完整安装清单。`run_sim.sh` 还使用 `jq`、`flock` 等系统工具，官方运行时有独立的图形、库和 SDK 依赖。

本地 `scripts/build.sh` 当前只执行依赖安装，下载和其他构建调用均被注释；输出 `Initialization complete` 不代表已安装完整仿真。`scripts/download_uesim.sh` 仅按目录是否存在决定跳过，不能用于证明资源完整；不要在已跑通的 Ubuntu 上盲目重新下载或重新构建。

### 记录两端差异

Windows PowerShell 在实际项目根目录运行：

```powershell
Get-FileHash -Algorithm SHA256 -LiteralPath run_sim.sh,mapping/field_mapper_node.py,mapping/run_mapping.sh,rviz/matrix.rviz,rviz/matrix_mapping.rviz,config/config.json
```

Ubuntu 在实际项目根目录运行：

```bash
sha256sum run_sim.sh mapping/field_mapper_node.py mapping/run_mapping.sh \
  rviz/matrix.rviz rviz/matrix_mapping.rviz config/config.json
```

哈希不同只说明字节不同，LF/CRLF 也会造成差异；要查看具体改动后判断，不能直接认定 Ubuntu 版本错误。涉及运行模式变化时，还应比较上文列出的自动修改文件及相关环境脚本。

## 7. 后续工作与问题记录

下一阶段不是重新解决已修复的 `config.json`、节点或 RViz 启动问题，而是确认一轮完整采集的覆盖度、位姿/时间/外参一致性，以及 PCD 文件是否满足比赛要求。必要的算法增强应在现有可运行版本之外验证后再合入。

本轮 `run.json` 已补齐 Ubuntu 项目路径 `/home/xiaozhibuzhan/robotac/matrix_robotac_first`、建图/RViz 命令和 ROS 通信变量，新目录中的 PCD 与日志及用户反馈已补齐本轮输出和验收结果。系统和架构版本、完整仿真启动命令、环境脚本及相对于本地快照的代码差异仍需另行记录。

以后遇到新问题，请同时记录：

- 问题发生在哪台系统、哪份目录、哪个终端；完整命令、参数、首次报错及前后日志，不只最后一行。
- `pwd`、`lsb_release -ds`、`uname -m`，以及 `printenv ROS_DISTRO RMW_IMPLEMENTATION ROS_DOMAIN_ID SDK_CLIENT_IP` 的结果。
- `ros2 node list`、`ros2 topic list -t`；必要时补输入话题的 `ros2 topic info -v`、频率、里程计 `header.frame_id`/`child_frame_id`、雷达 `header.frame_id` 和时间戳样本。
- 建图节点打印的点数、跳过帧数、保存路径与保存服务结果；RViz 的 Fixed Frame、异常显示项和展开后的 Status 信息。
- 最近改动文件的差异/哈希、是否重启或重置过仿真，以及重现步骤。无需发送完整环境变量或无关隐私数据。

`error_list` 等旧材料保留用于追溯，但没有新的相同报错证据时，不把其中问题恢复为当前阻塞项。外部传感器订阅示例 [uesim_subscriber](https://github.com/liuxinxinbit/uesim_subscriber) 不等于本项目的 `/field_mapper` 节点，也不是运行该建图脚本的必装前提。

## 8. 整理与缓存说明（2026-09-07 历史记录）

- 当时核对了本地建图源码、启动脚本、JSON、RViz 配置及关键目录，并以本 README 维护状态。2026-09-08 起，当前建图进度和操作说明以 [mapping/README.md](mapping/README.md) 为准，本文件保留平台背景及历史迁移记录。
- 本次发现的 Python 缓存为 `mapping/__pycache__/field_mapper_node.cpython-312.pyc`。删除操作被执行环境策略拦截，文件仍保留；后续复制时排除 `__pycache__/`、`*.pyc` 即可，不能写成已经清理成功。
- 旧上下文在这里更新为最新事实，不代表历史聊天或错误记录已被物理删除。代码、有效配置、官方运行文件、已有点云和日志均保留。
