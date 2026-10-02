# 手动重新建图与比赛换场景：Ubuntu 操作指南

更新：2026-10-02。按用户确认，本指南针对同一套官方 UE/MuJoCo 仿真换场景。复用现有建图与导航扩展，不修改官方代码，不覆盖已验收地图。以下标准参数用于当前平坦、世界地面高度为 0 的场地。

完整流程：**关闭导航 → 手控采集新场地 → 结束录制 → 生成 PCD/原始栅格 → 生成导航派生图 → 保存场地配置 → 预览与实际导航。**

## 1. 最新日志与这次重建重点

`navigation/run/run_20261002_205420_pybkp01d/` 有 9 次点击：5 次到达、3 次无法规划、1 次目标余量不足。5 次到达全部没有重规划，最长约 38.96 m 的路线用时 126.20 s。此前速度与半路恢复问题本轮未复现。

3 次规划失败的目标位于白色地面，但按机器人足迹计算后，与机器人所在主区域分离；原因是 `No path in known free space after footprint inflation`。本地按日志所用同一张图复现如下：

| 当前旧地图中的区域 | 现象 | 优先重新采集的位置 |
| --- | --- | --- |
| 目标约 (7.3, -7.0) | 落在独立小区域，入口可用宽度不足 | 约 (6.7, -7.5) 周围入口、走廊地面和两侧 |
| 目标约 (19.7, 6.0) | 原始白色区仅通过很细的连接相通，足迹计算后断开 | 约 (18.9, 6.2) 周围入口和转角 |
| 点击约 (7.95, -6.60) | 自身余量约 0.18 m，不足要求的 0.40 m | 核对实际障碍边界和缺测地面 |

这些坐标仅用于定位**当前旧图**的补扫区域，不是新场景坐标，也不能当作 `2D Pose Estimate` 的出生点。

当前 `robot_radius: 0.35` 加 `safety_margin: 0.05`，意味着机器人中心周围至少需要约 0.40 m 已知自由空间，还要考虑栅格的保守边界。因此“目标像素是白色”和“机器狗有连续通道到达”是两回事。上述入口附近存在未知区域，值得补扫；日志本身不能证明所有被阻挡区域实际都可通行。

## 2. 先确认本次要设置的内容

| 设置 | 当前场地先用 | 何时改变 |
| --- | --- | --- |
| 场地版本名 `MAP_ID` | 如 `field_a_20261002_210000` | 每次新采集或生成新版本都用新名字 |
| 全局坐标系 | `world` | 只有实际里程计父 frame 改变才改；改名字不等于坐标转换 |
| 点云/里程计 topic | `/front_lidar` / `/odom/mujoco_odom` | 以新场景实际 topic 为准，同套仿真通常沿用 |
| 雷达安装偏移 | 建图 `--sensor-z 0.30`；导航 `sensor_translation: [0, 0, 0.30]` | 机器人或雷达安装改变时重新核实 |
| 世界地面高度 | 建图 `--ground-z 0.0`；导航 `ground_z: 0.0` | 根据实际世界地面确认，不能用雷达安装高度代替 |
| 原始地图分辨率 | `--pgm-resolution 0.05` | 每格 5 cm；变细不能补出未测量地面 |
| 最终建图地面带 | `--ground-band 0.12` | 地面中心上下各 12 cm，先检查高度与回波再调整 |
| 导航派生图地面带 | `--band 0.06` | 地面中心上下各 6 cm，与上一项不是同一个参数 |
| 有界地面补全 | `--max-edge 0.30` | 只在局部实测支持内补全，优先补采，不盲目调大以跨过缺测区 |
| 机器人足迹 | `0.35 + 0.05 m` | 依据机器人真实外形和步态核实，不为了连通而缩小 |
| 导航速度 | `0.40 m/s / 0.60 rad/s` | 沿用本轮已跑通配置；不控制官方键盘 demo 的速度 |

**当前导航是单平面二维地图。** 若新场景有多层、明显坡道或世界地面高度不为 0，需要另核对地面提取、地图显示平面和点选高度；不能承诺只改 `ground_z` 就适配。`--sensor-z` 始终是安装外参，不用于“把地面调到 0”。

## 3. 准备终端，退出导航

使用 Ubuntu 已经跑通的工程，而不是 Windows 精简 Git 副本。每个新 Bash 终端先执行：

```bash
TASK2_PROJECT="$HOME/robotac/matrix_robotac_first"  # 改为实际已有工程
cd "$TASK2_PROJECT"
source /opt/ros/humble/setup.bash
# 如原来还需 source 其他 overlay，按现场已验证的顺序执行。
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1
/usr/bin/python3 --version
```

这些通信值沿用当前已验证的同机仿真设置。系统 Python 应为 3.10.x，不进入 Conda。

1. 先在导航终端 Ctrl+C，等待退出并确认机器狗停稳；建图时不同时运行 `--drive` 导航。
2. 当前场地重建时保留已工作的官方仿真和 Zenoh，不重复启动。
3. 比赛换场景时，先结束上一场地录制，再按官方方法选新场景、重启仿真并回到规定起点；使用目标机已经验证的官方启动命令与参数，不照抄无参数 `run_sim.sh`。
4. 一次录制只对应同一场景和连续坐标系。录制中不重置仿真、不传送机器人、不混录两个场地。

终端分工：

| 终端 | 用途 |
| --- | --- |
| A | 已工作的官方仿真/运控 |
| B | 已工作的 Zenoh |
| C | 官方 SDK 键盘手控 |
| D | 采集、结束后离线生成地图 |
| E（可选） | 查看 topic 与文件 |

只读检查输入；收不到新消息时先修复环境，不开始正式采集：

```bash
ros2 topic echo /odom/mujoco_odom --once --qos-reliability best_effort
ros2 topic echo /front_lidar --field header --once --qos-reliability best_effort
```

## 4. 手动控制并采集

### 4.1 终端 C：官方键盘控制

```bash
cd "$TASK2_PROJECT/deps/zsibot_sdk/demo/zsl-1/cpp/build"
./highlevel_demo
```

使用现场已编译、已经跑通的程序，不为了本次建图重编译或修改官方源码。以下按键由当前官方 C++ 源码核对，按键焦点必须在这个终端，**不需要 Enter**：

| 按键 | 动作 |
| --- | --- |
| `2` | 请求起立；等待至少 4 秒，再发运动命令 |
| 小写 `w / s` | 前进 / 后退 |
| 小写 `a / d` | 左移 / 右移 |
| 小写 `q / e` | 左转 / 右转 |
| 小写 `c` | 请求零速度停止 |

**这个 demo 的平移指令固定为 1 m/s、转向固定为 1 rad/s；松开按键不会自动发送停止。** 行走短段后按 `c`，转到所需朝向后按 `c`，观察确实停稳。退出前先 `c`，再 Ctrl+C；不要把 Ctrl+C 本身当成停车命令。

`navigation/config/local.yaml` 中的速度只对导航扩展有效，不能降低这个 demo 的速度。若现场已有可调速度的官方手控工具，可优先沿用其已验证的低速操作；当前 C++ demo 没有速度配置项。不要一边开 demo 一边运行自动导航。

### 4.2 终端 D：开始一轮新采集

先给这轮场地一个名字，用于后续导航地图和配置；本次以同类平坦场地为例：

```bash
cd "$TASK2_PROJECT"
MAP_ID="field_a_$(date +%Y%m%d_%H%M%S)"
printf '本轮场地版本：%s\n' "$MAP_ID"

bash mapping/run_mapping_rviz.sh --mode capture \
  --topic /front_lidar \
  --odom-topic /odom/mujoco_odom \
  --global-frame world \
  --sensor-z 0.30 \
  --ground-z 0.0 \
  --ground-band 0.12 \
  --voxel 0.03 \
  --min-observations 3
```

**不要在这一步加 `--pcd-file`。** 默认会建立新的 `maps/run_日期时间_随机后缀/`，把原始 bag、日志、参数和最终输出放在同一轮目录；不会覆盖以前的图。记下终端打印的本轮路径。

采集窗口的 RViz 使用 `Fixed Frame: lidar`，显示当前扫描；这时没有累计地图、没有最终 PCD 是正常现象。无需在这个窗口点 `2D Pose Estimate` 或 `Publish Point`，手动移动由终端 C 完成。

### 4.3 怎样走，才能改善导航地图

1. 先录一个包含直墙、地面、转角的小范围，结束后按下节生成检查。短测通过再完整采集；每次采集都会产生新目录。
2. 完整采集覆盖主通道、支路入口、狭窄口和预计要到达的区域；不要只沿外圈绕一遍。
3. 对同一走廊从不同位置和方向重复观察，让相邻扫描覆盖重叠。重点是获得有一定宽度的连续地面证据，不能只留下细线状轨迹。
4. 转角前后和遮挡后方分别走近观察。在已确认可行的实际地面上补扫缺口；原地转圈可以看更多方向，但不能代替换位置补地面盲区。
5. 当前旧图优先复扫第 1 节两个入口。新场景则逐个检查通道、入口和目标区域，不沿用旧图坐标。

地图分辨率更高、PCD 点数更多都不自动等于连通更好。墙面重影要查位姿/时序；地面缺测要补采或核实地面高度；真实窄路则不能靠参数变成可通行。

## 5. 结束采集，生成本轮原始地图

先在终端 C 按 `c`，确认停稳，再在终端 D 按 Ctrl+C。等待 `Capture stopped cleanly` 和命令行返回；程序会打印本轮 `finalize_pcd.sh` 命令。

在终端 D 设置准确路径，**替换下面占位目录，不直接使用最近目录猜测**：

```bash
cd "$TASK2_PROJECT"
MAP_RUN="$TASK2_PROJECT/maps/run_替换为刚才打印的本轮目录"
test -f "$MAP_RUN/run.json"
test -f "$MAP_RUN/raw_bag/metadata.yaml"
ros2 bag info "$MAP_RUN/raw_bag"
```

任一步找不到文件就先改路径；检查 bag 中有点云、里程计，记录时长覆盖了本轮行走。

然后执行：

```bash
bash mapping/finalize_pcd.sh "$MAP_RUN" \
  --pose-source auto \
  --deskew auto \
  --ground-z 0.0 \
  --ground-band 0.12 \
  --pgm-resolution 0.05 \
  --pgm-obstacle-height 0.15
```

等待命令结束，不需要 `ros2 bag play`。`auto` 沿用当前官方仿真已验证的位姿来源处理；换数据格式后仍需核对报告。若比赛对位姿来源有不同限制，须按实际规则选择，不能仅因仿真提供了字段就视作允许使用。

检查本轮结果：

```bash
ls -lh "$MAP_RUN/field_map.pcd" "$MAP_RUN/field_map.pgm" \
       "$MAP_RUN/field_map.yaml" "$MAP_RUN/finalize_report.json"
/usr/bin/python3 -m json.tool "$MAP_RUN/finalize_report.json"
/usr/bin/python3 -B mapping/inspect_ground_pcd.py \
  "$MAP_RUN/field_map.pcd" --ground-z 0.0
```

应有 `success: true`；重点看 `processing.fused_scans`、拒绝原因、`ground_points` 和 `occupancy_map`。这些计数证明生成过程，不证明每个通道已经采全；还须检查实际地面覆盖和墙体形状。

原始 PGM 里 `0` 为障碍、`254` 为已观测自由格、`205` 为未知。同格既有地面又有高于地面 15 cm 的点时，障碍优先。不要用图片缩放、修改 YAML 分辨率、涂白灰区的方法补采样。

## 6. 生成供导航使用的派生图

**新场景必须同时指定新 YAML 和新 PCD。** `navigation.prepare_map` 不带源参数时仍指向旧的 2026-09-11 基准；只换 `--output` 会得到旧场地的另一份图。

继续在终端 D：

```bash
cd "$TASK2_PROJECT"
printf '源数据：%s\n场地版本：%s\n' "$MAP_RUN" "$MAP_ID"

/usr/bin/python3 -B -m navigation.prepare_map \
  --source-map "$MAP_RUN/field_map.yaml" \
  --pcd "$MAP_RUN/field_map.pcd" \
  --output "navigation/maps/$MAP_ID" \
  --ground-z 0.0 \
  --band 0.06 \
  --max-edge 0.30
```

若换过终端，先手动恢复 `TASK2_PROJECT / MAP_RUN / MAP_ID` 三个变量，再执行。`--source-map`、`--band` 是导航派生程序的真实参数名；不要写成 `--source-yaml` 或 `--ground-band`。

输出包含：

| 文件 | 用途 |
| --- | --- |
| `field_map.yaml / field_map.pgm` | 导航实际加载的地图对 |
| `robot_clearance.pgm` | 默认机器人足迹下能放下机器人中心的区域 |
| `inferred_support.pgm` | 本次根据局部实测地面推断补全的区域 |
| `report.json` | 原始来源、哈希、补全数量、连通性等 |

`navigation/maps/$MAP_ID` 必须是**尚不存在的新目录**；重做时改版本名，不删除旧图。补全只在符合边长、高度和平面条件的实测地面三角形内部进行，未知区不会整片填白，原图障碍不会被覆盖。

报告中的 `components` 是足迹可通行连通域数量，但并非必须全图只有一个连通域；场外零散区域可能无关。关键是比赛要走的起点、通道和目标是否属于同一个可达区域。报告内自动选择的示例路径不等于全部目标验收，也不是本轮机器狗位置。

## 7. 保存场地配置，预览后再导航

先退出手控程序：终端 C 按 `c`，确认停稳，再 Ctrl+C。在终端 D 创建这张图独立的配置：

```bash
cd "$TASK2_PROJECT"
MAP_CFG="navigation/config/$MAP_ID.yaml"
cp -n navigation/config/local.yaml "$MAP_CFG"
nano "$MAP_CFG"
```

这里以现场已有、已经跑通的 `local.yaml` 为模板，保留 SDK 地址、topic、外参和速度。修改**原有 `map:` 那一行**为本轮真实名字，例如：

```yaml
map: navigation/maps/field_a_20261002_210000/field_map.yaml
frame: world
ground_z: 0.0
robot_radius: 0.35
safety_margin: 0.05
max_speed: 0.40
max_yaw_rate: 0.60
```

示例场地名字必须换成终端打印的 `MAP_ID`；**YAML 不会展开 `$MAP_ID`**。上面只是需要核对的条目，不用它覆盖整份配置，也不要重复添加同名键。Nano 按 Ctrl+O、Enter 保存，Ctrl+X 退出。

先离线检查和预览：

```bash
/usr/bin/python3 -B -m navigation.preflight --offline --config "$MAP_CFG"
bash navigation/run_navigation.sh --config "$MAP_CFG"
```

确认检查输出的 `map` 是新场地图，预览启动显示 `PREVIEW (no SDK motion)`。导航 RViz 的 Fixed Frame 为 `world`，与采集窗口的 `lidar` 不同；二者各自使用对应配置。

在导航 RViz 中：

1. 用 `2D Pose Estimate` 标定**当前实际位置和朝向**，结合墙角和走廊核对雷达/地图对齐，不照抄旧场地位姿。
2. 勾选 `Footprint Clearance (optional)`，结合 `Navigation Map` 对比通道是否在足迹计算后断开。可暂时关闭其中一个图层避免互相遮挡。
3. 用 `Publish Point` 依次点入口两侧、支路尽头和比赛目标附近，确认都能规划。预览不移动，约 15 秒无进展后取消是正常现象；可以继续测试其他目标。
4. 若点是白色但规划失败，看 `No path` 或 `Goal lacks known-free clearance`，不要只看像素颜色判断可达。

预览完成后 Ctrl+C 退出，再运行：

```bash
bash navigation/run_navigation.sh --config "$MAP_CFG" --drive --stand-up
```

沿用此前已验证的时间模式；若现场确实使用仿真时钟，预览和控制均按原流程追加 `--use-sim-time`。重启后重新初始定位，先 1 m、2 m，再测试通道和长路线。

日后换回该场地，只需显式指定其配置：

```bash
bash navigation/run_navigation.sh \
  --config navigation/config/field_a_20261002_210000.yaml \
  --drive --stand-up
```

把例子中的名字改成实际保存的文件。新场地 B 重复采集流程并生成 `field_b_...` 的地图和配置，不复用 A 的源数据、坐标或原点。导航启动时加载地图；修改文件后必须退出并重启，不会自动热切换。

## 8. 参数重做与数据保留

### 8.1 什么时候只重做处理，什么时候必须重采

| 现象 | 优先处理 |
| --- | --- |
| 走廊在原始扫描中就没扫到或被遮挡 | 换观察位置重新采集；参数不能生成缺失回波 |
| 地面带候选长期为 0 | 检查世界地面高度、点云是否收到、坐标/外参是否正确 |
| 墙面明显双层或整体错位 | 检查位姿、时序和外参；不通过填地面掩盖 |
| 原始图地面已有小范围支持，但自由格零碎 | 从同轮 PCD/YAML 生成派生图，核对 `--band` 和 `--max-edge`，默认先不变 |
| 原始图通道可见，足迹图断开 | 对比真实通道宽度、灰色缺测区和误标障碍，优先补采入口与两侧 |
| 更换机器人或雷达安装 | 重新核实足迹、外参和控制，不仅替换 map 路径 |

保留每轮 `raw_bag/`、`run.json`、`field_map.pcd/.pgm/.yaml`、生成报告，以及对应 `navigation/maps/场地版本/`、`navigation/config/场地版本.yaml`。它们不都是导航运行必需品，但能在比赛前重做参数和追溯版本。

`finalize_pcd.sh` 默认会重写该轮地图和 `finalize_report.json`；`finalize.log` 追加。因此不要直接在已验收旧轮次上反复试参数。

### 8.2 可选：同一原始 bag 另存一版处理结果

仅在旧轮次是 `capture`、已经正常结束且 `raw_bag/metadata.yaml` 存在时使用。下面用新处理目录和指向旧 bag 的符号链接，保留旧地图和旧报告：

```bash
OLD_RUN="$MAP_RUN"
export TASK2_PROJECT OLD_RUN
bash <<'BASH'
set -euo pipefail
cd "$TASK2_PROJECT"
test -f "$OLD_RUN/raw_bag/metadata.yaml"
test -f "$OLD_RUN/run.json"
REPROCESS=$(mktemp -d "$TASK2_PROJECT/maps/reprocess_$(date +%Y%m%d_%H%M%S)_XXXXXX")
cp -a "$OLD_RUN/run.json" "$REPROCESS/run.json"
ln -s "$(realpath "$OLD_RUN/raw_bag")" "$REPROCESS/raw_bag"
bash mapping/finalize_pcd.sh "$REPROCESS" \
  --pcd-file "$REPROCESS/field_map.pcd" \
  --pgm-file "$REPROCESS/field_map.pgm" \
  --map-yaml "$REPROCESS/field_map.yaml" \
  --ground-z 0.0 --ground-band 0.12 --pgm-resolution 0.05
printf '新处理结果：%s\n' "$REPROCESS"
BASH
```

这段先用相同参数演示保留原件的流程；对照具体问题后一次只调整一个有依据的参数。需要继续生成导航图时，将 `MAP_RUN` 改成最后打印的新处理目录，给 `MAP_ID` 新版本名，再执行第 6、7 节。

符号链接依赖旧 bag，备份或迁移时必须同时保留真实 bag；该新目录并非独立完整备份。只有 PCD、没有原始 bag 的历史轮次，不能用这个命令还原原始扫描。

## 9. 换场地时的最短检查单

1. 官方新场景与机器人已启动，上一场录制已结束。
2. 新 capture 轮次正常结束，点云和里程计均有记录。
3. 新 PCD/YAML/PGM 与生成报告成功，地面和通道覆盖已检查。
4. `prepare_map` 明确指定同轮新 YAML、新 PCD、新输出目录。
5. 独立场地配置的 `map:` 指向新图，其他同平台已验证参数保留。
6. 导航预览重新定位，检查实际比赛通道与目标可达。
7. 关闭手控，再运行对应配置的 `--drive`；先短程再长程。

当前程序和本指南均不会自动替你选择官方新场景或采集路线；这些由参赛者手动完成。无需修改官方源码、删除 build、重新安装整套仿真，也无需清空旧地图。
