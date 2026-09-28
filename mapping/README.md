# RViz 场地三维建图

更新日期：2026-09-24。本页记录 `bash mapping/run_mapping_rviz.sh` 相关扩展的当前实现、Ubuntu 测试进度与验收方法。官方 `run_sim.sh`、`config/`、`src/`、`rviz/matrix.rviz` 保留不改；本轮仅重构新增的 `mapping/` 功能。`rviz/matrix_mapping.rviz` 保留为历史扩展配置，不再是默认启动配置。平台背景见 [README_GPT.md](../README_GPT.md)，当前参数以本页和程序 `--help` 为准。

## 任务一完成：建图验收通过

**任务一（场地三维建图）已解决。最新目标 Ubuntu 实测建图精准高效，覆盖度与几何质量完全符合比赛要求。** 验收轮次为 [`run_20260911_223510_ujmvehqs`](../maps/run_20260911_223510_ujmvehqs/)，采用 `--mode capture` 采集后离线最终建图：`finalize_report.json` 中 `success=true`，5,512 帧扫描全部融合、0 帧拒绝，最终输出 987,319 个点、35,863 个地面体素，并生成 597×482 的 5 cm 栅格 [`field_map.pgm`](../maps/run_20260911_223510_ujmvehqs/field_map.pgm) 与 [`field_map.yaml`](../maps/run_20260911_223510_ujmvehqs/field_map.yaml)；离线融合耗时 203.3 秒，不再因在线等待上限丢弃已记录扫描。

关键产出见 [`field_map.pcd`](../maps/run_20260911_223510_ujmvehqs/field_map.pcd)、[`finalize_report.json`](../maps/run_20260911_223510_ujmvehqs/finalize_report.json) 与 [`run.json`](../maps/run_20260911_223510_ujmvehqs/run.json)。本轮使用官方 `rviz/matrix.rviz` 副本（SHA256 已记录）和 ROS2 Humble / Zenoh / `ROS_DOMAIN_ID=89` 环境，采集与最终建图分开执行，符合比赛提交流程。

## 当前进度：采集与最终建图分开

**本流程已通过最新目标 Ubuntu 复测并验收（见上节“任务一完成”）。默认边行动边记录原始扫描与里程计，官方 RViz 只显示当前扫描；结束采集后通过命令离线筛选、融合并生成最终 PCD。**

最新反馈为 [`run_20260909_223222_pcvf9aea`](../maps/run_20260909_223222_pcvf9aea/)。日志仍说明累计地图工作会阻塞后续扫描：

| 最新 Ubuntu 实测指标 | 结果 |
| --- | ---: |
| 首末周期统计覆盖窗口 | 876.82 秒（约 14 分 37 秒，非完整录制时长） |
| 最后周期累计接收 / 融合帧 | 8,799 / 3,708 |
| 周期窗口内接收 / 融合频率 | 9.98 / 4.17 Hz（按首末计数差计算） |
| 运动拒帧 `motion_uncertain` | 3,622 |
| 过期 / 溢出 | 1,297 / 143 |
| `pose_gap` / `pose_need_past` | 22 / 1 |
| 含融合路径的 32 个周期耗时采样：中位数 / P90 | 93.4 / 154.74 ms |
| 仅运动拒绝的 137 个周期耗时采样：中位数 | 7.4 ms |
| 最后两分钟含融合路径 7 个周期采样：中位数 | 134.1 ms |
| 最后两分钟累计图发布 `publish_ms` 采样范围 | 519.1～693.1 ms |
| 最后两分钟显示更新：中位数 / 最大值 | 27.87 / 50.58 ms（7 个含融合采样） |
| 最后两分钟 `save_ms` 采样范围 | 141.2～213.6 ms |
| 最终 PCD 点数 | 849,366 |

精确统计、日志行号、PCD 头和五个源文件 SHA256 见 [latest_run_analysis.json](diagnostics/live_scan_20260910/latest_run_analysis.json)。本轮原始 bag 的逐帧 CDR 核验、嵌入位姿公式和 ODOM 对照见 [embedded_pose_analysis.json](diagnostics/live_scan_20260910/embedded_pose_analysis.json)。这些耗时来自周期状态采样，不是逐帧完整分布；日志字段由并发回调更新，相邻状态与阶段行不是同一帧的原子快照。`publish_ms`、`save_ms` 是最近一次完成操作的耗时，可能在多条状态中重复出现。最后周期状态早于无时间戳的最终保存，因此 876.82 秒不能作为完整录制时长。`run.json` 证明这轮实际加载的是 `matrix_mapping.rviz` 副本；RViz 日志没有点云错误。证据表明批量融合优化已经有效，但整图发布与保存仍会随地图扩大产生明显停顿；运动筛选也仍会拒绝部分扫描，不能把所有未融合帧都归因于性能。

当前默认入口使用 `--mode capture`：不启动在线融合节点，不构造 `/field_map`、地面或分类图层，也不定期序列化整图。原始 `/front_lidar` 与 `/odom/mujoco_odom` 保存在本轮 rosbag2 中，供结束采集后重复离线处理。原在线融合保留为 `--mode online` 对照，仍只通过官方 RViz 显示当前扫描，默认在保存服务或正常退出时写 PCD。

## 历史 Ubuntu 测试

以下记录用于追溯精度、重影、地面与性能问题；其中的参数与显示方式属于当时版本，不是本轮默认操作。

历史验收基准 [`run_20260908_152708_phj0ivtz`](../maps/run_20260908_152708_phj0ivtz/) 使用 ROS2 Humble、`rmw_zenoh_cpp`、`ROS_DOMAIN_ID=89`、`SDK_CLIENT_IP=127.0.0.1`。最终 PCD 为 828,489 个有限 XYZ 点，用户确认精度、转弯重影、场地建模与 PCD 输出正常。其 `|z| <= 0.05 m` 点仅 3,772 个，占 0.455%；844 个低点具有局部水平面几何支持，地面覆盖不足。统计及图示保留在 [ground_baseline](diagnostics/ground_baseline/)。

随后两次实测使用旧地面增强版本的默认值：主图 3 cm / 3 帧确认，地面 10 cm / 同体素 3 帧确认，运动窗口 [-0.35, +0.10] s，整帧 5°/s 和 0.5 m/s 门控，5 秒自动保存。

| 实测指标 | [第一次：221046](../maps/run_20260908_221046_pmfzmchk/) | [第二次：224019](../maps/run_20260908_224019_rt8aznzy/) |
| --- | ---: | ---: |
| 有效雷达采集时间 | 约 26 分 29 秒 | 约 6 分 47 秒 |
| 最后周期统计：输入 / 融合帧 | 15,763 / 2,813 | 4,076 / 856 |
| 融合帧占输入比例 | 17.85% | 21.00% |
| 运动拒帧 `turning_or_settling` | 4,364 | 2,070 |
| `queue_expired` / `queue_overflow` | 2,073 / 4,342 | 513 / 294 |
| `pose_gap` / `pose_timeout_gap` | 1,245 / 916 | 209 / 130 |
| 处理时间采样中位数 / P90 | 293.9 / 401.5 ms | 216.9 / 298.4 ms |
| 队列等待采样中位数 / P90 | 582.0 / 761.0 ms | 303.5 / 739.87 ms |
| 最终 PCD 有限点数 | 721,493 | 379,969 |
| 地面通道已确认点 | 4,787 | 1,326 |
| PCD 地面高度 ±5 cm 内点数 | 9,531 | 6,284 |
| 局部水平面证据占据的 20 cm XY 网格 | 867 | 317 |
| RViz 四通道图像错误次数 | 15,922 | 4,076 |

完整精确值、源文件 SHA256、PCD 高度统计和两图比较见 [run_comparison.json](diagnostics/efficiency_20260908/run_comparison.json)。第一次采集结束按最后带时间戳的保存近似；第二次按 `Lidar stopped` 的时间减停流时长估计，排除了末尾约 157.5 秒没有雷达输入的时间。处理与等待耗时来自约 5 秒一次的状态采样，不能当作逐帧耗时分布。最终保存可能比最后周期状态多几个已确认点。

上一轮复测 `run_20260908_234515_6yb_w55x` 的原始数据位于用户提供的 `F:/matrix_robotac_first/maps/`。这次确实运行了 adaptive 和跨扫描地面版本，**地面数据增加，但效率仍不合格**。详见 [234515 日志分析](diagnostics/efficiency_20260909/new_run_analysis.json)，包含四个源文件的 SHA256。

| 234515 实测指标 | 结果 |
| --- | ---: |
| 最后周期统计：接收 / 融合帧 | 7,757 / 1,754（22.61%） |
| 过期 / 溢出 / 主动跳过 | 1,992 / 637 / 858 |
| 运动整帧拒绝 / 位姿相关拒绝 | 2,273 / 229 |
| 完整处理路径 80 个抽样中位耗时 | 420.25 ms |
| 融合 / 显示更新阶段抽样中位 | 245.08 / 58.84 ms |
| 最后近静止 149.85 秒接收 / 融合 | 1,499 / 273（10.00 / 1.82 Hz） |
| 同一静止段运动拒绝增量 | 0 |
| 同一静止段过期 / 溢出 / 主动跳过 | 711 / 310 / 111 |
| 最终 PCD 有限点 / 地面高度 ±5 cm 点 | 628,469 / 20,590 |
| 最后周期地面确认体素 | 18,932 |
| RViz 四通道图像错误 | 7,813 次 |

另有 71 个只执行运动拒绝的抽样，中位仅 5.15 ms，不能与完整建图耗时混合。最后近静止段仍持续丢帧，证明问题独立于运动门控；旧 `display_update` 包含预览消息构造和发布，不能全部归因于显示字典。以上是周期状态抽样，非完整逐帧分布，最后统计也早于最终保存。

234515 这轮 `run.json` 没有配置哈希，无法重建 Ubuntu 启动时的实际配置或用户界面操作。随后版本从新增建图配置移除 Image 插件，并在启动时保存配置副本及文件哈希；最新 223222 运行已具备该记录。本轮直接使用官方 `matrix.rviz` 副本，保留其原有显示设置，不修改官方文件。

这些记录确认了以下问题：

- 雷达约 10 Hz，而处理耗时已经明显超过 100 ms。地图增大后，整图序列化、发布、保存和 Python 逐点工作加重，过期、溢出及位姿缺口累积；继续原地等候不能解决计算积压。
- 第二次约 50.8% 的输入帧被旧运动门控拒绝。加速经过区域后，那里缺少足够已接受扫描，地图不能靠等待补回未观测区域。
- 两次有效扫描中仅约 1.97% / 1.94% 的点进入地面高度带；旧平面筛选仅保留高度带点的 11.54% / 15.67%，之后还有同体素重复命中要求。密集雷达线可能占满最近邻名额，邻近扫描线又不能提供同帧支持，稀疏地面更难确认。
- 快图中只有 0.527% 的点距慢图最近点超过 10 cm；反向却有 17.65% 的慢图点在快图中找不到 10 cm 内邻点。主要证据指向漏覆盖和密度不足，两个实测图不是独立真值，不能据此声称绝对精度合格或完全不存在重影。

## 本轮处理逻辑

### 运行期间只采集并显示当前扫描

`run_mapping_rviz.sh` 默认启动原始数据记录和 RViz。RViz 加载官方 `rviz/matrix.rviz` 的逐字节副本，保留 `Fixed Frame: lidar`，不再以 `world` 覆盖它。官方 PointCloud2 订阅 `/front_lidar`，采用 Best Effort / Volatile / depth 5，`Decay Time: 0`，只显示最新局部扫描。官方配置中的 Image、Grid、Axes 也保持原样；它们不是累计地图图层。

记录器保存原始点云和里程计，不执行邻域搜索、地面判别、体素融合、累计图发布或周期 PCD 保存。采集开销仍取决于输入带宽、ROS 传输和磁盘写入，不能承诺零丢帧，但不会再叠加随地图点数增长的在线建图开销。无需在每个转角停稳后才扫描：沿场地连续移动、平缓转向并让相邻观察有重叠即可。RViz 原始扫描是否出现，与最终某个点是否通过几何筛选是两件事。

### 结束后离线筛选并生成 PCD

停止机器狗运动后，先正常结束本轮采集，再运行 `finalize_pcd.sh`。离线程序分两遍读取 rosbag2：先建立完整位姿时间轴，再按扫描时间处理原始点云。筛选不再与 10 Hz 实时输入竞争，也不因计算超过在线 0.5 秒墙钟等待上限而丢弃已记录扫描。计算较慢只会延长离线完成时间；原始记录中的缺帧、位姿缺口、无效时间戳和不可靠测量仍需拒绝并记录。

离线过程复用已验证的位姿插值、几何与运动筛选、主图体素确认和跨扫描地面支持；最后只输出通过规则的实际观测点。主图采用 3 cm 体素和 3 个不同扫描时间戳确认，地面采用 10 cm 融合。PCD 中没有为了显示效果生成的整片地板，也没有未观测空隙的填充。

对可验证的绝对逐点时间字段，可按各点对应的插值位姿去畸变；字段含义或时间覆盖无法确认时，继续采用 `adaptive` 保守运动筛选。当前 UE 仿真格式还在每帧前两条 `intensity=111` 记录中嵌入雷达位姿；离线程序在 `--pose-source auto` 下严格识别该格式，剥除两条元数据后直接使用雷达位姿，避免把 header 时刻里程计误差放大成整帧拒绝。若格式不匹配则回退原 odom 路径，并在报告中记录 `embedded_pose_invalid`。逐点 timestamp 全零时仍报告为不可用，不编造逐点时间。重复 XYZ 在邻域筛选前只保留一个，并记录 `duplicate_points`，避免仿真占位点占用输入和让图面产生假密度。后者用扫描前后窗口内的平移和转动幅度估计测量不确定性，超过 `--max-motion-error 0.05` 米的点不参与融合。`--scan-duration 0.10` 秒仍是需要结合输入元数据核实的扫描跨度假设，不能为了增加点数随意减小。

边移动边采集并不意味着任意速度下的点都可靠。过快平移或急转仍可能让最终保留点减少；离线处理能避免计算积压导致的舍帧，不能恢复传感器从未记录的回波。建议短测先以约 0.3～0.4 m/s 直行、平缓转弯，在最终 PCD 中核对墙面与地面覆盖后再采集完整场地。

### 保留实测地面和已验证融合

主图继续使用 0.2 m 半径内至少 2 个其他不同坐标邻点，3 cm 体素、3 个不同扫描时间戳确认、未确认候选 2 秒未观测则过期。同帧重复点不能增加确认次数。

地面候选为 `world z=0 ±0.12 m`；支持来自最近 1 秒、最多 6 个已接受扫描，并先按空间抽样。默认要求 0.6 m 内至少 6 个不同坐标支持点，PCA 法向倾斜不超过 15°，平面残差与点到面距离不超过 3 cm，且具有二维展布。墙体上下文保留，避免把孤立墙脚判成水平地面。每个候选还需在 0.3 m 内获得至少 2 个不同扫描的同平面支持，不要求不同帧恰好命中同一个体素。

通过检查的真实回波进入地面融合，最后与主图按细体素去重后保存到 XYZ PCD。运行期间没有黄色地面图层；地面结果应以最终 PCD、`finalize_report.json` 中的 `ground` 统计和 `inspect_ground_pcd.py` 的局部平面证据检查。没有原始地面回波的位置不会生成地面。换场景时核实全局 `--ground-z`，不要用雷达安装偏移 `--sensor-z` 强行移动地面。

PointCloud2 二进制 NumPy 解码、连续数组批量体素更新和定时垃圾回收沿用上一轮实现。联合 Bash 入口默认设置 `OPENBLAS_NUM_THREADS=1`、`OMP_NUM_THREADS=1`、`MKL_NUM_THREADS=1`，减少小矩阵多线程争用；其他终端环境不变。可用 `MAPPING_BLAS_THREADS` 对照线程数。

### 原在线模式仅作对照

```bash
bash mapping/run_mapping_rviz.sh --mode online
```

在线模式保留 8 帧有界队列、0.5 秒等待上限及 `ordered` 顺序处理，不外推位姿或无上限积压。它默认传入 `--live-display none --save-policy final`：RViz 仍直接看官方 `/front_lidar`；融合节点不更新或发布累计显示，不周期保存，显式保存服务与正常退出才写 PCD。`--queue-policy latest`、`--motion-policy strict` 和 `--gc-policy default` 仅用于有针对性的历史对照。

`matrix_mapping.rviz` 中旧的累计图、地面、分类与当前扫描预览设置只保留供追溯；默认采集流程和在线对照都不依赖它。直接运行 `run_mapping.sh` 的默认行为不同，不能用它替代本页的新联合入口流程。

## 参数与适用阶段

联合入口默认 `--mode capture`。下表中的几何、运动与地面参数用于最终建图；在线队列和显示参数只适用于 `--mode online`。默认采集阶段不执行这些筛选。

| 参数 | 当前默认值 | 说明 |
| --- | --- | --- |
| `--mode` | `capture` | 联合入口运行模式；online 为在线融合对照 |
| `--deskew` / `--pose-source` | `auto` / `auto` | 验证逐点时间后去畸变；优先使用已验证的嵌入雷达位姿，`odom` 可强制旧路径 |
| `--topic` / `--odom-topic` | `/front_lidar` / `/odom/mujoco_odom` | 输入点云和位姿 |
| `--global-frame` / `--sensor-z` | `world` / `0.30` m | 实际位姿父坐标系和雷达相对机体安装高度 |
| `--voxel` / `--min-observations` / `--pending-ttl` | `0.03` m / `3` / `2.0` s | 主图持久融合 |
| `--motion-policy` | `adaptive` | 按估计点位移筛选；`strict` 为历史速度门控 |
| `--scan-duration` / `--max-motion-error` | `0.10` s / `0.05` m | 假设扫描跨度及估计运动位移上限 |
| `--motion-window` / `--motion-lookahead` | `0.10` / `0.10` s | 扫描前后检查长度，adaptive 每侧至少为 scan-duration |
| `--max-pose-gap` | `0.10` s | 完整窗口内相邻位姿间隔；旧名 `--max-pose-age` 为别名 |
| `--max-angular-speed` / `--max-linear-speed` | `5.0` °/s / `0.5` m/s | 仅 strict 策略使用的速度上限 |
| `--max-cloud-wait` / `--cloud-queue-size` | `0.5` s / `8` | 有界输入队列 |
| `--queue-policy` | `ordered` | 按顺序处理未过期帧；latest 为旧积压舍帧策略 |
| `--gc-policy` | `scheduled` | 定时循环回收；default 为 Python 自动回收对照 |
| `--min-range` / `--max-range` | `0.1` / `60` m | 雷达局部量程；max-range 为 0 时关闭上限 |
| `--outlier-radius` / `--min-neighbors` | `0.2` m / `2` | 主图同帧邻点筛选；radius 为 0 时关闭 |
| `--ground-z` / `--ground-band` | `0.0` / `0.12` m | 全局预期地面高度及上下容差 |
| `--ground-radius` / `--ground-voxel` | `0.6` / `0.10` m | 地面平面支持半径及确认后融合体素 |
| `--ground-min-observations` | `2` | 地面局部不同扫描支持数，1～6 |
| `--disable-ground` | 默认关闭此开关 | 显式传入时关闭补充地面链路 |
| `--live-display` / `--save-policy` | 联合在线入口：`none` / `final` | 不输出融合显示；只在服务或退出时保存 |
| `--publish-period` / `--display-voxel` | `1.0` s / `0.06` m | 仅旧在线累计显示模式使用 |
| `--preview-points` | `12000` | 仅融合节点的旧扫描预览使用；官方原始扫描不受它影响 |
| `--autosave` | `15.0` s | 仅显式 `--save-policy periodic` 时使用；新流程不周期写 PCD |
| `--cell-size` / `--max-step` | `0.20` / `0.22` m | 原有高度差分类参数 |

3 cm 和 10 cm 是融合采样尺度，不是绝对精度承诺。`--max-pose-gap` 不应通过设大来掩盖位姿缺口；`--scan-duration` 应按实际扫描元数据核实，不能为了提高帧数随意减小。

## 同步到目标 Ubuntu

本轮更新包为 [mapping_live_scan_final_pcd_20260910.zip](updates/mapping_live_scan_final_pcd_20260910.zip)，解压到现有项目根目录，保留官方文件；具体同步与哈希校验见 [本轮同步说明](updates/README_live_scan_final_pcd_20260910.md)。不要误用历史 `mapping_efficiency_20260909.zip`；它仍是在线累计地图版本。

保持目录结构同步以下联合入口扩展文件，尤其不要漏掉采集和离线生成入口。备份已经跑通的扩展版本以便对照；无需覆盖官方文件或重新构建整个仿真。

```text
mapping/cloud_io.py
mapping/deskew.py
mapping/embedded_pose.py
mapping/occupancy_map.py
mapping/runtime_gc.py
mapping/field_mapper_node.py
mapping/point_filters.py
mapping/ground_surface.py
mapping/pose_buffer.py
mapping/voxel_fusion.py
mapping/run_mapping_rviz.py
mapping/run_mapping_rviz.sh
mapping/capture_mapping.py
mapping/finalize_mapping.py
mapping/finalize_pcd.sh
mapping/run_mapping.sh
mapping/clean_pcd.py
mapping/inspect_ground_pcd.py
mapping/README.md
```

`mapping/tests/` 可用于目标机回归，`mapping/diagnostics/` 保留分析与基准证据，不参与正常启动。不要同步 `__pycache__/` 或 `.pyc`。数值依赖仍是 NumPy 和 SciPy；默认采集与离线流程另需目标 ROS 环境的 `ros2 bag`、`rosbag2_py` 和 SQLite3 存储支持。仅可选诊断绘图另需 Matplotlib。缺少依赖时使用 ROS Humble 对应的系统 Python：

```bash
sudo apt install python3-numpy python3-scipy ros-humble-rosbag2 \
  ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins
python3 -c "import numpy, scipy, rosbag2_py; print(numpy.__version__, scipy.__version__)"
```

## 运行、结束采集与生成 PCD

各终端沿用已验证环境，若有额外环境脚本则保持原 source 顺序：

```bash
cd ~/robotac/matrix_robotac_first
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
export SDK_CLIENT_IP=127.0.0.1
```

保留原来的仿真、路由器和 SDK 控制终端。先让 Zenoh 路由器就绪；已有正常实例时不重复启动。

| 终端 | 命令 |
| --- | --- |
| 1：仿真 | `./run_sim.sh` |
| 2：路由器 | `ros2 run rmw_zenoh_cpp rmw_zenohd` |
| 3：控制 | 在 `deps/zsibot_sdk/demo/zsl-1/cpp/build` 运行 `./highlevel_demo` |
| 4：采集与 RViz | `bash mapping/run_mapping_rviz.sh` |

联合入口检查依赖后启动采集器和 RViz；不要同时再启动一份 `run_mapping.sh`。单独打开 `.rviz` 文件不会启动记录器。默认每次建立唯一 `maps/run_日期时间_随机后缀/`，包含 `raw_bag/`、运行日志、`run.json` 和官方 `matrix.rviz` 副本。采集时还没有最终 `field_map.pcd` 属于正常现象。

RViz Fixed Frame 为 `lidar`，直接显示原始局部扫描；最终 PCD 默认仍位于里程计的 `world` 坐标系。`--global-frame` 只设置最终坐标系名称，须与实际里程计父坐标系一致，不会把官方 RViz 改成世界坐标累计视图。官方配置按原始内容复制到本轮目录后加载，保存本次 RViz 布局不会修改官方文件。`run.json` 保留实际命令、配置与扩展文件哈希以及本轮建图参数。

连续完成路线后，停止机器狗运动，在终端 4 按 **Ctrl+C**，等待记录器正常关闭、rosbag 元数据完成以及入口退出。此操作不会关闭其他终端的仿真、路由器和 SDK，也不会自动执行最终建图。终端会给出本轮离线命令；在相同 ROS 环境执行，将示例路径替换成本次目录：

```bash
bash mapping/finalize_pcd.sh maps/run_日期时间_随机后缀
```

默认读取本轮 `run.json` 中的建图参数与 `raw_bag/`，输出 `field_map.pcd`、标准二进制 `field_map.pgm`、配套的 `field_map.yaml`、`finalize.log` 和 `finalize_report.json`。PGM 使用 5 cm 栅格，像素值遵循 ROS `map_server`：`0` 是障碍，`254` 是有实测地面证据的自由格，`205` 是未观测区域；不对缺测区域插值填充，同一格同时有地面和高于地面 15 cm 的点时障碍优先。YAML 的 `origin` 是点云 XY 包围盒左下角，图像首行对应最大 Y，可直接交给 ROS 地图加载工具。先完成录制再处理，不能对仍在写入的 bag 运行最终建图。原始 bag 保留，可以重新筛选；命令后追加建图参数可覆盖本轮记录值，例如：

```bash
bash mapping/finalize_pcd.sh maps/run_日期时间_随机后缀 --voxel 0.03
```

需要调整比赛地图分辨率或输出位置时，可显式传入 `--pgm-resolution`、`--pgm-padding`、`--pgm-obstacle-height`、`--pgm-file` 和 `--map-yaml`；默认输出与 `field_map.pcd` 同目录。`finalize_report.json` 的 `occupancy_map` 节点记录图像尺寸、原点以及 free/occupied/unknown 栅格计数。

离线 `--deskew auto` 为默认：只有逐点绝对时间字段可验证时才启用去畸变。`--deskew off` 可关闭作对照；`--deskew required` 拒绝并统计没有可用逐点时间的扫描，不退回单姿态筛选；整轮没有确认点时返回失败并保留原有 PCD。运动位移及 strict 速度门限只作用于单姿态路径；逐点补偿路径按各点时间检查位姿覆盖。以报告中的 `deskewed_scans`、`single_pose_scans`、`timing_reasons`、拒绝原因和最终保存成功信息为准。报告也记录生成时算法文件的 SHA256，便于区分采集后重新处理的版本。

仅 `--mode online` 启动融合服务。在线对照如需主动保存，在机器狗停止后执行：

```bash
ros2 service call /field_mapper/save std_srvs/srv/Trigger '{}'
```

服务应返回 `success: true`，路径和点数属于本轮；继续扫描后正常退出仍会保存新增变化。默认 capture 模式没有这个服务，应使用 `finalize_pcd.sh`。在线 `/field_mapper/clear` 会清空地图并写入空 PCD，不是刷新显示按钮。

## 诊断与验收

默认 capture 模式首先核对记录器已订阅 `/front_lidar` 和 `/odom/mujoco_odom`，RViz 能及时显示当前扫描，本轮 `raw_bag/` 正在写入。此时没有 `Map started` 或累计确认点数是正常的；它们属于最终建图或在线融合日志。正常结束采集后，可只读查看录制时长、两个 Topic 的消息数与类型：

```bash
ros2 bag info maps/run_日期时间_随机后缀/raw_bag
```

官方扫描显示采用 Best Effort，显示刷新次数不等于 bag 记录帧数，也不等于最终通过筛选的扫描数。地图覆盖和地面是否存在应在离线生成完成后核对。

| 日志字段或状态 | 判断方法 |
| --- | --- |
| 录制时长 / Topic 消息数 | 核对是否覆盖整条路线，雷达记录频率是否接近输入频率，里程计是否连续 |
| `finalize_report.json` / `finalize.log` | 核对实际读取、融合、拒绝计数，逐点去畸变是否启用，以及最终保存路径和点数 |
| `motion_rejected_points` / `motion_uncertain` | 保守运动筛选丢弃估计位移过大的点 / 整帧；结合实际去畸变模式判断 |
| `pose_gap` / 位姿相关拒绝 | 原始记录的位姿窗口不完整；离线等待不能补出未记录位姿 |
| `invalid_cloud` / `invalid_poses` | 点云字段、负载或位姿无效；保留首次错误和元数据 |
| `nonmonotonic_stamp` / `pose_invalid_stamp` | 时间重复、倒退或无效；仿真时间重置后应重新采集 |
| `processing_ms` / `Pipeline: stages_ms` | 离线处理用时影响完成时间；在线模式还会影响实时输入积压 |
| `queue_expired` / `queue_overflow` | 在线模式的墙钟积压指标；默认离线流程不应因计算速度丢弃已记录扫描 |
| `publish_ms` / `save_ms` | 默认流程无运行期累计发布和周期 PCD 保存；最终保存仍需实际磁盘时间 |

离线报告 `ground.stage_point_totals` 中 `range_valid` 为量程及对齐筛选后的点数，`band_input` 为全局地面高度带候选，`plane_supported` 为获得几何平面支持的点数，`temporal_confirmed` 为进一步获得不同扫描局部支持的点数；`ground_points` 为最终已融合地面体素数。`ground.last_scan` 保存最后一帧的阶段数据。在线对照中的同类数据见 `Ground stages` 日志。阶段累计点数包含不同帧的重复观测，不能等同 PCD 唯一点数。

`band_input` 长期为 0 时核实地面回波和全局高度；有候选但没有 `plane_supported` 时核实局部二维支撑；平面点有而 `temporal_confirmed` 不增时，检查同一局部面是否获得不同扫描支持。

目标 Ubuntu 复测建议：

1. 先连续行走一段包含地面、直墙和转角的短路线，保持平缓转向。确认当前扫描显示及时，记录器持续写入；不再等待累计地图在 RViz 中补齐。
2. 停止运动并正常结束录制，用 `ros2 bag info` 核对消息数与时长，再执行 `finalize_pcd.sh`。比较录制帧数、最终融合帧数、拒绝原因和离线用时，区分采集丢失、几何拒绝与最终计算耗时。
3. 检查最终 PCD 的墙面是否重影、结构是否完整，地面是否有真实水平面支持；RViz 的局部扫描不能替代这一项。相同原始 bag 可用同一路线数据复测参数，避免路线变化混入算法比较。
4. 短测通过后再采集完整场地，保留本轮 bag、PCD、日志、报告、`run.json` 与路线用时。结束采集后无需等在线队列消化数分钟；最终离线处理的用时应单独记录。

只读检查最终 PCD，将路径替换为实际本轮目录：

```bash
python3 -B mapping/inspect_ground_pcd.py maps/run_日期时间_随机后缀/field_map.pcd
python3 -B mapping/clean_pcd.py maps/run_日期时间_随机后缀/field_map.pcd --inspect
```

地面诊断默认高度带是 `z=0 ±0.05 m`，比融合筛选的 ±0.12 m 窄；地面高度不同时传入相应 `--ground-z`。低点可能是墙脚，局部水平面证据和 XY 占据格数也不是测绘真值面积。诊断不传 `--output-dir` 时仅打印结果。

历史累计 PCD 只有最终点集，`mapper.log` / `rviz.log` / `run.json` 也不包含完整逐帧输入，因此旧运行目录不能补录成新流程所需的 raw bag。新流程保留原始扫描和轨迹，允许离线重做筛选；仍未实现 SLAM 回环、外参重新标定或固定时间偏移标定。

## 本地验证与历史边界

本轮本地回归共 259 项：258 项通过，1 项 POSIX 实际进程信号测试因当前 Windows 环境跳过；原 strict 合成转弯回归通过。覆盖采集参数与 QoS、停止及失败处理、离线位姿时间轴、逐点时间字段校验、UE 嵌入雷达位姿解码、重复点去除、无 ROS 节点复用融合、关闭累计发布、PCD 原子保存、地面保留以及 PGM/YAML 栅格输出。连续 1.2 m/s、60°/s 的六帧合成扫描在默认三帧确认和地面规则下恢复已知墙面与稀疏地面，并保留故意缺测的地面孔；这是已知时间戳的合成验证，不是目标机精度结论。验证摘要见 [local_validation.json](diagnostics/live_scan_20260910/local_validation.json)。

目标 Ubuntu 尚未运行本轮重构，真实 ROS CDR 读取、录制吞吐和 POSIX 信号关闭仍需复测，不能将上一轮用户验收直接延伸到新采集与离线链路。

上一轮效率版本 **200 项测试和原 strict 转弯回归通过**；这是历史验证结果，不代表本轮采集/离线重构已经通过目标 Ubuntu 验收。[完整处理基准](diagnostics/efficiency_20260909/pipeline_efficiency_benchmark.json) 比较当时上一包与 20260909 效率实现：固定 50 万点地图、每帧 18,000 点、180 帧、默认 3 cm / 3 帧确认、地面开启。旧/新单帧处理中位数为 117.08 / 78.03 ms，P95 为 192.27 / 82.46 ms。计入周期发布、状态、保存、维护后的每帧平均总开销为 137.68 / 83.88 ms；最终 PCD、地面、代表点、分类范围和代表点归属均完全一致。

这是 Windows 下两个独立进程的静止合成实验，使用真实二进制输入布局和 fake ROS，未测 ROS 消息生成类、传输、RViz、真实输入排队。18 秒模拟采集覆盖 1/15 秒维护，不覆盖 120 秒全代维护；仍存在约 205 ms 的最大单帧处理长尾，不能声称稳定零丢帧。当时建议用静止 3 分钟覆盖完整维护周期；新默认流程按上文连续采集再离线处理验收。该历史基准可按下面命令复现：

```bash
python3 -B mapping/tests/pipeline_efficiency_benchmark.py \
  --output mapping/diagnostics/efficiency_20260909/pipeline_retest.json
```

上一轮 185 项本地测试及原 strict 策略合成转弯回归通过，但未解决新实测效率问题。20260909 版本新增批量显示、循环回收、队列与消息输出回归，覆盖二进制解码、位姿窗口、运动位移筛选、过载队列、主图与地面确认、显示/完整 PCD 区分、保存清空和联合入口退出。运行：

```bash
python3 -B -m unittest discover -s mapping/tests -v
python3 -B mapping/tests/turning_regression.py
```

合成转弯实验使用人为注入的 80 ms 里程计接收延迟、四次 90° 转向和异常扫描，属于已知条件下的回归，不是现场实测误差。本轮采集/离线流程的目标机性能、最终精度与地面覆盖仍须按上面的 Ubuntu 流程验收，本地通过不能代替该结论。

上一轮 [voxel_efficiency_benchmark.json](diagnostics/efficiency_20260908/voxel_efficiency_benchmark.json) 记录 Windows 本地合成基准：每帧 18,000 个点、已有 499,840 个确认地图点、连续更新 20 帧，融合与输出数组准备的合计中位耗时从 169.80 ms 降至 62.80 ms，约快 2.70 倍；逐帧几何及输出完全一致。此基准不包含点云解码、过滤、ROS、RViz 或磁盘 I/O，不能解释为目标 Ubuntu 整条处理链路加速 2.70 倍。基准使用 5 cm 体素、预先确认的地图以隔离融合更新成本，不替代默认 3 cm / 3 帧确认的完整建图验收。

在项目根目录可复现基准，需同时保留 `mapping/diagnostics/efficiency_20260908/baseline_voxel_fusion.py`；以下命令另写复测报告，保留本轮原报告：

```bash
python3 -B mapping/tests/efficiency_benchmark.py \
  --output mapping/diagnostics/efficiency_20260908/voxel_efficiency_retest.json
```

历史异常柱体已修正：旧版本把同一 XY 格的最低点和最高点填成实心柱；当前只显示实际观测表面。更早 `run_20260907_222552_cum_a6nv` 的数千米极值、`run_20260908_011428_w7thn174` 的转弯副本和 `progress1.webm` 属于历史问题，不与 9 月 8 日两次记录混为同一时间序列。`run_20260908_152708_phj0ivtz` 已完成精度/重影用户验收，但其低地面覆盖不能标成通过。

上一轮地面增强曾通过 146 项本地测试，随后本页记录的两次 Ubuntu 实测暴露性能和地面覆盖不足；该历史测试数不作为本轮新实现验收。旧日志中的四通道图像错误与退出阶段 `publisher's context is invalid` 保留在原记录；文件成功保存与退出完全无报错是两个不同结论。所有旧 PCD 和日志保留供追溯。
