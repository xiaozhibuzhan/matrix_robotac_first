# matrix_robotac_first

更新：2026-10-20。**任务一、二原有成果保留；任务三新增独立 MuJoCo 仿真平台，0.3.6 已在 Ubuntu 官方原生动力学运行中完成三方向固定靶五发巡回。** 实物接入目前处于联网、视频与硬件适配准备阶段。任务一、二的现场结论与变更见 [CHANGELOG.md](CHANGELOG.md)。

任务三入口与最新说明：

- [独立仿真平台与 Ubuntu 安装](task3_sim/README.md)
- [0.3.6 第五发切换修复与五发验证](task3_sim/OFFICIAL_STANDUP_036.md)
- [实物连接、SSH、视频和后续适配](task3_sim/HARDWARE_CONNECT_20261020.md)
- [最新 Ubuntu 五发离线复核](task3_sim/validation/ubuntu036_five_shot_review.json)

最新五发顺序为 `2 → 5 → 9 → 2 → 5`，均十环、累计 50 分、余弹 0。完整 0.3.6 验收尚未全部完成；回传记录的跨机里程文件哈希差异保留在复核报告中。仓库包含任务三源码、图片来源记录和精选验收证据，不含虚拟环境、原始运行记录或官方编译运行时；Ubuntu 仍需保留原官方工程。

## 当前推进路线

1. 保留任务一地图、普通导航和已经成功的宝箱场景修复；已应用补丁的 Ubuntu 无需重新安装。
2. 任务二正式操作沿用 [普通导航触碰宝箱](navigation/TREASURE_UBUNTU.md)，启动不加 `--treasure`，不使用旧双点与宝箱 CLI 流程。
3. 任务二以 UE 中宝箱真实消失及官方记分牌增分为成功依据，用户已确认完成。
4. 任务三使用 [独立仿真平台](task3_sim/README.md) 验证识别、转向、瞄准与逐发计分，实物按 [连接与适配说明](task3_sim/HARDWARE_CONNECT_20261020.md) 分阶段接入；[原准备计划](TASK3_PLAN.md) 保留作规则与资源参考。

已确认的成功命令：

```bash
bash navigation/run_navigation.sh --config navigation/config/treasure_local.yaml --drive --stand-up
```

`treasure_local.yaml` 只指定配置文件；`Publish Point` 每次选一个普通导航目的地。保留 `--drive`、`--stand-up`，起立后至少等待 4 秒和 `TASK2_READY`。旧专用模式仍有现场问题，已从正式操作停用，实验代码保留追溯。

## 按用途阅读

| 用途 | 唯一维护入口 |
| --- | --- |
| Ubuntu 安装、升级、环境、普通导航、停止和回退 | [navigation/OPERATIONS_UBUNTU.md](navigation/OPERATIONS_UBUNTU.md) |
| 任务二已验收的普通导航触碰流程 | [navigation/TREASURE_UBUNTU.md](navigation/TREASURE_UBUNTU.md) |
| 宝箱从场外移入场内、Ubuntu 应用与还原 | [scene_tools/README.md](scene_tools/README.md) |
| 宝箱场景、正式命令与旧模式误用排查 | [navigation/TREASURE_TROUBLESHOOTING.md](navigation/TREASURE_TROUBLESHOOTING.md) |
| 当前能力、现场证据和后续安排 | [PROJECT_STATUS.md](PROJECT_STATUS.md) |
| 任务二规则、目标与阶段验收 | [TASK2_PLAN.md](TASK2_PLAN.md) |
| 任务三规则待确认项、官方资源与首个开发目标 | [TASK3_PLAN.md](TASK3_PLAN.md) |
| 现场验收与项目更新记录 | [CHANGELOG.md](CHANGELOG.md) |
| 重新采集地图、比赛换场景及切图 | [navigation/MAP_REBUILD_UBUNTU.md](navigation/MAP_REBUILD_UBUNTU.md) |
| 任务一采集与建图 | [mapping/README.md](mapping/README.md) |
| 导航实现、参数与开发接口 | [navigation/README.md](navigation/README.md) |
| 官方宝箱实现审计与本次坐标修改边界 | [OFFICIAL_TREASURE_REVIEW.md](OFFICIAL_TREASURE_REVIEW.md) |
| 旧日志、已解决故障及文档清理记录 | [docs/archive/README.md](docs/archive/README.md) |

## 当前迁移版本与目录

导航历史增量包为 `navigation/updates/task2_treasure_20261015_docs_cleanup.zip`，功能代码与 selection_fix 相同。本次更新不改运动代码，已跑通的 Ubuntu 无需重装。旧导航 ZIP 内的双点/专用模式教程已停用，现行操作以仓库中的本页和任务二手册为准。

| 目录 | 用途 |
| --- | --- |
| 本工作区 `E:\各种比赛\robortac\matrix_robotac_first` | 包含官方运行时的开发原件 |
| 其下同名 `matrix_robotac_first/` | 独立 Git 发布副本；提交推送前从上一级工作区同步 |
| Ubuntu 示例 `~/robotac/matrix_robotac_first` | 已经跑通官方仿真的实际安装目录 |

导航包只增量更新自有 `navigation/`，保留 Ubuntu 现场配置、地图、日志和任务记录。**本次单独授权的例外**：`scene_updates/treasure_inside_v1_20261015.zip` 提供 `scene_tools/` 工具，在校验、备份后仅修改官方 `RobotAC_Level_1.uexp` 五个宝箱的 XY 坐标。触发与计分蓝图、MuJoCo、SDK、`run_sim.sh`、官方配置、任务一程序和原图均未改动。GitHub 精简副本不含完整官方运行时，不能替代 Ubuntu 原安装。
