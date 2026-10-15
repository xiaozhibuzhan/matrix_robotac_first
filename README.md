# matrix_robotac_first

更新：2026-10-15。任务一建图已完成，任务二普通点选导航已在 Ubuntu 实测到达停稳。宝箱流程已实现并通过离线验证，官方场景中的完整触发流程仍待验收。

## 当前推进路线

1. 已确认用户截图来自 **UE 的 UeSim 窗口**，且已完整重启过官方仿真；这两项不再作为待确认事项。
2. 先切换 **FreeCam**，升高视角观察全场，确认真实宝箱的位置。当前 FollowCam 被墙体遮挡，截图不足以证明全场没有宝箱。
3. 若全场仍看不到宝箱，按 [场景与点选排查](navigation/TREASURE_TROUBLESHOOTING.md) 收集目标 Ubuntu 的进程、实际关卡与资源证据。
4. 找到真实宝箱后，再按 [宝箱操作手册](navigation/TREASURE_UBUNTU.md) 验证“两点选择 → 自主接近 → 受控接触 → 观察消失 → 确认记录”。

`new-round`、`select` 和 RViz 点选只管理本地任务，**不会生成官方宝箱**。任务二采用人工选目标的半自动方式；当前不要求自动识别或搜索宝箱。

## 按用途阅读

| 用途 | 唯一维护入口 |
| --- | --- |
| Ubuntu 安装、升级、环境、普通导航、停止和回退 | [navigation/OPERATIONS_UBUNTU.md](navigation/OPERATIONS_UBUNTU.md) |
| 宝箱配置、两点选择、接触、确认和回合恢复 | [navigation/TREASURE_UBUNTU.md](navigation/TREASURE_UBUNTU.md) |
| UE 画面无宝箱、点击后没有任务、误点恢复 | [navigation/TREASURE_TROUBLESHOOTING.md](navigation/TREASURE_TROUBLESHOOTING.md) |
| 当前能力、现场证据和后续安排 | [PROJECT_STATUS.md](PROJECT_STATUS.md) |
| 任务二规则、目标与阶段验收 | [TASK2_PLAN.md](TASK2_PLAN.md) |
| 重新采集地图、比赛换场景及切图 | [navigation/MAP_REBUILD_UBUNTU.md](navigation/MAP_REBUILD_UBUNTU.md) |
| 任务一采集与建图 | [mapping/README.md](mapping/README.md) |
| 导航实现、参数与开发接口 | [navigation/README.md](navigation/README.md) |
| 官方宝箱实现的只读审计 | [OFFICIAL_TREASURE_REVIEW.md](OFFICIAL_TREASURE_REVIEW.md) |
| 旧日志、已解决故障及文档清理记录 | [docs/archive/README.md](docs/archive/README.md) |

## 当前迁移版本与目录

本次文档整理后的导航增量包：`navigation/updates/task2_treasure_20261015_docs_cleanup.zip`，配套同名 `.zip.sha256`。功能代码与 10 月 15 日 selection_fix 版本相同，整合了随包手册并移除了过时教程。已有 selection_fix 的目标机无需为文档整理重装；需要迁移时统一按操作手册执行。

| 目录 | 用途 |
| --- | --- |
| 本工作区 `E:\各种比赛\robortac\matrix_robotac_first` | 包含官方运行时的开发原件 |
| 其下同名 `matrix_robotac_first/` | 独立 Git 发布副本；提交推送前从上一级工作区同步 |
| Ubuntu 示例 `~/robotac/matrix_robotac_first` | 已经跑通官方仿真的实际安装目录 |

只增量更新自有 `navigation/`，保留 Ubuntu 现场配置、地图、日志和任务记录。官方 UE、MuJoCo、SDK、`run_sim.sh`、官方配置、任务一程序和原图保持不变。GitHub 精简副本不含完整官方运行时，不能替代 Ubuntu 原安装。
