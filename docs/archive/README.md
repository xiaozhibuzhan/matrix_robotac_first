# 历史记录与文档清理

整理日期：2026-10-16。本目录仅用于追溯，旧记录中的“最新”、待办项和安装命令均指当时版本。当前操作统一从 [项目 README](../../README.md) 进入。

任务二现已通过普通导航、触碰消失与官方计分完成。旧 `--treasure` 双点、本地回合与 contact/confirm 流程已从正式操作停用；源码和历史 Git 版本保留追溯，不把旧教程当作任务三前置条件。最新现场记录见 [CHANGELOG](../../CHANGELOG.md)。

## 保留的记录

| 记录 | 用途 |
| --- | --- |
| [README_GPT.md](README_GPT.md) | 任务一与早期迁移背景；根目录同名文件仅保留兼容入口 |
| [LOG_REVIEW_20261009.md](LOG_REVIEW_20261009.md) | 10 月 9 日新图与导航现场数据 |
| [LOG_REVIEW_20261010_1009.md](LOG_REVIEW_20261010_1009.md) | 旧回合/通信故障分析，含后续结论修正 |
| [LOG_REVIEW_20261010_1025.md](LOG_REVIEW_20261010_1025.md) | 两轮只有中心点接收记录的证据 |
| [codexfix.md](codexfix.md) | 与机器人操作无关的个人工具权限排障记录 |

## 已合并并移除的旧教程

以下文件从原位置移除，避免再沿用旧包名和已被替代的步骤：

| 原文件 | 有效内容现在的位置 |
| --- | --- |
| MIGRATION_UBUNTU.md | navigation/OPERATIONS_UBUNTU.md：统一安装、备份、保留本机数据及回退 |
| navigation/STARTUP_FIX_20261002.md | 同上：单控制器、每次新进程 stand-up、等待 4 秒和 TASK2_READY |
| navigation/VELOCITY_FIX_20261002.md | 同上及 navigation/README.md：SDK 合法速度域、停止与退出约定 |
| navigation/LONG_ROUTE_FIX_20261002.md | 同上及 navigation/README.md：0.40 m/s、连续恢复额度和日志字段 |
| navigation/TREASURE_CLI_FIX_20261010.md | 当前导航包包含 Humble 接口修复；操作统一到宝箱手册 |
| TREASURE_SELECTION_GUIDE_20261010.md | navigation/TREASURE_UBUNTU.md 与 TREASURE_TROUBLESHOOTING.md：两点状态与误点恢复 |
| navigation/TREASURE_SCENE_AND_CLICK_20261015.md | navigation/TREASURE_TROUBLESHOOTING.md：UE 已确认后的场景检查与点击诊断 |

## 已解决故障的结论

| 时间 | 证据与结论 |
| --- | --- |
| 10 月 2 日起立 | 旧启动顺序先发零速 move 导致 SDK 12295；扩展已调整为先 standUp、等待后控制 |
| 10 月 2 日低速 | SDK 拒绝部分低于合法非零速度范围的指令；已在扩展适配，不能把所有 SDK 错误视为未连接 |
| 10 月 2 日长路线 | 旧恢复额度按整段累计，约 59 秒提前耗尽；已改按连续失败计数，取得规定进展后复位，不是靠增加总时限解决 |
| 10 月 9 日新图 | 两轮 9 个目标：6 次到达、1 次总时限、2 次实时障碍停止；不能合并为同一种失败 |
| 10 月 10 日 CLI | Humble Subscription 没有 get_publisher_count；已改节点级 count_publishers 并保留 ACK/重试 |
| 10 月 10 日 SDK | 5 次省略 stand-up 的新连接被拒；每次新控制进程保留该参数，外观站立不等于允许 move |
| 10 月 10 日两点 | 匹配回合的两轮日志只收到中心，没有第二点接收或拒绝记录；不能推断用户只点击一次 |
| 10 月 15 日两点 | 已收到后续点击，但被二维平面检查拒绝，旧逻辑又清空选择；当前增加阶段、XYZ 日志和停稳健康状态下重试 |

旧 MD 原文另有工作区备份：diagnostics/document_cleanup_20261015/documents_before_cleanup.zip。10 月 16 日改写前文档与场景包另备份于 diagnostics/task2_acceptance_20261016/before_document_update.zip。原日志、地图和任务一快照保留；过时双点教程还可在 Git 提交 eaa6a3e 中追溯。
