# matrix_robotac_first
This is my first project for the competition of robotac that four foots robot dog

## 当前进度

- 任务一建图已通过 Ubuntu 验收，说明见 [mapping/README.md](mapping/README.md)。
- 任务二第一目标：在 RViz 点一个可达位置，机器狗自主到达并可靠停住。
- 独立导航首版及 36 项离线测试已完成；Ubuntu 的 ROS2、SDK、真实仿真运动闭环仍待现场验收。
- 本仓库保存开发扩展、文档、配置快照和必要地图，**不包含完整官方仿真运行时**。

## 入口

| 内容 | 位置 |
| --- | --- |
| 项目说明与迁移历史 | [README_GPT.md](README_GPT.md) |
| 任务二目标、范围与建议 | [TASK2_PLAN.md](TASK2_PLAN.md) |
| 导航启动、参数、验收与增量迁移 | [navigation/README.md](navigation/README.md) |
| Ubuntu 增量包及校验文件 | [navigation/updates](navigation/updates/) |
| 任务一基准地图 | [maps/run_20260911_223510_ujmvehqs](maps/run_20260911_223510_ujmvehqs/) |
| 本次归档文件与排除范围 | [REPOSITORY_CONTENTS.json](REPOSITORY_CONTENTS.json) |

## Ubuntu 使用原则

继续使用已经跑通的官方 Ubuntu 项目，按导航说明只增量迁移 `navigation/`；不要整目录覆盖现场工程。官方 `src/`、SDK 二进制、运行库和构建目录仍需保留原安装。

本仓库不上传 UE 程序、官方运控/物理仿真构建产物、SDK 共享库/安装包、机器人网格资源、原始 rosbag 和运行日志。`src/` 中保存的是已有配置快照，不是可独立重建官方运行时的完整源码。只有克隆本仓库不足以启动完整仿真；不要删除原有官方 build 目录。

默认启动为预览，不发送机器人控制指令；确认坐标对齐后才显式加 `--drive`。具体步骤和保护机制以导航说明为准。

## 本地工作区与仓库副本

原始工作区保持在上一级 `matrix_robotac_first/`，其中保留官方运行时及大文件；本 Git 仓库是其同名子目录中的开发归档副本。后续修改原始工作区后，需要将相应开发文件同步到仓库副本再提交，不能只推送旧副本。
