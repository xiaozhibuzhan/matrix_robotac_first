# Ubuntu 迁移说明

更新日期：2026-10-02。当前任务二第一目标的完整流程见 [README.md](README.md)，按第 1～2 节准备文件并安装，再按第 3～7 节检查环境、预览、点选导航与停止；安装失败的恢复方法见第 8 节，常见问题见第 9 节。

只携带增量 ZIP 到 Ubuntu 时，解压后的 [navigation/OPERATIONS_UBUNTU.md](navigation/OPERATIONS_UBUNTU.md) 也包含完整流程，可以离线阅读。它与根 README 的第 0～9 节同步维护。

本次迁移范围是自己的 `navigation/` 扩展。先校验下载文件与暂存包，再把旧 navigation 整目录备份到独立目录，最后安装并重新校验；旧参数逐项恢复到 `navigation/config/local.yaml`。官方运行时、配置与任务一已验收成果不整目录覆盖。

目标环境沿用已经跑通的 Ubuntu 22.04 / ROS2 Humble / 系统 CPython 3.10 和原通信配置。GitHub 精简仓库不含完整官方仿真与 SDK 二进制，不能独立替代原安装。任务二代码和离线验证已完成，Ubuntu 的 ROS2 / SDK / 仿真运动闭环仍须现场验收。

历史记录中用户已确认 `config.json`、节点通信问题已解决，`rviz/matrix_mapping.rviz` 已能运行。旧版“以 Windows 覆盖整个 Ubuntu 项目”“排除所有 build 目录”“只允许 config.json 不同”的建议不再适用。官方程序可能就在 `src/robot_mc/build/export/` 和 `src/robot_mujoco/simulate/build/`，不能当普通缓存删除。任务一历史背景见 [README_GPT.md](README_GPT.md)，当前建图操作见 [mapping/README.md](mapping/README.md)。
