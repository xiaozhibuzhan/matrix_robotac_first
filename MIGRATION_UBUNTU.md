# Ubuntu 迁移说明

更新日期：2026-09-07。迁移说明已统一到项目根目录的 [README.md](README.md)，请以其中“当前状态与目标”和“后续迁移到 Ubuntu 的原则与步骤”为准。

当前用户已确认：

- `config.json` 问题已解决，节点通信问题已解决。
- `rviz/matrix_mapping.rviz` 已可以运行，Ubuntu 迁移进展顺利。
- 后续以 Ubuntu 已跑通的环境为运行基准，先备份、比较差异，再增量同步；Windows 快照尚未与其逐项对齐。

旧版说明中“以 Windows 覆盖整个 Ubuntu 项目”“排除所有 build 目录”“只允许 config.json 不同”的建议不再适用。官方运行程序可能位于 `src/robot_mc/build/export/` 和 `src/robot_mujoco/simulate/build/`，不能当作普通缓存删除。

实际路径、依赖与权限、运行配置变更、两端差异检查及缓存状态均记录在根 README，本文件不再维护另一套命令和参数。历史迁移报错只用于追溯，不代表现在仍然发生。
