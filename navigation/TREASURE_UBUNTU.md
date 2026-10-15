# 任务二正式操作：普通导航触碰宝箱

更新：2026-10-16。用户已在 Ubuntu 确认：移入场内的宝箱可触碰后消失，官方记分牌增加对应积分，普通导航完成任务二。后续统一使用以下入口，**不加 `--treasure`**：

```bash
bash navigation/run_navigation.sh --config navigation/config/treasure_local.yaml --drive --stand-up
```

`treasure_local.yaml` 是沿用的现场配置文件名，不会自动启用宝箱专用模式。`--drive` 才会加载 SDK 并发送运动命令；`--stand-up` 依赖它，不能省略 `--drive`。此次成功命令已由用户单独确认。

## 1. 当前正式路线与停用事项

正式路线为：UE 观察真实宝箱 → RViz 普通单点导航 → 机器人与宝箱触发重叠 → 宝箱消失、官方计分 → 选择下一箱。

旧 `--treasure` 专用模式在用户现场仍有问题，**从正式操作与后续任务前置条件中停用**。它的两点选择、本地回合、`treasure_cli new-round / select / contact / confirm / resume` 不再用于本次已验收路线。实验源码保留用于追溯，本次不改已经跑通的导航运动代码，也不宣称旧模式已修复。

当前依据官方画面与记分牌判断成功，不要求旧程序出现 `current_attempt`、`READY_CONTACT` 或 `CONFIRMED`。普通导航状态 `ARRIVED` 只说明到达停稳，不能单独证明宝箱触发。

## 2. 准备场景、地图与配置

1. 沿用已验收的 Ubuntu 22.04、ROS2 Humble、系统 Python 与通信设置。终端环境、普通导航安装见 [OPERATIONS_UBUNTU.md](OPERATIONS_UBUNTU.md)。
2. 已成功应用场景位置补丁的机器无需重装。新机器才按 [scene_tools 手册](../scene_tools/README.md) 校验、备份并应用，修改场景后必须完整重启官方仿真。
3. 当前场景使用已验收的地图和本机 `navigation/config/treasure_local.yaml`；保留 SDK 地址、雷达外参、足迹与障碍检查。换场景按 [重建与切图指南](MAP_REBUILD_UBUNTU.md) 更新。
4. 使用原来成功的官方仿真命令与参数。新的任务二演练从指定起点开始；已在正常工作的同一回合中不必因更新文档重启。

沿用原有现场配置，不用模板覆盖它。需要核对的当前参数为：

```yaml
map: navigation/maps/field_a_20261009_170749/field_map.yaml
goal_timeout: 300.0
progress_timeout: 15.0
max_speed: 0.40
max_yaw_rate: 0.60
```

这些是已有长路线配置建议，不表示此次用户反馈逐项重新标定了参数。尤其 box-2 需要绕行，直线距离不能代替实际路径长度。

## 3. 启动与定位

在已加载现场 ROS 环境的导航终端、实际原工程根目录执行：

```bash
cd "$HOME/robotac/matrix_robotac_first"
bash navigation/run_navigation.sh --config navigation/config/treasure_local.yaml --drive --stand-up
```

- 不加旧模式参数，不同时运行 `highlevel_demo`、手动遥控或另一个导航实例。
- 起立后至少等 4 秒，期间不要重复启动。等待 SDK 日志 `TASK2_READY` 与导航 `DRIVE enabled`，外观已站立不能替代连接就绪。
- 在 RViz 用 `2D Pose Estimate` 输入机器人当前实际位置和朝向，以两面不同方向的墙核对点云对齐；它不会移动机器人。
- 若现场原来使用 `--use-sim-time`，继续按原操作手册一致设置，不因这次文档更新切换时钟。

## 4. 单个宝箱与下一箱

1. 在 UE 确认真实箱子的位置，按 [位置表与示意图](../scene_tools/README.md#1-新位置与接近点) 对照地图。先测起点房间的 box-1，中心约 `(1.00,-3.00)`。
2. RViz 的 `Publish Point` 在普通模式下每次只选**一个导航目的地**。不要照旧教程连续点“箱子中心、接近点”；第二次普通点选会替换当前目标。
3. 沿用本次成功的单点操作前往宝箱区域；首次或定位有偏差时，可先到附近可达空地停稳、核对画面，再点新的最终目标。两次操作是两个独立普通导航目标，不是旧模式的两点选择。
4. 观察机器人触碰时宝箱是否消失，核对官方记分牌对应增分；两项都满足才记录该箱成功。无需额外运行本地 confirm。
5. 停稳并核对位置后选下一箱。若宝箱消失但旧地图/实时雷达仍阻挡路线，先读停止原因、核对传感器，不以本地确认命令代替实际排查。

普通模式保留全部足迹、未知区、静态/实时障碍和 SDK 保护。没有旧模式的目标圈临时允许功能；若目标被拒或途中保护停车，不关闭检查强行触碰，也不把官方消失误认为程序自动更新了原始 PGM。当前成功经验以用户已跑通的场景为准。

## 5. 状态、停止与记录

另开已加载相同 ROS 环境的终端查看普通导航状态：

```bash
ros2 topic echo /task2/status
```

需要取消当前目标时：

```bash
ros2 service call /task2/cancel std_srvs/srv/Trigger '{}'
```

结束导航在导航终端 Ctrl+C，等待进程退出并核对实际停止。退出 echo 只结束查看，不会停车。

保留本轮 `navigation/runs/run_.../` 中 `run.json`、`config.yaml`、`node.log`、`sdk.log`、`events.jsonl`，另保存官方记分牌前后画面、消失箱数、起止时间。普通模式不自动生成旧宝箱回合文件，也不自动维护其弹药估算；任务三实际可用弹药仍需核对官方状态与赛规。

任务二已按用户现场反馈完成。后续工作见 [任务三准备](../TASK3_PLAN.md)，重复演练作为稳定性补充，不再把旧实验模式修复列为任务三的前置条件。
