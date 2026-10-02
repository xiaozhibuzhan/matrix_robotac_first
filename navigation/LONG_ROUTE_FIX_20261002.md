# 2026-10-02：提高巡航速度并修复长路线半路停止

适用日志：`navigation/run/run_20261002_180042_hhck6zcr/`。本次只修改独立 `navigation/` 扩展与文档；官方代码、SDK 库、运行配置和任务一原始地图保持不变。

## 1. 日志说明了什么

这一轮 SDK 已连接并进入 `TASK2_READY`，没有 SDK 速度错误；节点与 RViz 最后都正常退出。五个被接受的目标如下，距离是点击时到目标的直线距离，耗时包含规划、转弯与停稳：

| 目标 | 目标坐标（地图 m） | 起点到目标直线距离 | 耗时 | 结果 |
| --- | --- | --- | --- | --- |
| 1 | (2.406, -0.194) | 2.456 m | 12.71 s | ARRIVED |
| 2 | (16.722, 3.678) | 14.907 m | 98.27 s | ARRIVED，恢复 1 次 |
| 3 | (13.685, -5.834) | 9.901 m | 83.13 s | ARRIVED，恢复 2 次 |
| 4 | (25.565, -8.438) | 12.090 m | 59.18 s | STOPPED，已恢复 3 次 |
| 5 | (21.261, -0.194) | 3.885 m | 28.48 s | ARRIVED，恢复 1 次 |

第 4 个目标的停车原因是：

```text
Tracking error crosses blocked cells; reselect a goal
```

`node.log` 第 54 行接受目标，第 59、66、70 行恢复，第 75 行停止。停止时约在 `(17.392, -0.813)`，当前格仍可通行，但到下一个路点的线段因跟踪偏移而越过不可通行格；此前整段路线共用的 3 次恢复额度已经耗尽。**它在约 59 秒停止，没有触发 180 秒总时限或 15 秒无进展时限。**

慢的原因也能从日志确认：非零前向指令中位数约 0.20 m/s，已受旧上限限制。第 4 条路线实际绕行约 22.31 m，有 16 段，其中 6 段不足 0.5 m；停止前约 16.5 秒用于对齐转向。旧规划有些位置仅比最低足迹要求多约 1 mm 余量，轻微偏移就容易触发重规划。

## 2. 本次改动

- 前进上限 `0.20 → 0.40 m/s`，偏航上限 `0.35 → 0.60 rad/s`；转弯前和终点附近继续降速并检查实际停稳。加速度模型保持 `0.20 m/s²`。
- 规划与路径简化优先选择额外约 `0.10 m` 的余量，取消不必要的固定长度切段。保留原足迹、障碍、目标和合法窄路；窄路只是代价较高，不会被额外硬性封死。第 4 条路线重算约 22.70 m，最小额外余量约 11.24 cm。
- 将“整个目标最多恢复 3 次”改为“没有取得足够进展时，最多连续恢复 3 次”。恢复后，剩余路径减少至少 0.75 m，且实际净位移至少 0.375 m，才复位连续计数；总恢复数继续记录。停在同一处反复失败仍会取消目标。
- 转向时根据实际航向误差的改善判断进展，避免正在有效旋转却被当作原地卡住。重规划不会重置 180 秒总目标时限。
- 预测检查按命令速度和测量速度中的较大值计算制动距离。接近拐点时减少过度延伸到下一段的直线预测，但检查距离始终不短于完整制动距离。
- 复杂路线在后台规划，等待期间持续输出零速度并处理 ROS 传感器；取消或替换目标后，旧规划结果不能恢复旧目标。这处理了审查中发现的另一个隐患：同步规划耗时可能超过 0.35 秒，导致误判里程计断流。
- 启动终端显示实际速度上限；事件日志增加规划路径，状态增加当前路点、剩余路径长度与连续恢复数，方便后续区分转向等待和真正异常。

新障碍、数据断流、无进展、总时限或连续恢复失败仍然会停车取消目标。本次不新增动态障碍绕行，也不关闭这些检查。

## 3. Ubuntu 怎样更新

### 第一步：传包、校验、备份后安装

先退出旧导航，确认停稳。官方仿真与 Zenoh 保持原有已验证状态，关闭 `highlevel_demo` 等其他运动发送者。

重新传输**本次同一批生成**的两个文件：

```text
navigation/updates/task2_point_navigation_20260926.zip
navigation/updates/task2_point_navigation_20260926.zip.sha256
```

文件名没有变化，必须重新复制两个文件，不能仅凭日期判断是否最新。按 [操作指南第 2 节](OPERATIONS_UBUNTU.md#2-ubuntu-校验备份与安装) 的完整代码执行 SHA256 校验、暂存解压、manifest 检查和备份安装。只替换 `navigation/`，不要覆盖整个官方工程。

记下安装程序打印的 `TASK2_BACKUP` 路径。旧配置和旧运行日志随旧扩展保留在那里；无需删除日志或重新编译官方工程。

### 第二步：创建并核对现场配置

在同一个 Ubuntu Bash 终端，`TASK2_PROJECT` 应为已有可运行工程路径：

```bash
cd "$TASK2_PROJECT"
cp -n navigation/config/default.yaml navigation/config/local.yaml
cp -a navigation/config/local.yaml \
  "navigation/config/local.before_speed_$(date +%Y%m%d_%H%M%S).yaml"
nano navigation/config/local.yaml
```

先对照旧备份恢复已经验证的地图路径、topic、SDK 地址与雷达外参。然后**修改现有同名项**，不要追加重复 YAML 键：

```yaml
max_speed: 0.40
max_yaw_rate: 0.60
acceleration: 0.20
max_tracking_replans: 3
replan_progress_distance: 0.75
goal_timeout: 180.0
progress_timeout: 15.0
```

`replan_progress_distance` 是新项，旧配置没有时补一行。Nano 按 **Ctrl+O → Enter** 保存，再 **Ctrl+X** 退出。其他现场参数保持已验证的值。

**只替换 default.yaml 不够：启动使用 local.yaml 时，其中旧的 0.20 / 0.35 会覆盖新默认值。** 不要通过缩小足迹、提高容许误差或延长超时来代替本次更新。

### 第三步：不运动检查

按 [操作指南第 3 节](OPERATIONS_UBUNTU.md#3-终端分工与环境) 加载现场已工作的 Humble、overlay 和 Zenoh 环境，再逐条执行；任一步失败先处理，不进入实际控制：

```bash
cd "$TASK2_PROJECT"
/usr/bin/python3 -B -m navigation.preflight --offline --config navigation/config/local.yaml
/usr/bin/python3 -B -m unittest discover -s navigation/tests -v
/usr/bin/python3 -B -m navigation.preflight --drive --config navigation/config/local.yaml
```

检查 JSON 的 `control_settings`：`max_speed` 是 `0.4`，`max_yaw_rate` 是 `0.6`，`replan_progress_distance` 是 `0.75`。预期 `Ran 96 tests`，全部 `OK`。`preflight --drive` 只检查文件和导入，不连接或驱动机器人。测试所需的任务一源地图要求见操作指南第 4 节。

### 第四步：短程确认后再走长路线

```bash
bash navigation/run_navigation.sh --config navigation/config/local.yaml --drive --stand-up
```

沿用现场已经验证的时间模式；此前确实用仿真时钟时，继续按操作指南追加 `--use-sim-time`。启动时确认：

```text
Loaded limits: max_speed=0.40 m/s, max_yaw_rate=0.60 rad/s
```

起立等待 4 秒由程序完成，不要重复启动。SDK 日志应有 `TASK2_READY`，节点日志应有 `DRIVE enabled`。

1. 每次重启导航都重新用 **2D Pose Estimate** 标定当前实际位置和朝向，不照抄上述旧日志坐标。
2. 用 **Publish Point** 先点约 1 米、再约 2 米的开阔点，确认转向、到点及制动正常。`ARRIVED` 后再观察至少 3 秒。
3. 再复测此前失败的长路线。上限加倍不等于全程用时减半，拐点仍要减速、停稳和转向。
4. 查看 `/task2/status`：`tracking_replans` 为整段总恢复数，`tracking_replan_streak` 为连续恢复数。前者超过 3 不再单独触发停止；后者只有取得足够进展后才复位。
5. 保留本轮完整 `navigation/runs/run_.../`。若再次 `STOPPED`，连同 `config.yaml / events.jsonl / node.log / sdk.log` 一起反馈；新版日志中的 `planned_path` 能重建当时路线。

需要取消时在已加载环境的另一终端执行：

```bash
ros2 service call /task2/cancel std_srvs/srv/Trigger '{}'
```

需要退出时在导航终端 Ctrl+C，等待回到提示符并确认实际停稳。回退方法见操作指南第 8 节。

## 4. 已验证结果与边界

在真实派生地图上，以相同规划器、0.60 rad/s 转向上限、0.20 m/s² 加速度参数和 0.25 秒响应滞后模型对比：

| 日志路线 | 0.20 m/s 上限 | 0.40 m/s 上限 | 用时减少 |
| --- | --- | --- | --- |
| 目标 2 | 104.15 s | 69.00 s | 33.7% |
| 目标 3 | 80.15 s | 57.55 s | 28.2% |
| 目标 4 | 171.10 s | 130.60 s | 23.7% |

六种组合均到达并测量停稳，终点误差小于 0.10 m。**这些是同条件离线模型结果，不是 Ubuntu SDK 新版实测，也不能和第 1 节现场耗时直接比较。** 为确定性比较，表内不计真实后台规划耗时；后台等待、传感器更新与旧结果作废另有专项测试。报告、坐标与源文件哈希见 [长路线离线报告](validation/long_route_validation.json)；全套 96 项测试和官方文件核验见 [验证报告](validation/local_validation.json)。

新速度下的真实步态、雷达叠加、制动距离、窄通道通行和长时间里程计漂移仍须按上述步骤复测。默认总目标时限仍为 180 秒；更远或更曲折的路线若确实超过它，日志会明确写 `Goal time limit exceeded`，应与本轮恢复额度耗尽区分。
