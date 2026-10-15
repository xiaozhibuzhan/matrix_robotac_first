# 任务二宝箱操作：选择、接近、接触与确认

更新：2026-10-15。当前迁移包为 task2_treasure_20261015_docs_cleanup.zip，功能代码与 selection_fix 版本相同，包含 Humble CLI 与点选误点恢复修复。安装、备份、环境、普通导航和回退统一维护在 [OPERATIONS_UBUNTU.md](OPERATIONS_UBUNTU.md)，本页只维护宝箱专用流程。

## 1. 开始前的条件

目标环境为 Ubuntu 22.04、ROS2 Humble、系统 Python 3.10。普通导航应先验证可到达停稳。用户已授权修正场外宝箱摆放，独立补丁只改官方关卡五箱 XY；官方触发/计分蓝图、MuJoCo、SDK 和任务一文件不变。

用户视频已确认五个宝箱排在地板外。先按 [宝箱位置修复手册](../scene_tools/README.md) 在 Ubuntu 应用、完整重启 UE，检查五箱位于地板上且持续存在；先验收起点附近 box-1。new-round、select 和 RViz 标记不会生成或搬动官方宝箱，单纯更新导航也不含此场景修复。

已实现流程为“人工选箱 → 自主到接近点 → 显式受控接触 → 人工观察消失 → 停稳确认 → 下一箱”。已有离线验证，官方仿真中的完整流程和实际触发范围仍待现场验收。

官方负责宝箱触发、消失和内部计分。本地宝箱数、弹药数和赛规分数只是记录与估算；new-round / finish 不代替裁判起止计时或结束示意。

## 2. 安装与现场配置

需要升级时，按 [统一操作手册第 1～2 节](OPERATIONS_UBUNTU.md#1-准备迁移文件windows) 传输当前 ZIP 及同次校验文件、暂存检查、备份并安装。它保留 local.yaml、*_local.yaml、missions 和额外地图，旧日志保存在备份中。回退见该手册第 8 节。已经安装 selection_fix 的用户无需为本次文档整理重新安装。

每个导航/命令终端按 [第 3 节](OPERATIONS_UBUNTU.md#3-终端分工与环境) 加载同一套现场 ROS 环境；下文均在实际 Ubuntu 工程根目录运行。

### 2.1 创建本机宝箱配置

从已经跑通的现场配置复制，保留 SDK 地址、端口、雷达外参和机器狗尺寸：

```bash
cd "$TASK2_PROJECT"
test -f navigation/config/local.yaml
cp -n navigation/config/local.yaml navigation/config/treasure_local.yaml
nano navigation/config/treasure_local.yaml
```

修改现有同名顶层键，不追加重复 YAML 键：

```yaml
map: navigation/maps/field_a_20261009_170749/field_map.yaml
goal_timeout: 300.0
progress_timeout: 15.0
max_speed: 0.40
max_yaw_rate: 0.60
```

300 秒适用于此前长绕行路线的受控复测；15 秒无进展、传感器保护和 SDK 看门狗仍启用。没有旧 `local.yaml` 时，先按原手册完成普通导航配置。换场景先按 [重建与换图指南](MAP_REBUILD_UBUNTU.md) 生成新图，再修改 `map`。此示例图只用于当前场景。

```bash
/usr/bin/python3 -B -m navigation.preflight --offline --config navigation/config/treasure_local.yaml
/usr/bin/python3 -B -m navigation.preflight --drive --config navigation/config/treasure_local.yaml
/usr/bin/python3 -B -m navigation.treasure_cli --help
```

这些预检查不会驱动机器人。

## 3. 预览与正式启动

按原手册运行官方仿真、通信与传感器。另开“导航终端”和“命令终端”，两者都进入同一工程、source 同一 ROS 环境并使用相同 `ROS_DOMAIN_ID`；每个新终端都需要重新设置工程路径。

先在导航终端预览：

```bash
bash navigation/run_navigation.sh --config navigation/config/treasure_local.yaml --treasure
```

命令终端查看状态：

```bash
/usr/bin/python3 -B -m navigation.treasure_cli status
```

无 `--drive` 时没有 SDK 运动，也不能确认真实宝箱成功。预览中机器狗不走，导航无进展保护可能触发，这是预期行为。检查图层后 Ctrl+C 结束预览。

开始一轮新的正式试验时，按已验证的官方命令重启仿真、回到指定起点并恢复宝箱。应用位置补丁后必须完整重启官方仿真，再 new-round；只有官方仿真确实未重启、仍属同一回合才按第 5 节恢复记录。导航终端运行：

```bash
bash navigation/run_navigation.sh --config navigation/config/treasure_local.yaml --treasure --drive --stand-up
```

起立后至少等待 4 秒，期间不重复启动。**本目标仿真每次新建控制进程均保留 `--stand-up`**：10 月 10 日日志中，省略它的 5 次启动均被 SDK 以 `0x3007` 拒绝，包含前一次导航已成功的情况；外观站立不等于新的 SDK 连接允许 move。等待 `sdk.log` 出现 `TASK2_READY`、导航显示 `DRIVE enabled` 后再定位。不要同时运行 `highlevel_demo`、手动遥控或第二个导航实例。若原现场确定需要 `--use-sim-time`，按原手册在预览和控制中一致添加；不要只为了消除错误随意切换。

在 RViz 用 `2D Pose Estimate` 对齐实际位置及朝向；它不会搬动机器狗。用两面不同方向的墙核对近、远处点云。新的官方回合执行下面的 new-round；若官方仿真未重启、要继续已有回合，跳到第 5 节 resume，不执行这里的 new-round：

```bash
/usr/bin/python3 -B -m navigation.treasure_cli new-round
/usr/bin/python3 -B -m navigation.treasure_cli status
```

记录 `status` 命令输出中的 `round.round_id`（动作 ACK 则嵌套在 `status.round.round_id`）。已有活动回合必须先 `finish`，不能通过重复 `new-round` 重置计时。

## 4. 一个宝箱的完整操作

### 4.1 两次点选

```bash
/usr/bin/python3 -B -m navigation.treasure_cli select box-1 --radius 0.18
```

**0.18 m 只是命令示例，未被标定为官方宝箱尺寸。** 允许范围为 0.05～0.30 m；所选圆必须包含整个宝箱水平占据，同时与墙和其他物体隔离。该圆会显示在 RViz。目标圈涉及未知栅格、占据跨越边界或与墙连通时会拒绝，不能为通过检查随意放大圈。

用 `Publish Point` 按顺序点两次：

1. 宝箱中心，必须对应官方画面中的实际宝箱。
2. 已知可达的接近点，程序允许距中心 **0.55～1.00 m**，建议从约 0.75～1.00 m 的清晰空地接近；不要隔墙选点。

点选时保持二维地图显示，可临时隐藏 Robot Pose、Robot Goal and Live Obstacles 与额外点云，减少拾取三维对象的机会；这只改变 RViz 显示，后端障碍检查仍启用。两点完成后恢复图层。

第二点有效后才开始普通自主导航。箱子不会自动被识别或搜索。`select` 后 `selection_step: WAIT_CENTER`；第一点有效后为 `WAIT_APPROACH`，此时 `current_attempt` 仍可为空；第二点有效后才创建任务。10 月 15 日起，停稳且健康时误点会保留待选阶段并给出原因，可以修正同一个点；传感器/定位异常及运动中误点仍可能取消动作，应先看 `status`，不要连续盲点。`Single click: true` 时每点之前重新按 Publish Point；连续点选模式下，第二点完成后立即切回 Move Camera。

每次点击后重新运行 status，读取新快照；select 当时打印的 JSON 不会自动变化。若没有推进，按 [点击诊断表](TREASURE_TROUBLESHOOTING.md#2-点击后没有任务或后面点不了) 判断下一步，不只检查 current_attempt 是否为 null。

### 4.2 到达后显式开始接触

等待 `round.current_attempt.state` 为 `READY_CONTACT`，普通导航状态应为 `ARRIVED`。此时不自动向宝箱移动：

```bash
/usr/bin/python3 -B -m navigation.treasure_cli status
/usr/bin/python3 -B -m navigation.treasure_cli contact
```

默认速度 **0.05 m/s**、最大累计位移 **0.80 m**、最多 **30 秒**、5 秒无进展停车。先原地对齐，再低速接近；距点击中心约 0.25 m 时发零速并核对停稳。这些是保守工程起点，不保证该距离适合所有宝箱或场景。

控制器只对明确选定、与其他障碍隔离的目标圆作临时允许，保留原足迹、未知区、圈外静态与实时障碍、定位健康和 SDK 保护。UE 的重叠触发物可能不产生阻挡点云；此时仍可沿已知安全地面接近，目标身份由人判断，不以“雷达没点”推断宝箱消失。

`contact` 返回成功 ACK 只表示开始执行。末端状态 `WAIT_CONFIRM` 只表示已停住，**不代表宝箱触发成功**。未消失就不确认；检查定位、触发区域或日志后再决定重试，不能关闭保护强行前进。

### 4.3 观察消失后确认

只有亲眼看到所选宝箱消失，才运行；若接近途中已经消失，可以立即发此命令：

```bash
/usr/bin/python3 -B -m navigation.treasure_cli confirm --disappeared
/usr/bin/python3 -B -m navigation.treasure_cli status
```

程序先持续输出零速度，待测量停稳连续满足 0.8 秒后才计入。确认 ACK 可能仍在等待停稳；应看到 `round.current_attempt.state: CONFIRMED` 和 `round.confirmed_count` 增加，再选下一箱。若普通导航已使宝箱消失，也可在 `READY_CONTACT` 时直接人工确认，不必额外前进。

确认后，仅在本轮内存地图中移除该孤立宝箱占据并更新规划器；原始 PGM/YAML 不变，新回合恢复原图。该箱子编号和重叠目标区域会被去重，换编号也不能重复计数。

## 5. 多个宝箱、结束及恢复

下一箱换用 `select box-2 --radius 实测半径`，重复上面的两次点击与确认。一次只执行一个人工选择的目标，没有自动宝箱坐标和自动排序。

```bash
/usr/bin/python3 -B -m navigation.treasure_cli finish
/usr/bin/python3 -B -m navigation.treasure_cli status
```

`navigation/missions/<round_id>.json` 保存回合、目标历史、确认数、剩余时间和本地分数估算；`navigation/runs/run_.../` 保存本轮日志，关注 `events.jsonl` 的 `treasure_command / treasure_click / treasure_confirmed / treasure_contact_stopped / treasure_memory_map`。拷回的旧日志可能另在 `navigation/run/`。

命令均使用 `/usr/bin/python3 -B -m navigation.treasure_cli` 前缀：

| 命令 / 情况 | 行为 |
| --- | --- |
| `cancel` | 取消当前动作并请求零速，仍需核对实际停止 |
| `skip` | 不计成功，跳过当前目标，再选另一箱 |
| `retry` | 从失败、取消或跳过状态重新导航到原接近点，不自动接触 |
| `finish` | 停止运动、结束本地回合，不代替官方结束示意 |
| ACK 超时 | 结果未知，先 `status`；CLI 会在等待期内用相同 ID 重试，节点不会重复执行同一请求 |
| CLI Ctrl+C | 仅退出命令终端的客户端，已接受的动作仍可能在进行；需要停车另发 `cancel` |
| 导航 Ctrl+C / RViz 关闭 | 本轮导航回收并请求停车，旧动作不自动恢复 |
| `WAIT_CONFIRM` 但箱子还在 | 不确认；先 `cancel` 或 `skip`。`retry` 仍须通过普通地图起点检查 |
| 失败位置已在原箱子膨胀区 | 普通导航可能拒绝起点；不要虚假确认清图。退出导航后用既有手动方式恢复到安全接近位置，重新启动和定位 |

所有 CLI 默认等待 15 秒，可用 `status --timeout 30`；没有节点时检查 `--treasure`、ROS 环境、DOMAIN 和导航日志。

**仅当官方仿真未重启、仍为同一回合、原图哈希和路径不变时才能恢复。** 重新启动扩展、先用 `2D Pose Estimate` 对齐当前实际位置，再执行：

```bash
/usr/bin/python3 -B -m navigation.treasure_cli resume 实际round_id
/usr/bin/python3 -B -m navigation.treasure_cli status
```

已确认宝箱保留，未完成动作变成取消状态，需显式 `retry`。退出期间的时间仍计入；墙钟倒退会拒绝恢复。官方重启、换场景或换图后用 `new-round`，不要恢复旧记录。重新设置初始位姿也会取消未完成动作，不能边移动边反复定位。

## 6. Ubuntu 现场验收

以下尚待目标系统执行，建议每次保留完整日志：

1. 安装 manifest 通过，普通导航不加 `--treasure` 仍能到达停稳。
2. 无 `--drive` 时无 SDK 运动，不能登记真实成功。
3. 独立重启官方仿真，单宝箱完整闭环至少重复 3 次；记录实际消失时的位置，核对默认接近终点是否足够。
4. 到达不自动计数，未消失不确认；同一 ID 或同一位置不重复计数。
5. 取消、未知区域、邻墙目标、无关实时障碍和传感器异常均能中止动作。
6. 连续完成至少两个不同宝箱，确认后的下一条路线可正常规划，画面与记录一致。
7. 同回合扩展重启不恢复旧运动、不增加剩余时间；新仿真重新建立回合、恢复原地图。
8. 完整演练包括官方起点与结束示意，本地 `finish` 不能替代裁判操作。

失败时区分“未到达、未触发、未确认、未停稳”并保存日志及对应画面，不凭“没加分”直接提高速度、缩小足迹或扩大目标圆。
