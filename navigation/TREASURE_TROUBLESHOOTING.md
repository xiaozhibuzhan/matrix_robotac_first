# 宝箱场景与 RViz 点选排查

更新：2026-10-15。本页维护当前故障判断与只读检查。安装见 [OPERATIONS_UBUNTU.md](OPERATIONS_UBUNTU.md)，完整动作见 [TREASURE_UBUNTU.md](TREASURE_UBUNTU.md)。

## 1. 先确认真实宝箱

用户已确认截图来自 **UE 的 UeSim 窗口**，并已完整重启过官方仿真。截图右上角为 FollowCam，低位跟随视角被墙遮挡，只能看到部分场地，不能据此断言整个关卡没有宝箱。

先在当前窗口检查，不必为这个观察重复启动：

1. 点击右下角 FreeCam/FollowCam，确认右上角变为 FreeCam。
2. 将键盘焦点放到场景；按 E 升高到能越过墙体观察的高度。
3. 用 W/A/S/D 移动观察，Q 降低，查看全场及被墙遮挡的区域；自由相机不用于控制机器狗行走。
4. 找到真实箱子后核对其与地图的位置关系，再选择中心与接近点；若全场仍没有箱子，执行第 3 节取证。

这些按键依据本地官方 DefaultInput 与 Spectator 资源，目标机实际视角操作仍需现场验证。没有确认具体鼠标按键的捕获行为，不额外假定右键或中键操作。

**RViz 点击、new-round、select 不会在官方画面生成宝箱。** 扩展只管理人工标注、运动与确认，地图 PGM 也不会自动显示官方箱子模型。

## 2. 点击后没有任务或后面点不了

每点一次后重新执行 `/usr/bin/python3 -B -m navigation.treasure_cli status`，不要继续看 select 当时打印的旧快照。

| 当前状态 | 含义与操作 |
| --- | --- |
| selection_step 为 WAIT_CENTER | 等待有效中心点；current_attempt 为空属正常 |
| selection_step 为 WAIT_APPROACH，center 已有坐标 | 等待有效接近点，只补点接近点 |
| current_attempt 为 APPROACHING | 两点已接受，正在导航；切回 Move Camera，避免额外点选中止 |
| current_attempt 为 READY_CONTACT | 接近点已到达停稳，才可 contact |
| current_attempt 为 FAILED / CANCELLED | 已有任务但动作未完成；先读 reason，再按手册 retry / skip |
| selection 与 current_attempt 均为空 | 读 reason 和回合状态；可能尚未 select，或选择因异常中止；不能直接 contact |

Single click: true 时，每次点之前重新启用 Publish Point；若改为连续点选，两点结束立即切回 Move Camera。误点后是否保留选择取决于当前状态，不能沿用旧版“一律清空”的判断。

### 已确认的 10 月 15 日日志

`navigation/run/1015/run_20261015_143108_i_au58en/events.jsonl`：

| 北京时间 | 行号 | 事件 |
| --- | --- | --- |
| 14:33:40 | 288 | new-round 成功 |
| 14:36:13 | 580 | select 成功 |
| 14:41:35 | 1190 | contact 被拒，因为还没到接近点 |
| 14:42:14 | 1265 | 中心点 (0.421894, -2.020685) 接受 |
| 14:44:04 | 1474 | contact 仍被拒，没有接近点/到达 |
| 14:45:29 | 1636 | 后续点击被拒：Click the 2D map plane |
| 14:45:36 | 1651 | 选择已被旧逻辑清空，要求重新 select |
| 14:46:02 | 1701 | 继续点击仍要求 select |

这次并非第二点一直没有到达节点。原代码要求点击 XYZ 有限、且 z 距地面不超过 0.20 米。第二次收到的点未通过该检查，旧错误处理清空了已选中心，所以出现“后面点不了了”。旧日志没有该点原始 XYZ，不能确定具体点中了哪个物体。节点没有崩溃，SDK 就绪；更早一轮普通导航已到达并停稳。

## 3. 全场仍无宝箱时核对实际场景

1. 当前窗口身份与完整重启已经确认，不重复排查这两项。先保留 FreeCam 全场画面和正在运行的场景。
2. 本地官方 RobotAC_Level_1 资源包含 5 个 CashBox 实例，默认地图配置指向它，但不能证明目标 Ubuntu 使用相同文件或当前仍有 5 个存活宝箱。
3. 本地 run_sim.sh 虽给 MAPNAME 赋值，实际启动 UE 的命令没有使用它。不能仅修改场景编号来保证恢复宝箱，也不改官方脚本、资源或配置。
4. 一次收集下方证据，核对实际 UE 进程参数、默认关卡、当前 UE 日志与资源哈希；不向没有确认箱子的位置执行 contact。

### 一次收集场景证据

在目标 Ubuntu 工程根目录执行。它只读取官方文件和进程，并在 `diagnostics/` 新建一个检查目录，不启动/停止机器人，不改官方配置：

```bash
cd "$HOME/robotac/matrix_robotac_first"
bash <<'BASH'
set -eu
test -f run_sim.sh
mkdir -p diagnostics
TASK2_SCENE_CHECK="$(mktemp -d "$PWD/diagnostics/scene_check_20261015_XXXXXX")"
{
    date -Is
    printf '\nRunning simulator processes:\n'
    pgrep -af 'UeSim|robot_mujoco|run_sim.sh' || true
    printf '\nConfigured default maps:\n'
    grep -nE 'GameDefaultMap|EditorStartupMap' src/UeSim/Linux/UeSim/Config/DefaultEngine.ini || true
    printf '\nOfficial resource hashes:\n'
    sha256sum src/UeSim/Linux/UeSim/Content/Maps/RobotAC/RobotAC_Level_1.umap \
        src/UeSim/Linux/UeSim/Content/Maps/RobotAC/RobotAC_Level_1.uexp \
        src/UeSim/Linux/UeSim/Content/BluePrint/robotac/BP_CashBox.uasset \
        src/UeSim/Linux/UeSim/Content/BluePrint/robotac/BP_CashBox.uexp || true
} > "$TASK2_SCENE_CHECK/scene_check.txt" 2>&1
cp -p run_sim.sh "$TASK2_SCENE_CHECK/run_sim.sh.txt"
if test -f src/UeSim/Linux/UeSim/Config/DefaultEngine.ini; then
    cp -p src/UeSim/Linux/UeSim/Config/DefaultEngine.ini "$TASK2_SCENE_CHECK/DefaultEngine.ini.txt"
fi
if test -f src/UeSim/Linux/UeSim.log; then
    cp -p src/UeSim/Linux/UeSim.log "$TASK2_SCENE_CHECK/UE_launch.log"
fi
if test -f src/UeSim/Linux/UeSim/Saved/Logs/UeSim.log; then
    cp -p src/UeSim/Linux/UeSim/Saved/Logs/UeSim.log "$TASK2_SCENE_CHECK/UE_saved.log"
fi
printf '场景检查目录：%s\n' "$TASK2_SCENE_CHECK"
BASH
```

把打印的目录与 FreeCam 全场画面一起保存。UE 可能关闭了部分日志输出，因此没有加载关卡日志不能证明场景不存在；这次也不要为日志而擅自改官方启动参数。

## 4. 点选交互修复内容

当前版本（包含 selection_fix 的交互修复）：

- 在回合有效、传感器与定位正常、测量停稳、零命令且没有活动任务时，误点保留待选箱子或已选中心，提示补点，不再因一次误点必须重选全套。
- 增加 `selection_step`：`WAIT_CENTER` 等待中心，`WAIT_APPROACH` 等待接近点；成功接近点后该字段为空，使用 `round.current_attempt` 查看任务。
- 记录每次收到的点击 XYZ、frame、回合及阶段。高度错误会显示实际 z 与期望地面高度；拒绝事件记录 `selection_preserved`。
- 点击后立即更新宝箱状态及标记。
- 传感器异常、定位变化、SDK 故障、移动中额外点选仍按原逻辑中止。不放宽地面、距离、足迹、隔离或障碍检查。

这只修复导航扩展的交互，不生成 UE 宝箱，也不代表已经修复官方场景显示。无法从现有资料断言官方箱子不存在的原因。

## 5. 安装入口

当前文档整理包为 task2_treasure_20261015_docs_cleanup.zip。安装、现场配置保留和回退统一见 [OPERATIONS_UBUNTU.md](OPERATIONS_UBUNTU.md)，不再维护另一份覆盖步骤。已安装 selection_fix 的目标机无需为本次文档整理重装。

## 6. 官方宝箱可见后再做点选复测

导航/命令终端按原手册加载相同 ROS 环境，启动保留 `--treasure --drive --stand-up`，等待就绪，设置初始位置及朝向。按官方场景是否仍为同一回合，正确建立或恢复本地回合。

点选前可暂时取消 RViz 的 **Robot Pose**、**Robot Goal and Live Obstacles** 以及额外点云显示，让鼠标只拾取二维地图。这里只隐藏显示，不会关闭后端障碍检查。两点完成后恢复这些图层；隐藏标记时自然看不到目标圈。保持地图本身显示。

1. `select` 成功应为 `WAIT_CENTER`，在二维地图点真实宝箱中心。
2. 变为 `WAIT_APPROACH` 后，点距中心 0.55～1.00 米的可达地面。
3. 若误点被拒但仍为 `WAIT_APPROACH`，按提示重新点接近点，不再重复中心点。
4. `Single click: true` 时每次点选先按 Publish Point；取消该选项则可连续点，两点完成后立即切回 Move Camera，避免第三点中止导航。
5. `current_attempt.state` 变为 `APPROACHING` 后等待自主到达；直到 `READY_CONTACT` 才能 contact。
6. 只有官方画面确认宝箱消失才执行 `confirm --disappeared`。

## 7. 验证范围

162 项离线测试通过，包含 14 项新增点选恢复测试；Python 3.10 语法检查通过。128 个受保护文件未变化，任务一原始地图校验通过。6 个此前审计的官方宝箱/关卡资源与 10 月 9 日记录一致。目标 Ubuntu 的新交互、FreeCam 全场观察结果和宝箱实际触发仍待现场验证；UE 窗口身份已经由用户确认。本次只整理文档，不新增现场验收结论。
