# 任务二场景与正式操作排查

更新：2026-10-16。用户已确认普通导航触碰宝箱、宝箱消失和官方记分成功，任务二完成。正式流程见 [TREASURE_UBUNTU.md](TREASURE_UBUNTU.md)，场景应用/还原见 [scene_tools](../scene_tools/README.md)。

## 1. 当前唯一推荐的任务二启动命令

```bash
bash navigation/run_navigation.sh --config navigation/config/treasure_local.yaml --drive --stand-up
```

不加 `--treasure`。旧专用模式在现场仍有问题，已从操作说明停用；不再让操作者通过 new-round、双点选择、contact 或 confirm 来完成当前任务二。其源码保留追溯，本次没有修复或删除实验实现。

| 现象 | 核对与操作 |
| --- | --- |
| treasure_local.yaml 名字里有 treasure | 只是配置文件名，不启用特殊模式；无需重命名已跑通的现场文件 |
| 加了 --stand-up 却没加 --drive | 当前启动器拒绝该组合；实际控制应保留二者 |
| 看不到 current_attempt / READY_CONTACT | 普通模式没有旧宝箱回合，不是当前成功流程的错误依据 |
| 连续点击中心和接近点后目标改变 | 普通 Publish Point 每次选一个目的地，后一次会替换前一次；不要沿用旧双点教程 |
| 到达了但宝箱没消失 | ARRIVED 只证明停稳；核对实际位置与官方画面，不以到达替代触发 |
| 宝箱消失但本地没确认记录 | 当前以官方画面和记分牌为准，不执行旧本地 confirm |
| 目标不可达或实时障碍停车 | 保留足迹/未知区/障碍检查，核对地图、定位与日志后重新选择 |

## 2. 仍需保留的启动注意事项

每次新控制进程保留 `--stand-up`，起立后至少等 4 秒，等待 SDK 的 `TASK2_READY` 与导航 `DRIVE enabled` 再定位点选。不要同时运行 highlevel_demo、遥控或另一导航进程。画面站立不代表新 SDK 连接就绪。

用 `2D Pose Estimate` 对齐当前实际位置及朝向；它不会移动机器狗。用两面墙核对近远处点云。点击须落在二维地图平面，Single click 模式每次点前重新启用 Publish Point，操作完成切回 Move Camera。

节点退出时读 `node.log` 与 `sdk.log`。本次用户未提供新的专用模式失败日志，旧问题原因不追加猜测，不写成 SDK、参数或点选已被修复。

## 3. 另一台机器场景不一致时

当前摆放问题已解决；仅在迁移后画面不符时做以下检查：

1. 在实际原工程执行 `/usr/bin/python3 -B scene_tools/treasure_layout.py status --project "$PWD"`，应为 PATCHED。
2. 场景补丁必须在关闭 UE 后应用，之后完整重启官方仿真；只重启导航无效。对同一个实际工程执行补丁和启动。
3. FreeCam 升高观察全场，核对 [五箱位置表](../scene_tools/README.md#1-新位置与接近点)。原箱子排在场外并非窗口身份或 FollowCam 问题。
4. 若资源为 PATCHED 而位置仍旧，保存进程、默认关卡与哈希证据。原 run_sim.sh 的 MAPNAME 没有实际传给 UE，不能仅切换场景编号来保证换图。

在已加载环境的 Ubuntu 原工程执行，只读取和保存证据：

```bash
mkdir -p diagnostics
TASK2_SCENE_CHECK="$(mktemp -d "$PWD/diagnostics/scene_check_XXXXXX")"
{
    date -Is
    pgrep -af 'UeSim|UnrealGame|robot_mujoco|run_sim.sh' || true
    grep -nE 'GameDefaultMap|EditorStartupMap' src/UeSim/Linux/UeSim/Config/DefaultEngine.ini || true
    sha256sum src/UeSim/Linux/UeSim/Content/Maps/RobotAC/RobotAC_Level_1.umap \
        src/UeSim/Linux/UeSim/Content/Maps/RobotAC/RobotAC_Level_1.uexp
} > "$TASK2_SCENE_CHECK/scene_check.txt" 2>&1
printf '场景检查目录：%s\n' "$TASK2_SCENE_CHECK"
```

## 4. 历史问题如何看待

10 月 15 日旧模式日志中曾收到中心点，后续点被二维平面检查拒绝，旧处理又清空选择；当时增加了 selection_step、XYZ 日志和停稳健康状态下的点击重试。这是旧实验的历史结论，不是本次普通导航成功的必要步骤。

10 月 16 日用户反馈旧专用模式仍有问题，普通导航则成功完成。两者分别记录：**任务二完成，旧模式停用，旧模式根因未新增确认。** 原教程可在 Git 提交 eaa6a3e 及开发机文档备份中追溯，当前不再重复提供旧 CLI 命令。

后续安排见 [任务三准备](../TASK3_PLAN.md)，新一轮需要保留普通导航日志、官方记分牌前后画面与实际使用命令。
