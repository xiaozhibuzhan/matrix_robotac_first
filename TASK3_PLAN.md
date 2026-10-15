# 任务三准备：规则、官方接口与首个闭环

更新：2026-10-16。任务一和任务二已完成，正式移动入口沿用已成功的普通导航，不启用旧宝箱专用模式。本页是开发准备，不表示任务三功能已实现。

## 1. 当前可以复用什么

| 成果 | 使用方式 |
| --- | --- |
| 当前 PGM/YAML、重新建图与切图流程 | 沿用当前场景；若任务三换场景，重新采集和核对地图 |
| RViz 普通单点导航 | 前往观察或任务位置，停稳后执行后续动作 |
| SDK 起立、运动与异常停车 | 每个新进程保留 drive、stand-up，至少等 4 秒和 TASK2_READY，单一控制器 |
| 官方宝箱触发与记分 | 已成功；作为任务二完成证据，不代表任务三弹药接口已打通 |
| UE/ROS 相机与官方任务资源 | 可开展只读接口调查；实际消息和动作仍需验证 |

任务二的成功导航命令继续保留：

```bash
bash navigation/run_navigation.sh --config navigation/config/treasure_local.yaml --drive --stand-up
```

配置文件名不切换模式。此次不改成功路线、足迹或障碍保护，也不自动增加发射行为。

## 2. 已定位的官方线索

本次解析了下列 cooked 资源的名称、导入及导出表，未完整还原它们的执行链。证据见 [official_assets.json](diagnostics/task3_preparation_20261016/official_assets.json)。**资源名或事件名是调查入口，不能单独当作可调用的射击接口。**

| 位置（均在 src/UeSim/Linux/UeSim/Content/BluePrint 下） | 已确认内容与下一步 |
| --- | --- |
| `Robot/MujoCoSim_Xgb` | 引用 `BP_BulletRobotAc`，有 ROS String 接收事件、Weapon 组件及 Gamepad_RightShoulder 输入事件；需还原触发条件、消息载荷和当前配置是否启用 |
| `robotac/BP_BulletRobotAc` | 有 BeginOverlap、BeginPlay、Tick 导出；需查碰撞对象、销毁及有效命中条件 |
| `robotac/BP_RobotACCharacter` | 引用 `BP_RobotACSCORE` 和 `UpdateAttackScore`；需核对与任务三目标的实际关系，不能先认定目标类型 |
| `robotac/BP_RobotACSCORE` | 存在 `UpdateAttackScore`、`UpdateBoxScore`、`UpdateQRScore` 函数；需核对命中反馈、分值及是否对外发布 |
| `IROS/BP_rifle` | 有 FireInTheHole 名称和 Gamepad_RightShoulder 事件，但属于另一个资源分支；不能直接当作当前机器狗的接口 |

另已核对：

- `deps/zsibot_sdk/include/zsl-1/highlevel.h` 没有公开射击函数；不能凭空调用 SDK fire/shoot。
- 当前 `config/config.json` 的 `robot.weapon` 是空字符串；先查取值与装载逻辑，不填猜测值、不为准备工作改官方配置。
- 当前相机配置为 `/front_camera/image/compressed`，1920×1080、FOV 120、10 Hz；这些来自本地配置，不等于目标 Ubuntu 当前实际消息与相机标定。

## 3. 先核对的规则

以下尚未取得足够信息，不写死到实现：

1. 任务三的正式目标、允许的操作方式、计分/扣分和结束条件。
2. 是否沿用当前关卡与起点，目标是静止还是移动，以及参赛者能否人工指定目标。
3. 任务二所得弹药的继承、上限、消耗规则；重启仿真是否清空，官方状态如何读取。
4. 是否允许直接人工触发、ROS/UDP 控制或仅用规定的官方入口。

此前任务二规则提到“每箱获得任务三弹药 1 发”，只说明赛规关联；现有普通导航没有实现这条官方状态查询或转移。以官方实际状态和适用规则为准。

## 4. 建议推进顺序与验收

| 顺序 | 工作 | 可检查的通过条件 |
| --- | --- | --- |
| 1 | 核对规则和实际场景 | 目标、输入、计分、弹药和结束口径明确 |
| 2 | 还原当前官方动作入口 | 明确被当前机器人使用的事件、消息类型/载荷及触发条件，不能只凭关键词 |
| 3 | 官方最小人工闭环 | 若规则确为射击，先在机器人停稳时触发一次官方发射，看到真实弹体/命中和官方反馈，记录是否扣弹药 |
| 4 | 相机与坐标验证 | 确认实际话题、图像方向和时间戳，核对相机与机器人朝向关系 |
| 5 | 自有最小控制扩展 | 按已核实规则实现人工指定或许可方式选目标 → 对准 → 显式单次动作 → 核对官方结果 |
| 6 | 联合与重复验收 | 普通导航到位、停稳、任务动作、官方反馈；再验证无命中、弹药不足和取消等情况 |

建议首个开发目标：**机器人停稳时，通过核实的官方入口完成一次有效任务动作，并由官方画面/记分反馈确认。** 若任务三为射击，就是单次发射与有效命中闭环；先不承诺未知规则下的自动识别、连续发射或整场策略。

## 5. Ubuntu 可先保存的只读证据

保留已经成功的场景和环境。在已加载同一 ROS 环境的终端进入实际原工程执行；下列命令不会发送运动或发射命令：

```bash
cd "$HOME/robotac/matrix_robotac_first"
mkdir -p diagnostics
TASK3_CHECK="$(mktemp -d "$PWD/diagnostics/task3_check_XXXXXX")"
date -Is > "$TASK3_CHECK/time.txt"
ros2 topic list -t > "$TASK3_CHECK/topics.txt"
ros2 service list -t > "$TASK3_CHECK/services.txt"
ros2 node list > "$TASK3_CHECK/nodes.txt"
ros2 topic info -v /front_camera/image/compressed > "$TASK3_CHECK/camera_info.txt" 2>&1
printf '任务三只读检查目录：%s\n' "$TASK3_CHECK"
```

相机话题不存在也保留错误结果，再按 topics.txt 核对实际名称。暂不发布猜测的 ROS/UDP 载荷，不把候选按键写成已确认发射操作。

每次后续实验保存命令、实际配置、机器人和场景状态、动作前后弹药/分数、图像与官方反馈。本次只完成准备，不扩大先前对宝箱坐标的官方资源修改授权。
