# 2026-10-02：三轮运行日志与 SDK 低速修复

## 按旧到新读取的结论

下表使用北京时间。三轮 run.json 的代码哈希与上一版发布代码一致，说明上次启动顺序修复已经迁移到 Ubuntu。

| 运行 | 实际模式 | 日志证据 | 结论 |
| --- | --- | --- | --- |
| 16:46:24，`run_20261002_164624_vavk1iaq` | drive + stand-up | `connect success!`、`TASK2_READY`；16:46:29 节点启动，16:46:38 初始化，16:46:57 点目标；随后 `invalid velocity in x-axi` / `12307 (0x3013)` | 已完成连接及 4 秒起立等待；这轮记录的失败是低速指令被拒绝，不能归因于未等待 |
| 16:51:36，`run_20261002_165136_z4c3f6ko` | drive + stand-up | 同样连接并就绪；有短暂转向反馈，前进时同样报 `0x3013` | 同一个速度范围兼容问题，并非 SDK 没有连上 |
| 17:00:10，`run_20261002_170010_l0co2lj1` | preview | run.json 没有 `--drive`；node.log 明确 `PREVIEW: SDK not loaded; no robot commands sent`；最后无进展停止 | 预览不会发运动指令；正常没有 sdk.log，另开 highlevel_demo 不会改变导航模式 |

`SDK worker failed; restart navigation` 表示 SDK 子进程已经因错误退出，不等于最开始没有连接成功。SDK 日志第一行 `ROBOT SDK is Running, APP Command or other input is InValid!` 也不是连接失败判据。

## 真正的问题

官方 `deps/zsibot_sdk/docs/api_zsl-1.md` 的 move 接口要求：

- 前向 vx：`0`，或绝对值至少 `0.05 m/s`。
- 侧向 vy：`0`，或绝对值至少 `0.1 m/s`；本导航始终为 0。
- 偏航 yaw：`0`，或绝对值至少 `0.02 rad/s`。

旧控制器按 `0.20 m/s²`、`20 Hz` 平滑起步，第一条非零前向指令约 `0.01 m/s`，落在 SDK 禁止范围。文档虽把 `0x3013` 简写成“速度命令过大”，本次具体报错明确是非零速度低于下限。

官方 C++ demo 注释中提到巡逻模式，但本地头文件、Python 用法与可见二进制接口没有给出可用的低速模式参数；本次继续使用有证据的三参数 move，不添加猜测的第四参数。

## 本次修复

1. 导航控制器单独累计期望起步速度，达到 `0.05 m/s` 前输出零；达到后才输出 SDK 接受的速度。
2. 接近中间拐点时使用合法的最低前进速度，再由原位置条件与实际停稳反馈决定换段。不能简单把所有低于 0.05 的指令丢掉，否则会卡在拐点前。
3. 实际输出仍受最大速度、足迹、预测运动与实时障碍检查；SDK worker 仅校验速度范围，不在传输末端私自抬高速度。取消、到达、故障和停车清除起步累计状态。
4. 不允许将 max_speed 设置低于 `0.05` 或 max_yaw_rate 低于 `0.02`；启动前明确报配置错误。到达仍发精确零速度，并保留原位置/反馈速度/持续时间判据。
5. SDK 返回错误时，记录具体 vx/yaw 和错误码，避免只看到“worker failed”而误判网络。
6. 三轮退出时 `rclpy.try_shutdown()` 内仍出现重复关闭竞态。现在只在异常类型和内容都明确为“已关闭”且 context 确已失活时忽略；其他退出异常继续保留。

SDK 的可用速度范围决定指令有 `0 → 0.05 m/s` 的阶跃，无法表示中间速度。累计起步不等于保证真实加速度或制动距离；需要 Ubuntu 短程复测。没有放宽停稳判据，也没有修改官方 SDK、仿真或任务一地图。

## 重新测试：只保留一个控制客户端

1. 在旧导航终端 Ctrl+C 退出。在 `./highlevel_demo` 所在终端 Ctrl+C 退出该 demo；保留官方仿真、运控与 Zenoh。**highlevel_demo 是另一套键盘控制客户端，不是导航必须依赖的 SDK 服务。** 它和导航都使用本地端口 `43988`，不应同时运行。
2. 重新复制最新 `task2_point_navigation_20260926.zip` 和同次生成的 `.zip.sha256`。文件名未变，务必重新传输；按 [操作指南](OPERATIONS_UBUNTU.md) 第 2 节校验、备份、增量安装，并恢复 local.yaml。
3. 终端 C 按操作指南第 3 节加载原来的 ROS 环境；从工程根目录运行：

```bash
cd "$HOME/robotac/matrix_robotac_first"
/usr/bin/python3 -B -m navigation.preflight --drive --config navigation/config/local.yaml
bash navigation/run_navigation.sh --config navigation/config/local.yaml --drive --stand-up
```

预检查失败则停在该步。两个命令逐条执行。现场仍沿用此前已验证的系统时钟方式，不因本次速度问题额外切换 /clock。

4. 让 C 保持运行。4 秒起立等待由程序自动完成，观察本轮 sdk.log 的 `TASK2_READY` 和 node.log 的 `DRIVE enabled`；这些输出主要在日志里，不要求启动终端每秒刷屏。
5. 在 D（同样加载 ROS 环境）查看：

```bash
ros2 topic echo /task2/status
```

6. 等机器狗静止，在 RViz 用 2D Pose Estimate 设置实际位置与朝向，再用 Publish Point 点距当前位置约 1 米的开阔点。先通过前进、转向、`ARRIVED` 后持续停稳，再测试 2 米与绕障。
7. 若仍失败，不打开 highlevel_demo 叠加控制，也不连续重复点选已锁定的实例。按操作指南取消/退出，保存本轮 `navigation/runs/run_*/` 全部日志；区别它与 Windows 回传目录 `navigation/run/`。

本次 70 项离线测试通过，128 个受保护文件及任务一源地图哈希未变。完整结果见 [local_validation.json](validation/local_validation.json)。这些测试覆盖 SDK 速度域、起步、拐点、停车与退出竞态；真实 SDK/步态闭环仍需上述现场复测。
