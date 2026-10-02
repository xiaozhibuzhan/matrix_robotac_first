# 2026-10-02：实际导航启动退出修复与重试

## 结论与证据

用户的 `--drive --stand-up` 命令正确。本次失败发生在导航扩展的 SDK 启动顺序，尚未进入目标跟踪。

检查了回传的四轮日志：

| 运行目录 | 模式 | 结论 |
| --- | --- | --- |
| `run_20261002_141002_7lxdaarl` | preview | 已收到传感器、定位和点选；无运动时无进展停止符合预览逻辑；退出时报 ROS 重复 shutdown |
| `run_20261002_142016_tdql0snm` | drive + stand-up | SDK 连接成功，第一次 move 被状态机拒绝，约 1 秒后退出 |
| `run_20261002_142921_uijiwdmx` | preview | 已收到定位和点选；无进展停止符合预览逻辑；退出时报 ROS 重复 shutdown |
| `run_20261002_143412_lkrmk64j` | drive + stand-up | 与第一次实际导航相同 |

最新一轮 `sdk.log` 第 4～6 行：

```text
connect success!
Cannot transition to 'move' state: must transition to 'standUp' first.
{"sdk_error": "SDK returned 12295"}
```

`12295 = 0x3007`，官方 SDK 文档定义为“状态机切换失败”，并规定仅在站立状态下切入 move。**零速度 `move(0,0,0)` 也会请求切换状态。** 旧代码先发这个零指令、后处理 stand-up 参数，因此还没有调用 standUp 就退出。父进程的 `SDK worker readiness pipe closed` 和终端的 `Navigation exited` 是随后产生的退出提示。

日志中的 SDK 已连接；单轮 Zenoh expr_id 错误和 RViz GLSL 提示不解释两轮共有的这次 SDK 拒绝。若更新后地图仍不能显示，再单独检查 RViz 图形问题。

回传源代码哈希与本地修复前版本一致。官方证据位于完整工程 `deps/zsibot_sdk/demo/zsl-1/python/examples/highlevel_demo.py`（standUp 后等待 4 秒才 move）与 `deps/zsibot_sdk/docs/api_zsl-1.md`（move 返回码/调用条件）。官方文件没有修改。

## 本次改动

- `sdk_worker.py`：连接后，若有 `--stand-up`，先 standUp 并等待 4 秒，再检查零速度指令，成功后才发出 `TASK2_READY`。等待参考官方 demo，不代表已经通过现场站稳测量；后续仍按里程计和定位条件验收。
- SDK 非零返回码仍按错误退出，记录调用名称、十进制与十六进制错误码。未传 stand-up 时不会擅自调用站立；若被状态机拒绝，检查停稳状态后使用带 stand-up 的命令。
- `sdk_bridge.py`：就绪管道关闭时附上本轮 sdk.log 路径。
- `node.py`：处理 ROS 外部关闭，使用 `rclpy.try_shutdown()`，避免退出时重复关闭 context。未知异常仍会抛出。
- `package_extension.py`：同时排除 `run/` 和 `runs/`，保留回传日志原件，避免放入迁移包。

## Ubuntu 重试步骤

1. 退出旧导航并确认机器狗静止；关闭其他 SDK demo/键盘运动控制程序，保留已工作的官方仿真和 Zenoh。
2. 从 Windows **重新复制本次生成的 ZIP 和 SHA256 文件**。文件名仍是 `task2_point_navigation_20260926.zip` 与 `.zip.sha256`，只看名字无法判断新旧。
3. 按 [操作指南](OPERATIONS_UBUNTU.md) 第 2.1～2.3 节执行校验、暂存检查、旧导航备份和替换。只替换 navigation，旧参数和日志仍在备份中；对照恢复 local.yaml，不覆盖官方文件。
4. 在 Ubuntu 新终端按操作指南第 3 节加载环境，工程路径沿用日志中的实际位置：

```bash
TASK2_PROJECT="$HOME/robotac/matrix_robotac_first"
cd "$TASK2_PROJECT"
/usr/bin/python3 -B -m navigation.preflight --drive --config navigation/config/local.yaml
```

5. 检查通过后，在同一终端重新运行原命令：

```bash
bash navigation/run_navigation.sh --config navigation/config/local.yaml --drive --stand-up
```

6. SDK 起立等待至少 4 秒，期间不要重复启动。启动成功应保持 RViz 和导航运行，`sdk.log` 中出现 `TASK2_READY`，`node.log` 中出现 `DRIVE enabled`，状态进入 `WAIT_INITIAL_POSE`。再设置机器狗实际位置和朝向，点约 1 米开阔点测试，不直接点击宝箱。

新日志正常仍写入 `navigation/runs/`（复数）；Windows 中的 `navigation/run/`（单数）是此次回传位置，两者不要混淆。若再次退出，保留新一轮完整 run 目录，重点检查新的 sdk.log 错误，不继续看旧轮次。

## 验证范围

离线共 **54 项测试通过**，其中 SDK 回归使用模拟状态机：旧启动顺序重现 `12295`，修复后先站立再发零速度；同时覆盖失败不发 READY、等待中取消、未授权站立及退出清理。128 个受保护文件和任务一 PCD/PGM/YAML 哈希均未变化。

这些检查没有连接实际 SDK 或运行 Ubuntu ROS。修复后站立、到达与停稳仍需按上面的步骤现场验证。
