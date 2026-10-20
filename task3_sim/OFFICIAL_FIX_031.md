# 任务三 0.3.1：Ubuntu 启动、机器狗显示与三块靶修复

本次只更新 `task3_sim/`。官方 ELF、SDK、模型、步态策略、任务一二和旧单靶入口保留。修复后的官方步态仍需在 Ubuntu 实跑，Windows 的图像回归不代替动力学验收。

## 本次记录说明了什么

已读取 `offical_300` 及回传的 `official_20261019_201947_4851`、`official_20261019_202255_5304` 两场记录。官方只读状态桥已在 Ubuntu 编译成功，依赖预检为 ready；两场分别收到 945、947 个有效物理状态包。因此此次失败不应归因于状态桥未编译或完全没有物理程序。

但两场均在 `Official SDK connection timeout` 处停止，SDK 指令账本为空、射击为 0，尚未完成 SDK 连接和站起。程序打印 `TASK3_PROCESS_EXIT code=2` 后清理了窗口。另有 `mc_ctrl` 最终退出码 `-6`，表示该原生进程发生 SIGABRT；旧记录缺少清理前状态，不能证明它发生在超时之前还是终止收尾期间。Python 故障日志为空也不能排除子进程异常。详细审计见 `validation/ubuntu_official_030_failure_review.json`。

## 三个原因与修改

1. **只看见环境：**任务三将机器人放入 geom group 3，供射线排除自身；官方原生观察窗口默认只显示组 0、1、2。现在独立的原生场景副本将机器人放入可见组 0、计分碰撞面放入隐藏组 5，并设置穿过敞开顶部的俯视初始相机。Python 相机/裁判镜像仍使用机器人组 3、计分面组 2。两份模型的质量、惯性、关节、执行器、几何接触等物理数组逐项核对一致，仅观察设置不同。

2. **SDK 超时及窗口关闭：**`mc_ctrl.log` 明确记录 `client_ip: 192.168.234.1`、端口 43997/43900 绑定失败、`Cannot assign requested address` 和 `sdk init failed!`。官方 `librobot.so` 通过环境变量 `SDK_CLIENT_IP` 选择本地绑定 IP；缺省为旧硬件地址。0.3.0 只修改副本 `sdk_config.yaml` 的回传地址，遗漏了这一独立设置。现在只在任务三创建的子进程中设置官方支持的 `SDK_CLIENT_IP=127.0.0.1`，无需改网卡或官方文件。即使控制器进程仍活着，也检查 SDK 初始化失败日志并报告具体原因。启动阶段、启动期间收到的真实物理状态、每个进程清理前的退出状态、是否主动终止/强杀均会保存。

3. **前方两块完整靶：**额外板是 ID 42 干扰物，其外观和正式绿色靶相同。现在两个默认配置只有左 ID 2、前 ID 5、右 ID 9 三块正式靶；干扰验收另加一张 20 cm 白底 AprilTag 贴片，没有绿色靶环或计分碰撞体。默认画面正前方只有一个靶，四面真实图片背景保留。

另修复 Ubuntu 单元测试夹具创建临时假 ELF 后漏设执行权限的问题。这是测试夹具问题，不能通过放宽生产执行权限检查处理。完整验收会记录各阶段真实退出码、失败/跳过原因，优先运行原生链路，失败后尽早导出日志。

网络设置依据官方 [deploy.md 的 SDK_CLIENT_IP 说明](../deps/zsibot_sdk/docs/deploy.md#23-sdk_client_ip) 和 [faq.md](../deps/zsibot_sdk/docs/faq.md)。本地静态概览见 [机器狗可见性图](validation/official_fix031_native_overview.png)，正前方相机图见 [单个正式靶](validation/official_fix031_front_rgb.png)；前者只用于验证显示，不是步态运行截图。

## 在 Ubuntu 更新

将新的 ZIP 和 `.sha256` 放在同一目录，在该目录执行；保留原来已编译的 `task3_sim/local/native/` 和 Ubuntu 虚拟环境：

```bash
sha256sum -c task3_official_dynamics_031_20261019.zip.sha256
unzip -o task3_official_dynamics_031_20261019.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
source /opt/ros/humble/setup.bash
task3_sim/.venv/bin/python -c 'import sys,task3_sim; print(sys.version); print(task3_sim.__version__); assert sys.version_info[:2]==(3,10); assert task3_sim.__version__=="0.3.1"'
```

本次不改 C++ 桥，无需重新编译。不要复制 Windows 的虚拟环境或已生成 XML。若 `local/native/task3_official_state_bridge` 确实不存在，才重新执行 `bash task3_sim/build_official_bridge.sh`。

先在 Ubuntu 图形桌面终端单独验证一次：

```bash
bash task3_sim/run_official_task3.sh --preflight
bash task3_sim/run_official_task3.sh --show --viewer --max-seconds 240
```

启动顺序会打印 `native_processes → physics_feedback → sdk_connect → sdk_connected → standing_up → ready`。最先出现的是官方原生物理窗口；SDK 连接、站起确认后才创建 Python 图像/镜像窗口。成功后 Python 结果窗口保持显示，按 Esc/Q 或关闭 Python 窗口退出。若启动失败，程序会停止控制并关闭自己的原生窗口，原因保留在终端和本次 `summary.json/error.log/official_runtime/mc_ctrl.log`；不要把仍然运动的错误场景当作验收通过。

确认能显示机器狗并进入 `ready` 后，执行完整验收：

```bash
bash task3_sim/accept_official_ubuntu.sh
```

请回传最后打印的 `TASK3_RETURN_ARCHIVE ...tar.gz` 文件。原生验收需要三靶正确身份、一次扣弹、逐靶分数、真实关节运动和 SDK 调用账本等证据；不能仅看屏幕出现绿色靶或 `SUCCESS`。若单独运行仍报错，也可将该次日志打包：

```bash
TASK3_LAST="$(find task3_sim/runs -maxdepth 1 -type d -name 'official_[0-9]*' | sort | tail -n 1)"
test -n "$TASK3_LAST"
tar --exclude='*/official_runtime/mc' --exclude='*/official_runtime/zsibot_robots' \
    --exclude='*/official_runtime/simulate/build' \
    -czf "$TASK3_LAST-return.tar.gz" -C "$(dirname "$TASK3_LAST")" "$(basename "$TASK3_LAST")"
printf '%s\n' "$TASK3_LAST-return.tar.gz"
```

## 本地验证边界

修复的组显示、物理数组一致性、SDK 环境配置、启动失败记录、Linux 执行权限和验收拒绝错误证据均由本地契约测试检查。三方向图像与干扰贴片已进行实际 MuJoCo 渲染闭环，完整固定/随机回归结果与源码哈希见 `validation/official_fix031_local_checks.json`。原工程保护报告见 `validation/official_fix031_preservation.json`。

本次结果：230 项单元测试中 227 通过、3 项按平台条件跳过；15 场固定加 20 场预先保存的随机场景 **35/35 符合预期**，包括预先声明的缺靶、弹药不足、超时负例。97 发全部命中正确 ID、10 环，最大实际偏差 8.961 mm。独立默认命令行三靶成功并退出 0。55 个 Python 源码在正式回归前后哈希一致，保留的旧单靶 14 个文件/资产及入口函数 AST 未变；原工程 1,189 个受保护文件未变。

本机仍为 Windows，不能执行官方 Linux SDK/控制器。新版本能否完成官方原生站起、转向和逐靶射击，以上述 Ubuntu 回传为准；不将静态显示或运动学图像结果记为官方步态通过。
