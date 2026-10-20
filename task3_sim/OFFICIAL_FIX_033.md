# 任务三 0.3.3：启动后窗口关闭修复

本次只修改 `task3_sim/`。官方 SDK、ELF、加密步态策略、原模型、任务一二及旧单靶入口保留，不需要重新编译状态桥或安装依赖。

## 本次终端记录说明了什么

已读取用户提供的 `office_320_error`。其中 0.3.2 的可视运行 `official_20261020_000221_3437` 和验收 `official_acceptance_20261020_000307_3668/native_nominal` 均完成 SDK 连接及站起，约 15 秒时首次出现 WAIT 后退出。验收前的 Ubuntu 312 项单元测试全部通过，但原生场景没有进入射击阶段。

两次 Python 错误分别是：

- `official physics publication is stale, future, or uses a discontinuous clock`。
- `Out of range float values are not JSON compliant`。

第一条旧错误将过期与时钟异常混在一起，无法仅凭该行确定当时属于哪一种。本地复现确认了一个实际缺陷：刚被桥接收、但源发布已经超时的合法旧包会被永久锁为故障。第二条已明确复现：无效反馈的时间戳为 `-inf`，严格 JSON 日志无法写入；SDK 读取失败时还可能包含 NaN 向量。控制器本来已安全 WAIT，却被诊断写入异常终止。

终端记录显示 Python 捕获异常、输出 `TASK3_ERROR`、退出码为 2，窗口随资源清理关闭。原生进程清理前仍活着，`mc_ctrl=-6` 发生在明确请求终止之后，不能据此判断为本次窗口关闭的先因。当前本地 `runs_ubuntu` 尚无这两场 Oct20 的原始文件，以上判断来自完整终端及原 0.3.2 源码复现，不声称已经读到缺失的逐帧数据。

## 修正后的行为

1. 数据包先做结构、有限值、四元数、序号和时钟校验。仅合法过期包会丢弃，后续真实新包可恢复；保留原发布时间，不重打时间戳。未来时间、源时钟或序号回退、格式损坏、非有限物理数据仍拒绝并退出。
2. 渲染器、AprilTag 检测和观察窗口在官方进程启动前初始化。预热图全部丢弃，既不进入控制，也不进入计分。
3. 渲染同步遇到短时缺失反馈，先停止当前 SDK 控制、取消指令租约和旧稳定窗口，再最多等待 **2 秒**。只接受停止时刻之后的新鲜真实发布；持续断流或进程退出仍保存错误并结束。这个等待上限不放宽 **0.15 秒** 的图像/反馈新鲜度。
4. 每次恢复均撤销旧图像权限和射击资格，排除恢复前排队图像。WAIT、PROBING、LOST 等安全状态只停止已有控制，不因旧图触发新的 standUp 或模式切换。有效新图触发的正式模式切换仍遵守官方站起流程。
5. 射击前遇到过期图像、无效反馈或物理源断流，立即撤销保持并拒射，**不等待恢复后沿用旧 READY 开火**。8 mm 瞄准阈值、至少 0.4 秒稳定窗口、同 ID 核验、实际 SDK 调用账本和有限弹药事务保留。
6. 仅在诊断落盘时将 NaN/∞ 写为 JSON `null`，另记录 `telemetry_serialization.nonfinite_paths/nonfinite_values`。控制对象仍保留无效标志和原始数值，不会以零姿态、新时间或虚假分数代替。事件新增反馈失败原因，summary 新增旧包分类和恢复过程。

终端可能短暂显示 `WAIT` 或 `TASK3_STATE_RECOVERY`，恢复后必须重新取得真实图像并积累稳定窗口。持续出现恢复或反复 WAIT 仍需通过回传日志检查性能和源时间，不可视为已通过比赛验收。

## Ubuntu 更新

先结束上一场任务三，将本版 ZIP 和 `.sha256` 放在同一目录，在该目录执行：

```bash
sha256sum -c task3_official_dynamics_033_20261020.zip.sha256
unzip -o task3_official_dynamics_033_20261020.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
source /opt/ros/humble/setup.bash
task3_sim/.venv/bin/python -c 'import sys,task3_sim; print(sys.version); print(task3_sim.__version__); assert sys.version_info[:2]==(3,10); assert task3_sim.__version__=="0.3.3"'
bash task3_sim/run_official_task3.sh --preflight
bash task3_sim/run_official_task3.sh --show --viewer --max-seconds 240
```

保留 Ubuntu 的 `task3_sim/.venv` 和 `task3_sim/local/native/`；不要复制 Windows 虚拟环境或旧运行生成的 XML。若这里的工程目录不同，只需替换解压与 cd 的路径。

可视运行退出后，再执行完整验收：

```bash
bash task3_sim/accept_official_ubuntu.sh
```

回传最后打印的 `TASK3_RETURN_ARCHIVE ...tar.gz`，同时保留可视运行所在的 `runs/official_.../`。验收失败也会导出证据。完整验收包括单元测试、预检、三场官方动力学和实际渲染视觉回归；是否成功射击以逐发账本、原生状态、正确靶 ID 与验收报告共同判断。

## 验证范围

开发机验证详见 `validation/official_fix033_local_checks.json`。恢复与日志测试使用明确标注的注入 SDK/源数据，只验证异常链路；实际渲染回归仍使用运动学视觉后端。这些结果不能替代 Ubuntu 上官方 ELF/SDK 的运行，更不能证明实物已验收。0.3.2 PI 精瞄尚未获得官方动力学成功射击证据，本版需继续完成该项验收。

本地完整单元测试 344 项：341 通过、3 项 POSIX 信号测试按 Windows 平台跳过，零失败。62 份 Python 源码通过 Python 3.10 语法检查，5 份 Bash 脚本通过语法检查。受保护的原工程 1,189 个文件无变化；另核验旧单靶文件、入口函数、官方策略绑定文件和物理 ELF。

本版实际渲染回归为 15 场固定与 20 场预先声明的随机场景，35/35 符合预期，保留缺靶、无弹、超时负例。97 发均对应正确靶 ID、10 环，最大实际偏差约 8.961 mm。62 份 Python 源码在回归前后哈希一致；独立运动学命令行也正常退出，三靶各 10 环、余弹 2 发。详细场景见 `validation/directional_actual_render_033.json`。
