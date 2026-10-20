# Ubuntu 任务三修复与验收（2026-10-19 fix1）

这次更新只覆盖 `task3_sim/` 自有源码，保留 `task3_sim/.venv/`、`runs/`、`local/`，不覆盖官方模型、SDK 或任务一、二工程。依赖版本未变，已经安装成功的虚拟环境可以继续使用。

## 日志说明了什么

用户回传的终端记录里有几个不同的问题：

| 现象 | 依据及含义 |
| --- | --- |
| `--headless` 没有 MuJoCo 窗口 | 预期行为；仍在后台真实渲染 RGB 图像，结果保存在 `runs/`。只有 `--viewer` 打开 MuJoCo 场景，`--show` 打开相机画面。 |
| 交互窗口约 2.6 秒后消失 | 旧版在单次发射完成后直接退出。`121905`、`121946` 两次记录完成 10 环且没有终端崩溃信息；单凭窗口消失不能判断崩溃。修复版完成后保留观察窗口。 |
| `TASK3_SUCCESS` 后出现 `malloc(): mismatching next->prev_size (unsorted)`、`Aborted (core dumped)` | 这是原生内存错误。`121755` 的无窗口运行及集成验收第三场景均出现过；并非正常退出，也不是模型路径错误。结果文件写出成功不能证明进程正常退出。 |
| `Error, more than one new minima found.` | 旧 AprilTag 原生位姿求解器的诊断输出；它没有阻止这两次运行命中，但应与上述堆损坏分开处理。 |
| Ubuntu 提示 `python3.10 has stopped unexpectedly` | 系统在报告 Python 进程崩溃，不能据此认定 Python 安装坏了。旧报告弹窗也不能代替本次运行退出码。 |
| `/absolute/path/to/xgb/xgb.xml` 不存在 | 文档占位路径被直接执行。现有工程无需 `--model-root` 参数，默认会找到原模型。 |
| `sha256sum` 查找文件名末尾的 `$'\r'` | 旧版校验文件在 Windows 写成了 CRLF。修复版校验文件固定 LF；不是因为 ZIP 文件名应带回车。 |

回传的 `summary.json` 显示旧版默认场景确实命中 10 环，`model_sources_unchanged=true`。这说明视觉闭环已经在 Ubuntu 执行，但仍需处理进程的原生崩溃与交互窗口生命周期。

## 本版修复内容

版本 `0.1.1` / `fix1` 包含以下修改：

- 核查固定版本 `pupil-apriltags==1.0.4.post11` 的源码后，确认其析构顺序存在释放后访问：先释放 Tag family，再销毁仍引用该 family 的 detector。自有 `perception.py` 改用受管理的 detector，先销毁 detector，再释放 family，并保证重复关闭安全。没有修改安装目录里的第三方包或官方代码。
- 继续使用真实 AprilTag 检测得到 ID 和角点，由 OpenCV 的平面 PnP 求公制位姿；不再调用会输出 `more than one new minima found` 的原生位姿求解路径。目标位置仍来自图像，没有引入场景真值控制。
- 交互运行完成后保留观察窗口；退出时按资源依赖顺序关闭窗口、渲染器和检测器。若清理异常，记录 `CLEANUP_ERROR`，避免把资源清理失败标成正常成功。
- 启动脚本保存完整终端输出与真实进程退出码，启用 faulthandler；修复 SHA-256 校验文件的 CRLF 换行，并确保更新包中的 Bash 脚本使用 LF。

第三方析构缺陷已经由源码确认，且与回传的堆损坏现象相符；尚未拿到用户那两次崩溃的原生调用栈，因此不能断言它是那两次崩溃的唯一来源。修复是否消除目标 Ubuntu 环境的崩溃，仍需用下方步骤验收。

修复版开发机验证：33 项单元测试、六种真实渲染场景全部通过。另在 Windows 开发机实际执行 `--show --viewer --realtime --hold-seconds 2`：命中 10 环，结果为 `SUCCESS`，`shutdown_completed=true`、`perception_closed=true`、无清理异常，观察 2 秒后正常退出，进程退出码为 0。该次任务阶段实际约 2.579 秒，对应 2.55 秒仿真时间。测试摘要见 [fix1_validation.json](validation/fix1_validation.json)。这只覆盖该开发机和该交互命令；Ubuntu 桌面与驱动环境仍需复验。

## 更新现有 Ubuntu 工程

把下面两个新文件放到 `~/Downloads`：

- `task3_single_target_20261019_fix1.zip`
- `task3_single_target_20261019_fix1.zip.sha256`

然后执行：

```bash
cd "$HOME/Downloads"
sha256sum -c task3_single_target_20261019_fix1.zip.sha256
unzip -l task3_single_target_20261019_fix1.zip
unzip -o task3_single_target_20261019_fix1.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
```

校验应显示 `OK`，压缩包所有文件应位于 `task3_sim/`。如果第一步失败，先核对下载的两个文件，校验成功后再解压。本修复不需要再次执行 `apt`、`pip` 或 `setup_ubuntu.sh`，也不需要删除虚拟环境或重装 Python、ROS、显卡驱动。

## 先验收进程，再查看窗口

先运行无窗口检查：

```bash
bash task3_sim/run_task3.sh --headless
echo $?
```

应当有 `TASK3_SUCCESS`、`ring=10 ammo_remaining=4`，最后显示 `TASK3_PROCESS_EXIT code=0`，随后 `echo $?` 输出 `0`。本次 `summary.json` 还应包含 `shutdown_completed=true`。无窗口模式本身不会弹出 MuJoCo 页面。

查看 MuJoCo 场景和相机画面：

```bash
bash task3_sim/run_task3.sh --show --viewer --realtime
echo $?
```

完成后程序保存结果、停止机器人并保留窗口。关闭场景窗口、在相机窗口按 Esc 或按 Ctrl+C 退出观察；已经成功完成的结果应保持成功，随后退出码为 0。若只需要 MuJoCo 场景可用：

```bash
bash task3_sim/run_task3.sh --viewer --realtime
```

如果希望观察 10 秒后自动退出：

```bash
bash task3_sim/run_task3.sh --show --viewer --realtime --hold-seconds 10
```

`--hold-seconds 0` 恢复任务完成后立即退出。观察阶段不会再次发射，也不会继续运行自主控制。关闭发生在任务完成前仍属于取消。

再运行自动测试和六种真实渲染场景：

```bash
task3_sim/.venv/bin/python -m unittest discover -s task3_sim/tests -v
MUJOCO_GL=egl task3_sim/.venv/bin/python -X faulthandler -m task3_sim.validate
echo $?
```

应完成全部六场，顶层 `validation.json` 中 `all_passed=true`，最后退出码为 0。阻挡、Tag 不可见、无弹药场景应按照各自预期停止或脱靶；验收通过不表示这些场景都应该命中。

## 如果修复后仍然崩溃

启动脚本会打印本次 `TASK3_TERMINAL_LOG` 的完整路径。该目录中的 `terminal.log` 保存标准输出、标准错误及进程退出码；`fault.log` 保存 Python faulthandler 信息，正常情况下可以为空。原生崩溃不一定有普通 Python 的 `error.log`。

可在工程根目录查看最新一次启动脚本运行的记录：

```bash
TASK3_LAST_RUN="$(ls -td task3_sim/runs/run_*/ | head -n 1)"
cat "$TASK3_LAST_RUN/terminal.log"
cat "$TASK3_LAST_RUN/fault.log"
cat "$TASK3_LAST_RUN/summary.json"
```

不要只看 `summary.json` 的 `SUCCESS`：若终端仍有 `Aborted`、`Segmentation fault`，或 `TASK3_PROCESS_EXIT` 为 134/139，就仍然存在原生崩溃。保留该次整个运行目录及终端记录便于定位；不要靠关闭 Ubuntu 崩溃报告来判断修好了。

修复版在开发机的测试与 Ubuntu 真实桌面/驱动验收是两个环节。目标机是否消除原生退出崩溃，以以上命令的实际退出码和日志为准。
