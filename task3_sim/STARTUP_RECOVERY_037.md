# 任务三 0.3.7：缺少状态桥的启动恢复

更新：2026-10-21。本次只改任务三启动检查和迁移说明，保留 0.3.6 的第五发修复、识别、运动、瞄准、弹药与计分策略。

## 本次日志结论

`update/task3/log/` 中四轮 0.3.6 运行（`official_20261021_012051_4148`、`official_five036_20261021_012124`、`official_20261021_012336_4406`、`official_20261021_012532_4560`）均报告：

```text
Missing state_bridge: /home/xiaozhibuzhan/robotac/matrix_robotac_first/task3_sim/local/native/task3_official_state_bridge
TASK3_PROCESS_EXIT code=2
```

四轮摘要均为 `ERROR`、`shutdown_completed=true`、`cleanup_errors=[]`，`fault.log` 均为空，射击数均为 0。预检 `official_preflight_20261021_012035_7920f8b8/preflight.json` 已显示 `ready=false`，唯一错误也是缺少状态桥。这是缺依赖后的主动退出；预检失败应先处理，再启动任务。

状态桥是任务三自有的只读程序，读取官方 eCAL RobotState，向本机渲染和控制适配层交付真实状态。它使用官方原版头文件和库在目标 Ubuntu 编译，输出到 `task3_sim/local/native/`。更新包不带本机编译产物，新建工程、删除旧工程或清理 `local/` 后需要重建。此前 `update/task3/README.md` 新工程段落漏列编译步骤，本次已补齐。

## 先恢复现有 0.3.6，无需先换包

先关闭本次任务三；其他官方仿真、任务一二或 demo 若仍在运行，按原流程退出并确认停止。在 **Ubuntu 图形桌面上的 Bash 终端**执行，路径改成实际工程：

```bash
cd "$HOME/robotac/matrix_robotac_first"
source /opt/ros/humble/setup.bash
bash task3_sim/build_official_bridge.sh
```

只构建 `task3_sim/native/` 的自有状态桥，不重编译或修改官方 SDK、运控和模型。脚本最后自动执行完整依赖预检，**最后必须为 `"ready": true` 且退出码 0**；只看到 CMake `Built target` 还不够。

若编译失败，停在这一步。仅当提示对应编译工具缺失时补装：

```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config libprotobuf-dev
```

若缺 `/usr/include/robot_sdk.pb.h`、eCAL CMake 包或 `librobot_sdk`，按工程内 `task3_sim/OFFICIAL_DYNAMICS_030.md` 的原版 `.deb` 依赖步骤补齐，不生成替代 protobuf 定义。若提示其他官方进程、端口占用或 DISPLAY 缺失，按预检报告处理对应项目。使用自定义 `bridge_executable` 时，还需核对它指向本机正确的桥文件；默认构建输出为上述 `local/native/` 路径。

编译与预检成功后启动基础三发：

```bash
bash task3_sim/run_official_task3.sh \
  --config task3_sim/config/multi_official.json \
  --show --viewer --max-seconds 240
```

五发巡回诊断使用：

```bash
bash task3_sim/run_official_task3.sh \
  --config task3_sim/config/multi_official_five_shots.json \
  --show --viewer --max-seconds 240
```

成功后应有三条或五条 `TASK3_SHOT`，五发顺序 `2,5,9,2,5`。任务结束后窗口保留，关闭结果窗口再检查最后 `TASK3_PROCESS_EXIT code=0`，并对本轮实际目录执行 `task3_sim/.venv/bin/python -m task3_sim.validate_official <实际运行目录>`。预检通过只说明依赖满足，命中和源文件完整性仍需本轮 Ubuntu 运行证据。

## 0.3.7 改进与安装

运行入口在创建机器人适配器、渲染资源、检测器和窗口之前执行只读依赖检查，保存 `startup_preflight.json`，并写入摘要的 `official_startup_preflight`。缺桥时额外输出：

```text
TASK3_REPAIR cwd=... command=bash task3_sim/build_official_bridge.sh
```

检查失败仍返回 2，不自动安装系统包、不启动官方进程或发送 SDK 命令。依赖齐全时保留原来的“图形预热在原生启动之前”顺序；正式启动仍在进程锁内重新检查，拒绝后来出现的冲突。

把新 ZIP 和同名 `.sha256` 复制到 Ubuntu 同一下载目录，在该目录逐条执行；任何一步失败先停止：

```bash
sha256sum -c task3_startup_fix_037_20261021.zip.sha256
unzip -o task3_startup_fix_037_20261021.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
task3_sim/.venv/bin/python -c 'import task3_sim; print(task3_sim.__version__); assert task3_sim.__version__=="0.3.7"'
```

已有虚拟环境保留。**新包也不含桥的可执行文件**，缺少时仍须运行前面的编译命令；首次安装或迁机，先 `setup_ubuntu.sh` 创建本机环境，再编译桥。旧 0.3.6 ZIP 和本次用户回传日志保持原样。

本地 25 项相关回归检查通过，覆盖缺桥的失败时机、诊断/修复指令、正常依赖放行、原有生命周期、源文件保护和包边界。Windows 未执行官方 Linux SDK/ELF，不能据此宣称目标 Ubuntu 已恢复。详细本地记录见 `task3_sim/validation/startup037_local_checks.json`。
