# RK3588 板端适配与部署（0.3.8）

更新：2026-10-22。部署目标为机器狗板载 RK3588、64 位 Linux、CPython **3.10.12**。系统发行版尚未收到确认，安装前以板上检测结果为准。

本版完成架构选择、独立依赖安装和**真实 RTSP 图像的 AprilTag 身份识别**。官方 SDK 提供 aarch64/cp310 库，Python 3.10.12 与其 ABI 匹配；官方 x86 仿真 ELF 仍只能在 x86 工作站执行。原官方库、控制器、模型和任务一、二不做修改。

此入口尚未串起实物自动转向、瞄准及发射。视频只输出 ID、像素角点和质量信息，不输出未经标定的距离/靶心，不伪造相机采集时间。`ready=true` 只表示本次依赖检查通过，`autonomous_hardware_ready` 仍为 `false`。SDK 导入检查不会构造 `HighLevel`，不调用 `initRobot`，不获取控制权。

## 需要传什么

将下列两件自有更新文件传到板上，例如 `~/Downloads/task3_038/`：

- `task3_arm64_038_20261022.zip`
- `task3_arm64_038_20261022.zip.sha256`

ZIP 解压后是 `task3_sim/`，包含兼容保留的仿真源代码，但板端安装只安装 `requirements_hardware*.txt`。不需要上传 x86 虚拟环境、仿真模型、x86 状态桥、ROS/UE 仿真进程或历史 `runs/`。

**官方 SDK 不包含在这个代码 ZIP 中。** 保留板上主办方原始 SDK；可以用 `--sdk-root` 指定它的根目录，也可将原版 SDK 复制到新工程的 `deps/zsibot_sdk/`。`--sdk-root` 指向含有 `lib/` 的根目录，不是 `.so` 所在目录。只读识别/预检需要原版 ZSL-1 的这两个文件：

```text
<SDK_ROOT>/lib/zsl-1/aarch64/libmc_sdk_zsl_1_aarch64.so
<SDK_ROOT>/lib/zsl-1/aarch64/mc_sdk_zsl_1_py.cpython-310-aarch64-linux-gnu.so
```

开发机原件 SHA256：

```text
bacd06930fc054be431e5dd5363c38146fc69796b280356504c7157d19b3919b  libmc_sdk_zsl_1_aarch64.so
1e7b53ffeefc7914f224686f675d2caf610c4a536e660ed5099e88e548817c2c  mc_sdk_zsl_1_py.cpython-310-aarch64-linux-gnu.so
```

这是本次核验原件的指纹，不要求替换主办方后来发布的版本。其他版本须另做兼容核对。不要将 x86 库改名放到 aarch64 文件夹；预检会检查 ELF 的机器类型，而不只看名字。本版型号配置为 **ZSL-1**，不能据 RK3588 CPU 推断其他狗型号也兼容。

## 在板上解压与安装

以下命令全部在 **RK3588 终端**执行。示例使用独立目录 `~/robotac_task3`，也可解压到现有工程根目录；ZIP 不覆盖 `deps/`、`src/` 或现场 `local/`。保留原工程备份。

```bash
set -euo pipefail
cat /etc/os-release
uname -m
python3 --version
# 应为 aarch64（arm64 别名也接受）、Linux、CPython 3.10.x；32 位 armv7l 不支持。

mkdir -p "$HOME/robotac_task3"
cd "$HOME/Downloads/task3_038"
sha256sum -c task3_arm64_038_20261022.zip.sha256
unzip -o task3_arm64_038_20261022.zip -d "$HOME/robotac_task3"
cd "$HOME/robotac_task3"
```

先把官方 SDK 放到 `~/robotac_task3/deps/zsibot_sdk/`，或者将下面 `TASK3_SDK_ROOT` 改为板上实际官方 SDK 根目录。后续每个终端都要重新设定这个变量；脚本不改动官方环境配置：

```bash
TASK3_SDK_ROOT="$HOME/robotac_task3/deps/zsibot_sdk"
test -f "$TASK3_SDK_ROOT/lib/zsl-1/aarch64/libmc_sdk_zsl_1_aarch64.so"
test -f "$TASK3_SDK_ROOT/lib/zsl-1/aarch64/mc_sdk_zsl_1_py.cpython-310-aarch64-linux-gnu.so"
```

若系统确认为 Ubuntu/Debian 且缺少以下依赖，再安装；不要替换系统 Python 或升级官方 SDK：

```bash
sudo apt-get update
sudo apt-get install python3.10-venv python3.10-dev build-essential libgl1 libglib2.0-0
```

如果系统软件源没有 `python3.10-*`，先根据 `/etc/os-release` 配好对应开发包，不能改用 Python 3.11 来加载 cp310 库。

```bash
bash task3_sim/setup_hardware_ubuntu.sh --sdk-root "$TASK3_SDK_ROOT"
```

这会新建 `task3_sim/.venv_hardware/`，保留仿真的 `.venv/`。ARM 上 NumPy/OpenCV 使用 aarch64 wheel；`pupil-apriltags==1.0.4.post11` 没有 Linux ARM64 wheel，因此在本板编译原版源码，使用独立环境中的 CMake 3.x、Ninja 和最多两个并行构建任务。没有安装 MuJoCo，也不启动 ROS/官方控制器。首次安装需要网络和编译时间。

## 三步板端检查

每次 `--output` 使用新路径，已有文件/非空目录会被拒绝，防止覆盖记录。

```bash
TASK3_CHECK_DIR="task3_sim/runs/rk3588_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TASK3_CHECK_DIR"
set -o pipefail

# 1. 架构、两份原版 ELF、动态依赖、NumPy/OpenCV、AprilTag 原生检测与释放
bash task3_sim/run_hardware_task3.sh --preflight \
  --sdk-root "$TASK3_SDK_ROOT" --output "$TASK3_CHECK_DIR/preflight.json" \
  2>&1 | tee "$TASK3_CHECK_DIR/preflight_terminal.log"

# 2. 仅导入官方 Python 模块；不构造机器人客户端或连接机器人
bash task3_sim/run_hardware_task3.sh --sdk-import-check \
  --sdk-root "$TASK3_SDK_ROOT" --output "$TASK3_CHECK_DIR/sdk_import.json" \
  2>&1 | tee "$TASK3_CHECK_DIR/sdk_import_terminal.log"
```

两项均应退出码 0、`ready=true`，第二项 `sdk_imported=true`，两项 `sdk_initialized=false`。若 `ldd` 出现 `not found`，根据缺失库补齐同架构系统依赖；不要从 x86 机器复制动态库。

第三步使用官方视频服务的**真实地址**。官方格式为 `rtsp://<实际相机地址>:8554/test`；板载运行也不能直接假定服务监听 `127.0.0.1`。填入实际地址后执行：

```bash
read -r -p '输入实际 RTSP 地址: ' TASK3_RTSP_URL
bash task3_sim/run_hardware_task3.sh --video-only \
  --rtsp "$TASK3_RTSP_URL" --seconds 30 \
  --output "$TASK3_CHECK_DIR/video" \
  2>&1 | tee "$TASK3_CHECK_DIR/video_terminal.log"
```

SSH 默认不加 `--show`，直接保存图像和日志；在板上图形桌面才加 `--show`，按 `q` 或 Esc 结束。没有窗口不代表视频没有运行。此版本使用 OpenCV/FFmpeg CPU 解码，不假设已经接入 RK3588 的 MPP/RGA 加速；先测实际帧率再决定是否优化。

正常结束应显示 `OBSERVATION_COMPLETED`，并保存：

- `video/first_frame.jpg`：真实相机首帧；检查清晰度、方向、畸变与 Tag 大小。
- `video/detections.jsonl`：每帧 ID、像素角点、质量、帧尺寸和本机解码时间。
- `video/summary.json`：帧数、见过的 ID、退出状态和资源释放结果。

`OBSERVATION_COMPLETED` 表示成功持续读帧，不等于一定认到靶；还需查看 `observed_ids` 和首帧。断流/超时退出码 2，保留已识别帧，释放相机与检测器，不自动发送任何动作。流中没有可信的采集时间时记录 `source_acquisition_timestamp_s=null`；解码完成时间不作为实物运动/发射的新鲜度依据。

默认 `target_ids=[2,5,9]` 是示例。现场配置使用独立副本：

```bash
mkdir -p task3_sim/local
cp -n task3_sim/config/hardware_rk3588.json task3_sim/local/hardware_rk3588.json
# 修改 local 文件的 target_ids 为现场三个固定靶的真实 ID
# 在 --video-only 命令追加：--config task3_sim/local/hardware_rk3588.json
```

未列入配置的 Tag 仍记录，但 `declared_target=false`，便于核对干扰。检测不到 Tag 时先看首帧、实际家族、光线/尺寸，不把仿真中默认相机内参直接用于实物。

## 后续接入与平台边界

板端完成上述三步并回传整个 `TASK3_CHECK_DIR` 后，下一步才是实机相机内参与畸变、Tag 实测尺寸/靶心映射、相机/发射器外参和机体控制响应核对，再接可信实机状态、小角度转向停车、单靶无发射瞄准、真实单发及三靶巡回。已有仿真目标调度与弹药账本可复用；实物状态来源、控制参数和发射协议必须实际接入。当前仿真的有界 PI 参数与射线成绩不能当作实物验收或成绩。

RK3588 使用 `setup_hardware_ubuntu.sh` / `run_hardware_task3.sh`。`setup_ubuntu.sh` / `run_official_task3.sh` / `build_official_bridge.sh` 继续用于 x86 仿真工作站；板端缺少这个仿真状态桥是正常的，不能通过删掉架构保护来运行它。不要运行包含跳跃/后空翻的官方完整 demo 作为联通测试。

离线部署要提前在同架构、同 Python/系统环境准备 wheel 缓存，包括在 ARM 构建的 AprilTag wheel，并实际断网试装；Windows/x86 `.venv` 无法移植。原版 SDK/机器人系统仍由主办方工程提供。

本次开发机验证涵盖架构与 ABI 错配拒绝、ARM/x86 库选择、SDK 仅导入不实例化、真实 Tag 图像识别、流失败与释放、仿真回归和打包边界。**没有在 RK3588 实板执行编译或机器人动作**；实际板端验收以本节输出为准。

官方参考：[部署说明](https://github.com/zsibot/genisom_l1_sdk_old/blob/main/docs/deploy.md)、[视频说明](https://zsibot.github.io/genisom_l1_sdk_old/video)。
