# 任务三 0.3.8 更新包

发布：2026-10-23。面向 RK3588、64 位 Linux、CPython 3.10.12，提供独立环境安装、官方 ARM64 SDK 检查和真实 RTSP 的 AprilTag ID/角点识别；保留 x86 仿真源码。实物自动转向、瞄准和发射仍需状态、标定和设备协议接入。

- [代码更新包](task3_arm64_038_20261022.zip)
- [SHA256 校验文件](task3_arm64_038_20261022.zip.sha256)
- [板端完整部署与验收命令](RK3588_DEPLOY_038.md)
- [包内容核验记录](ARM038_PACKAGE_CHECK.json)
- [开发机验证记录](../../task3_sim/validation/arm038_local_checks.json)

将 ZIP 和 `.sha256` 放在同一目录，在板上执行（安装路径可调整）：

```bash
set -euo pipefail
sha256sum -c task3_arm64_038_20261022.zip.sha256
mkdir -p "$HOME/robotac_task3"
unzip -o task3_arm64_038_20261022.zip -d "$HOME/robotac_task3"
cd "$HOME/robotac_task3"
TASK3_SDK_ROOT="/实际的官方SDK根目录"
bash task3_sim/setup_hardware_ubuntu.sh --sdk-root "$TASK3_SDK_ROOT"
bash task3_sim/run_hardware_task3.sh --preflight \
  --sdk-root "$TASK3_SDK_ROOT" --output task3_sim/runs/rk3588_preflight.json
```

SDK 根目录需要含 `lib/zsl-1/aarch64/` 下的原版动态库和 cp310 绑定，ZIP 不包含这些原件。虚拟环境在板上重新创建，不上传 Windows/x86 `.venv`。完整文档包含系统依赖、SDK 仅导入检查、真实视频命令、输出核对和后续步骤。

更新包名称中的日期为构建日期 2026-10-22，GitHub 发布同步于 2026-10-23；文件保持原包字节和 SHA256。开发机 419 项测试通过、3 项平台相关跳过，不等于已经通过 RK3588 实板验收。
