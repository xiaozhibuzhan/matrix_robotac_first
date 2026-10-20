# 图片靶与背景更新（0.1.2，2026-10-19）

用户确认：彩色纹理图片作为背景，带 AprilTag 的绿色图片作为固定靶。当前阶段进入单靶外观与感知扰动验证；Ubuntu 首个里程碑的评价见 [Ubuntu 运行评估](UBUNTU_REVIEW_20261019.md)。

## 本版的行为

- 图片背景是位于靶后方的 MuJoCo 纹理墙，固定靶同样通过三维平面纹理真实渲染。不会在检测结果或相机画面上合成目标位置。背景只参与显示，不参与射线碰撞；真实障碍仍参与最近表面碰撞。
- 原始素材在 `assets/background_source.jpg` 和 `assets/fixed_target_source.png`，原图和原有水印保留。靶图中的标记实际为 `tag36h11` **ID 2**，继续使用原图片中的标记。
- 原图片标记宽高有约 7% 差异。派生纹理对整张图做单应矫正，使黑色标记为正方形。Tag 到环心的偏移和打印环线由同一映射标定；打印环线按椭圆计分，并裁切到绿色区域，避免白框和标记区域获得环分。
- **黑色 Tag 边长 0.16 m 是当前仿真假设**，并非用户照片提供的实测值。由此得到的靶板约 0.645 × 0.682 m，不能用作正式赛事尺寸。原图只有 320 × 319 像素，环线拟合与绿色区域边界也只是近似。来源、哈希、变换和几何见 `assets/provenance.json`。
- 新增 `config/photo_target.json`；原 `config/single_target.json` 和无参数启动的合成场景保持原样。默认控制门限、图像新鲜度与稳定门控不因换图而放宽。
- 修正成功后 Ctrl+C 的日志管道问题：旧 `tee` 与 Python 同时收到 SIGINT，日志管道提前关闭可导致最终输出失败、退出码 1。新版 `tee -i` 保持日志管道，成功后退出仍应为 0，成功前取消仍为非零；原生崩溃 134/139 保留。清理期间的重复 Ctrl+C 延后处理，避免中途放弃资源清理。
- MuJoCo 默认隐藏左右面板，并将观察相机置于可看见机器狗、射线和靶的方向。这个第三人称视角与用于定位的机载相机互相独立。
- 增加可复现的曝光、伽马、模糊与噪声选项。它们处理真实渲染输出，属于简单的传感器压力模型，并非实机相机标定模型。

代码、素材和结果仍在 `task3_sim/` 内，官方工程、SDK、模型和任务一、二入口均不修改。更新包不包含 `.venv/`、`runs/`、`runs_ubuntu/`、`local/`，继续使用已安装成功的依赖。

## 更新 Ubuntu

将 `task3_single_target_20261019_appearance.zip` 与同名 `.zip.sha256` 放到 `~/Downloads`，先执行校验，确认显示 `OK` 后再解压：

```bash
cd "$HOME/Downloads"
sha256sum -c task3_single_target_20261019_appearance.zip.sha256
```

```bash
unzip -o task3_single_target_20261019_appearance.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
bash task3_sim/run_task3.sh --headless --config task3_sim/config/photo_target.json
echo $?
```

成功应为 `TASK3_SUCCESS`、目标 ID 2、10 环，最后 `TASK3_PROCESS_EXIT code=0`，随后 `echo $?` 为 `0`。无需重新执行 pip/setup，不需要更改官方模型路径。

查看场景及相机画面：

```bash
bash task3_sim/run_task3.sh --config task3_sim/config/photo_target.json --show --viewer --realtime
echo $?
```

成功后保留窗口，按相机窗口 Q/Esc、关闭窗口或 Ctrl+C 退出。请特别验收成功后 Ctrl+C 应返回 0；成功前取消应返回 2。只想定时观察可增加 `--hold-seconds 10`。MuJoCo 面板默认隐藏，需要时可用该查看器的界面切换操作显示。

## 自动验收与适用范围

```bash
task3_sim/.venv/bin/python -m unittest discover -s task3_sim/tests -v
MUJOCO_GL=egl task3_sim/.venv/bin/python -X faulthandler -m task3_sim.validate
MUJOCO_GL=egl task3_sim/.venv/bin/python -X faulthandler -m task3_sim.validate_appearance
echo $?
```

第一套真实渲染验证保留原合成靶六场景。第二套包含 15 个预先规定的图片靶场景：基准、背景变暗/变亮/平铺、曝光变暗/变亮、模糊、噪声、组合扰动、左偏起点、斜靶、更远的靶、射线遮挡、靶不可见、无弹药。

图片靶基准要求 10 环、实际脱靶不超过 10 mm；可见靶扰动用例要求至少 9 环、脱靶不超过 20 mm。成功场景应只消耗一发且清理成功；遮挡应报告未命中，无靶与无弹不发射。这是本轮预先规定的调试门槛，**不是赛事门槛**，也不能与原场景的较小环半径直接比较命中分数。结果保存在本次 `runs/appearance_.../validation.json`，包含每场实际误差、检测帧比例、期望值和失败检查项。

15 个固定场景仅覆盖单张靶图、单张背景和指定噪声种子，不能给出统计成功率或实机可靠性结论。开发机验证记录见 [本版验证摘要](validation/appearance_validation.json)；图片版在用户 Ubuntu 的运行验收须另行执行上述命令，不能把上一版 Ubuntu 的成功记录算作新版验证。

## 可调参数

先将配置复制到 `task3_sim/local/`，避免更新覆盖现场调参。图片路径相对 `task3_sim/` 包目录，因此复制配置到其他目录后仍能解析，无需 Windows 盘符或剪贴板临时路径。

```json
"appearance": {
  "target": {"kind": "image", "asset": "assets/fixed_target_rectified.png", "brightness": 1.0},
  "background": {
    "asset": "assets/background_source.jpg", "brightness": 1.0,
    "tile": [1.0, 1.0], "width_m": 12.0, "height_m": 8.0,
    "distance_behind_target_m": 1.0
  },
  "lighting": {"ambient": 0.5, "diffuse": 0.5},
  "camera_effects": {
    "exposure": 1.0, "gamma": 1.0, "blur_sigma_px": 0.0,
    "noise_std_u8": 0.0, "seed": 0
  }
}
```

靶与背景采用自发光材质，以保留所给图案的颜色；`lighting` 主要影响机器人、地面和阴影。要改变图片靶的明暗，请调整 `target.brightness` 或相机 `exposure`；要改变背景明暗，调整 `background.brightness`。背景 `tile` 改变纹理重复次数，作为受控变化使用；原图长宽比和墙面长宽比不一致时会有拉伸，背景不用于米制标定。

下一轮建议扩大为多种真实场地背景、干扰 Tag、预先保存种子的姿态/距离组合，记录失败图和误射率；随后验证掉帧与视频延迟，再扩展三固定靶、补射与正式计分。运动学基座仍未验证真实四足步态，也不能把理想相机内外参下的毫米级成绩直接用于实机精度承诺。
