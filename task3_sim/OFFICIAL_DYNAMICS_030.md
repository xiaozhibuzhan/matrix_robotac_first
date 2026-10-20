# 任务三 0.3.0：按方位切靶与官方四足动力学接入

> 历史版本说明。0.3.0 的 Ubuntu 回传已暴露 SDK 绑定、机器狗显示与默认干扰靶问题，现已发布 [0.3.1 修复与运行命令](OFFICIAL_FIX_031.md)。更新时使用 0.3.1 包；下文保留架构和首次依赖安装参考。

这次更新只写入 `task3_sim/`。任务一、二、官方模型、SDK、步态控制器和原启动脚本保持原件；不调用会改配置和全局结束进程的 `run_sim.sh`。旧单靶入口保留。

## 已确认的 Ubuntu 记录

最新回传为 **0.2.1 三方向、四面背景版本**，不是旧的一排三靶：11 固定场景 + 20 随机场景 **31/31 符合预期**，85 发全部命中正确 ID、10 环，总计 850 分，最大偏差 8.961 mm。27 场 SUCCESS，另外 1 场 PARTIAL、2 场 NO_AMMO、1 场 TIMEOUT 是预先声明的负例。三次独立运行的进程退出码均为 0，故障日志为空。详细来源见 `validation/ubuntu_three_directions_021_review.json`。

旧日志同时证实：后两靶各逆时针绕约 270°。这些记录使用运动学底盘，不能据此声称官方四足步态已通过。

## 切靶算法

默认 `config/multi_fixed.json` 改为 `mission.search_mode=fixed_directions`：

- 三个方向先验相对首次有效反馈的机身朝向为 `0、+π/2、−π/2`；任务层只有待射击 ID 和 Tag/相机标定，不接收靶心世界坐标，也没有 ID 到方向的预设映射。
- 从真实图像识别所有合法 ID，将 Tag 位姿、相机外参与**图像采集时刻对应的反馈姿态**转换为方位缓存。延迟图像使用反馈插值；跨模式等待的盲区禁止插值。缓存按时间和位移失效。
- 已见目标按最短角双向转向；未见目标从三个尚未被其他 ID 占据的方向中选最近方向，停车等待新画面探查。探查完仍缺失时停车，按原期限标记失败后处理下一靶。
- 方位缓存只帮助导航；每次射击仍要求当前目标 ID 的新鲜、高质量图像、稳定姿态和独立射线命中判定。已完成目标不再射击，每发弹药有唯一目标和计分记录。

默认 ID2 在左、ID5 在前、ID9 在右，配置可以改变 ID 分配和射击顺序。ID42 移到前方旁侧，保证新算法真的遇到并忽略干扰 Tag。四面图片背景保留。左右靶相差 180°时按反馈选择最短角；小幅瞄准偏移可能使这一段经过后半区，但不会为找靶额外绕一圈。

`summary.mission.search.observed_bearings`、各靶 `search_probes` 和 `search_travel_rad` 保留来源与转向记录。15 个固定验收场景包括身份换位、顺序改变、缺靶、弹药不足、延迟掉帧等，另有 20 个预先保存的随机场景。

## 官方动力学链路

显式选择 `config/multi_official.json` 才启用官方动力学。默认单靶和 `multi_fixed.json` 仍可用于快速视觉回归，不能当动力学结果。

```text
真实渲染 RGB → AprilTag / 切靶 / 视觉瞄准
                       ↓
              官方 HighLevel SDK
                       ↓
         原版 mc_ctrl r + 原版步态策略
                       ↓ mujoco_cmd（原版 eCAL 协议）
      原版 robot_mujoco ELF 执行接触动力学
                       ↓ mujoco_state（原版 RobotState）
      只读桥 → 基座 + 十二关节状态 → 渲染/射线镜像
```

官方原生程序实际执行物理步进、关节力矩和足地接触。Python 只按真实收到的完整状态构建渲染/裁判镜像，调用 `mj_forward` 更新变换，**不调用 `mj_step`，不积分基座，不生成腿部动画或替代步态**。官方原生程序没有输出 RGB 话题，因此相机图像来自状态镜像的 MuJoCo 渲染，不是原生观察窗口的截图。原生物理库为官方 3.3.0，Python 渲染库沿用 3.3.7。

运行前将官方 ELF、模型、策略和资源复制到本次 `runs/.../official_runtime/` 并核对哈希。只改副本中的场景选择、`motor_platform_type: 5` 和 SDK 回传地址 `127.0.0.1:44888`。控制器环境沿用 `ROBOT_TYPE=XG`，官方代码与策略不替换。只结束本次创建的 PID；发现其他官方仿真器/控制器时拒绝启动，运行中若状态或命令话题出现多个发布者则退出。

官方模型的惯性、关节轴与限位、12 个力矩执行器、52 项传感器数据布局保持一致。原 `imu` 位于基座原点、无附加旋转，因此 `RobotState.position/quat` 可对应基座。首次姿态遵守原程序 `mj_makeData` 行为：基座高度 0.65 m、关节初值 0，**不自动加载 home keyframe**。请勿在原生观察窗口手动重置、载入 keyframe 或暂停，否则本次验收无效。

当前导入器仍使用先前的派生 STL 兼容修复，最大顶点位移上界约 0.866 mm，官方原 STL 不变。相机、激光、靶面尺寸与接触参数仍需按实际设备标定；本平台可验证控制链和视觉算法，不代表已完成实物精度或安全认证。

## 官方 SDK 调用约束

以随仓库的 `highlevel.h`、API 文档、Python demo 及实际绑定为依据：

- `initRobot → checkConnect → standUp`；站起至少等待官方示例的 4 秒，并要求连续新鲜、静稳的反馈与站立模式。
- 粗转向用 `move(vx,vy,yaw_rate)`。非零最小值为 vx 0.05、vy 0.1、yaw 0.02；更小请求归零并记录，不擅自加速。
- 站立精瞄用 `attitudeControl(0,pitch_rate,yaw_rate,0)`，允许 yaw 与 pitch 同时非零，连续小速度不套用 move 死区。
- move 与姿态模式间切换：当前模式归零 → 真实反馈静稳 → standUp → 至少 4 秒及反馈确认。切换完成只准备模式，丢弃等待前的旧指令/READY，重新取图并重新累计稳定窗口。
- 每个 SDK 状态码都检查；本次所附 Python 绑定经只读反汇编确认返回整数。默认不接受 `None` 代替成功。
- 缺帧、旧反馈、过时射击画面或瞬时抖动禁止射击；短时不稳重新观察，断链/坏数据/指令看门狗故障停止本次运行，不回退到运动学。独立看门狗默认 1 秒；源图像和反馈的新鲜度门槛仍为 0.15 秒。

`RobotState.time_stamp` 没有规定单位，不能冒充仿真秒。这里用 eCAL 发送与接收时间的差、桥接收单调时钟映射**物理发布时刻**，保留源序号与原时间值。它不是对实物摄像头曝光时间的标定。SDK getter 没有采样时间，只记录为诊断；任务反馈来自官方物理程序模拟传感器，不能给 getter 人为补上一个“新鲜时间”。协议证据见 `validation/official_runtime_contract_research_20261019.json`。

## Ubuntu 22.04 更新与依赖

将本次 ZIP 和 `.sha256` 放在 Ubuntu 同一目录，执行：

```bash
sha256sum -c task3_official_dynamics_030_20261019.zip.sha256
unzip -o task3_official_dynamics_030_20261019.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
bash task3_sim/setup_ubuntu.sh
task3_sim/.venv/bin/python -c 'import sys; assert sys.version_info[:2]==(3,10), sys.version'
```

现有 Ubuntu 3.10 虚拟环境可以继续使用。不要从 Windows 复制 `.venv` 或 `generated/scene.xml`；派生场景需在 Ubuntu 当地生成。

此次需要**官方原生运行依赖**，不仅是 Python 的 `pip install mujoco`。若官方程序依赖尚未安装，在工程目录执行以下系统安装命令；若已安装可直接进入编译：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config protobuf-compiler libprotobuf-dev \
  libglfw3-dev libyaml-cpp-dev libopencv-dev libspdlog-dev libeigen3-dev \
  libboost-all-dev libhdf5-dev coinor-libipopt-dev libblas-dev liblapack-dev
sudo apt-get install -y \
  ./deps/ecal_5.13.3-1ppa1~jammy_amd64.deb \
  ./deps/zsibot_common_0.6.1_x86_64_Linux.deb \
  ./deps/mujoco_3.3.0_x86_64_Linux.deb \
  ./deps/onnx_1.51.0_x86_64_jammy_Linux.deb \
  ./deps/lcm_1.5.1-1_amd64.deb
source /opt/ros/humble/setup.bash
bash task3_sim/build_official_bridge.sh
```

ROS Humble 是官方原生程序依赖；若 `/opt/ros/humble/setup.bash` 不存在，需要先完成官方 Ubuntu 22.04 的 ROS Humble 安装。编译脚本最后运行依赖预检，`preflight.json` 会列出 `ldd` 未找到的具体库、文件或端口冲突。停止你此前启动的任务一、二进程后再做本次验收；程序不会帮你结束其他任务。

官方 MuJoCo ELF 强制创建 GLFW 窗口，必须在可用的 Ubuntu 图形桌面终端运行。`--headless` 只关闭 Python 的相机/镜像窗口，官方物理窗口仍存在。无显示的服务器可另用 `xvfb-run -a` 提供虚拟显示；桌面验收无需此步骤。

## 启动与完整验收

建议先看一次官方步态与相机画面：

```bash
bash task3_sim/run_official_task3.sh --show --viewer --max-seconds 240
```

会出现官方物理观察窗口，以及 Python 相机/镜像窗口。任务完成后 Python 结果窗口保持显示，在 Python 窗口按 Esc/Q 或关闭该窗口退出，官方物理窗口由程序清理；要自动退出加 `--hold-seconds 0`。请等待完整退出再做下一次运行。`--realtime` 无需添加，官方链路按真实墙钟运行。

完整验收使用一个新输出目录，包含本地契约测试、35 场运动学真实图像回归，以及三场**官方原生动力学**：默认布局、ID/顺序换位、起点/距离/延迟/掉帧。各原生场景上限为站起完成后的 240 秒，失败不重抽参数、不降低指标。

```bash
bash task3_sim/accept_official_ubuntu.sh
```

结束会打印：

```text
TASK3_ACCEPTANCE PASS/FAIL .../acceptance.json
TASK3_RETURN_ARCHIVE .../official_acceptance_....tar.gz
```

把该 `.tar.gz` 回传即可。即使 FAIL，脚本也尽量打包现有日志；复制的官方大文件不打包，保留配置、哈希清单、逐帧状态、图片与日志。若失败发生在编译阶段，回传编译终端输出和最新 `official_preflight_*/preflight.json`。

对已有的一次官方运行，也可以单独审计：

```bash
task3_sim/.venv/bin/python -m task3_sim.validate_official task3_sim/runs/official_实际目录名
```

验收器检查三靶唯一身份与一次扣弹、每发至少 9 环且误差不超过 2 cm、新鲜且静稳的实际射线样本、SDK 指令账本和模式切换、粗转时至少六个关节的真实 q/qd 变化、源时间与图像对应、官方源文件/ELF 哈希、只关闭自身进程，以及终端 `TASK3_PROCESS_EXIT code=0`。最终看 `official_validation.json` 的 `all_passed`，不能只看屏幕上的 TASK3_SUCCESS。

## 当前验证边界

本地正式视觉回归 **35/35** 符合预期，97 发全部为正确 ID 的 10 环，最大偏差 8.961 mm，验证期间全部 55 个 Python 源文件哈希保持不变。默认场景为 19.95 模拟秒，对照本次 Ubuntu 0.2.1 的 38.05 模拟秒减少约 47.6%；这是路线/模拟时间对比，不是同机墙钟性能对比。完整报告为 `validation/directional_actual_render_030.json`。独立 CLI 也得到 30 分、余弹 2、进程退出 0，见 `validation/directional_cli_030.json`。

单元测试共 213 项，210 通过、3 项因平台条件跳过。原工程保护清单 1,189 个文件无变化（1,181 个 SHA256、8 个 ROS 记录只核对元数据；build 等目录按清单排除）。原单靶的 14 项文件/资产及运行函数 AST 保持一致。见 `validation/directional_dynamics_030_local_checks.json` 和 `validation/directional_dynamics_preservation_030.json`；官方 ELF 另有研究证据哈希及每次运行的逐文件 staging 清单。

开发机为 Windows：官方 SDK 与控制器是 Linux ELF，当前没有可用 Linux 执行环境。代码、派生模型、Python 3.10 语法、真实图像算法与注入契约测试可在本地检查；**C++ 桥在 Ubuntu 的编译与官方步态实跑结果待上述命令回传**。不会把模拟夹具或运动学成绩标成官方动力学验收通过。
