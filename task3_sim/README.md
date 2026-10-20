# 任务三独立 MuJoCo 仿真平台

更新：2026-10-20，当前版本 **0.3.6**。修复第五发前 `standUp 0x3007` 的适配层原因：官方 SDK 在尚未站立时累计切换次数，现补上真实到达站立状态后的官方接口确认，再等待完整四秒；不重试或忽略非零返回码。源码和本机回归可验证，五发官方原生运行仍须 Ubuntu 验收，详见 [0.3.6 第五发切换修复](OFFICIAL_STANDUP_036.md)。默认基础弹药仍为 **3 发**，保留 0.3.5 的逐发累计计分、界面回执和 `shots.csv`，四发十环为 40 分、五发为 50 分。图像/反馈 0.15 秒、8 mm、0.4 秒门槛保留，官方原件、任务一二与旧单靶不变。

`bash task3_sim/run_official_task3.sh --show --viewer --max-seconds 240` 启动默认基础三发官方动力学；额外用 `--config task3_sim/config/multi_official_five_shots.json` 明确测试第五发。首次使用需编译只读状态桥，已有可用桥无需重复编译。`bash task3_sim/accept_official_ubuntu.sh` 包含独立的官方原生五发巡回诊断及视觉回归，并导出回传包。官方原件、任务一二与旧单靶保持独立。

最新 Ubuntu 回传已完成一次 0.3.6 原生五发任务：顺序 `2,5,9,2,5`，全十环、50 分、余弹 0。离线复核 57/58 项通过，唯一未通过项是跨机可变里程文件的源哈希核对；旧 0.3.5 完整验收不能替代本版全套验收。详见 [最新五发复核](validation/ubuntu036_five_shot_review.json)。实物连接步骤与尚待接入的硬件接口见 [SSH、网络和视频说明](HARDWARE_CONNECT_20261020.md)。

以下基础入口的首个里程碑为 **MuJoCo 相机真实渲染 → AprilTag3 检测与位姿 → 自主瞄准 → 最近碰撞面的射线计分**。默认单靶和运动学视觉回归不启动官方运行链；显式 `multi_official.json` 会启动隔离副本中的官方进程，只连接本机回环地址。

`0.2.1` 将多固定靶纠正为机器狗初始朝向的 **9 点钟（左，ID 2）、12 点钟（前，ID 5）、3 点钟（右，ID 9）**，四周使用原背景图片建成四面真实纹理墙。每轮按 ID 升序各射一发，有余弹则继续新一轮；0.3.5 默认共享弹药为 3 发。默认场景仅含这三块正式靶，ID 42 仅在干扰验收场景作为白底 Tag 贴片出现。用 `--config task3_sim/config/multi_fixed.json --max-seconds 180` 启用，详见 [三方向布局](THREE_DIRECTIONS_20261019.md) 和 [多固定靶历史说明](MULTI_FIXED_20261019.md)。原合成单靶默认入口和图片单靶配置保留。

`0.1.2` 引入用户提供的彩色背景与图片固定靶，使用独立配置 `config/photo_target.json`，以及 15 种受控外观/相机扰动验证。图片素材说明见 [图片靶说明](APPEARANCE_20261019.md)。用户回传的 Ubuntu 记录已确认原 0.2.0 多靶、基础单靶、图片场景及 0.2.1 三方向布局正常运行；0.3.3 已有两次官方原生三靶射击成功记录，0.3.4 五发巡回在第四发后遇到官方模式切换错误，详见当前版说明。

`0.1.1 / fix1` 的历史问题与修复见 [Ubuntu 修复说明](UBUNTU_FIX_20261019.md) 和 [开发机验证记录](validation/fix1_validation.json)。后续用户回传的 Ubuntu 证据见本次评估；更新包继续保留原虚拟环境、运行记录和官方工程，无需重复安装依赖。

基础运动学入口使用原 xgb 模型的外形、关节、惯性与资产，基座按速度指令和一阶响应运动，腿部保持初始姿态。它用于视觉与控制接口回归，不能作为四足平衡验收。新增官方动力学入口通过官方 `move/attitudeControl` 控制，实际物理由原版 ELF 执行，Python 仅渲染真实状态。所有入口的射线分数均为自有仿真裁判结果，不是官方 UE 记分牌结果。

## Ubuntu 22.04 安装和首次运行

在已经跑通任务一、二的原工程目录解压本目录即可。Ubuntu 目标为 **x86_64、Python 3.10**；不需要重新安装或升级原工程的 ROS/SDK。

```bash
cd "$HOME/robotac/matrix_robotac_first"

# 系统依赖仅在缺少时安装；Python 包全部放进 task3_sim/.venv。
sudo apt-get update
sudo apt-get install python3.10-venv libgl1 libglib2.0-0 libegl1 libgl1-mesa-dri

bash task3_sim/setup_ubuntu.sh
bash task3_sim/run_task3.sh --headless --max-seconds 25
```

脚本只用 `task3_sim/.venv/bin/python` 安装固定版本依赖，并在子进程中清除继承的 `PYTHONHOME/PYTHONPATH`，避免已加载的 ROS Python 路径混入；调用终端不受影响。无需激活虚拟环境，也不要使用 `sudo pip`。依赖版本见 [requirements.txt](requirements.txt)。若工程复制自 Windows，不要复制 `.venv`，在 Ubuntu 重新执行安装脚本。

无窗口运行仍然进行真实 OpenGL 相机渲染；脚本默认使用 `MUJOCO_GL=egl`。有图形桌面时，可观看相机画面与被动场景窗口：

```bash
bash task3_sim/run_task3.sh --show --viewer --realtime
```

不要同时加 `--headless`。`--viewer` 打开 MuJoCo 场景窗口，`--show` 打开 OpenCV 相机窗口；`--headless` 和不带这些选项的运行均不显示窗口。窗口只用于观察；完成前在相机窗口按 Esc 或关闭场景窗口会取消本次运行。`--realtime` 让仿真时间接近墙钟时间；默认尽快计算。`--max-seconds` 是仿真时间上限。

交互模式在完成后保存结果并保留窗口供观察，机器人停止，也不会再次发射。关闭场景窗口、在相机窗口按 Esc 或按 Ctrl+C 可退出观察。需要完成后立即退出时加 `--hold-seconds 0`；例如 `--hold-seconds 10` 表示完成后再显示 10 秒。无窗口模式不等待。

模型默认只读加载工程中的 `src/robot_mujoco/zsibot_robots/xgb/xgb.xml` 与其 `assets/`，在现有工程里直接运行即可。需要显式指定时，下面仍使用工程中真实的默认目录：

```bash
bash task3_sim/run_task3.sh --headless \
  --model-root "$HOME/robotac/matrix_robotac_first/src/robot_mujoco/zsibot_robots/xgb" \
  --config task3_sim/config/single_target.json
```

只有原模型确实位于其他目录时，才把上面的 `--model-root` 改成实际包含 `xgb.xml` 的目录。不要复制不存在的占位路径。

首次运行按需在本次 `runs/` 目录生成兼容 MuJoCo 3.3.7 的派生模型资产。原机身 STL 超过该版本的面数限制，因此派生副本采用 1 mm 网格顶点聚类；单顶点位移上界约 0.866 mm，具体面数与处理记录见该次模型来源记录。原 STL 文件保持不变。派生几何也意味着当前平台不能证明原模型的动力学精度。

靶面显示网格与射线碰撞盒分开：碰撞盒采用可被 `mj_ray` 识别的不透明几何，在相机渲染时通过第 2 组可见性关闭。直接用其他查看器打开生成的 XML 时，也需要关闭第 2 组显示以观看纹理；不要把碰撞盒改成透明来隐藏，否则可能导致射线穿过它。

## 如何验收

默认输出到独立的 `task3_sim/runs/run_<时间>_<随机标识>/`。也可用 `--output task3_sim/runs/my_first_run` 指定一个新目录或空目录；程序拒绝覆盖已有结果。

成功时终端输出 `TASK3_SUCCESS`，正常退出码为 0；取消、超时、未命中或 Python 异常为 2。使用启动脚本时还应确认最后一行 `TASK3_PROCESS_EXIT code=0`；原生库崩溃可能发生在结果已经写出之后，不能只凭 `TASK3_SUCCESS` 判断进程正常结束。检查同目录文件：

| 文件 | 内容 |
| --- | --- |
| `summary.json` | 状态、耗时、弹药、射线命中点、环数、位姿诊断与源模型完整性 |
| `initial_rgb.png` / `final_rgb.png` | 未叠加标注的 MuJoCo 相机观测；若配置了相机扰动，此处包含该扰动 |
| `final_annotated.png` | 实际检测到的 Tag、估计靶心和控制状态 |
| `overview.png` | 场景全景，用于核对机器人与靶纸关系 |
| `events.jsonl` | 每帧检测、位姿、控制指令、稳定判断与发射记录 |
| `shots.csv` | 多靶逐发回执：序号、轮次、预期/实际 ID、环数、有效分、累计分与弹药变化 |
| `config.json` | 本次实际使用的配置快照 |
| `generated/` | 本次派生场景、纹理、资产及模型来源/哈希记录 |
| `error.log` | 进入运行阶段后的异常堆栈，仅出错时产生 |
| `terminal.log` | 启动脚本记录的标准输出、标准错误及真实 Python 退出码 |
| `fault.log` | Python faulthandler 诊断输出；正常运行时可为空 |

单靶验收时确认 `summary.json` 的 `status` 为 `SUCCESS`、`shot.ring` 为期望环数、`model_sources_unchanged` 和 `shutdown_completed` 均为 `true`，并检查图像确实包含可识别的目标。默认单靶只自动发射一次。默认多靶为基础 3 发，顺序 `2,5,9`、余弹 0；逐发次数以 `shot_count`、`shots.json` 和 `shots.csv` 核对。`mission.total_score` 与 `shot_score_sum` 均累计所有有效发分，每靶 `score_sum` 为累计分、`ring` 为最新分；原逐靶最新分合计移至 `latest_target_score_total`。四发十环总分 40，五发为 50。1、2、6 发是非比赛边界测试；5 发是额外弹药/重复访问诊断，不能据此推定本场获准弹药。

## 模块、坐标与接口

| 模块 | 职责 |
| --- | --- |
| `scene.py` / `appearance.py` | 从原 xgb 模型生成独立场景，加载合成靶或标定后的图片靶与背景纹理 |
| `sensor.py` | 对真实渲染帧施加可复现的曝光、模糊与噪声压力条件 |
| `backend.py` | MuJoCo 相机与基座运动适配器；含速度滞后、速度限制、指令超时停车 |
| `perception.py` | 受管理的 `pupil-apriltags` AprilTag3 ID/角点检测，OpenCV 平面 PnP 公制位姿；输入仅为 RGB 图像 |
| `geometry.py` | Tag 到靶心偏移、相机/激光外参、射线与靶面误差 |
| `controller.py` | 瞄准、停稳、多帧稳定、图像和反馈新鲜度及发射条件 |
| `scoring.py` | MuJoCo `mj_ray` 最近表面碰撞、环数、有限弹药与最后一次成绩 |
| `interfaces.py` | 独立于 MuJoCo/ROS/SDK 的图像、运动和射击接口 |
| `multi_config.py` / `multi_scene.py` | 三方向多靶、按 ID 派生纹理与四面背景墙，兼容旧单墙配置 |
| `multi_perception.py` / `mission.py` | 逐 ID 公制标定、目标状态、自动切换与一次性发射权限 |
| `multi_scoring.py` / `multi.py` | 多靶最近交点裁判、共享弹药、逐发账本与运行入口 |
| `transport.py` / `validate_multi.py` | 保留采集时间戳的延迟/掉帧注入与固定、随机验收 |
| `official_aim.py` / `official_backend.py` | 官方后端专用有界 PI、实际姿态输入保持、原生反馈与射击前最终门控 |

控制器只接收 `camera/laser/tag/control` 参数、图像检测结果及机器人反馈；场景中靶心真值不传入控制器。仿真真值只供射线裁判和决策后的诊断比较使用。Tag 在靶心上方，目标为偏移后的**环心**。

长度用米、角度用弧度、时间戳用仿真秒。机体坐标为 X 前、Y 左、Z 上；OpenCV 相机与 Tag 坐标为 X 右、Y 下、Z 前。正偏航向左，正俯仰使发射方向向下。`pitch_rate_rad_s` 是当前仿真扩展，不能直接等同于官方 `move(vx, vy, yaw_rate)`。

机器人接口保留 `Connect/StandUp/ReadFeedback/Execute/Stop` 形态，射击接口保留 `shooting_init/shooting_reload/shooting_fire/shooting_shutdown` 形态，后续适配实机时替换后端。没有给官方 SDK 增加未经证实的射击函数。

## 参数与尚未证实的比赛细节

`config/single_target.json` 中的靶纸大小、环半径、Tag 家族/尺寸、相机内参和场地位置是可复现的**仿真参数**；相机/激光安装初值参考 `demo/config/config.yaml`，并非当前实机重新标定值。渲染与检测使用同一组内参；当前渲染器要求 `fx=fy` 且主点居中，不支持直接加载任意真实内参和畸变。

环半径顺序为 10、9、8、7、6 环。射线命中环外或脱靶记 3 分，边界算内环；这些均为可替换的仿真裁判约定。单靶模式中，已消耗弹药的脱靶会把当前目标的最后成绩更新为 3，且保留实际 `target_id=null`。命中障碍会被最近碰撞面阻挡；射线忽略机器人自身与仅供显示的几何。无弹药和未初始化调用不扣弹，不计新成绩。靶纸与比赛正式尺寸、擦边规则、脱靶归属仍须正式规则确认。

多靶模式计算本次任务认可的逐靶环数与总分，并支持图像交付延迟、抖动、掉帧、断流测试；仍未接入 RTSP、实机串口、任务二奖励弹药、时间奖励或三次尝试最好成绩。后续接入官方视频流时，应保持图像接口和采集时间戳语义，并重新标定内参、外参和延迟保护。

把现场配置放到 `task3_sim/local/`，避免后续代码包覆盖：

```bash
mkdir -p task3_sim/local
cp task3_sim/config/single_target.json task3_sim/local/single_target.json
bash task3_sim/run_task3.sh --headless --config task3_sim/local/single_target.json
```

## 原工程保留与迁移包

所有新增代码、运行结果、派生资产和虚拟环境均位于 `task3_sim/`。不改原 `run_sim.sh`、`config/`、`src/`、`deps/`、导航、建图和地图。更新包只包含自有 `task3_sim/` 源文件与用户提供的图片素材；排除 `.venv/`、`runs/`、`runs_ubuntu/`、`updates/`、`local/`、缓存和 `*_local.json`，不分发或覆盖原模型、SDK 与 UE 资源。

制作迁移包：

```bash
python3 task3_sim/package_update.py
```

得到 `task3_sim/updates/task3_standup_fix_036_20261020.zip` 与 `.zip.sha256`。校验文件和包内 Bash 脚本使用 LF 换行。压缩包内部全部路径均以 `task3_sim/` 开头，并带 `PACKAGE_MANIFEST.json` 的逐文件 SHA-256。复制这两个文件到 Ubuntu，在压缩包所在目录校验后解压到**现有原工程根目录**：

```bash
sha256sum -c task3_standup_fix_036_20261020.zip.sha256
unzip -l task3_standup_fix_036_20261020.zip
unzip -o task3_standup_fix_036_20261020.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
bash task3_sim/run_task3.sh --headless --config task3_sim/config/multi_fixed.json --max-seconds 180
echo $?
```

已成功执行过 `setup_ubuntu.sh` 的这次修复无需重装；首次安装或迁移到另一台机器时才按前文创建 Ubuntu 虚拟环境。`echo $?` 应为 0。

不要复制 Windows 虚拟环境与生成后的场景文件；导出的 MJCF 含当前主机的绝对模型/资产路径，Ubuntu 必须通过运行入口重新生成本地场景。官方模型与 SDK 的授权及归属沿用原工程；本包没有把它们复制成自有源码。

可在试验前后检查原工程完整性：

```bash
python3 task3_sim/preserve.py snapshot --manifest task3_sim/runs/before.json
bash task3_sim/run_task3.sh --headless
python3 task3_sim/preserve.py verify --manifest task3_sim/runs/before.json \
  --report task3_sim/runs/preservation_report.json
```

默认检查原 MuJoCo 模型/资产、运控源码、SDK、配置、脚本、导航、建图、地图、场景工具、当前说明文档和原 RobotAC_Level_1 的 `.umap/.uexp`。排除 `.git/build/install/log` 等目录；巨大的 `.db3/.mcap` 只检查大小及修改时间，其余纳入文件检查 SHA-256。它不是全部 UE 二进制的完整哈希审计，报告会列出实际覆盖范围；可重复传入 `--path` 自定义快照范围。工具只读取受保护文件，不做还原或覆盖。

## 故障定位和验证边界

- 窗口几秒后消失、Ubuntu 提示 Python 3.10 停止、`malloc()`/`Aborted`、校验文件名尾随 `\r`：按 [Ubuntu 修复与验收说明](UBUNTU_FIX_20261019.md) 定位历史问题，再安装当前更新包。当前包保留相关修复，无需重装系统 Python。
- `EGL` 初始化失败：检查显卡驱动与 `libegl1`。纯 CPU 环境可安装 `libosmesa6` 后执行 `MUJOCO_GL=osmesa bash task3_sim/run_task3.sh --headless`。有桌面环境可用 `MUJOCO_GL=glfw ... --show`。
- `libGL.so.1` 或 `libgthread-2.0.so.0` 缺失：补齐 `libgl1`、`libglib2.0-0`，再运行。
- `xgb.xml` 不存在：需要原模型及其资产，精简 Git 发布副本未必包含这些文件；用 `--model-root` 指向已有原件。
- `LOST/TIMEOUT`：检查原始图像、Tag ID、尺寸、相机外参、目标可见性及检测质量阈值。
- `UNREACHABLE`：检查目标是否在前方、俯仰限制及激光/相机外参；不要用跳过发射门限解决。

运行自动测试：

```bash
task3_sim/.venv/bin/python -m unittest discover -s task3_sim/tests -v
```

再执行实际相机渲染的集成验收：

```bash
MUJOCO_GL=egl task3_sim/.venv/bin/python -m task3_sim.validate
```

它分别运行默认初始位姿、另一侧初始位姿、倾斜靶、射线被阻挡、Tag 不可见和无弹药六种情况，将每个场景的图像、日志与汇总写入新的 `runs/validation_*/`。检查顶层 `validation.json` 的 `all_passed=true`，其中失败场景应当正确停止或返回脱靶，而非强行发射成功。`--output` 可指定新目录，`--model-root` 可指定原模型位置。

测试包括靶心偏移、坐标修正方向、时间戳和停稳保护、射线遮挡、环边界、有限弹药、最后成绩以及更新包边界。开发机为 Windows；Ubuntu 的真实 OpenGL 渲染和桌面环境仍须在目标机器上验收。Python 3.10 语法及 Linux x86_64 固定依赖可独立检查，但不能代替目标机运行。

原版的 2026-10-19 开发机记录：Python 3.12.3、MuJoCo 3.3.7 完成真实渲染闭环，默认配置在 2.55 秒仿真时间、52 帧后自动命中 10 环，实际射线距靶心约 4.39 mm，剩余 4 发。原版 14 项自动测试和六种实际渲染集成场景均通过；全部 Python 源文件通过 Python 3.10 语法检查，四个固定依赖均成功取得 Linux x86_64/Python 3.10 二进制 wheel。摘要证据保存在 [validation/local_validation.json](validation/local_validation.json)。这些是原版历史结果。最初用户回传的 Ubuntu 日志暴露过退出崩溃；随后 0.2.0 验收已确认原生清理与正常退出。本次 0.2.1 三方向修正仍须在 Ubuntu 目标机复验，见 [当前说明](THREE_DIRECTIONS_20261019.md)。大体积运行图像和原模型不放进迁移包。
