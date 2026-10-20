# 多固定靶：运行、状态和验收（0.2.1）

本次实现「不同 AprilTag ID 的固定靶 → 自动切换 → 逐靶瞄准射击 → 共享有限弹药与逐发记录」。默认目标顺序为 **2 → 5 → 9**，ID **42** 是干扰靶，初始弹药 **5 发**。默认场景预期各射一发、三靶各 10 环，共 30 分，剩余 2 发；`SUCCESS` 表示三靶均有效命中，本身不保证满分。没有移动靶，也不自动补射。

0.2.1 已纠正比赛布局：以机器狗初始朝向为基准，**左侧 9 点钟为 ID 2、正前方 12 点钟为 ID 5、右侧 3 点钟为 ID 9**。三个靶均距初始中心 2.7 m、靶心高 0.5 m，朝向场地中心。钟点方向不是 AprilTag ID。四周为同一张原背景图片的四面 MuJoCo 纹理墙，场景观察窗口从开放屋顶上方查看。详见 [本次纠正记录](THREE_DIRECTIONS_20261019.md)。

独立配置为 `task3_sim/config/multi_fixed.json`。默认合成单靶、`config/photo_target.json` 图片单靶以及任务一、二的场景、入口、官方代码与模型保留。仅修改 `task3_sim/` 内的自有源码，未增加依赖。机器人仍复用原 xgb 外形和接口，以带速度滞后的运动学基座运行；此结果不能证明四足步态或实机姿态能力。

## Ubuntu 更新与启动

把新 ZIP 和同名 `.zip.sha256` 复制到 Ubuntu。进入这两个文件所在目录后执行：

```bash
sha256sum -c task3_three_directions_20261019.zip.sha256
unzip -o task3_three_directions_20261019.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"

# 首先无窗口验收，仍使用真实 MuJoCo 相机渲染。
bash task3_sim/run_task3.sh --headless \
  --config task3_sim/config/multi_fixed.json --max-seconds 180
echo $?

# 再观察相机与场景两个窗口；完成后保留结果，等待关闭或 Esc/Q。
bash task3_sim/run_task3.sh --show --viewer --realtime \
  --config task3_sim/config/multi_fixed.json --max-seconds 180
```

已跑通 0.1.2 的 Ubuntu 虚拟环境继续使用，不需要重装。新机器仍先执行 `bash task3_sim/setup_ubuntu.sh`；目标 Python 3.10、MuJoCo 3.3.7。包不包含 `.venv`、历史运行、现场 `local/` 配置或官方模型。不要把 Windows 虚拟环境或 `generated/` 场景复制到 Ubuntu，入口会从当地模型重新生成。

正常结果为 `TASK3_SUCCESS`，最后须同时出现 `TASK3_PROCESS_EXIT code=0`。窗口停留期间仿真和弹药均停止变化；可加 `--hold-seconds 5` 在结果展示五秒后退出。部分完成、无弹药、取消、超时或异常返回 2，不能只凭已有命中图判断整场成功。

原单靶回归命令继续可用：

```bash
bash task3_sim/run_task3.sh --headless
bash task3_sim/run_task3.sh --headless --config task3_sim/config/photo_target.json
```

## 目标身份和图像链路

每个目标独立配置 ID、Tag 黑边尺寸、Tag 到靶心的公制偏移。每帧先从真实 RGB 解码 ID，再按该 ID 的尺寸求位姿、按该 ID 的偏移求靶心。一个场景内的目标和干扰靶不能重用 ID。同一帧若出现同一目标 ID 的重复检测，会停车并禁止射击。

控制器只接收相机/激光标定、任务目标 ID、逐 ID Tag 标定和机器人反馈；世界位置、碰撞体、环区真值和干扰靶列表均不传给它。射击后，独立射线裁判按最近实际交点确定命中了哪个靶，随后才核对计划目标。场景坐标仅用于建场、裁判和事后误差核验。

继续使用已确认的彩色背景和绿色图片靶。默认亮度下原 ID 2 图片不变，ID 5、9、42 在每次运行的派生纹理里替换标记及其一格宽白色留边；这些派生补丁之外的图片像素保持原样，计分仍使用原图片标定。转换、标记角点、补丁范围与文件哈希记入 `generated/model_provenance.json`，原始素材保持不变。当前黑边 0.16 m 及板尺寸仍是仿真假设，现场必须实测标定。

## 状态与射击规则

| 目标状态 | 含义 |
| --- | --- |
| `PENDING` | 待射击，等待按配置顺序选中 |
| `ACTIVE` | 当前目标，识别、瞄准或等待停稳 |
| `LOST` | 丢失达到阈值，停止射击并等待重新识别 |
| `COMPLETED` | 已正确命中，永久退出本次目标队列 |
| `FAILED` | 缺靶超时、脱靶、串靶、无弹药或取消，保留原因 |

默认每靶最多一次射击。三方向配置使用 `mission.search_mode=full_rotation`，以正偏航方向搜索；每个目标从实际反馈累计转角，达到一周（6.283 rad）扫描预算且仍无有效检测时，发送停车指令停止继续搜索，不会用世界坐标直接转向靶。截止目标期限前仍接受新鲜视觉重识别，并可继续视觉瞄准；不会重新获得扫描预算。速度滞后仍可造成少量滑行，这不是物理转角的 360° 硬限。45 秒目标期限到达后标为 `target_timeout` 并处理后续目标；全局上限使用 180 仿真秒。切换时先停车，创建新目标的稳定判断；找回目标后也重新计时。没有该选择项的旧配置继续采用原局部搜索。

任何实际射击都要求新鲜、唯一的图像采集时间戳、匹配 ID、合格检测、有效反馈、可达靶心、误差合格、机器人停稳和稳定观察。图像缺失时发送停车指令，不射击；过期、重放、乱序和长时间缺帧撤销发射权限。短时无帧可以暂停稳定计时，空白等待时间不累计为有效观察；长间隔和重新识别仍重置稳定计时。采集时间戳不会被交付时间重写。

射击与扣弹记账作为一个短事务处理 Ctrl+C；每发只接受一次当前目标的有效回执。最后一发已完成任务时关闭窗口不把成功覆盖成取消。整场可为 `SUCCESS`、`PARTIAL`、`FAILED`、`NO_AMMO`、`CANCELLED`；顶层 `TIMEOUT` 表示超过全局 `--max-seconds`，逐靶报告仍保留完成结果和超时原因。

## 计分与运行文件

计分为自有仿真裁判约定，不是官方比赛记分牌。实际最近交点决定物理命中 ID 和环数：10～6 环按图片标定区域判定，环外、障碍或脱靶保留 3 分物理记录。**任务认可分数只计正确目标的有效命中**；错靶或干扰靶不把任何目标标为完成，目标队列不会串靶。失败、未射击目标的任务认可分数为 0。

| 文件或字段 | 用途 |
| --- | --- |
| `summary.json` | 总状态、逐靶结果、弹药、清理、源模型完整性 |
| `summary.mission.total_score` / `credited_scores` | 本场任务认可总分 / 逐靶分数 |
| `summary.last_scores` | 实际命中任务靶的物理最后成绩；与任务认可分开 |
| `mission.json` | 每靶状态、次数、原因、分数、完整状态历史；附任务认可账本 |
| `shots.json` / `summary.shots` | 每发物理账本：序号、计划 ID、实际 ID、环数、命中/原因、弹药前后、交点、仿真发射时间；图像采集时间另在事件中记录 |
| `events.jsonl` | 每个仿真轮次的采集/交付时间、图像年龄、检测、控制和发射；射击事件附事后任务状态 |
| `target_<ID>_shot_<序号>_rgb.png` | 每发所用真实 RGB，未叠加射线或真值 |
| `target_<ID>_shot_<序号>_annotated.png` | 逐发 ID、目标状态和估计靶心 |
| `overview.png` | 全部靶、机器人与历史射线，仅供展示，不进入感知 |

例如三发物理账本应依次是 `2→2, 5→5, 9→9`，弹药 `5→4→3→2`，三个目标均 `COMPLETED`，`attempts=1`。任务认可账本另保留 `correct_target_hit` 和 `credited_ring`，便于识别物理分数与任务信用的区别。

## 延迟、掉帧和可复现随机验收

下面使用真实渲染和原生 AprilTag 检测，保存每个场景的配置、抽样参数和结果，不重抽失败样本：

```bash
# 固定身份映射、缺靶、干扰 Tag、有限弹药、延迟/掉帧/断流等验收。
env -u PYTHONHOME -u PYTHONPATH MUJOCO_GL=egl \
  task3_sim/.venv/bin/python -m task3_sim.validate_multi --random-cases 0

# 同样的固定场景，再加 100 组随机条件；可能运行数分钟。
env -u PYTHONHOME -u PYTHONPATH MUJOCO_GL=egl \
  task3_sim/.venv/bin/python -m task3_sim.validate_multi \
  --random-cases 100 --seed 20261019

# 单元测试，以及原有合成/图片单靶实际渲染回归。
env -u PYTHONHOME -u PYTHONPATH MUJOCO_GL=egl \
  task3_sim/.venv/bin/python -m unittest discover -s task3_sim/tests -v
env -u PYTHONHOME -u PYTHONPATH MUJOCO_GL=egl \
  task3_sim/.venv/bin/python -m task3_sim.validate
env -u PYTHONHOME -u PYTHONPATH MUJOCO_GL=egl \
  task3_sim/.venv/bin/python -m task3_sim.validate_appearance
```

验收不仅检查成功：中间目标不可见时须跳过并保留两靶成绩；只有两发时须保留已完成靶、标记无弹药；全是过期图像时不应开火；断流须经过丢失和重新稳定；干扰 ID 必须确实被解码、始终不被选为目标。每发核对图像年龄、目标唯一性、质量、反馈、误差、实际命中 ID、弹药和任务认可账本。正确命中要求至少 9 环且实际距靶心不超过 20 mm。

随机范围为起点 X/Y 各 ±0.10 m、偏航 ±0.15 rad、俯仰 0～0.08 rad，靶位置比例 0.9～1.1（三方向靶距世界原点 2.43～2.97 m，相机到靶的实际距离另随起点变化），曝光 0.75～1.25、图像噪声标准差 0～4、交付延迟 0～50 ms、掉帧概率 0～15%。种子和每组参数均保存。该范围内的通过率仅说明已测试配置，不能外推到任意场地、视角、遮挡或网络条件。

`observation` 可配置 `delay_s`、`jitter_s`、`drop_probability`、`drop_every_n`、`blackout_intervals_s`、`seed` 和 `max_queue`。默认多靶场景无额外网络扰动。注入层保存原始采集时刻、每帧最多交付一次、丢弃过时乱序到达项；等待队列有长度上限。此层模拟软件交付异常，尚未接入实际 RTSP 网络链路。

现场修改请复制配置到 `task3_sim/local/` 再用 `--config` 指向副本，后续更新包会保留该目录。用户提供的 Ubuntu 0.2.0 验收已通过，审计见 [ubuntu_multi_020_review.json](validation/ubuntu_multi_020_review.json)；本次三方向和四面墙修改仍须在目标机确认。

## 0.2.0 一排三靶历史开发机验收记录

2026-10-19，Windows、Python 3.12.3、MuJoCo 3.3.7：

- 11 个固定场景加 100 个预先定义范围的随机场景，共 **111/111 场景验收通过**。其中 107 场 `SUCCESS`，另外 1 场缺靶 `PARTIAL`、2 场 `NO_AMMO`、1 场过期图像 `TIMEOUT`，均符合事先定义的预期。
- 100/100 随机场景全部完成三靶；总计 325 发均对应正确 ID、各 10 环。实际靶心误差 3.32～8.27 mm，中位数 5.34 mm；随机任务用时 8.15～12.05 仿真秒。
- 116 项单元测试：113 项通过，3 项 POSIX 专用测试在 Windows 跳过。原有 6 个合成单靶和 15 个图片单靶场景全部回归通过。
- 独立双窗口多靶进程完成三靶、余弹 2 发，展示结果 2 秒后正常退出，退出码 0，原生清理无错误。完整多场景验收进程也正常退出 0。
- 验收前后 41 个 Python 文件哈希一致，全部通过 Python 3.10 语法解析；这不等同于在 Python 3.10 上运行过新版。
- 原工程核对 1,189 个文件无变化，其中 1,181 个按 SHA-256、8 个大体积 ROS bag 按大小和修改时间检查。两份原单靶配置、核心模块、启动脚本、原始图片与上一版包一致，单靶入口函数行为未变。

可迁移的历史摘要见 [validation/multi_fixed_validation.json](validation/multi_fixed_validation.json)。完整原始记录位于开发机 `task3_sim/runs/multi_validation_acceptance_20261019/`，单靶回归和双窗口记录位于 `runs/multi_release_verification/`。随后用户提供的 Ubuntu 0.2.0 记录也已验收通过。这些历史成绩来自一排三靶，不能代替本次三方向布局验证；当前结果见 [THREE_DIRECTIONS_20261019.md](THREE_DIRECTIONS_20261019.md)。大体积运行目录不放进更新包。
