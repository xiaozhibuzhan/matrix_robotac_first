# 任务三 0.3.4：按 ID 循环射击并用尽弹药

> 本文保留 0.3.4 历史配置和验证结果。当前基础弹药为 **3 发**；重复射击改为逐发累计计分，四发十环为 40 分、五发为 50 分，更新命令见 [0.3.5 说明](OFFICIAL_SCORE_035.md)。下表的 1、6 发属于非比赛边界测试，5 发是额外弹药诊断；均不表示比赛基础弹药数量。

本版只修改 `task3_sim/`。官方 ELF、SDK、加密步态策略、原模型、任务一二及旧单靶保留；Ubuntu 现有虚拟环境与状态桥继续使用，无需重装依赖或重新编译。

## 最新 Ubuntu 记录

0.3.3 的两次原生运行已实际完成识别、转向、定位与计分：

- `official_20261020_005721_3401`：ID 2/5/9 各 10 环，射击 3 发、剩余 2 发，最大实际偏差 5.864 mm。
- `official_acceptance_20261020_010228_3639/native_nominal`：同样三靶各 10 环、余弹 2 发，最大实际偏差 7.924 mm。

完整验收尚未完成：旧验证器只接受 move/attitude，错误拒绝新增的 `stop_current_mode`。19 条被误判的执行全部是全零且没有射击；所有非零输入都满足真实图像和反馈的新鲜度要求。本版为安全停止增加专门审计，并继续要求实际停止对应控制族。后两场原生与视觉验收此前因此跳过，不能当作已通过。详细证据见 `validation/ubuntu_official_033_success_review.json`。

## 新射击策略

默认 `config/multi_fixed.json` 与 `config/multi_official.json` 使用：

```json
"shooting_policy": "round_robin"
```

每轮将声明的固定靶按 **数值 AprilTag ID 升序**各访问一次，配置文件顺序或靶在左/前/右的位置不决定射击顺序。例如三个 ID 为 2、5、9：

| 初始弹药 | 射击顺序 | 正常结束余弹 |
| --- | --- | --- |
| 1（异常边界测试） | 2 | 0 |
| 3（基础弹药） | 2、5、9 | 0 |
| 5（额外弹药诊断） | 2、5、9、2、5 | 0 |
| 6（非比赛压力测试） | 2、5、9、2、5、9 | 0 |

每次访问最多一发。第一轮结束但有余弹时开始新轮；末轮允许只打部分靶。每发记录 `shot_index`、`round_index`、`target_attempt`、预期/实际 ID、得分、射击前后弹药。逐靶累计发数与正确命中次数保留，界面显示当前轮次、弹药与每靶发数/最新分。

已完成一次访问并不永久禁止再次射击；同一访问的重复回调、已消费令牌、旧图像和旧 READY 仍禁止扣弹。每次射击事务结束后撤销保持，下一次访问创建新的瞄准控制器、取得新图像并重新满足稳定窗口。官方图像/反馈 0.15 秒、8 mm、至少 0.4 秒、真实姿态保持/SDK 账本与最后物理检查均保留。

射前失靶超时或精瞄停滞的靶会记录失败并退出本场后续巡回，其他可用靶继续；已发出的未命中或串靶会真实扣弹，下一轮仍可重新尝试。所有可用靶都退出时有界结束，保留剩余弹药，绝不盲射耗弹。正常弹空且每个声明靶至少一次正确命中时为 SUCCESS；弹药不足以覆盖所有靶时为 NO_AMMO。全局超时、操作取消和实际故障仍会结束本场。

旧自定义配置缺少 `shooting_policy` 时继续使用 `single_pass`：按原声明顺序每靶一次。希望自定义配置使用新策略时显式加上 `round_robin`。`max_target_attempts=1` 现在表示每次访问最多一发，不能绕过弹药与新图像门控。

## 计分约定

保持当前自有射线裁判的约定，不改为累加所有子弹：逐靶 `ring` 为该靶最新一次尝试的 credited 分数（未命中或串靶为 0），`total_score` 为各靶最新 credited 分数之和。五发全 10 环时逐靶总分仍为 30；独立的 `shot_score_sum=50` 是逐发累计统计。原 `last_scores` 继续记录实际物理靶的最后环数，完整账本用于区分串靶和有效归分。

这保留了既有仿真裁判语义，不宣称已核实官方比赛采用最后分、最高分或累计分；比赛官方分数仍以正式规则及官方裁判系统为准。本次用户授权的调整是多轮射击策略。

## Ubuntu 更新与复验

先结束上一场任务三，将 ZIP 和 `.sha256` 放在同一目录，在该目录执行：

```bash
sha256sum -c task3_repeat_targets_034_20261020.zip.sha256
unzip -o task3_repeat_targets_034_20261020.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
source /opt/ros/humble/setup.bash
task3_sim/.venv/bin/python -c 'import sys,task3_sim; print(sys.version); print(task3_sim.__version__); assert sys.version_info[:2]==(3,10); assert task3_sim.__version__=="0.3.4"'
bash task3_sim/run_official_task3.sh --preflight
bash task3_sim/run_official_task3.sh --show --viewer --max-seconds 240
```

查看 `shots.json` 与 `mission.json`：五发应为 `2,5,9,2,5`，对应轮次 `1,1,1,2,2`、每靶发数 `2,2,1`、余弹 0。完成后窗口继续保留供观察，按 Esc/Q 或关闭窗口退出。然后运行：

```bash
bash task3_sim/accept_official_ubuntu.sh
```

回传最后打印的 `TASK3_RETURN_ARCHIVE ...tar.gz`。此验收会检查三场官方原生五发巡回及视觉回归，包括身份/方向交换、随机起点/距离、干扰 Tag、延迟/掉帧和少弹/空弹负例。

## 开发机验证结果

- 单元测试 379 项：376 项通过，3 项 POSIX 前台进程组信号测试在 Windows 跳过；进程退出码为 0。
- 真实 MuJoCo RGB 回归 40 场全部符合预期：20 个固定场景和 20 个预先声明种子的随机场景。35 场 SUCCESS、4 场预期 NO_AMMO、1 场预期过期图像 TIMEOUT，反例保留。
- 共实际发射 183 发，全部命中对应 ID 且为 10 环；最大真实射线偏差 8.961 mm。8 mm 图像推算误差门槛保持，图像推算误差与真实射线偏差分别记录。
- 独立 CLI 五发顺序 `2,5,9,2,5`，轮次 `1,1,1,2,2`，余弹 0、逐靶最新总分 30、逐发累计统计 50；清理完成且进程正常退出。
- 65 个 Python 文件通过 Python 3.10 语法检查，5 个 Bash 脚本通过语法检查。原工程 1,189 个文件记录、旧单靶 14 个文件与 4 个入口函数、官方运行策略所绑定的文件和状态桥源码均保持一致。
- 回归前后源码哈希一致；更新包将逐项核对已测源码哈希，不以重新抽样覆盖失败。

开发机检查见 `validation/repeat034_local_checks.json`，真实渲染报告见 `validation/repeat034_actual_render.json`，原工程保护见 `validation/repeat034_preservation.json`。本地运动学视觉与注入 SDK 测试不能代替新版 Ubuntu 原生五发巡回验收；0.3.4 官方原生五发验收尚待目标机执行。
