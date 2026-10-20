# 任务三 0.3.5：逐发累计计分、可见射击记录、基础三发

本次只修改 `task3_sim/`，保留官方 SDK、ELF、策略、状态桥、原模型、任务一二及旧单靶。上一任务的弹药数据接入只给方案，尚未实现。

## 最新 Ubuntu 记录的真实结果

两次最新 0.3.4 原生运行均实际射击 **4 发**：

- `official_20261020_020620_3364`
- `official_acceptance_20261020_020951_3603/native_nominal`

| 发次 | 轮次 | 预期/实际 ID | 环数 | 射前→射后弹药 | 逐发累计 |
| --- | --- | --- | --- | --- | --- |
| 1 | 1 | 2 / 2 | 10 | 5 → 4 | 10 |
| 2 | 1 | 5 / 5 | 10 | 4 → 3 | 20 |
| 3 | 1 | 9 / 9 | 10 | 3 → 2 | 30 |
| 4 | 2 | 2 / 2 | 10 | 2 → 1 | 40 |

物理账本、任务账本、事件和逐发原生静止/新图/姿态保持门控相互吻合。旧版总分仅合计各靶最新一次成绩，第四发更新 ID 2 的 10 环后仍为 30；旧 `shot_score_sum` 已是 40。第四发和第一发同靶，射线在场景中接近重叠，不能靠数可见射线确定发数。

第五发未发生。再次选择 ID 5 时，官方 `standUp` 返回 `12295 (0x3007)`，程序以 ERROR/退出码 2 结束，余弹 1；并非弹空、关窗或全局超时。本次计分/显示修正不声称修复此官方模式切换故障。另有双次取时造成的发布年龄算术审计不一致，单独保留在报告中，未放宽新鲜度门限。原始日志未改，详见 `validation/ubuntu_034_four_shot_review.json`。

## 本次改动

`mission.total_score` 改为所有已接受的有效发分累计；`shot_score_sum` 保留为相同累计值。串靶、未命中不增加有效总分，已实际发射仍扣弹和记一发。超时、取消或故障不丢弃此前成绩。

每靶 `score_sum` 为该靶累计有效分，`ring` 仍为最新一次成绩；`latest_target_score_total` 单独保存原来的各靶最新分合计。因此四发十环总分 40、ID 2 累计 20；五发十环总分 50，最新分合计仍可为 30，不再把后者显示成总分。

相机窗口增加独立旁栏，持续显示 `Shots fired`、`Total score`、余弹、轮次、每靶发数和最近五发回执。MuJoCo 场景窗口也显示相同文字。原始 RGB 和 AprilTag 检测输入保持不变；射线仍按真实命中点绘制。

每次真实发射后终端会打印，例如：

```text
TASK3_SHOT index=4 round=2 intended_id=2 actual_id=2 ring=10 credited=10 total_score=40 ammo_before=2 ammo_remaining=1
```

新增 `shots.csv`，每发一行，包含序号、轮次、目标、环数、有效分、累计分和弹药变化；每次射后停车及最终清理时保存。若实际物理发射后任务账本拒绝提交，CSV 仍保留物理发数，明确标记 `mission_accepted=False`、`credit_status=unconfirmed`，有效分留空，原故障仍报告为 ERROR。`shots.json`、`mission.json` 和 `events.jsonl` 继续保留原始证据。

## 比赛基础弹药与测试弹药

`config/multi_fixed.json` 与 `config/multi_official.json` 默认 `simulation.ammo_count=3`，按 ID `2 → 5 → 9` 完成基础三发。有额外弹药时仍按升序巡回，每次重访重新取图、定位、瞄准和积累稳定窗口。

正常比赛说明以基础三发为准。1、2 发为少弹/异常边界，0 发为空弹保护，6 发为非比赛压力测试；它们验证程序不会越界发射，不能解释为比赛允许这些基础弹药数。5 发是额外弹药/重复访问的诊断数量，也不自动代表裁判已批准本场五发。旧版文档已加历史标记及边界说明。

## 上一任务决定弹药：推荐方案（尚未接入）

推荐将**基础 3 发、上一任务获得的额外弹药、裁判确认的本局有效弹药 N**分别记录。优先使用赛前裁判确认的 N 来设置本局发射预算；缺少官方机器可读接口时，由操作员一次性录入裁判确认值，并记录确认来源。之后若上一任务能稳定导出结果，再接入自动导入，仍保留裁判确认/更正途径，避免猜测数据或沿用上一场旧值。

若主办方明确允许“始终射击五次，裁判只计前 N 发”，可采用备用方案：保留全部五发真实记录，另列前 N 发有效分及其余发次，不删除、改序或掩盖额外射击。需要确认的核心是额外发射本身是否允许；“现场裁判处理”尚不足以确定。因此当前推荐默认三发、按裁判确认的预算执行；不把固定五发作为默认比赛策略。本次只提出方案，未实现上一任务导入、录入界面或前 N 发裁定功能。

## Ubuntu 更新与检查

关闭上一场程序，把 ZIP 与 `.sha256` 放到 Ubuntu 同一目录，在该目录执行：

```bash
sha256sum -c task3_score_ammo_035_20261020.zip.sha256
unzip -o task3_score_ammo_035_20261020.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
source /opt/ros/humble/setup.bash
task3_sim/.venv/bin/python -c 'import task3_sim; print(task3_sim.__version__); assert task3_sim.__version__=="0.3.5"'
bash task3_sim/run_official_task3.sh --preflight
bash task3_sim/run_official_task3.sh --show --viewer --max-seconds 240
```

默认应看到基础三发逐条打印，顺序 `2,5,9`，十环时总分 `10 → 20 → 30`，余弹 `2 → 1 → 0`。检查本次 `shots.csv`、`shots.json` 和界面发数一致。完整验收命令：

```bash
bash task3_sim/accept_official_ubuntu.sh
```

回传最后 `TASK3_RETURN_ARCHIVE` 指向的压缩包。该脚本的三场官方原生测试继承当前基础三发配置；视觉回归另有明确的五发、六发及边界场景。基础三发验收通过不能据此推定第五发官方 standUp 故障已解决。

## 本机验证结果

- 全套 391 项测试：388 项通过，3 项 POSIX 信号测试在 Windows 跳过；进程退出码 0。
- 20 场固定场景真实 RGB 回归全部符合预期：15 SUCCESS、4 预期 NO_AMMO、1 预期过期图像 TIMEOUT；共实际发射 59 发，全部对应 ID 且 10 环，最大真实偏差 7.316 mm。
- 独立四发/五发 CLI 均正常退出，分别得到累计 40/50 分，CSV 为 4/5 行；界面旁栏与第四发留存图片已检查。
- 66 个 Python 文件通过 Python 3.10 语法检查，5 个 Bash 脚本语法通过；测试源码哈希前后一致。
- 原工程 1,189 个文件记录、旧单靶 14 个文件及 4 个入口函数、官方策略绑定文件与原生桥源码保持一致，旧更新包未覆盖。

本次修改集中在计分、弹药默认值和可见记录，运行 20 个固定场景；未把上一版的 20 个随机场景冒充成本版重测结果。开发机验证结果见 `validation/score035_local_checks.json`，本次实际渲染报告见 `validation/score035_actual_render.json`，第四发旁栏示例见 `validation/score035_four_shot_receipt.png`。Windows 无法执行官方 Linux SDK/ELF，0.3.5 官方原生验收仍待 Ubuntu 回传；已知第五发前 standUp 故障及发布年龄日志问题保留为未解决项。
