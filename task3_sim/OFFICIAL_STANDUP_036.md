# 任务三 0.3.6：第五发前 standUp 切换故障

本次修复限制在 `task3_sim/`。官方 SDK、控制器、策略、物理 ELF、原模型、任务一二及旧单靶均保留。基础弹药仍为三发，五发配置明确用于重复访问诊断；上一任务弹药接入未实现。

## 原因与修改

用户两份 0.3.4 Ubuntu 记录均实际发射 `2 → 5 → 9 → 2` 四发，仍余一发；第五次选择 ID 5 时，`standUp` 返回 `12295 / 0x3007`。不是弹空或全局超时。

两次失败都恰好发生在第 11 次 `standUp` 调用。只读检查原版 x86_64 SDK 机器码确认：调用时尚未到达站立模式 1，会增加该接口的内部计数；前十次可以返回 0，第十一次返回 `0x3007`。只有在实际模式 1 下再次调用 `standUp`，该计数才归零。读取反馈本身不会清零。终端的 `transition to standUP failed!` 与此分支一致；源码适配层此前每次发出请求后只等待反馈，遗漏了到位确认。静态证据、字节位置和两份原始记录的校验见 `validation/standup036_sdk_evidence.json`。此二进制结论仅适用于校验过的原版 x86_64 库，实物或不同固件仍须复验。

修复后的每次切换执行：

1. 撤销原命令，只向当前拥有的控制模式发零，等待真实新鲜反馈停稳。
2. 调用官方 `standUp` 发起请求，严格检查返回码。
3. 等待 SDK 模式确实到达 1，同时真实 RobotState 连续更新并满足静止窗口。
4. 在模式 1 下再次调用官方 `standUp` 完成确认，严格检查返回码。
5. 从最后这次确认开始再等待完整四秒，并检查持续新鲜的站立、静止反馈。
6. 本次旧图和旧指令失效；下一步重新取图、定位、瞄准、积累射击稳定窗口。

以上阶段共用一个绝对切换期限。任何非零返回、断连、反馈停止或超时均终止；没有对 `0x3007` 的盲重试，没有清除已锁定的故障，也没有写入 SDK 内存、修改官方库或伪造模式。图像/反馈 0.15 秒、误差 8 mm、射击稳定窗口 0.4 秒均保留。新增事件 `standup_requested` 和 `standup_acknowledged` 记录两阶段，所有实际接口调用和返回码仍完整保存，原独立验收按最后一次 `standUp` 检查四秒等待。

另修正物理日志双次取时造成的算术不一致：同一条记录的 `task_time_s` 和 `publication_age_s` 现在共享一次真实时钟读取。原始发布时刻不变，新鲜度门槛和独立审计容差不变，旧 Ubuntu 日志不改写。

## Ubuntu 更新和五发验证

关闭旧程序，将新 ZIP 与 `.sha256` 放在同一目录，在该目录执行：

```bash
sha256sum -c task3_standup_fix_036_20261020.zip.sha256
unzip -o task3_standup_fix_036_20261020.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
source /opt/ros/humble/setup.bash
task3_sim/.venv/bin/python -c 'import task3_sim; print(task3_sim.__version__); assert task3_sim.__version__=="0.3.6"'
bash task3_sim/run_official_task3.sh --preflight
TASK3_FIVE_RUN="task3_sim/runs/official_five036_$(date +%Y%m%d_%H%M%S)"
bash task3_sim/run_official_task3.sh \
  --config task3_sim/config/multi_official_five_shots.json \
  --show --viewer --max-seconds 240 --output "$TASK3_FIVE_RUN"
task3_sim/.venv/bin/python -m task3_sim.validate_official "$TASK3_FIVE_RUN"
```

已有可用环境和状态桥无需重装、重编译。交互完成后窗口保留供观察；关闭窗口或按 Esc 退出观察后再执行独立验收命令。提前关窗会取消任务。

五发验收要求：终端出现五条 `TASK3_SHOT`；目标顺序 `2,5,9,2,5`；余弹依次 `4,3,2,1,0`；全十环时累计分 `10,20,30,40,50`。`shots.csv`、物理账本和任务账本均应有五条一致记录，最后状态 `SUCCESS`，独立 `official_validation.json` 的 `all_passed=true`。不要仅凭看到第五块目标或出现转向就认定第五发已发射；少一发、模式切换异常、反馈审计失败都应保留为失败。

完整验收现在额外包含 `native_repeat_five_shots`，不会只用默认三发来判断第五发修复：

```bash
bash task3_sim/accept_official_ubuntu.sh
```

完整验收保留基础三发、身份换位、起点/距离/延迟/掉帧原生场景和固定/随机视觉回归，并导出 `TASK3_RETURN_ARCHIVE` 指向的回传包。任何前置阶段失败会如实记录，未启动的场景标记为 skipped。请回传此压缩包，以验证真实 Linux SDK 和官方动力学中的第五发。

## 验证范围

Windows 可以执行接口回归和真实 RGB 视觉回归，不能执行官方 Linux CPython 3.10 SDK/ELF。新增回归模拟真实异步模式切换和 SDK 累计计数规则，覆盖超过十次切换、到位确认、错误返回、超时、断连、停更反馈、旧图授权撤销及日志单次取时。它证明适配层故障路径已修正，不代替 Ubuntu 五发原生验收。最终本机结果记录在 `validation/standup036_local_checks.json`。

本机最终结果：

- 全套 402 项测试：399 通过，3 项 POSIX 信号测试在 Windows 跳过，无失败；其中新增 11 项切换/日志回归全部通过。
- 20 个固定场景真实 RGB 回归全部符合预期：15 SUCCESS、4 预期 NO_AMMO、1 预期过期图像 TIMEOUT；包含五发和六发重新访问场景，共实际发射 59 发，全部目标对应且十环，最大真实偏差 7.316 mm。
- 测试前后 67 个 Python 源文件哈希一致；全部通过 Python 3.10 语法检查，5 个 Bash 脚本通过语法检查。验收脚本的四场配置生成已本机检查，弹药分别为 3、5、3、3。
- 原工程 1,189 个受保护文件记录、旧单靶 14 个文件与 4 个入口函数、官方策略绑定文件、状态桥源码和物理 ELF 均保留；0.3.3、0.3.4、0.3.5 更新包未覆盖。

本次本机渲染重测的是 20 个固定场景，未把历史随机场景结果作为本版重测。官方 Linux SDK/ELF 未在 Windows 执行，Ubuntu 五发原生验收尚未完成。
