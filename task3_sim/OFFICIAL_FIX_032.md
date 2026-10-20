# 任务三 0.3.2：识别到靶却不射击

本次只修改 `task3_sim/`。官方 ELF、SDK、加密步态策略、原模型、任务一二与旧单靶入口保留。**本地 Windows 检查不代表 Ubuntu 官方动力学已验收。**

## 最新记录的结论

已读取 `official_20261019_211859_3364` 与 `official_acceptance_20261019_212457_3604/native_nominal`。

第一场已完成 SDK 连接、站起、三方向搜索和 AprilTag 识别，所有 SDK 返回码均为 0。但 1,476 帧停留在 AIMING，从未进入 SETTLING/READY，也从未调用射击门控。ID 2/5/9 最后稳定偏差约为 **54.6/164.4/155.7 mm**，大于 8 mm 门槛，所以每靶 70 秒超时、0 发射击、剩余弹药 5 发。不是识别到 ID 就应射击，也不是弹药或射线计分被漏记。

原因是原精瞄纯 P 外环按角速度积分响应设计，但此工程所带官方平衡策略对持续输入产生有限姿态响应，留下稳态误差。官方文档仍将 `attitudeControl(roll_vel,pitch_vel,yaw_vel,height_vel)` 定义为 rad/s、m/s；此修复不重定义官方 API，也不修改官方实现。

日志还证实：清零姿态输入会改变已保持姿态。例如 156.053 秒清零后，在下一次 standUp 前，偏航从约 0.0388 rad 变为 0.1079 rad。因此“对准 → 姿态清零 → 等待 → 开火”会破坏本策略的对准。

第二场在首次渲染后发安全 WAIT/零指令时，因为反馈已过期而锁定故障。零指令现在优先停止已经拥有的控制模式；非零运动仍要求真实、新鲜的反馈。

详细证据见 `validation/ubuntu_official_031_no_fire_review.json` 与 `validation/official_attitude_response_032.json`。退出清理后的 `mc_ctrl=-6` 单独记录，未被当作此次零分的原因，也未被改写为退出 0。

## 修正后的控制链

1. 大角度搜索继续走官方 `move`，精瞄走官方 `attitudeControl`。两模式切换仍先停止、站起至少 4 秒、重取图像，不使用旧图继续控制。
2. 仅官方后端加入两轴 PI 外环，P 复用原图像误差控制量，I 累积图像推算角误差。默认 Ki=0.8/s，有积分限幅、输入限幅和防饱和；没有读取靶心真值或直接改机器人姿态。旧单靶和运动学控制算法保留。
3. SETTLING/READY 冻结上一次**完整**姿态输入，保持 P+I；实际新图像仍须在靶心 8 mm 内，连续稳定至少 0.4 秒，真实角速度不大于 0.02 rad/s、速度不大于 0.01 m/s。输入非零本身不能证明静止。射击前检查原生反馈、SDK 模式 21、真实指令账本、同靶身份及完整保持窗口，再以当前真实物理状态做射线计分。站起后已恰好对准且一直没有非零指令时，也允许模式 1 下的全零静止窗口。
4. 去掉官方保持路径在射击前的两次清零；最终门控与射线/扣弹事务共用指令锁，避免后台停止指令插入事务。掉帧、失靶、无效观察、换靶、结束或异常仍撤销保持并停止。
5. SDK 同参数续发与识别解耦，配置周期 0.04 秒（25 Hz），由每 0.025 秒检查一次的线程执行；实际频率需由 Ubuntu 调用账本确认。续发保留原始指令时间和原图有效截止，绝不制造新采集时间。图像截止或反馈失鲜后，停止当前模式、取消续发；断连、SDK 错误或原始指令看门狗超时仍锁定故障。保持曾被零输入打断时，重建至少 0.4 秒的观察窗口，旧 READY 权限不能复用。官方运行循环不再在处理图像后额外固定等待 0.05 秒。
6. 持续有效精瞄在 12 秒内无至少 2 mm 改善，且已饱和或角误差几乎不变时，以 `aim_response_stalled` 失败并切换目标；不扣弹、不补分。图像不连续不能用来判定连续停滞。70 秒总目标超时仍保留，作为另一个边界。

本策略保持响应只绑定 `config/official_aim_profile.json` 中的 SDK、mc_ctrl、命令映射库、平衡 policy 与 odom 五个 SHA256。版本变化会拒绝沿用该响应假设。接实物前还须重新确认固件/策略、单轴响应、真实图像与反馈时间戳、独立急停和真实射击接口；本包仍只连接回环仿真，不会向实机发送命令。仿真射线得分仍是任务三自有裁判结果。

## Ubuntu 更新和运行

将新 ZIP 与 `.sha256` 放在同一目录，先结束上一场任务三，在该目录执行：

```bash
sha256sum -c task3_official_dynamics_032_20261019.zip.sha256
unzip -o task3_official_dynamics_032_20261019.zip -d "$HOME/robotac/matrix_robotac_first"
cd "$HOME/robotac/matrix_robotac_first"
source /opt/ros/humble/setup.bash
task3_sim/.venv/bin/python -c 'import sys,task3_sim; print(sys.version); print(task3_sim.__version__); assert sys.version_info[:2]==(3,10); assert task3_sim.__version__=="0.3.2"'
```

保留 Ubuntu 的 `.venv` 和 `task3_sim/local/native/`；本次不改 C++ 桥，不必重新编译或安装依赖。不要传 Windows 虚拟环境或旧运行生成的 XML。

先做可视运行：

```bash
bash task3_sim/run_official_task3.sh --preflight
bash task3_sim/run_official_task3.sh --show --viewer --max-seconds 240
```

终端新增 `TASK3_AIM id=... phase=... error_mm=... yaw_input=... pitch_input=...`，便于观察误差是否下降。成功应逐靶经过 AIMING/SETTLING/READY，三发对应三个正确 ID，余弹 2 发；最终分数以 `shots.json`、原生状态和独立验收为准。达到门槛前不射击是正确行为。结果保存后窗口保持显示，可按 Esc/Q 退出。

然后执行完整验收：

```bash
bash task3_sim/accept_official_ubuntu.sh
```

请回传最后打印的 `TASK3_RETURN_ARCHIVE ...tar.gz`。验收会先跑单元测试和依赖预检，再跑三场官方动力学（默认、交换身份/顺序、起点距离延迟掉帧），最后跑视觉回归。失败仍导出本次证据；不要单凭识别到 Tag、静态画面或 Windows 视觉回归认定步态通过。

## 开发机验证

本地结果和源码哈希见 `validation/official_fix032_local_checks.json`，35 场实际渲染视觉回归见 `validation/directional_actual_render_032.json`，官方源文件保护见 `validation/official_fix032_preservation.json`。PI 测试中的有界滞后模型只验证外环逻辑，不是官方控制器或真实机器狗的替代验收。

Windows 完整单元测试 312 项：309 通过、3 项 POSIX 信号测试按平台跳过；58 份 Python 源码通过 Python 3.10 语法检查，5 份 Bash 脚本通过语法检查。受保护的原工程 1,189 个文件未变，旧单靶 14 个文件/资产与入口函数 AST 未变。独立运动学命令行三靶各 10 环、余弹 2 发、退出码 0。所有这些均不替代 Ubuntu 官方步态射击验证。

实际 MuJoCo 图像闭环回归为 15 场固定 + 20 场预先声明的随机场景，**35/35 符合预期**，其中保留缺靶、无弹、超时等负例。97 发均命中正确 ID、10 环，最大实际偏差 8.961 mm；58 份 Python 源码在回归前后哈希一致。该回归的运动后端仍为原运动学视觉后端，不能据此认定本次 PI 已在官方原生策略中成功射击。
