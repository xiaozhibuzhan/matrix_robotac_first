# 官方宝箱实现定位与接入边界

日期：2026-10-09。只读检查当前官方发布包的资源、蓝图字节码、配置和 SDK 接口；未运行仿真，未修改官方文件。

**2026-10-15 更新：** 用户视频确认五箱放在场外，并明确授权修改官方宝箱位置。现已调整 `RobotAC_Level_1.uexp` 五个 `cash_lux` 根组件的 XY，原件已备份；本页以下蓝图和计分分析仍有效，相关蓝图未改。Ubuntu 应用、坐标表、校验和回退统一见 [场景位置修复](scene_tools/README.md)。10 月 9 日“未修改”描述属于当时审计，不再代表当前关卡资源状态。

**已找到宝箱实现，位于 UE 的 `BP_CashBox` 蓝图中。官方已实现重叠触发、宝箱消失和内部计分；自有导航扩展现已实现到达、受控接触和人工消失确认，完整官方现场触发仍待验收。当前操作见 [宝箱手册](navigation/TREASURE_UBUNTU.md)。**

## 1. 文件位置

以下均相对项目根目录；蓝图的 `.uasset` 与同名 `.uexp` 配套保存。

| 文件 | 已确认作用 |
| --- | --- |
| [BP_CashBox.uasset](src/UeSim/Linux/UeSim/Content/BluePrint/robotac/BP_CashBox.uasset) / [BP_CashBox.uexp](src/UeSim/Linux/UeSim/Content/BluePrint/robotac/BP_CashBox.uexp) | 宝箱 Actor、重叠事件、延迟、销毁自身及调用计分 |
| [BP_RobotACSCORE.uasset](src/UeSim/Linux/UeSim/Content/BluePrint/robotac/BP_RobotACSCORE.uasset) / [BP_RobotACSCORE.uexp](src/UeSim/Linux/UeSim/Content/BluePrint/robotac/BP_RobotACSCORE.uexp) | 宝箱内部计分、内部计时及其他任务计分 |
| [BP_RobotACStatistic.uasset](src/UeSim/Linux/UeSim/Content/BluePrint/robotac/BP_RobotACStatistic.uasset) / [BP_RobotACStatistic.uexp](src/UeSim/Linux/UeSim/Content/BluePrint/robotac/BP_RobotACStatistic.uexp) | 统计 UI，`SetScore` 将分数和耗时写入文本控件 |
| [RobotAC_Level_1.umap](src/UeSim/Linux/UeSim/Content/Maps/RobotAC/RobotAC_Level_1.umap) / [RobotAC_Level_1.uexp](src/UeSim/Linux/UeSim/Content/Maps/RobotAC/RobotAC_Level_1.uexp) | 当前打包关卡中的宝箱实例和计分对象 |
| [DefaultEngine.ini](src/UeSim/Linux/UeSim/Config/DefaultEngine.ini) 第 62、72 行 | 编辑器起始地图和游戏默认地图均指向 `RobotAC_Level_1` |

当前目录是打包发布内容，没有提供 UE 游戏模块的可读 C++ Source 目录或完整可编辑蓝图工程。这里的逻辑保存在编译后的蓝图资源中，因此仅搜索 `.cpp/.py` 或中文“宝箱”会漏掉它。不能将二进制资源当作普通文本源码直接编辑。

## 2. 宝箱触发与消失逻辑

已解析 `BP_CashBox` 的导入、导出及执行字节码，确认以下调用顺序：

```text
ReceiveActorBeginOverlap(OtherActor)
  → ExecuteUbergraph_BP_CashBox，入口 242
  → Delay(0.5)
  → 返回入口 236，跳到 15
  → GetAllActorsOfClass(BP_RobotACSCORE_C)
  → 如果计分对象数组为空，结束
  → K2_DestroyActor(self)
  → 取第 0 个计分对象，调用 UpdateBoxScore()
```

这里的触发来自 **UE Actor 的 BeginOverlap 重叠事件**，不是读取 RViz 的到达状态，也不是导航程序主动调用“领取宝箱”。实际测试应先验证机器人何时进入宝箱的重叠区域，以及随后是否消失，不应预设必须用较大速度撞击模型。

需要区分以下边界：

- `OtherActor` 被事件保存，但本宝箱执行图中没有进一步的机器人类型、距离或速度筛选。是否能产生重叠事件仍取决于双方碰撞组件、通道和配置，不能据此断言所有物体都会触发。
- 资源包含 `OverlapAll`、`cash_lux` 静态网格及相关碰撞通道名称；本次没有完整还原网格碰撞形状、机器人接触部件和触发区域尺寸，因此不能给出可靠的“距中心多少米即可成功”参数。
- 若场景没有计分对象，这条执行分支不会销毁宝箱。当前打包关卡中确实存在一个 `BP_RobotACSCORE_C` 实例。
- `ReceiveBeginPlay` 和 `ReceiveTick` 在这个宝箱蓝图中均进入空结束分支，没有另一个距离轮询触发逻辑。

证据见 [宝箱字节码解析](diagnostics/treasure_review_20261009/cashbox_bytecode.txt)。该执行图在 `.uexp` 的脚本起点为 `0x295`，序列化脚本长 337 字节、虚拟机脚本长 301 字节，解析长度与资源声明一致。事件入口与原始字节另见 [资源证据](diagnostics/treasure_review_20261009/asset_evidence.json)。

## 3. 当前关卡与官方内部计分

当前 `RobotAC_Level_1.umap` 导出表中有：

- 5 个 `BP_CashBox_C` 实例，名称为 `BP_CashBox_C_1` 至 `BP_CashBox_C_5`；
- 1 个 `BP_RobotACSCORE_C` 实例。

这是对**当前本地打包关卡**的核实，不代表比赛换场景后数量仍为 5，也不表示程序需要硬编码这 5 个宝箱的位置。

`BP_RobotACSCORE.UpdateBoxScore` 的实际逻辑包含：

```text
remaining_boxes = 场景中剩余 BP_CashBox_C 的数量
score = min(max(100 - remaining_boxes * 20, 0), 100)
timecost = GetGameTimeInSeconds(self) - starttime
```

其函数导出编号为 8，`.uexp` 函数起点为 `0x196c`，脚本范围为 `0x1d1d–0x21fd`。函数中还构造 JSON 对象并设置 `Level = "maze2"`，但没有在这条函数路径中发现向外发送、保存文件或打印成功事件的调用。

官方蓝图还存在内部计时：开始时查找带 `Engine` 标签的机器人；Tick 中，在其 `MainBody.RelativeLocation` 距原点超过 100 UE 单位且 `starttime == 0` 时记录游戏时间。宝箱更新时再计算 `timecost`。统计 UI 的 `SetScore` 会把分数与耗时写入文本控件。

**这个 0～100 内部分数及游戏时间口径，不能直接替代此前截图中任务二的 30 分赛制、整分钟时间奖励和裁判计时。** 后续应区分“官方仿真内部反馈”与“按正式赛规计算的记录”，不要因为找到该函数就改写比赛规则。

## 4. 是否已有外部成功反馈接口

目前可以确认官方 UE 内部有成功处理机制；**尚未找到可直接由自有导航程序订阅的“宝箱成功”ROS 话题、服务或 SDK 查询接口。**

已检查的范围包括：

- `BP_CashBox` 的完整执行图、`BP_RobotACSCORE` 的相关函数、统计 UI，以及 Content 资源中的相关引用；
- 官方 MuJoCo、运控目录中随包提供的可读文件与配置；
- [SDK highlevel.h](deps/zsibot_sdk/include/zsl-1/highlevel.h)，其中公开的是连接、运动、位姿/速度、电量和控制模式等接口，没有宝箱状态接口。

Score 资源内出现 ROS 订阅组件、UDP 组件和 JSON 名称，并不能证明已经对外发布宝箱成功状态。当前没有目标 Ubuntu 的实时话题清单或触发前后通信记录，也没有完整 UE 源码，因此不能排除尚未定位的原生组件或运行时接口。

`BP_ScoreRecorder` 另有 `Score / Finish / Failed / Render` 状态字段，但未发现其承接本次 CashBox 计分调用链的证据，不应仅凭文件名将其当作宝箱反馈接口。

## 5. 对后续工作的修正

1. **复用官方宝箱行为。** 不需要在自有程序里重新实现官方宝箱生成、删除或修改仿真得分。
2. **先验证一个宝箱的触发边界。** 用已稳定的路线到达附近，再进行受控低速接近，记录机器人位置、画面中消失时刻和导航雷达；核对是否在真实触发前被障碍保护拦停。
3. **确认成功反馈来源。** 首版继续由人观察宝箱消失并确认；目标 Ubuntu 上可只读对照触发前后的 ROS 话题、官方日志与统计界面，若找到稳定外部接口，再接入自动确认。
4. **验收已有的最终接近模式。** 当前扩展已限定接近距离、速度和时间，仅临时允许人工圈定的孤立目标区域；仍需用实际触发边界核对参数，保留圈外障碍和其他保护。
5. **记录仍需去重。** 点击目标、导航到达、官方宝箱触发和参赛者确认保持独立，未消失不记成功。

10 月 9 日仅新增检查文档和诊断证据。10 月 15 日新增位置修复只改关卡五箱 XY，未更改上述重叠事件、销毁、计分、运动控制或现场运行配置；实际 UE 加载、落地和触发仍需 Ubuntu 验收。
