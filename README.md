# 取块机构 · 八轴独立调试

`catch_block_test.cpp` 是应用入口。`Start()` 接入双 CAN 并初始化调试邮箱，`Update()` 调用一次机构控制周期。`catch_block` 处理 Keil Watch 命令，`catch_block_driver` 管理六台 M2006/C610 和两台 J4310。本版用于空载确认单轴方向、反馈和参数。

## 接入与接线

三个 `.cpp` 和配套头文件均已收录在 Keil 的 `Users` 组中。在 `MainFrame.cpp` 包含 `catch_block_test.hpp`，并在现有 `MainFrameCpp()` 中调用一次 `CatchBlock_Register()`。返回 `true` 表示已注册，返回 `false` 表示应用槽位不足。注册后每台电机仍须通过 `Init` 命令逐台初始化。原有通过 `catch_block.hpp` 调用注册函数的代码也可继续使用。

| Watch 轴号（左 / 右） | 功能 | CAN1 左侧 | CAN2 右侧 |
|---:|---|---|---|
| 0 / 3 | 滑车升降，ADRC 位置 | C610 ID 1 | C610 ID 1 |
| 1 / 4 | 齿条开合，ADRC 位置 | C610 ID 2 | C610 ID 2 |
| 2 / 5 | 皮带输送，ADRC 速度 | C610 ID 3 | C610 ID 3 |
| 6 / 7 | 长臂俯仰，位置＋速度 | J4310 设备 ID 0x07，控制帧 0x107，反馈 ID 0x17 | J4310 设备 ID 0x08，控制帧 0x108，反馈 ID 0x18 |

右侧 C610 从旧版的 4、5、6 改为 1、2、3。左右是独立总线，各自发送一组 `0x200` 电流指令。右侧四轴使用 A 板通用 CAN2 口 J4、J8、J11，其中一个口用短分支接两个节点。按官方针脚图配线，并在每条总线物理主干两端配置终端电阻。[DJI A 型板用户手册 §1.12](https://rm-static.djicdn.com/tem/RoboMaster%E5%BC%80%E5%8F%91%E7%89%88%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf)

应用启动时将已初始化的 CAN2 接入空闲的副 CAN 映射，或复用已有的 CAN2 映射。右侧初始化返回 `NoBus` 时，检查副路映射和 CAN2 外设状态。右侧轴的设备 ID 与左侧重复，接线和注册都须保持两路独立。

装配时核对两路总线的设备及反馈 ID 占用。J4310 的设备 ID 须预先设好，库的 `Init` 会按该 ID 寻址并写设备 ID、反馈 ID 和位置＋速度模式（`POSANDVEL`）。上机前用达妙调试助手核对设备与库的 PMAX、VMAX、TMAX 一致。当前库默认分别为 12.5 rad、30 rad/s、10 N·m，这三个数定义协议量程。单轴调参仅更新应用参数。

空载逐轴执行初始化、使能、运动和停止，记录每轴的实测方向。八台接入后分别抓取 CAN1 和 CAN2，核对每路只有一组 C610 电流命令，记录总线利用率、反馈间隔和错误计数。验证右路反馈中断时右侧相应轴撤销目标，恢复反馈后通过人工确认再使能。

## Keil Watch

添加三个表达式。轴名使用 `left_lift`、`left_clamp`、`left_feed`、`right_lift`、`right_clamp`、`right_feed`、`left_pitch`、`right_pitch`。

| 表达式 | 用法 |
|---|---|
| `CatchBlock::tuning` | 可写。展开轴名编辑一组候选参数，随后用 Apply 提交给该轴。源码默认值在 `catch_block_driver.cpp` 顶部。 |
| `CatchBlock::debug` | 可写输入为 `axis`、`command`、`target` 和 `submit`。最后一步将 `submit` 加一。等 `processed == submit` 后看 `result`，再编辑下一条。一次只提交一条。 |
| `CatchBlock::feedback` | 只观察。各轴分别显示状态、故障、参数是否已应用、反馈计数及年龄、最新测量和已接受目标。`fresh` 表示已有可用且近期更新的运动反馈。 |

| 命令值 | 动作 | 参数与生效条件 |
|---:|---|---|
| 1 | Init | 初始化所选轴，每次 MCU 启动只接受一次。 |
| 2 | Apply | Idle 状态整组应用该轴候选参数。校验失败保留上次有效参数。 |
| 3 | Enable | 要求参数有效且反馈新鲜。M2006 位置轴从当前位置开始，速度轴从零速开始，首个计算周期提交零电流。J4310 先向失能中的电机发送当前反馈角和零速字段，等到新反馈确认仍失能且角度未变才发使能。收到使能反馈后进入 Active。 |
| 4 | MoveRelative | 升降、开合采用相对当前读数的转子编码增量。俯仰的 `target.position` 为相对当前反馈的角度增量（rad），`target.velocity` 为正的运行限速（rad/s）。 |
| 5 | RunSpeed | 仅输送轴，`target.velocity` 为带符号的电机侧 rpm。 |
| 6 | MoveAbsolute | 仅俯仰轴。`target.position` 为反馈坐标中的目标角度（rad），`target.velocity` 为正的运行限速（rad/s）。 |
| 7 | Stop | 升降、开合锁定当前读数，输送目标降至零。俯仰以反馈角为保持目标，采用该轴配置的正限速。等待使能期间收到 Stop 会取消使能。 |
| 8 | Disable | 撤销该轴目标并请求失能。故障记录保留。 |
| 9 | AckFault | 人工处理后确认应用故障。须有新反馈，J4310 须报告已失能且设备无故障。确认后回到 Idle，运动需再次 Enable 并提交新目标。 |

**左升降调试示例。** 选 `axis=0`，提交 `Init`，让电机静止接收反馈。C610 前 51 帧用于建立偏置，从第 52 帧开始取得运动测量。填写 `tuning.left_lift`，提交 `Apply`，确认 `result=Ok`、`configured=true`、`fresh=true`。提交 `Enable`，再将本次增量写入 `target.position` 并提交 `MoveRelative`。观察 `feedback.left_lift.position_code` 和 `commanded.position`。只改 `target` 或重复同一提交号都不会重复移动。

## 参数与反馈口径

- **M2006** 每轴可调 `wo_hz`、`wc_hz`、`tracking_coeff`、`inertia_kg_m2`、`kt_nm_per_a`、`damping`、`current_limit_a`、`speed_limit_rpm`、`planned_speed_rpm`。位置轴另有 `max_step_code`。带宽和模型起点取自工程的 C610 ADRC 位置示例，尚未针对机构辨识。电流、速度、规划速度以及位置轴的单步上限初始留为零，需填写后 Apply。C610 库电流满量程为 10 A，具体机构允许值待定。电机转子一圈为 8192 code，单步上限只限制一条指令的增量，机械行程和毫米换算仍需标定。
- **J4310** 左右俯仰分别配置 `min_angle_rad`、`max_angle_rad`、`velocity_command_limit_rad_s` 和 `overspeed_trip_rad_s`。运行速率上限须大于零且不超过实测超速阈值，超速阈值须小于驱动的反馈速度量程。提交相对或绝对位置时，运动方向由目标角度决定，正反两个方向都填正的 `target.velocity`。零值、负值或超出本轴限速的命令返回 `InvalidTarget`，保留上次目标。
- **反馈** C610 展示累计 code、减速前 rpm 和按库量程换算的原始符号电流。`device_enabled` 对 C610 表示软件输出使能，对 J4310 表示设备反馈的使能位。J4310 展示角度 rad、速度 rad/s、力矩 N·m 和原始错误码。方向沿用电机坐标，长臂竖直基准、左右镜像符号和地块高度尚待标定。
- **J4310 使能握手** 预置只发送当前角度和精确的零速字段。位置＋速度模式的报文只有位置和限速两个字段。握手期间禁止新的运动目标。使能前检测到角度变化时锁存 `EnablePositionChanged`，等待反馈或使能状态超时也会撤销目标并发失能。收到使能反馈后，以应答中的角度开始保持，使用该轴配置的正限速。实机验收需抓包确认预置帧先于使能帧，并验证设备在失能期间采用该预置值。反馈帧只有实测值与状态，无法读回电机内部目标。
- **M2006 看门狗** 使能前将目标对齐到当前编码器位置，清零速度、电流和 ADRC 前馈缓存。使能后以 50 ms 检查本轴的有效反馈和电流计算回调间隔。实测转子转速也参与保护，`overspeed_trip_rpm` 可逐轴配置，填 0 时取 `speed_limit_rpm`，显式阈值须大于等于该转速上限。保护触发后，同一 `0x200` 分组仅把故障轴的槽位置零，应用任务撤销该轴目标并锁存故障。反馈恢复且转速回到允许范围后，须先 `AckFault`，再 `Enable`。
- **J4310 看门狗** 反馈、应用发令间隔和使能握手期限均为 50 ms。触发后在应用任务的下一次检查中请求失能、撤销旧目标并锁存故障。Idle/Fault 状态每 50 ms 发失能帧请求反馈，恢复需人工确认和重新使能。
- **设备端响应** 软件看门狗监控反馈和本地控制时序。现有发送接口无法确认单帧上总线。A 板停机、Keil 暂停或 CAN 断线时，停机依赖设备端通信保护。实机需验证 C610 失联输出与 J4310 的 `TIMEOUT`，并确认零电流或失能后的机构运动，尤其是负载下的升降轴。
