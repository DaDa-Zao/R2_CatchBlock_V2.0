# 取块机构

单块 DJI A 板通过 CAN1 控制左侧、CAN2 控制右侧，每路接三台 M2006/C610 和一台 J4310。当前版本用于空载逐轴调试。

## 工程接入

Keil 的 `Users` 组已包含本模块。`catch_block_test.cpp` 提供 `Start()` 和 `Update()`。启动时需在 `MainFrameCpp()` 中调用一次 `CatchBlock_Register()`，声明见 `catch_block_test.hpp`。

## CAN 接线

| 功能 | Watch 轴号 左 / 右 | CAN1 左侧 | CAN2 右侧 |
|---|---:|---|---|
| 升降 · ADRC 位置 | 0 / 3 | C610 ID 1 | C610 ID 1 |
| 开合 · ADRC 位置 | 1 / 4 | C610 ID 2 | C610 ID 2 |
| 输送 · ADRC 速度 | 2 / 5 | C610 ID 3 | C610 ID 3 |
| 俯仰 · 位置＋速度 | 6 / 7 | J4310 ID `0x07`，反馈 `0x17` | J4310 ID `0x08`，反馈 `0x18` |

两路总线独立接线，各自主干两端接终端电阻。J4310 的设备 ID 须预先设好，初始化时写入位置＋速度模式和反馈 ID。上机前核对电机的 PMAX、VMAX、TMAX 与驱动库一致。右侧返回 `NoBus` 时检查 CAN2 初始化和副 CAN 映射。

## Keil Watch 调试

- `CatchBlock::tuning` 填参数。左右同功能轴共用一组，各自在 `Idle` 时执行一次 `Apply`。运行中的轴沿用上次生效的值。
- `CatchBlock::debug` 发命令。填写 `axis`、`command` 和所需的 `target`，最后把 `submit` 加一。等 `processed == submit` 后查看 `result`。
- `CatchBlock::feedback` 查看各轴的状态、故障和实测值。首次空载调试先 `Init`，填参数后 `Apply`。确认 `configured` 和 `fresh` 后执行 `Enable`，等状态变为 `Active` 再发运动命令。

| 参数组 | 首次调试要填的字段 |
|---|---|
| `lift`、`clamp` | `current_limit_a`（大于 0 且不超过 10 A）、`speed_limit_rpm`、`max_step_code` |
| `feed` | `current_limit_a`、`speed_limit_rpm` |
| `pitch` | `velocity_command_limit_rad_s`、`overspeed_trip_rad_s`。运行限速须为正，且不超过超速阈值 |
| `left_pitch_limits`、`right_pitch_limits` | 分别填写电机原始坐标下的 `min_angle_rad`、`max_angle_rad` |

三个 M2006 组各有一个 `adrc`，核心字段为 `wo_hz`、`wc_hz`、`inertia_kg_m2`、`kt_nm_per_a`、`damping`。在 `directions` 下逐轴设方向（`+1` 沿用电机方向，`-1` 反向）。

| 命令值 | 命令 | 用法 |
|---:|---|---|
| 1 | `Init` | 初始化选中的轴 |
| 2 | `Apply` | 在 `Idle` 时应用参数 |
| 3 | `Enable` | 按当前编码器位置或零速启动 |
| 4 | `MoveRelative` | 升降、开合填 `target.position`（转子编码增量，8192 code/圈）。俯仰填相对角度（rad）和正限速（rad/s） |
| 5 | `RunSpeed` | 输送填 `target.velocity`（带符号的转子 rpm） |
| 6 | `MoveAbsolute` | 俯仰填 `target.position`（rad）和 `target.velocity`（正限速，rad/s） |
| 7 | `Stop` | 位置轴保持当前位置，输送降到零速 |
| 8 | `Disable` | 撤销目标并请求失能 |
| 9 | `AckFault` | 排除故障、恢复反馈后确认。J4310 须先报告已失能且无设备故障 |

两类电机均有 50 ms 软件超时保护。故障恢复后须重新 `Enable` 并提交目标。上机先空载核对方向、J4310 使能前的当前位置和零速预置，以及两路 CAN 的利用率和设备失联保护。
