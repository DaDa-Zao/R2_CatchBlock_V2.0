#pragma once

#include <stdint.h>

namespace CatchBlock
{
// 轴号供 Keil Watch 选轴，顺序与 README 一致。
enum class Axis : uint32_t
{
  LeftLift,   // 左滑车升降
  LeftClamp,  // 左夹爪开合
  LeftFeed,   // 左皮带输送
  RightLift,  // 右滑车升降
  RightClamp, // 右夹爪开合
  RightFeed,  // 右皮带输送
  LeftPitch,  // 左长臂俯仰
  RightPitch  // 右长臂俯仰
};

// Watch 单轴命令，编号与 README 一致。
enum class Command : uint32_t
{
  None,         // 空命令
  Init,         // 初始化所选轴
  Apply,        // 应用本轴调参值
  Enable,       // 请求使能
  MoveRelative, // 相对当前位置移动
  RunSpeed,     // 按目标转速输送
  MoveAbsolute, // 俯仰移至绝对角度
  Stop,         // 停止当前动作
  Disable,      // 撤销目标并失能
  AckFault      // 人工确认故障
};

// 单次命令的处理结果。
enum class Result : uint32_t
{
  Ok,                 // 命令已接受
  InvalidAxis,        // 轴号无效
  InvalidCommand,     // 命令编号无效
  WrongState,         // 当前状态不允许此命令
  AlreadyInitialized, // 本轴已执行过初始化
  NoBus,              // 所选轴的 CAN 总线不可用
  NotConfigured,      // 本轴参数尚未应用
  InvalidTuning,      // 调参值未通过校验
  NoFeedback,         // 缺少有效的新反馈
  InvalidTarget,      // 目标无效或当前位置越界
  WrongAxisType,      // 本轴不支持此动作
  DeviceFault,        // 设备状态不满足操作条件
  RegistrationFailed  // 电机注册失败
};

// 单轴的当前运行状态。
enum class State : uint8_t
{
  Uninitialized, // 等待初始化
  Idle,          // 空闲，等待使能
  Enabling,      // 使能握手中
  Active,        // 已使能，可执行动作
  Fault,         // 故障已锁存
  InitFailed     // 初始化失败
};

// 单轴锁存的故障原因。
enum class Fault : uint8_t
{
  None,                  // 无故障
  FeedbackTimeout,       // 电机反馈超时
  DeviceError,           // 设备反馈状态异常
  EnableTimeout,         // 使能握手超时
  UnexpectedDisable,     // 运行中设备意外失能
  PitchRange,            // 俯仰实测角度越界
  Overspeed,             // 实测转速超限
  CommandTimeout,        // 控制输出更新超时
  EnablePositionChanged  // 预置后、使能前位置改变
};

struct AdrcParameters
{
  float wo_hz;          // 观测器带宽。
  float wc_hz;          // 控制带宽。
  float tracking_coeff;
  float inertia_kg_m2;
  float kt_nm_per_a;
  float damping;
  float planned_speed_rpm; // 规划转速，零表示沿用本轴转速上限。
};

struct AdrcTuning
{
  float current_limit_a; // 输出电流上限。
  float speed_limit_rpm; // 指令转速上限，单位为转子 rpm。
  float max_step_code; // 位置轴单次相对移动的转子编码增量上限。
  float overspeed_trip_rpm; // 实测转子转速阈值，零表示使用 speed_limit_rpm。
  AdrcParameters adrc;    // 按需展开，初次空载调试可沿用默认参数。
};

struct PosVelTuning
{
  float velocity_command_limit_rad_s; // 运行速率上限，保持当前位置时也采用此限速。
  float overspeed_trip_rad_s;         // 实测速度超过此值触发故障。
};

struct PitchLimits
{
  float min_angle_rad; // 电机原始反馈坐标中的最小允许角度。
  float max_angle_rad; // 电机原始反馈坐标中的最大允许角度。
};

// 各电机的安装方向，只允许 +1 或 -1。
struct AxisDirections
{
  int32_t left_lift;
  int32_t left_clamp;
  int32_t left_feed;
  int32_t right_lift;
  int32_t right_clamp;
  int32_t right_feed;
  int32_t left_pitch;
  int32_t right_pitch;
};

struct Tuning
{
  AdrcTuning lift;
  AdrcTuning clamp;
  AdrcTuning feed;
  PosVelTuning pitch;
  AxisDirections directions;
  PitchLimits left_pitch_limits;
  PitchLimits right_pitch_limits;
};

struct Target
{
  float position; // 按轴方向换算后的转子编码值或俯仰角度（rad）。
  float velocity; // 按轴方向换算后的转子转速（rpm），或俯仰轴的正限速（rad/s）。
};

struct Feedback
{
  State state;
  Fault fault;
  bool configured;
  bool fresh;
  bool device_enabled; // C610 为软件输出使能，J4310 为设备反馈位。
  uint32_t frames;
  uint32_t age_ms;
  int64_t position_code; // 方向换算后的转子累计编码值。
  float speed_rpm;
  float current_a; // 保留 C610 原始反馈电流的符号。
  float angle_rad;
  float velocity_rad_s;
  float torque_nm;
  uint8_t device_error;
  Target commanded;
};

struct FeedbackSet
{
  Feedback left_lift;
  Feedback left_clamp;
  Feedback left_feed;
  Feedback right_lift;
  Feedback right_clamp;
  Feedback right_feed;
  Feedback left_pitch;
  Feedback right_pitch;
};

// 左右同类轴共享候选参数，Apply 将其与所选轴的安装参数一并应用。
extern volatile Tuning tuning;

// 应用任务更新八轴状态，Watch 只读。
extern FeedbackSet feedback;

namespace Driver
{
/**
 * @brief 将已初始化的 CAN2 接入 Reactor70 的副 CAN。
 * @note 在应用启动时、首次注册电机前调用，保留已存在的副 CAN 映射。
 */
void InitBuses();

/**
 * @brief 执行指定轴的一条调试命令。
 * @note 由应用任务串行调用。相对位置取该轴最近一次有效反馈为基准。
 * @return Ok 表示已接受命令，设备使能状态另见反馈。
 */
Result Execute(Axis axis, Command command, const Target &target);

/**
 * @brief 刷新八轴反馈并维护使能和故障状态。
 * @note 每个应用周期在处理新命令前调用，已使能的俯仰轴在此发送位置和限速。
 */
void Update();
}
}
