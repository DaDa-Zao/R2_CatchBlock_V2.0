#pragma once

#include <stdint.h>

namespace CatchBlock
{
// 轴号和命令值供 Keil Watch 使用，顺序与 README 一致。
enum class Axis : uint32_t
{
  LeftLift,
  LeftClamp,
  LeftFeed,
  RightLift,
  RightClamp,
  RightFeed,
  LeftPitch,
  RightPitch
};

enum class Command : uint32_t
{
  None,
  Init,
  Apply,
  Enable,
  MoveRelative,
  RunSpeed,
  MoveAbsolute,
  Stop,
  Disable,
  AckFault
};

enum class Result : uint32_t
{
  Ok,
  InvalidAxis,
  InvalidCommand,
  WrongState,
  AlreadyInitialized,
  NoBus,
  NotConfigured,
  InvalidTuning,
  NoFeedback,
  InvalidTarget,
  WrongAxisType,
  DeviceFault,
  RegistrationFailed
};

enum class State : uint8_t
{
  Uninitialized,
  Idle,
  Enabling,
  Active,
  Fault,
  InitFailed
};

enum class Fault : uint8_t
{
  None,
  FeedbackTimeout,
  DeviceError,
  EnableTimeout,
  UnexpectedDisable,
  PitchRange,
  Overspeed,
  CommandTimeout,
  EnablePositionChanged
};

struct AdrcTuning
{
  float wo_hz;
  float wc_hz;
  float tracking_coeff;
  float inertia_kg_m2;
  float kt_nm_per_a;
  float damping;
  float current_limit_a;
  float speed_limit_rpm;
  float planned_speed_rpm;
  float max_step_code; // 位置轴单次相对移动的转子编码增量上限。
  float overspeed_trip_rpm; // 实测转子转速阈值，零表示使用 speed_limit_rpm。
};

struct PosVelTuning
{
  float min_angle_rad;
  float max_angle_rad;
  float velocity_command_limit_rad_s; // 运行速率上限，保持当前位置时也采用此限速。
  float overspeed_trip_rad_s;         // 实测速度超过此值触发故障。
};

struct Tuning
{
  AdrcTuning left_lift;
  AdrcTuning left_clamp;
  AdrcTuning left_feed;
  AdrcTuning right_lift;
  AdrcTuning right_clamp;
  AdrcTuning right_feed;
  PosVelTuning left_pitch;
  PosVelTuning right_pitch;
};

struct Target
{
  float position;       // C610 转子编码值或 J4310 反馈坐标角度（rad）。
  float velocity;       // C610 转子有符号转速（rpm）；俯仰轴正的运行限速（rad/s）。
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
  int32_t position_code;
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

// Watch 编辑候选参数，应用任务在 Apply 时读取选中轴。
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
Result Execute(Axis axis, Command command, const Target &target, uint32_t now_ms);

/**
 * @brief 刷新八轴反馈并维护使能和故障状态。
 * @note 每个应用周期在处理新命令前调用，已使能的俯仰轴在此发送位置和限速。
 */
void Update();
}
}
