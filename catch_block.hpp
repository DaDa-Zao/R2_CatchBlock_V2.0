#pragma once

#include "catch_block_driver.hpp"

namespace CatchBlock
{
struct Mailbox
{
  Axis axis;
  Command command;
  Target target;
  uint32_t submit;    // Watch 写完轴、命令和目标后，最后更新提交号。
  uint32_t processed; // 应用写完结果后回填本次提交号。
  Result result;
};

// 单条命令邮箱。Watch 等待 processed 等于 submit 后再编辑下一条。
extern volatile Mailbox debug;

class Controller
{
public:
  /**
   * @brief 接入双 CAN 并对齐调试邮箱的提交号。
   * @note 应用启动时调用，电机按调试命令逐轴初始化。
   */
  void Init();

  /**
   * @brief 刷新八轴反馈并处理本周期的调试命令。
   * @note 由应用任务串行调用，每个提交号只处理一次。
   */
  void RunCycle();

private:
  uint32_t last_submit_ = 0;
};
}

/**
 * @brief 将取块应用注册到 Reactor70。
 * @note 在 MainFrameCpp() 中、RTOS 启动前调用。电机由 Init 命令逐台初始化。
 * @return 注册成功或已注册时返回 true，应用槽位不足时返回 false。
 */
bool CatchBlock_Register();
