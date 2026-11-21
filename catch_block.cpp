#include "catch_block.hpp"
#include "bsp_halport.hpp"

namespace CatchBlock
{
volatile Mailbox debug{};

void Controller::Init()
{
  Driver::InitBuses();
  last_submit_ = debug.submit;
  debug.processed = last_submit_;
}

void Controller::RunCycle()
{
  Driver::Update();

  const uint32_t sequence = debug.submit;
  if (sequence == last_submit_)
  {
    return;
  }

  const Axis axis = debug.axis;
  const Command command = debug.command;
  const Target target{debug.target.position, debug.target.velocity};
  // 拷贝期间提交号改变，下一周期重新读取整条命令。
  if (sequence != debug.submit)
  {
    return;
  }

  last_submit_ = sequence;
  debug.result = Driver::Execute(axis, command, target, HAL_GetTick());
  debug.processed = sequence;
}
}
