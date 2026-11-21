#pragma once

#include "System.hpp"
#include "catch_block.hpp"

/**
 * @brief 取块机构在 Reactor70 中的单轴调试入口。
 * @note 框架注册时调用 Start，随后由应用任务周期调用 Update。
 */
class CatchBlockTest final : public Application
{
  SINGLETON(CatchBlockTest) : Application("CatchBlock")
  {
  }

  APPLICATION_OVERRIDE

private:
  CatchBlock::Controller controller_;
};
