#include "catch_block_test.hpp"

void CatchBlockTest::Start()
{
  controller_.Init();
}

void CatchBlockTest::Update()
{
  controller_.RunCycle();
}

bool CatchBlock_Register()
{
  static bool registered = false;
  if (!registered)
  {
    registered = System.RegistApp(CatchBlockTest::GetInstance());
  }
  return registered;
}
