#include <gtest/gtest.h>

#include <cstdio>

// Each registered fault scenario needs a fresh process, including its epoch.
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  const int selected = ::testing::UnitTest::GetInstance()->test_to_run_count();
  if (selected != 1) {
    std::fprintf(stderr, "Expected exactly one isolated test; selected %d\n", selected);
    return 2;
  }
  return result;
}
