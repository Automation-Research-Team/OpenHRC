#include <gtest/gtest.h>
#include <ohrc_common/math_utility.h>

TEST(MathUtilityTest, SGN) {
  EXPECT_EQ(math_utility::sgn(2.0), 1.0);
  EXPECT_EQ(math_utility::sgn(-2.0), -1.0);
  EXPECT_EQ(math_utility::sgn(0.0), 0.0);

  EXPECT_EQ(math_utility::sgn(2), 1);
  EXPECT_EQ(math_utility::sgn(-2), -1);
  EXPECT_EQ(math_utility::sgn(0), 0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}