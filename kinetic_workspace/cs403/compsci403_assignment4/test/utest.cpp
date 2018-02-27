#include "assignment4.h"
#include <gtest/gtest.h>

TEST(TestSuite, test_GenDiscDynWind)
{
	Eigen::Vector2f test_velocity (.5f, 1.5f);

	auto discwind = t_helpers::GenDiscDynWind(test_velocity, 10); ;

	ASSERT_GE(discwind.size(), 100);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}