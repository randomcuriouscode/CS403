#include "assignment4.h"
#include <gtest/gtest.h>

#define GTEST

TEST(TestSuite, test_GenDiscDynWind)
{
	Eigen::Vector2f test_velocity (0, 0);

	auto discwind = t_helpers::GenDiscDynWind(test_velocity, 10);

	for (auto it = discwind.begin(); it != discwind.end() ; it++)
	{
		std::cerr << "[" << *it << "]" << "" << std::endl;
	}

	ASSERT_GE(discwind.size(), 100);

	test_velocity = Eigen::Vector2f (.5f, -1.5f);

    discwind = t_helpers::GenDiscDynWind(test_velocity, 10);

	ASSERT_GE(discwind.size(), 100);

	test_velocity = Eigen::Vector2f(0, 1.5f);

	discwind = t_helpers::GenDiscDynWind(test_velocity, 10);

	ASSERT_GE(discwind.size(), 100);

	test_velocity = Eigen::Vector2f(0, -1.5f);

	discwind = t_helpers::GenDiscDynWind(test_velocity, 10);

	ASSERT_GE(discwind.size(), 100);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}