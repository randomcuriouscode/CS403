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

TEST(TestSuite, test_PointIsObstacle)
{
	Eigen::Vector2f radius (R_ROBOT, 0);

	t_helpers::ObstacleInfo oi;
	Eigen::Vector2f p (1,0); 

	ASSERT_TRUE(PointIsObstacle(p, .5f, 0.f, oi));

	ASSERT_EQ(oi.f(), (p - radius).norm());

	p = Eigen::Vector2f(1/3, 1/3);

	ASSERT_TRUE(PointIsObstacle(p, .5f, 1.5f, oi));

	ASSERT_TRUE(oi.f() > 1.3 && oi.f() < 1.4);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}