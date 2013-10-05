// Unit test for robotcontrol::GolayDerivative class
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/model/golay.h>

#include <gtest/gtest.h>

TEST(GolayDerivativeTest, testEmpty)
{
	robotcontrol::GolayDerivative<double, 1> d;

	ASSERT_FLOAT_EQ(0.0, d.value());

	d.put(1.0);

	ASSERT_FLOAT_EQ(0.0, d.value());
}

TEST(GolayDerivativeTest, testFirstDerivative)
{
	robotcontrol::GolayDerivative<double, 1> d;

	for(int i = 0; i < 9; ++i)
		d.put(i);

	ASSERT_FLOAT_EQ(1.0, d.value());
}

TEST(GolayDerivativeTest, testSecondDerivative_zero)
{
	robotcontrol::GolayDerivative<double, 2> d;

	for(int i = 0; i < 9; ++i)
		d.put(i);

	ASSERT_FLOAT_EQ(0.0, d.value());
}

TEST(GolayDerivativeTest, testSecondDerivative_constant)
{
	robotcontrol::GolayDerivative<double, 2> d;

	for(int i = 0; i < 9; ++i)
		d.put(i*i);

	ASSERT_FLOAT_EQ(2.0, d.value());
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
