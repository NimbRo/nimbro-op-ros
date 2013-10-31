// Unit testing of the utilities headers
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <cmath>
#include <gtest/gtest.h>
#include <utilities/utilities.h>
#include <config_server/SetParameter.h>

// Namespaces
using namespace util;

//
// MathConv tests
//

// Test: CubicSpline
TEST(MathConvTest, test_CubicSpline)
{
	// Create a cubic spline object
	CubicSpline cspline(2.0, 1.0, 4.0, 1.0, 0.8);

	// Test spline 1 (point symmetric cubic)
	EXPECT_DOUBLE_EQ(2.0, cspline(0.0));
	EXPECT_DOUBLE_EQ(3.0, cspline(0.4)); // Halfway point should be halfway due to symmetry
	EXPECT_DOUBLE_EQ(4.0, cspline(0.8));

	// Test spline 1 again with setParams method
	cspline.setParams(2.0, 1.0, 4.0, 1.0, 0.8);
	EXPECT_DOUBLE_EQ(2.0, cspline(0.0));
	EXPECT_DOUBLE_EQ(3.0, cspline(0.4)); // Halfway point should be halfway due to symmetry
	EXPECT_DOUBLE_EQ(4.0, cspline(0.8));

	// Test spline 1 again with static method
	EXPECT_DOUBLE_EQ(2.0, CubicSpline::eval(2.0, 1.0, 4.0, 1.0, 0.8, 0.0));
	EXPECT_DOUBLE_EQ(3.0, CubicSpline::eval(2.0, 1.0, 4.0, 1.0, 0.8, 0.4)); // Halfway point should be halfway due to symmetry
	EXPECT_DOUBLE_EQ(4.0, CubicSpline::eval(2.0, 1.0, 4.0, 1.0, 0.8, 0.8));

	// Test spline 2 (linear)
	cspline.setParams(1.0, 0.5, 2.0, 0.5, 2.0);
	EXPECT_DOUBLE_EQ(1.0, cspline(0.0));
	EXPECT_DOUBLE_EQ(1.2, cspline(0.4));
	EXPECT_DOUBLE_EQ(1.4, cspline(0.8));
	EXPECT_DOUBLE_EQ(1.6, cspline(1.2));
	EXPECT_DOUBLE_EQ(1.8, cspline(1.6));
	EXPECT_DOUBLE_EQ(2.0, cspline(2.0));

	// Test spline 3 (quadratic)
	cspline.setParams(0.0, 1.0, 0.0, -1.0, 1.0);
	EXPECT_DOUBLE_EQ(0.00, cspline(0.0));
	EXPECT_DOUBLE_EQ(0.16, cspline(0.2));
	EXPECT_DOUBLE_EQ(0.24, cspline(0.4));
	EXPECT_DOUBLE_EQ(0.25, cspline(0.5));
	EXPECT_DOUBLE_EQ(0.24, cspline(0.6));
	EXPECT_DOUBLE_EQ(0.16, cspline(0.8));
	EXPECT_DOUBLE_EQ(0.00, cspline(1.0));

	// Test that zero dT case doesn't make anything explode (Why should it? Other than div by 0... but still)
	EXPECT_NO_THROW(CubicSpline::eval(2.0, 1.0, 4.0, 1.0, 0.0, 1.0)); // Zero dT!
}

// Test: picut
TEST(MathConvTest, test_picut)
{
	// Check that we are getting the expected values
	EXPECT_DOUBLE_EQ(0.0, picut(-M_2PI));
	EXPECT_DOUBLE_EQ(M_PI_2, picut(-3.0*M_PI_2));
	EXPECT_DOUBLE_EQ(M_PI, picut(-M_PI));
	EXPECT_DOUBLE_EQ(-M_PI_2, picut(-M_PI_2));
	EXPECT_DOUBLE_EQ(0.0, picut(0.0));
	EXPECT_DOUBLE_EQ(M_PI_2, picut(M_PI_2));
	EXPECT_DOUBLE_EQ(M_PI, picut(M_PI));
	EXPECT_DOUBLE_EQ(-M_PI_2, picut(3.0*M_PI_2));
	EXPECT_DOUBLE_EQ(0.0, picut(M_2PI));
}

// Test: sign, sign0
TEST(MathConvTest, test_sign)
{
	// Check that we are getting the expected values (we want exact comparisons to be possible)
	EXPECT_EQ( 1.0, sign ( 5.0));
	EXPECT_EQ( 1.0, sign ( 0.0));
	EXPECT_EQ(-1.0, sign (-5.0));
	EXPECT_EQ( 1.0, sign0( 5.0));
	EXPECT_EQ( 0.0, sign0( 0.0));
	EXPECT_EQ(-1.0, sign0(-5.0));
}

// Test: coerce, coerceAbs, coerceMax, coerceMin
TEST(MathConvTest, test_coerce)
{
	// Check that we are getting the expected values (we want exact comparisons to be possible)
	EXPECT_EQ( 2.0, coerce(1.0, 2.0, 8.0));
	EXPECT_EQ( 5.0, coerce(5.0, 2.0, 8.0));
	EXPECT_EQ( 8.0, coerce(9.0, 2.0, 8.0));
	EXPECT_EQ(-8.0, coerceAbs(-9.0, 8.0));
	EXPECT_EQ( 1.0, coerceAbs( 1.0, 8.0));
	EXPECT_EQ( 8.0, coerceAbs( 9.0, 8.0));
	EXPECT_EQ(-5.0, coerceMax( 0.0,-5.0));
	EXPECT_EQ(-8.0, coerceMax(-8.0,-5.0));
	EXPECT_EQ( 0.0, coerceMin( 0.0,-5.0));
	EXPECT_EQ(-5.0, coerceMin(-8.0,-5.0));
}

// Test: zeroZ, withZ
TEST(MathConvTest, test_zeroZ_withZ)
{
	// Check that we are getting the expected values for zeroZ
	Eigen::Vector2d vec2d(1.41, 2.97);
	Eigen::Vector3d vec3da = zeroZ(vec2d);
	EXPECT_EQ(vec2d.x(), vec3da.x());
	EXPECT_EQ(vec2d.y(), vec3da.y());
	EXPECT_EQ(0.0      , vec3da.z());
	
	// Check that we are getting the expected values for withZ
	Eigen::Vector3d vec3db = withZ(vec2d, 3.83);
	EXPECT_EQ(vec2d.x(), vec3db.x());
	EXPECT_EQ(vec2d.y(), vec3db.y());
	EXPECT_EQ(3.83     , vec3db.z());
}

// Test: eigenToTF
TEST(MathConvTest, test_eigenToTF)
{
	// Check that we are getting the expected vector conversion
	Eigen::Vector3d eigenvec(1.68, 2.03, 3.25);
	tf::Vector3 tfvec = eigenToTF(eigenvec);
	EXPECT_EQ(eigenvec.x(), tfvec.x());
	EXPECT_EQ(eigenvec.y(), tfvec.y());
	EXPECT_EQ(eigenvec.z(), tfvec.z());
	EXPECT_EQ(0.0         , tfvec.w());
	
	// Check that we are getting the expected matrix conversion
	Eigen::Matrix3d eigenmat;
	eigenmat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
	tf::Matrix3x3 tfmat = eigenToTF(eigenmat);
	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
			EXPECT_EQ(eigenmat(i,j), tfmat[i][j]);
}

//
// Plotting tests
//

// Test: PlotManager
TEST(PlottingTest, test_PlotManager)
{
	// Test the plot manager naming
	PlotManager PM1;
	PlotManager PM2("");
	PlotManager PM3("/");
	PlotManager PM4("node1");
	PlotManager PM5("/node2");
	PlotManager PM6("/node3/");
	PlotManager PM7("~");
	PlotManager PM8("~subnode");
	PlotManager PM9("~moresub/");
	EXPECT_STREQ(PlotManager::DEFAULT_BASENAME.c_str(), PM1.getBasename().c_str());
	EXPECT_STREQ(PlotManager::DEFAULT_BASENAME.c_str(), PM2.getBasename().c_str());
	EXPECT_STREQ("/", PM3.getBasename().c_str());
	EXPECT_STREQ("/node1/", PM4.getBasename().c_str());
	EXPECT_STREQ("/node2/", PM5.getBasename().c_str());
	EXPECT_STREQ("/node3/", PM6.getBasename().c_str());
	EXPECT_STREQ("/utilities_test/", PM7.getBasename().c_str());
	EXPECT_STREQ("/utilities_test/subnode/", PM8.getBasename().c_str());
	EXPECT_STREQ("/utilities_test/moresub/", PM9.getBasename().c_str());
}

//
// ROS timing tests
//

// Test: RosTimeMarker
TEST(RosTimingTest, test_RosTimeMarker)
{
	// Declare variables
	double elapsed;

	// Create an instance of a RosTimeMarker
	RosTimeMarker RTM;

	// Verify initial state
	EXPECT_FALSE(RTM.haveMarker());
	EXPECT_LT(RTM.getElapsed(), 0.0);
	EXPECT_TRUE(RTM.hasElapsed(1000.0));

	// Set a marker and verify the changes
	ros::Time start = ros::Time::now();
	RTM.setMarker();
	EXPECT_TRUE(RTM.haveMarker());
	EXPECT_GE(RTM.getElapsed(), 0.0);
	EXPECT_FALSE(RTM.hasElapsed(1000.0)); // Assume 1000 seconds hasn't passed yet...!
	
	// Wait for 500ms to elapse
	while(!RTM.hasElapsed(0.5))
	{
		// Avoid infinite looping...
		if((elapsed = (ros::Time::now() - start).toSec()) > 0.7)
			break;
	}

	// Check that the amount of elapsed time is reasonable
	EXPECT_GE(elapsed, 0.49);
	EXPECT_LE(elapsed, 0.70);
	EXPECT_GE(RTM.getElapsed(), 0.49);
	EXPECT_LE(RTM.getElapsed(), 0.69); // Being a wee bit stricter here... (see infinite loop avoidance above)

	// Unset the marker and verify the changes
	RTM.unsetMarker();
	EXPECT_FALSE(RTM.haveMarker());
	EXPECT_LT(RTM.getElapsed(), 0.0);
	EXPECT_TRUE(RTM.hasElapsed(1000.0));
}

// Test: RosTimeTracker
TEST(RosTimingTest, test_RosTimeTracker)
{
	// Declare variables
	double elapsed;
	std::size_t m, N = 3;

	// Create an instance of a RosTimeTracker
	RosTimeTracker RTT(N);

	// Verify initial state
	for(m = 0;m < N;m++)
	{
		EXPECT_FALSE(RTT.haveMarker(m));
		EXPECT_LT(RTT.getElapsed(m), 0.0);
		EXPECT_TRUE(RTT.hasElapsed(m, 1000.0));
	}

	// Set some markers and verify the changes
	ros::Time start = ros::Time::now();
	for(m = 0;m < N;m++)
	{
		RTT.setMarker(m);
		EXPECT_TRUE(RTT.haveMarker(m));
		EXPECT_GE(RTT.getElapsed(m), 0.0);
		EXPECT_FALSE(RTT.hasElapsed(m, 1000.0)); // Assume 1000 seconds hasn't passed yet...!
	}

	// Wait for 500ms to elapse on marker 0
	while(!RTT.hasElapsed(0, 0.5))
	{
		// Avoid infinite looping...
		if((elapsed = (ros::Time::now() - start).toSec()) > 0.7)
			FAIL();
	}
	EXPECT_GE(elapsed, 0.49);

	// Wait for 800ms to elapse on marker 1
	while(!RTT.hasElapsed(1, 0.8))
	{
		// Avoid infinite looping...
		if((elapsed = (ros::Time::now() - start).toSec()) > 1.0)
			FAIL();
	}
	EXPECT_GE(elapsed, 0.79);

	// Wait for 1100ms to elapse on marker 2
	while(!RTT.hasElapsed(2, 1.1))
	{
		// Avoid infinite looping...
		if((elapsed = (ros::Time::now() - start).toSec()) > 1.3)
			FAIL();
	}
	EXPECT_GE(elapsed, 1.09);

	// Unset the markers and verify the changes
	for(m = 0;m < N;m++)
	{
		RTT.unsetMarker(m);
		EXPECT_FALSE(RTT.haveMarker(m));
		EXPECT_LT(RTT.getElapsed(m), 0.0);
		EXPECT_TRUE(RTT.hasElapsed(m, 1000.0));
	}
}

// Test: RosServiceCaller
TEST(RosTimingTest, test_RosServiceCaller)
{
	// Lacking something about the RosServiceCaller class that you can really test, we just test the
	// instantiation thereof and the negative response when a dummy service client is used.
	RosServiceCaller<config_server::SetParameter> RSC(0.6, 0.3);
	EXPECT_FALSE(RSC.callService());
	ros::ServiceClient m_srv_dummy;
	RSC.setServiceClient(m_srv_dummy);
	EXPECT_FALSE(RSC.callService()); // This should be within the 0.3s delay and automatically return false
}

//
// Main function
//
int main(int argc, char **argv)
{
	// Initialise ROS node
	ros::init(argc, argv, "utilities_test");
	
	// Run the required tests
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF