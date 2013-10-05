// Behaviour Control Framework - Go Behind Ball behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/control_layer/go_behind_ball.h>
#include <soccer_behaviour/control_layer/control_layer.h>

#define WAK_TINC			0.100	// Loop at 10Hz nominally...
#define DECAY_TIME			6.0  	// Keep data for 3 seconds before discarding it as too old
#define S_OFFSET			0.35  	// The x offset additional to X_OFFSET at which the robot should start slowing down
#define X_OFFSET			0.35 	// The target x offset to the ball
#define KICK_OFFSET 		0.38	// The maximum distance to allow start kicking
#define Y_OFFSET			-0.13	// The target y offset to the ball (+ve => left of the mid-sagital plane)
#define CMD_X_WALK			0.4  	// Gait command for normal walking speed
#define CMD_X_TIPTOE		0.2	// Gait command for tip-toeing forward
#define CMD_X_STOP			0.0	// Gait command to halt motion in x direction
#define CMD_W_CCW			0.4		// Turn on the spot angular command
#define CMD_X_MAX			0.7 	// Maximum x velocity gait command
#define CMD_Y_MAX			0.7 	// Maximum y velocity gait command
#define CMD_W_MAX			0.7 	// Maximum w velocity gait command
#define ALPHA_GAIN			3.0 	// Proportional gain for alpha control
#define BETA_GAIN			1.5 	// Proportional gain for beta control
#define ALPHA_EPS			0.1 	// Angle accuracy to which to line up the kick to
#define BETA_EPS			0.1 	// Angle accuracy to which to line up the goal to before starting a kick
#define KICK_DELAY			1.0 	// Delay between gait stopping and kick starting
#define POST_KICK_DELAY		1.0 	// Delay between end of kick and starting to walk towards ball again

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// Constructors
//

// Constructor
GoBehindBall::GoBehindBall(ControlLayer* L) : Behaviour(L, "GoBehindBall"), L(L), M(L->M), SM(L->SM), AM(L->AM)
{
}

//
// Function overrides
//

// Initialisation function
ret_t GoBehindBall::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Update function
void GoBehindBall::update()
{
}

// Compute activation level function
level_t GoBehindBall::computeActivationLevel()
{
	geometry_msgs::PointStampedConstPtr ballInfo = SM->ballPosition.read();
	return !!ballInfo && ((ros::Time::now() - ballInfo->header.stamp) < ros::Duration(3.0));
}

// Execute function
void GoBehindBall::execute()
{
	if(wasJustActivated())
		ROS_WARN("   GO BEHIND BALL just activated!");

	geometry_msgs::PointStampedConstPtr ballInfo = SM->ballPosition.read();
	gait::GaitCommand cmd;

	// Save the horizontal offset to the ball in local (short) variables
	double x = ballInfo->point.x;
	double y = ballInfo->point.y;

	// Calculate the length squared of the offset ray vector
	double BLen_sq = x*x + y*y;
	double SLen_sq = BLen_sq - Y_OFFSET*Y_OFFSET;

	double alpha;
	double beta;
	double SLen;

	// Avoid complex numbers
	if(SLen_sq > 0)
	{
		// Calculate offset ray angle (alpha)
		SLen = sqrt(SLen_sq); // Length of offset ray vector
		alpha = asin((SLen*y-x*Y_OFFSET)/BLen_sq);

		// Check what data we have available
		bool haveCompass = false;
		double compassGoal = L->compassGoalTarget(haveCompass); // This also returns in variable haveCompass whether we have a compass reading
		bool haveGoalPos = L->goalIsValid();
		// Note: m_lastGoalPos contains the data if haveGoalPos is true

		// Calculate offset ray to goal angle (beta)
		if(haveGoalPos)
		{
			Eigen::Vector2d target = L->m_lastGoalPos;
// 			ROS_INFO_STREAM_THROTTLE(0.2, "Using target m_lastGoalPos = " << target.transpose());
			double Ex = target.x() - x;
			double Ey = target.y() - y;
			double Sx = SLen * cos(alpha);
			double Sy = SLen * sin(alpha);
			beta = atan2(Sx*Ey-Sy*Ex,Sx*Ex+Sy*Ey);
// 			ROS_INFO_STREAM_THROTTLE(0.2, "Using goal detection: Calculated beta = " << beta);
		}
		else if(haveCompass)
		{
			beta = -alpha - compassGoal;
// 			ROS_INFO_STREAM_THROTTLE(0.2, "Using compass info: Heading = " << compassGoal);
// 			ROS_INFO_STREAM_THROTTLE(0.2, "Using compass info: Calculated beta = " << beta);
		}
		else
		{
// 			ROS_INFO_STREAM_THROTTLE(0.2, "Don't have compass OR goal!");
			beta = 0.0;
		}
		
		//beta = 0.0; // TODO: Comment this out - this makes the robot ignore beta control

		// Decide on an appropriate action
		if(SLen > X_OFFSET + S_OFFSET)
		{
			cmd.gcvX = CMD_X_WALK;
			cmd.gcvY = 0.0;
			cmd.gcvZ = ALPHA_GAIN * alpha;
			cmd.walk = true;
		}
		else if(SLen > X_OFFSET)
		{
			double u = (SLen - X_OFFSET)/S_OFFSET;
			cmd.gcvX = CMD_X_TIPTOE + u * (CMD_X_WALK - CMD_X_TIPTOE); // Ramp down x velocity for distance control
			cmd.gcvY = (1 - u) * (-BETA_GAIN * beta); // Ramp up y velocity for beta control
			cmd.gcvZ = ALPHA_GAIN * alpha; // Alpha control
			cmd.walk = true;
		}
		else
		{
			cmd.gcvX = CMD_X_STOP; // Keep distance the same (as we can't move backwards it seems)
			cmd.gcvY = -BETA_GAIN * beta; // Beta control
			cmd.gcvZ = ALPHA_GAIN * alpha; // Alpha control
			cmd.walk = true;
		}
	}
	else // Ball is closer to robot than |Y_OFFSET|...
	{
		// Define alpha as the angle directly to the ball in this case
		alpha = atan2(y,x);

		// Try to turn towards the ball (you're probably just going to kick it away by accident anyway)
		cmd.gcvX = CMD_X_STOP;
		cmd.gcvY = 0.0; // TODO: Beta control?? (and check beta in the IF below...)
		cmd.gcvZ = ALPHA_GAIN * alpha;

		// Keep walking...
		cmd.walk = true;
		ROS_INFO_THROTTLE(1, "I'm too close to the ball, but I'm trying to face it anyway!");
	}

	// Obstacle avoidance
	soccer_vision::DetectionsConstPtr detections = SM->visionDetections.read();
	if(detections)
	{
		Eigen::Vector2d ballVec(x, y);
		const std::vector<soccer_vision::ObstacleDetection>& obst = detections->obstacles;

		// See which object is most pressing
		int maxi = -1;
		double strength = 0.0, maxStrength = 0.0;
		Eigen::Vector2d obstacleVec(4.0, 0.0);
		for(unsigned i = 0;i < obst.size();i++)
		{
			obstacleVec << 0.5*(obst[i].left_lower_corner.x + obst[i].right_lower_corner.x), 0.5*(obst[i].left_lower_corner.y + obst[i].right_lower_corner.y);
			strength = adjustCmdForObstacle(cmd, ballVec, obstacleVec, true);
			if(fabs(strength) > maxStrength)
			{
				maxi = i;
				maxStrength = fabs(strength);
			}
		}
		
		// Adjust for the most relevant object
		if((maxStrength > 0.0) && (maxi >= 0) && (maxi < (int) obst.size()))
		{
			obstacleVec << 0.5*(obst[maxi].left_lower_corner.x + obst[maxi].right_lower_corner.x), 0.5*(obst[maxi].left_lower_corner.y + obst[maxi].right_lower_corner.y);
			adjustCmdForObstacle(cmd, ballVec, obstacleVec, false);
		}
	}

	AM->gaitCommand.write(cmd, this);
}

double GoBehindBall::adjustCmdForObstacle(gait::GaitCommand& cmd, Eigen::Vector2d targetVec, Eigen::Vector2d obstacleVec, bool calcOnly)
{
	// Calculate polar coordinates of obstacle and target
	double targetNorm = targetVec.norm();
	double targetAngle = atan2(targetVec.y(), targetVec.x());
	double obstacleNorm = obstacleVec.norm();
	double obstacleAngle = atan2(obstacleVec.y(), obstacleVec.x());

	// Calculate additional parameters
	double alpha = obstacleAngle - targetAngle;
	double perpObstacleDist = obstacleNorm * sin(alpha);
	double parallelObstacleDist = obstacleNorm * cos(alpha);

	// Constants
// 	const double maxCmdY = 0.7;
// 	const double maxCmdZ = 0.7;
	const double maxPerp = 1.1;
	const double maxAlpha = 1.0;
	const double cmdGainY = 0.7;
	const double cmdGainZ = 0.0;
	const double obstacleNormBuffer = 0.35;
	const double minObstacleNorm = 0.4;

	// Variables
	double strength = 0.0;

	// Calculate whether obstacle is worthy of our attention
	if((fabs(perpObstacleDist) <= maxPerp) && (fabs(alpha) <= maxAlpha) && (obstacleNorm <= (targetNorm - obstacleNormBuffer)) && (obstacleNorm >= minObstacleNorm))
	{
		strength = 1.0 - 0.6*fabs(perpObstacleDist)/maxPerp - 0.4*obstacleNorm/targetNorm;
		if(strength < 0.0) strength = 0.0;
		if(alpha >= 0.0)
			strength = -strength;

		if(!calcOnly) ROS_INFO_STREAM_THROTTLE(0.3, "WORST OBSTACLE: Perp dist = " << perpObstacleDist << " | Par dist = " << parallelObstacleDist << " | Strength = " << strength << " |");
		
		if(calcOnly)
			return strength;
		
		double extraCmdY = cmdGainY * strength;
		double extraCmdZ = cmdGainZ * strength;

		// Do a final check to try to avoid reacting to seeing yourself as an obstacle
		if(fabs(strength) <= 0.90)
		{
// 			cmd.gcvY += extraCmdY; // TODO: Uncomment all this
// 			cmd.gcvZ += extraCmdZ;
// 
// 			if(cmd.gcvY >  maxCmdY) cmd.gcvY =  maxCmdY; // TODO: Uncomment all this too
// 			if(cmd.gcvY < -maxCmdY) cmd.gcvY = -maxCmdY;
// 			if(cmd.gcvZ >  maxCmdZ) cmd.gcvZ =  maxCmdZ;
// 			if(cmd.gcvZ < -maxCmdZ) cmd.gcvZ = -maxCmdZ;

			ROS_WARN_STREAM_THROTTLE(0.3, "Obstacle avoidance is adding: Y[" << extraCmdY << "] and Z[" << extraCmdZ << "]");
		}
		
		return strength;
	}

	// Return value (zero = no obstacle avoidance performed)
	return 0.0;
}

// Inhibited function
void GoBehindBall::inhibited()
{
	if(wasJustDeactivated())
		ROS_WARN(" GO BEHIND BALL just deactivated! XXXXXXXXXXXXXXXXXXXXXXXXXX");
}
// EOF