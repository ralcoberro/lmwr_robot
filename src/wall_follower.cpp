
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <cstdlib> 			// Needed for rand()
#include <ctime> 			// Needed to seed random number generator with a time value
#include <string>
#include <math.h>

#define ULTRA_SENSOR_1 "ultra_1";
#define ULTRA_SENSOR_2 "ultra_2";

class PID {
public:
// Kp -  proportional gain
// Ki -  Integral gain
// Kd -  derivative gain
// dt -  loop interval time
// max - maximum value of manipulated variable
// min - minimum value of manipulated variable
PID(double dt, double max, double min, double Kp, double Kd, double Ki) :   _dt(dt),
									    _max(max),
									    _min(min),
									    _Kp(Kp),
									    _Kd(Kd),
									    _Ki(Ki),
									    _pre_error(0),
									    _integral(0)
{
}

double calculate(double setpoint, double pv) {
	// Calculate error
	double error = setpoint - pv;

	// Proportional term
	double Pout = _Kp * error;
	//ROS_INFO_STREAM("PID Pout" << Pout);

	// Integral term
	_integral += error * _dt;
	double Iout = _Ki * _integral;

	// Derivative term
	double derivative = (error - _pre_error) / _dt;
	double Dout = _Kd * derivative;

	// Calculate total output
	double output = Pout + Iout + Dout;
	//ROS_INFO_STREAM("PID Output" << output);

	// Restrict to max/min
	if( output > _max )
		output = _max;
	else if( output < _min )
		output = _min;

	// Save error to previous error
	_pre_error = error;

	return output;
}

void reset() {
	ROS_INFO_STREAM("PID Reset");
	_pre_error = 0;
	_integral = 0;
}

double _dt;
double _max;
double _min;
double _Kp;
double _Kd;
double _Ki;
double _pre_error;
double _integral;
};

class WallFollower {
public:
// Construct a new WallFollower object and hook up this ROS node
// to the simulated robot's velocity control and range topics
WallFollower(ros::NodeHandle& nh) : fsm(FSM_MOVE_FORWARD),
				    rotateStartTime(ros::Time::now()),
				    rotateDuration(0.f),
				    pid(0.1, 2, -2, 0.8, 2, 0) {

	front_sensor_last_meas = 0;
	right_sensor_last_meas = 0;
	left_sensor_last_meas = 0;
	wallFollowing = NONE;
	deviation = 0;

	
	// Initialize random time generator
	srand(time(NULL));
	// Advertise a new publisher for the simulated robot's velocity command topic
	// (the second argument indicates that if multiple command messages are in
	// the queue to be sent, only the last command will be sent)
	commandPub = nh.advertise<geometry_msgs::Twist>("lmwr_diff_drive_controller/cmd_vel", 1);

	// Subscribe to the simulated robot's ultrasonic sensors topic and tell ROS to call
	// this->commandCallback() whenever a new message is published on that topic
	ultraSub_1 = nh.subscribe("sensor/ultra_1_ultrasonic_front", 1, &WallFollower::commandCallback, this);
	ultraSub_2 = nh.subscribe("sensor/ultra_2_ultrasonic_front", 1, &WallFollower::commandCallback, this);
	ultraSub_3 = nh.subscribe("sensor/ultra_center_ultrasonic_front", 1, &WallFollower::commandCallback, this);
};

// Send a velocity command
void move(double linearVelMPS, double angularVelRadPS) {
	geometry_msgs::Twist msg; 	// The default constructor will set all commands to 0
	msg.linear.x = linearVelMPS;
	msg.angular.z = angularVelRadPS;
	commandPub.publish(msg);
};

// Process the incoming ultrasonic sensor message
void commandCallback(const sensor_msgs::Range::ConstPtr& msg) {
	std::string frame_id = msg->header.frame_id;		// This tell us which sonar is
	if(frame_id == "ultra_1_front_sensor")
		right_sensor_last_meas = msg->range;
	else if(frame_id == "ultra_2_front_sensor")
		left_sensor_last_meas = msg->range;
	else if(frame_id == "ultra_center_front_sensor")
		front_sensor_last_meas = msg->range;
	
	if(fsm == FSM_MOVE_FORWARD) {
		if(msg->range < MIN_PROXIMITY_RANGE_M) {
			ROS_INFO_STREAM("Range below thres: " << msg->range << " frame Id: " << frame_id);
			rotateStartTime = ros::Time::now();
			float duration_s = ROTATE_90_DEGREES_SECS;
			rotateDuration = ros::Duration(duration_s);
			int rot_right = (right_sensor_last_meas < left_sensor_last_meas);
			if(rot_right) {
				ROS_INFO_STREAM("Rotate RIGHT " << rotateDuration.sec << " seconds. LEFT Wall Following");
				fsm = FSM_ROTATE_RIGHT;
				wallFollowing = LEFT;
			}
			else {
				ROS_INFO_STREAM("Rotate LEFT " << rotateDuration.sec << " seconds. RIGHT Wall Following");
				fsm = FSM_ROTATE_LEFT;
				wallFollowing = RIGHT;
			}
		}
	}
	else if((fsm == FSM_ROTATE_RIGHT) || (fsm == FSM_ROTATE_LEFT)) {
		ros::Time isnow = ros::Time::now();		
		ros::Time total_rot = rotateStartTime + rotateDuration;
		if(isnow > total_rot) {		// Rotation Time Ended
			ROS_INFO_STREAM("Rotation Time Ended. FSM: " << fsm);
			if(front_sensor_last_meas < MIN_PROXIMITY_RANGE_M) {
				rotateStartTime = isnow;
				float duration_s = ROTATE_90_DEGREES_SECS;
				rotateDuration = ros::Duration(duration_s);
				ROS_INFO_STREAM("Front Range below thres: " << front_sensor_last_meas 
					<< ". Continue in rotation for: " << rotateDuration.sec << " secs");
			}
			else {
				fsm = FSM_WALL_FOLLOWING;
				// Set PID for Wall Following
				pid.reset();
				deviation = 0;
			}
		}
	}
	else if(fsm == FSM_WALL_FOLLOWING) {
		// Check which wall we are following
		if((wallFollowing == LEFT) && (frame_id == "ultra_2_front_sensor"))
		{
			deviation = pid.calculate(WALL_FOLLOWING_PROXIMITY_RANGE_M, left_sensor_last_meas);
			ROS_INFO_STREAM("LEFT Wall Following. Sensor: " << left_sensor_last_meas 
				<< " Deviation: " << deviation);
		}
		else if((wallFollowing == RIGHT) && (frame_id == "ultra_1_front_sensor"))
		{
			deviation = pid.calculate(right_sensor_last_meas, WALL_FOLLOWING_PROXIMITY_RANGE_M);
			ROS_INFO_STREAM("RIGHT Wall Following. Sensor: " << right_sensor_last_meas 
				<< " Deviation: " << deviation);
		}
		if(front_sensor_last_meas < MIN_PROXIMITY_RANGE_M) {
			rotateStartTime = ros::Time::now();
			rotateDuration = ros::Duration(ROTATE_90_DEGREES_SECS);
			ROS_INFO_STREAM("Front Range below thres: " << front_sensor_last_meas 
				<< ". Rotate for: " << rotateDuration.sec << " secs");
			if(wallFollowing == LEFT)
			{
				if(left_sensor_last_meas > 1)		// Could be a open zone
					fsm = FSM_ROTATE_LEFT;
				else
					fsm = FSM_ROTATE_RIGHT;
			}	
			else
			{ 
				if(right_sensor_last_meas > 1)
					fsm = FSM_ROTATE_RIGHT;
				else
					fsm = FSM_ROTATE_LEFT;
			}
		}
	}
 };

// Main FSM loop for ensuring that ROS messages are
// processed in a timely manner, and also for sending
// velocity controls to the simulated robot based on the FSM state
void spin() {
	ros::Rate rate(10); // Specify the FSM loop rate in Hz
	while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C

		if (fsm == FSM_MOVE_FORWARD) {
			move(FORWARD_SPEED_MPS, 0); // Move foward
		} 
		else if (fsm == FSM_ROTATE_RIGHT) {
			move(0, ROTATE_SPEED_RADPS); // Rotate right
		}
		else {
			move(0, -ROTATE_SPEED_RADPS); // Rotate left
		}
		if (fsm == FSM_WALL_FOLLOWING) {
			if(deviation < FORWARD_SPEED_MPS)
				move(FORWARD_SPEED_MPS - deviation, deviation); // Move foward and steering
			else
				move(0, deviation); // Move foward and steering
		} 

		ros::spinOnce(); 	// Need to call this function often to allow ROS to process incoming messages
		rate.sleep(); 		// Sleep for the rest of the cycle, to enforce the FSM loop rate
 	}
};

enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE_LEFT, FSM_ROTATE_RIGHT, FSM_WALL_FOLLOWING};
enum WALL {NONE, RIGHT, LEFT};

// Tunable parameters
const static float MIN_PROXIMITY_RANGE_M;
const static float WALL_FOLLOWING_PROXIMITY_RANGE_M;
const static float WALL_FOLLOWING_MAX_DEVIATION;
const static double FORWARD_SPEED_MPS;
const static double ROTATE_SPEED_RADPS;
const static int ROTATE_PI_DURATION_S;
const static double ROTATE_90_DEGREES_SECS;

protected:
ros::Publisher commandPub; 		// Publisher to the simulated robot's velocity command topic
ros::Subscriber ultraSub_1; 		// Subscriber to the simulated robot's Ultrasonic topic
ros::Subscriber ultraSub_2; 		// Subscriber to the simulated robot's Ultrasonic topic
ros::Subscriber ultraSub_3; 		// Subscriber to the simulated robot's Ultrasonic topic
enum FSM fsm; 				// Finite state machine for the wall follower algorithm
ros::Time rotateStartTime; 		// Start time of the rotation
ros::Duration rotateDuration; 		// Duration of the rotation

float front_sensor_last_meas;
float right_sensor_last_meas;
float left_sensor_last_meas;
float deviation;

enum WALL wallFollowing;

PID pid;

};

const float WallFollower::MIN_PROXIMITY_RANGE_M = 0.5;
const float WallFollower::WALL_FOLLOWING_PROXIMITY_RANGE_M = 0.4;
const double WallFollower::FORWARD_SPEED_MPS = 2;
const int WallFollower::ROTATE_PI_DURATION_S = 6;
const double WallFollower::ROTATE_SPEED_RADPS = M_PI/(2*WallFollower::ROTATE_PI_DURATION_S);
const double WallFollower::ROTATE_90_DEGREES_SECS = 1.2 * (2*WallFollower::ROTATE_PI_DURATION_S)/(2*M_PI);


int main(int argc, char **argv) {
	ros::init(argc, argv, "wall_follower"); 	// Initiate new ROS node named "wall_follower"
	ros::NodeHandle n;

	ROS_INFO_STREAM("Wall Follower. Waiting 10 seconds");
	ros::Duration(10).sleep();	

	WallFollower walker(n); 		// Create new random walk object
	walker.spin(); 				// Execute FSM loop
	return 0;
};

