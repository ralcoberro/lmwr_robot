
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <cstdlib> 			// Needed for rand()
#include <ctime> 			// Needed to seed random number generator with a time value
#include <string>
#include <math.h>

#define ULTRA_SENSOR_1 "ultra_1";
#define ULTRA_SENSOR_2 "ultra_2";

class RandomWalk {
public:
// Construct a new RandomWalk object and hook up this ROS node
// to the simulated robot's velocity control and laser topics
RandomWalk(ros::NodeHandle& nh) : fsm(FSM_MOVE_FORWARD),
				  rotateStartTime(ros::Time::now()),
				  rotateDuration(0.f) {
	
	// Initialize random time generator
	srand(time(NULL));
	// Advertise a new publisher for the simulated robot's velocity command topic
	// (the second argument indicates that if multiple command messages are in
	// the queue to be sent, only the last command will be sent)
	commandPub = nh.advertise<geometry_msgs::Twist>("lmwr_diff_drive_controller/cmd_vel", 1);

	// Subscribe to the simulated robot's ultrasonic sensors topic and tell ROS to call
	// this->commandCallback() whenever a new message is published on that topic
	ultraSub_1 = nh.subscribe("sensor/ultra_1_ultrasonic_front", 1, &RandomWalk::commandCallback, this);
	ultraSub_2 = nh.subscribe("sensor/ultra_2_ultrasonic_front", 1, &RandomWalk::commandCallback, this);
	ultraSub_3 = nh.subscribe("sensor/ultra_center_ultrasonic_front", 1, &RandomWalk::commandCallback, this);
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
	
	if (fsm == FSM_MOVE_FORWARD) {
		if(msg->range < MIN_PROXIMITY_RANGE_M) {
			ROS_INFO_STREAM("Range below thres: " << msg->range << " frame Id: " << frame_id);
			rotateStartTime = ros::Time::now();
			float duration_s = 1 + std::rand() % ROTATE_PI_DURATION_S;
			rotateDuration = ros::Duration(duration_s);
			//ROS_INFO_STREAM("Rotation Duration " << rotateDuration.sec << " seconds");
			int right_left = rand() % 2;
			if(right_left) {
				ROS_INFO_STREAM("Rotate RIGHT " << rotateDuration.sec << " seconds");
				fsm = FSM_ROTATE_RIGHT;
			}
			else {
				ROS_INFO_STREAM("Rotate LEFT " << rotateDuration.sec << " seconds");
				fsm = FSM_ROTATE_LEFT;
			}
		}
	}
	else {
		ros::Time isnow = ros::Time::now();		
		ros::Time total_rot = rotateStartTime + rotateDuration;
		if(isnow > total_rot) {
			if(msg->range < MIN_PROXIMITY_RANGE_M) {
				float duration_s = 1 + std::rand() % ROTATE_PI_DURATION_S;
				rotateStartTime = isnow;
				rotateDuration = ros::Duration(duration_s);
				ROS_INFO_STREAM("Range below thres: " << msg->range << " frame Id: " << frame_id << ". Continue in rotation for: " << rotateDuration << " secs");
			}
			else {
				ROS_INFO_STREAM("Free path. Moving Forward");
				fsm = FSM_MOVE_FORWARD;
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

		ros::spinOnce(); 	// Need to call this function often to allow ROS to process incoming messages
		rate.sleep(); 		// Sleep for the rest of the cycle, to enforce the FSM loop rate
 	}
};

enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE_LEFT, FSM_ROTATE_RIGHT};

// Tunable parameters
const static float MIN_PROXIMITY_RANGE_M;
const static double FORWARD_SPEED_MPS;
const static double ROTATE_SPEED_RADPS;
const static int ROTATE_PI_DURATION_S;
//const static std::string ULTRA_SENSOR_1;
//const static std::string ULTRA_SENSOR_2;

protected:
ros::Publisher commandPub; 		// Publisher to the simulated robot's velocity command topic
ros::Subscriber ultraSub_1; 		// Subscriber to the simulated robot's Ultrasonic topic
ros::Subscriber ultraSub_2; 		// Subscriber to the simulated robot's Ultrasonic topic
ros::Subscriber ultraSub_3; 		// Subscriber to the simulated robot's Ultrasonic topic
enum FSM fsm; 				// Finite state machine for the random walk algorithm
ros::Time rotateStartTime; 		// Start time of the rotation
ros::Duration rotateDuration; 		// Duration of the rotation

};

const float RandomWalk::MIN_PROXIMITY_RANGE_M = 0.5;
const double RandomWalk::FORWARD_SPEED_MPS = 2.0;
const int RandomWalk::ROTATE_PI_DURATION_S = 6;
const double RandomWalk::ROTATE_SPEED_RADPS = M_PI/(2*RandomWalk::ROTATE_PI_DURATION_S);

int main(int argc, char **argv) {
	ros::init(argc, argv, "random_walk_basic"); 	// Initiate new ROS node named "random_walk"
	ros::NodeHandle n;

	ROS_INFO_STREAM("Random Walk Basic. Waiting 10 seconds");
	ros::Duration(10).sleep();	

	RandomWalk walker(n); 			// Create new random walk object
	walker.spin(); 				// Execute FSM loop
	return 0;
};

