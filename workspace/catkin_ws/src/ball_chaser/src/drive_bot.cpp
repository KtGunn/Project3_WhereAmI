
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>


// Autogenerated
#include "ball_chaser/DriveToTarget.h"

// Motor commands
ros::Publisher gMotor_command_publisher;


// Service callback function
bool service_callback (ball_chaser::DriveToTarget::Request& req_msg,
			       ball_chaser::DriveToTarget::Response& resp_msg) {
    // Throttle the messaging to console
    static int throttle = 0;
    static int squawk = 500;

    if (throttle++ >= squawk) {
	// Log the request
	ROS_INFO ("...Driving v=%f rate=%f", (float)req_msg.linear_x, (float)req_msg.angular_z);
	throttle = 0;
    }
    
    // Compose the command to the motor drive
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = (float)req_msg.linear_x;
    motor_command.angular.z = (float)req_msg.angular_z;

    // The command is a single message (not two)
    gMotor_command_publisher.publish (motor_command);
 
    // Return message
    resp_msg.msg_feedback = "Drive2Targ done!";
    if (throttle == 0 ) {
	ROS_INFO_STREAM (resp_msg.msg_feedback);
    }
    return (true);
}

int main(int argc, char* argv[]) {

    ros::init (argc,argv, "drive_bot");
    ros::NodeHandle handle;

    int qsize = 10;
    gMotor_command_publisher = handle.advertise<geometry_msgs::Twist>("/cmd_vel", qsize);

    if (true) {
	// Wait for ROS start
	int start_time;
	while (not start_time) {
	    start_time = ros::Time::now().toSec() - start_time;
	}
    }

    // Define a safe_move 'service' ; a CALLBACK function is provided (handle_safe_move_request)
    ros::ServiceServer service = handle.advertiseService("/ball_chaser/command_robot", service_callback);

    ROS_INFO ("Ready to command the bot" );

    // Enter the events loop, i.e. 'wait forever'
    ros::spin ();

    return (0);
}
