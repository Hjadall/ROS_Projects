#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// Publisher for motor commands
ros::Publisher motor_command_publisher;

// Callback function for handling drive requests
// This function publishes the requested linear and angular velocities to the robot
// and sends feedback with the applied velocities
bool handle_command_robot_request(ball_chaser::DriveToTarget::Request& req, 
                                  ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("Drive request received - linear_x: %1.2f, angular_z: %1.2f", 
             (float)req.linear_x, (float)req.angular_z);

    // Create and populate a Twist message with the requested velocities
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish the velocities to the motor command topic
    motor_command_publisher.publish(motor_command);

    // Send feedback to confirm the velocities were applied
    res.msg_feedback = "Velocities set - linear_x: " + std::to_string(req.linear_x) +
                       " , angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a NodeHandle to interact with the ROS system
    ros::NodeHandle n;

    // Advertise a publisher for motor commands on the /cmd_vel topic
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Advertise the /ball_chaser/command_robot service and bind it to the callback
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_command_robot_request);
    ROS_INFO("Ready to receive drive commands");

    // Handle incoming service requests and ROS communication events
    ros::spin();

    return 0;
}

