#include <ros/ros.h>
#include <teleop_twist_joy/TeleopTwistJoy.hpp>
#include <ros/spinner.h>

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "teleop_twist_joy_node", true);
    
    ros::NodeHandle nodeHandle("~");

    teleop_twist_joy::TeleopTwistJoy teleopTwistJoy(nodeHandle);

    ros::spin();
    
    return 0;
}