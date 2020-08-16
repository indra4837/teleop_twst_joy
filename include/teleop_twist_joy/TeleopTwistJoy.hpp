#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

namespace teleop_twist_joy {

class TeleopTwistJoy {
public:
    TeleopTwistJoy(ros::NodeHandle& nodeHandle);

private:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber joyMovement_sub, joyMode_sub;
    ros::Publisher cmd_vel_pub, mode_pub;
    ros::Timer timer;

    std_msgs::String msg;

    bool current_machine_state = false; // true: engaged, false: disengaged
    bool current_machine_mode = false;  // true: autonomous, false: manual  

    int engage_button;                  // engage sweeper
    int mode_button1, mode_button2;     // switch modes (manual or autonomous)
    int mode_button3;     
    int brake_button;                   // emergency brake button
    int linear_, angular_;              // axis number for linear and angular motion
    double l_scale_, a_scale_;  
    long double accelerate_step_size, decelerate_step_size;
    long double current_speed;

    void joyMovementCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void joyModeCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void timerCallback(const ros::TimerEvent& event);
    void accelerate(long double *current_speed, double joyPosition);
    void decelerate(long double *current_speed);
    bool loadParam();
};

} // namespace teleop_twist_joy