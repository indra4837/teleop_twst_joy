#include "teleop_twist_joy/TeleopTwistJoy.hpp"

namespace teleop_twist_joy {

TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
	if(!loadParam()){
		ROS_ERROR("Error in loading the parameters.");
		// ros::requestShutdown();
	}

    msg.data = "none";

    cmd_vel_pub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    joyMovement_sub = nodeHandle_.subscribe("/joy", 100, &TeleopTwistJoy::joyMovementCallback, this);
    joyMode_sub = nodeHandle_.subscribe("/joy", 10, &TeleopTwistJoy::joyModeCallback, this);
    timer = nodeHandle_.createTimer(ros::Duration(1), &TeleopTwistJoy::timerCallback, this);
    mode_pub = nodeHandle_.advertise<std_msgs::String>("/sweeper_mode", 1);
}

void TeleopTwistJoy::timerCallback(const ros::TimerEvent& event) {
    mode_pub.publish(msg);
}

void TeleopTwistJoy::joyModeCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    if (joy->buttons[mode_button1] && joy->buttons[mode_button2] && joy->buttons[mode_button3]) {
        if (!current_machine_mode) {
            current_machine_mode = true;
                msg.data = "autonomous";
        } else {
            current_machine_mode = false;
                msg.data = "manual";
            }
    }
    if (!current_machine_mode) {
        // checking state of machine
        if (joy->buttons[engage_button]) {
            if (!current_machine_state) {
                current_machine_state = true;
            } else {
                current_machine_state = false;    
            }
        }
    }
}

void TeleopTwistJoy::accelerate(long double *current_speed, double joyPosition) {
    if (joyPosition > 0) {
        // from 0 to max
        while (*current_speed < l_scale_ * joyPosition) {
            *current_speed += accelerate_step_size;
        }
    } else if (joyPosition < 0) {
        // from 0 to min
        while (*current_speed > l_scale_ * joyPosition) {
                *current_speed -= accelerate_step_size;
        }
    }

    ros::Duration(0.01).sleep(); // prevent knee-jerk reaction form sudden joystick movements
}

void TeleopTwistJoy::decelerate(long double *current_speed) {
    if (*current_speed > 0) {
        // braking from forward
        while (*current_speed != 0) {
            *current_speed -= decelerate_step_size;
        }
    } else if (*current_speed < 0) {
        // braking from reverse
        while (*current_speed != 0) {
            *current_speed += decelerate_step_size;
        }
    }

    ros::Duration(0.01).sleep();
}

void TeleopTwistJoy::joyMovementCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    
    std_msgs::String msg;
    geometry_msgs::Twist twist;
    current_speed = 0;
    accelerate_step_size = 0.0001, decelerate_step_size = 0.000001;
    
    if (joy->buttons[brake_button]) {
        twist.linear.x = 0;
        twist.angular.z = 0;
        twist.linear.z = 1; // activate brake disc
    } else {
        if (current_machine_state && !current_machine_mode) {
            if (joy->axes[linear_] > 0) {
                if (current_speed == 0) {
                    // accelerate
                    accelerate(&current_speed, joy->axes[linear_]);
                } else if (current_speed > 0) {
                    // accelerate
                    accelerate(&current_speed, joy->axes[linear_]);
                } else if (current_speed < 0) {
                    // slow-brake
                    decelerate(&current_speed);
                }
            } else if (joy->axes[linear_] < 0) {
                if (current_speed == 0) {
                    // reverse
                    accelerate(&current_speed, joy->axes[linear_]);
                } else if (current_speed > 0) {
                    // slow-brake
                    decelerate(&current_speed);
                } else if (current_speed < 0) {
                    // reverse
                    accelerate(&current_speed, joy->axes[linear_]);
                }
            }
            twist.angular.z = a_scale_*joy->axes[angular_];
        }
    }
    
    twist.linear.x = current_speed;
    cmd_vel_pub.publish(twist);
}

bool TeleopTwistJoy::loadParam() {

    // TODO: Use getParam loop to retrieve parameters 
    if (!nodeHandle_.getParam("engage_button", engage_button)) {
        ROS_ERROR("Could not find engage_button parameter!");
        return false;
    }
    if (!nodeHandle_.getParam("brake_button", brake_button)) {
        ROS_ERROR("Could not find brake_button parameter!");
        return false;
    }
    if (!nodeHandle_.getParam("mode_button1", mode_button1)) {
        ROS_ERROR("Could not find mode_button1 parameter!");
        return false;
    }
    if (!nodeHandle_.getParam("mode_button2", mode_button2)) {
        ROS_ERROR("Could not find mode_button2 parameter!");
        return false;
    }
    if (!nodeHandle_.getParam("mode_button3", mode_button3)) {
        ROS_ERROR("Could not find mode_button3 parameter!");
        return false;
    }
    if (!nodeHandle_.getParam("axis_linear", linear_)) {
        ROS_ERROR("Could not find axis_linear parameter!");
        return false;
    }
    if (!nodeHandle_.getParam("axis_angular", angular_)) {
        ROS_ERROR("Could not find axis_angular parameter!");
        return false;
    }
    if (!nodeHandle_.getParam("scale_linear", l_scale_)) {
        ROS_ERROR("Could not find scale_linear parameter!");
        return false;
    }
    if (!nodeHandle_.getParam("scale_angular", a_scale_)) {
        ROS_ERROR("Could not find scale_angular parameter!");
        return false;
    }

    return true;
}

} // namespace teleop_twist_joy