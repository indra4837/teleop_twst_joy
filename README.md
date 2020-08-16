# Teleop Twist Joystick

This package contains a teleop_twist_node to send command velocities to the vehicle.

[Decision flowchart of teleop_twist_node](docs/teleop_node_decision_flowchart.jpeg)

## Using the package

### 1. Dependencies

Install `joy`

For ROS Melodic:

```
sudo apt-get install ros-melodic-joy
```

For more information please refer to the respective [github](https://github.com/ros-drivers/joystick_drivers) and [tutorial](http://wiki.ros.org/joy/Tutorials)

### 2. Installation

Build the `teleop_twist_joy` package using `catkin_make`

### 3. Setup the joystick controller

Connect the joystick to your computer. Follow the tutorial [Configuring and Using a Linux-Supported Joystick with ROS](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).

### 4. Subscribed Topics

`joy (sensor_msgs/Joy)`

Outputs the joystick state

### 5. Published Topics

`cmd_vel (geometry_msgs/Twist)`

Command velocity messages from Joystick commands

`sweeper_mode (std_msgs/String)`

Autonomous sweeper operation mode from Joystick commands

### 6. Parameters

`~engage_button (int, default: 0)`

Joystick button to engage/disengage machine

`~brake_button (int, default: 1)`

Joystick button for emergency brake

`~mode_button1 (int, default: 2)`

Joystick button to switch modes (autonomous/manual)

`~mode_button2 (int, default: 4)`

Joystick button to switch modes (autonomous/manual)

`~mode_button2 (int, default: 5)`

Joystick button to switch modes (autonomous/manual)

`~axis_linear (int, default: 1)`

Joystick axis for linear movement control

`~axis_angular (int, default: 3)`

Joystick axis for angular movement control

`~scale_linear (double, default: 10)`

Scale to apply to joystick linear axis for movement control

`~scale_angular (double, default: 10)`

Scale to apply to joystick angular axis for movement control
