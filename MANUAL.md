# Teleop Twist Package Design (V1)

## Current features

### 1. Accelerating the vehicle
- Holding the left joystick at different upwards throttling position
- linear.x value increaes in steps to desired value (step:0.0001)
- Maximum linear.x value: 1000.0
- If joystick let go, linear.x decreaes in steps and goes back to 0 (step:0.000001)
- Prevents the machine from abruptly braking if the left joystick is released
	
### 2. Decelerating/Soft Braking the vehicle
- Holding the left joystick at different downwards throttling position
- If linear.x > 0, will activate the decelerating/soft braking function
- Minimum linear.x value: 0.0
- Action: trigger brake discs slowly in steps to slow/stop vehicle

### 3. Reversing vehicle
- If linear.x == 0, wil activate reverse braking function at downwards throttling position
- linear.x value decreaes in steps to desired value (step: 0.0001)
- Minimum linear.x value: -1000.0
- If joystick let go, linear.x increaes in steps and goes back to 0 (step: 0.000001)
- Prevents the machine from abruptly braking if the left joystick is released
	
### 4. Emergency braking
- If circle-button (O) is pressed, linear.x = 0
- Brake-discs will be implemented and vehicle will come to complete stop immediately

## Future features (if required)

1. Button mappings
	- Using arrow buttons to trigger vehicle mode (forward, reverse)

## Current Button mappings

![Joystick Button Mappings](teleop_twist_joy/docs/joystick-diagram.jpg)
