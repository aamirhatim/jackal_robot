jackal_robot
============

Robot packages for Jackal

## Changes for Verizon

### Gradual acceleration and deceleration on network timeout

New functionality was added to the `jackal_base.cpp`, `jackal_hardware.cpp`, and `jackal_hardware.h` files to change the behavior if there has been a loss in signal and motor commands are stopped. If no command has been received for 0.2 seconds, the robot will decelrate to a gradual stop rather than stopping immediately. Once connection has been re-established, the robot will slowly accelerate to the commanded velocity. Once that velocity has been reached, the robot will respond to future velocity commands as it would under normal operation.

This feature was added to create a more seamless experience when the robot temporarily loses control. It is often dangerous to have the robot immediately start and stop when signal is spotty, so having a gradual deceleration/acceleration profile put in place creates a more predictable and safer response from the robot.