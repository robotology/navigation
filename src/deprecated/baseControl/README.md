# baseControl
 The module controls robot joints (e.g. wheels) in order to achieve the desired cartesian velocities commanded by the user. Robot reference frame is placed according to the following convention: X pointing forward, Y pointing left, Z pointing up. 
 
## YARP Connections
* **/baseControl/rpc**   standard rpc port
* **/baseControl/joystick:i** this port is used to receive joystick commands.
* **/baseControl/control:i** this port is used to receive Cartesian velocities from a user module. Commands received on the joystick port have the priority over the ones received on this port. 
* **/baseControl/aux_control:i** a service port allowing a secondary module to control the movements the robot. Commands received on the joystick port have the priority over the ones received on this port. 
* **/baseControl/odometry:o** this port publishes the current estimated position of the robot, expressed in the odometry reference system (i.e. the initial location of the robot when the module was launched defines x=0,y=0,theta=0). The output is in the format *x_position*, *y_position*, *theta_angle*.
* **/baseControl/motor_status:o** this port broadcast the current status of robot joints (i.e. joints control mode)

## ROS Connections
* **/cmd_vel@/baseControl** This ROS topic (type *geometry_msgs_Twist*) can be used to control robot from ROS (the functionality is the same as controlling the robot via */baseControl/control:i* port).
* **/odometry@/baseControl** This ROS topic (type *nav_msgs_Odometry*) broadcasts robot's odomety (analogously  to */baseControl/odometry:o* port).
* **/footprint@/baseControl** This ROS topic (type *geometry_msgs_PolygonStamped*) broadcasts robot's fooprint (it can be displayed in RVIZ).

## Rpc commands
The module accepts the following rpc commands, encoded as bottles of strings/values. 
* **help**  shows available rpc commands
* **run** Turns on robot motors, setting all joints control mode to the value specified by parameter *GENERAL::control_mode*.
* **idle** Turns off robot motors, setting all joints control mode to *VOCAB_CM_IDLE*
* **reset_odometry** Sets to zero the odometry of the robot, meaning that the current position of the robot becomes (x=0, y=0, theta=0).
* **set_prefilter <value>** Sets the frequency of the low-pass filter applied to user commands.
* **set_motors_filter <value>** Sets the frequency of the low pass filter applied to control values sent to each motor (e.g. motor speed/motor pwm).

 ## Parameters
   Parameters required by this device are:
   
  | Parameter name | SubParameter   | Type    | Units          | Default Value      | Required     | Description                                                       | Notes    |
  |:--------------:|:--------------:|:-------:|:--------------:|:------------------:|:-----------: |:-----------:|:---------------------------:|  
  | robot        |  -   | string  | -              | - | Yes          | Sets the name of the robot.                 |     &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    |
  | part        |  -    | string     | -            | -                  | Yes          | Sets the name of the part of the robot controlling the wheels.     |       |  
  | joystick_connect   |  -      | -      | -  |   -         | No          | If set, the module tries to automatically connect /baseControl/joystick:i with /joystickCtrl:o port                     | - |  
 | GENERAL            |  robot_type      |   string        | - | -        | Yes | Sets the kinematic model of the robot to be controlled     | Can be one of the following values: *cer*, *ikart_V1*, *ikart_V2* |
  | GENERAL            |  control_mode      |   string        | - | -        | Yes | Sets the control mode for the robot motors   | Can be one of the following values: *velocity_no_pid*, *velocity_pid*, *openloop_no_pid*, *openloop_pid*. |
   | GENERAL            |  max_linear_vel      |   double        | m/s | -        | Yes | Sets the robot maximum linear velocity     | -|
  | GENERAL            |  max_angular_vel      |   double        | deg/s | -        | Yes  |Sets the robot maximum angular velocity   | - |
  | GENERAL            |  max_linear_acc      |   double        | m/s^2 | -        | Yes| Sets the robot maximum linear acceleration  | -|
  | GENERAL            |  max_angular_acc      |   double        | deg/s^2 | -        | Yes | Sets the robot maximum angular acceleration | -|
  | GENERAL            |  use_ROS       |   bool        | - | -        | Yes | Enables ROS connections | -|
  | JOYSTICK   |  linear_vel_at_full_control      | double      | m/s  |    -        | Yes          | Maximum linear velocity when the joystick is at 100%                     | - |
  | JOYSTICK   |  angular_vel_at_full_control      | double      |  deg/s  |    -       | Yes          | Maximum angular velocity when the joystick is at 100%                     | - |
  | MOTORS   |  max_motor_pwm      | double      |  -  |    -       | Yes          | Maximum motor PWM when motors are controlled in openloop mode. | - |
  | MOTORS   |  max_motor_vel      | double      |  -  |    -       | Yes          | Maximum motor velocity when motors are controlled in velocity mode. | - |
  | MOTORS   |  motors_filter_enabled      | int      |  -  |    -       | Yes          | Enables a low pass filter on computed commands sent to the motors. | - |
 
 ## Additional Notes
 
The joystick input can be sent to the module either through a yarp port or by opening a device client. See [joystickCtrl](https://github.com/robotology/icub-main/tree/master/src/tools/joystickCtrl) module and *yarp::dev::IJoypadController* documentation.
