# fakeMobileBaseTest
 This module simulates a virtual robot, exsposing similar interfaces to [**baseControl**](https://github.com/robotology/navigation/tree/master/src/baseControl). The user can send navigation commands to test navigation modules with this virtual robot. The module computes perfect odometry corresponding to user commands and can eventually simulate wheels splippage by intentionally adding a systemic error to odometry data.
 
## YARP Connections
* **/fakeMobileBaseTest/rpc**   standard rpc port
* **/fakeMobileBaseTest/joystick:i** this port is used to receive joystck commands, with the same format used by baseControl
* **/fakeMobileBaseTest/control:i** this port is used to receive movement commands from a user modulel, using the same format used by baseControl. Commands received on the joystick port have the priority over the ones received on this port. 
* **/localizationServer/aux_control:i** a service port which allows a secondary module to control the movement the robot. The commands have the same format of the one accepted by control and joystick ports. Commands received on the joystick port have the priority over the ones received on this port. 
* **/fakeMobileBaseTest/odometry:o** this port publishes the current position of the robot, expressed in the odometry reference system (i.e. the initial location of the robot when the module was launched defines x=0,y=0,theta=0). The output is in the format *x_position*, *y_position*, *theta_angle*.
* **/fakeMobileBaseTest/localizationTfClient** service port used by an internally instatiated *tfClient* device to publish odometry data onto a *tfServer*. The published transform defines the transformation bewtween the following reference frames: *odom->mobile_base_body*.

## Rpc commands
The module accepts the following rpc commands, encoded as a bottle of strings/values. 
* **help**  shows available rpc commands
* **reset_odometry** set to zero the odometry of the robot, meaning that the current position of the robot becomes (x=0, y=0, theta=0).
* **relocalize <x> <y> <theta>** sets the current position of the robot to the value specified by the user.
* **go <linear_dir> <linear_vel>  <angular_vel>  <command_duration>** commands the robot to move in a defined direction with a defined speed for a specified amount of seconds, then stops.


 ## Parameters
   Parameters required by this device are:
   
  | Parameter name | SubParameter   | Type    | Units          | Default Value      | Required     | Description | Notes    |  
  |:--------------:|:--------------:|:-------:|:--------------:|:------------------:|:-----------: |:-----------:|:---------------------------:|
  | robot        |  -   | string  | -              | - | Yes          | Sets the name of the robot.                 |   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   -    |
  | part        |  -    | string     | -            | -                  | Yes          | Sets the name of the part of the robot controlling the wheels.     |   -    |
  | ODOMETRY_ERROR    |  x_gain     | double  | -              | 1.0                | No          | The real computed odometry is mutiplied by the specified gain.                              | If not specified, or if x_gain==1.0, the odometry is accurate. If x_gain<1.0 the odometry is degraded by an error similar to wheels slippage.    |
  | ODOMETRY_ERROR    |  y_gain     | double  | -              | 1.0                | No          | The real computed odometry is mutiplied by the specified gain.                              | If not specified, or if y_gain==1.0, the odometry is accurate. If y_gain<1.0 the odometry is degraded by an error similar to wheels slippage     |
 | ODOMETRY_ERROR    |  t_gain | double  | -            | 1.0                | No          | The real computed odometry is mutiplied by the specified gain.                              | If not specified, or if t_gain==1.0, the odometry is accurate. If t_gain<1.0 the odometry is degraded by an error similar to wheels slippage     |
 | holonomic            |  -   | -  | -     |   -              | Yes          | If set, the following robot dof are enabled: x,y,theta. Otherwise, only x,theta commands are allowed. | -     |
  | joystick_connect   |  -      | -      | -  |   -         | No          | If set, the module tries to automatically connect */fakeMobileBaseTest/joystick:i* with */joystickCtrl:o* port                     | - |

 ## Additional Notes
 
Some of the parameters (e.g.max_linear_vel, max_angular_vel etc) implemented by **baseControl** are currently not yet implemented in **fakeMobileBaseTest**. Thus setting them in **fakeMobileBaseTest.ini** does not have any effect.
Currently, the module dows not load any map, so it is not possibile to simulate collision of the robot against a wall. However, it is possibile to simulate the corresponding laser scan, by providing map data to fakeLaser device. See [RobotPathPlanner Example1](https://github.com/robotology/navigation/blob/documentation/README.md#robotPathPlannerExample1).
