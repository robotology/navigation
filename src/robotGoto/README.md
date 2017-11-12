# robotGoto
 This module performs a point-to-point local navigation. It receives a goal from the user either via RPC command or via yarp iNavigation2D interface and it computes the Cartesian velocities to be sent to  [**baseControl**](https://github.com/robotology/navigation/tree/master/src/baseControl) module. 
 
## YARP Connections
* **/robotGoto/rpc**   standard rpc port
* **/robotGoto/control:o** this port is used by the module to send cartesian velocities commands to baseControl module.
* **/robotGoto/status:o** this post broadcasts the internal status of the module (yarp::dev::INavigation2D::NavigationStatusEnum), expressed as a string.
* **/robotGoto/localizationClient** The input port used by the internally opened localization2DClient to receive the position of the robot in the environment.
* **/robotGoto/laser:i** The input port used by the internally opened Rangefinder2DClient to receive laser scan data.

## Rpc commands
The module accepts string rpc commands (meant to be issued from a user) or Vocab rpc commands (meant to be used from a software module/yarp interface *e.g. robotPathPlanner*):
### String commands
* **gotoAbs <x> <y> <angle in degrees>** Starts a navigation task to reach a goal defined in the map reference frame.
* **gotoRel <x> <y> <angle in degrees>** Starts a navigation task to reach a goal defined in the robot reference frame.
* **approach <angle in degrees> <linear velocity> <time>** This command allows the user to directly control the robot, sending a Cartesian command for a specified amount of time. It is useful to perform small adjustments in position when the robot is near the goal.
* **stop** Stops the current navigation task. It is also used to clear a previously aborted navigation task.
* **pause** Pauses the current navigation task until a resume command is issued. 
* **resume** Resumes the current navigation params.
* **quit** Terminates the module execution.
* **reset_params** Reset the parameters to the original value defined in the .ini file
* **set linear_tol <m>**  sets the ROBOT_TRAJECTORY::goal_tolerance_lin parameter.
* **set linear_ang <deg>**  sets the ROBOT_TRAJECTORY::goal_tolerance_ang parameter.
* **set max_lin_speed <m/s>** sets the ROBOT_TRAJECTORY::max_lin_speed parameter.
* **set max_ang_speed <deg/s>** sets the ROBOT_TRAJECTORY::max_ang_speed parameter.
* **set min_lin_speed <m/s>** sets the ROBOT_TRAJECTORY::min_lin_speed parameter.
* **set min_ang_speed <deg/s>** sets the ROBOT_TRAJECTORY::min_ang_speed parameter.
* **set obstacle_stop** sets the OBSTACLES_EMERGENCY_STOP::enable_obstacles_emergency_stop parameter.
* **set obstacle_avoidance** sets the OBSTACLES_AVOIDANCE::enable_obstacles_avoidance parameter.
### Vocab commands
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GOTOABS] <x> <y> <angle in degrees>** Starts a navigation task to reach a goal defined in the map reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GOTOREL]  <x> <y> <angle in degrees>** Starts a navigation task to reach a goal defined in the robot reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_STATUS]** Returns the current navigation status, expressed as a yarp::dev::INavigation2D::NavigationStatusEnum.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_STOP]** Stops the current navigation task. It is also used to clear a previouly aborted navigation task.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_SUSPEND]** Pauses the current navigation task until a resume command is issued. 
* **[VOCAB_INAVIGATION] [VOCAB_NAV_RESUME]** Resumes the current navigation params.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_CURRENT_POS]** Returns the current robot position in map reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_ABS_TARGET]** Returns the current navigation target expressed in the map reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_REL_TARGET]** Returns the current navigation target expressed in the robot reference frame.

## Parameters
Parameters required by this module are:

 | Parameter name | SubParameter   | Type    | Units          | Default Value      | Required     | Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | Notes    |  
 |:--------------:|:--------------:|:-------:|:--------------:|:------------------:|:-----------: |:-----------:|:--------:|  
 | ROBOT_GEOMETRY |  robot_radius  | double  | m              | -                  | Yes          | The robot is approximated by a circle with the specified radius. | -   |  
 | ROBOT_TRAJECTORY   |  robot_is_holonomic    | bool      | -  |   -         | Yes          | If set, the following robot dof are enabled: x,y,theta. Otherwise, only x,theta commands are allowed.  | - |  
 | ROBOT_TRAJECTORY   |  max_beta_angle    | double      | deg  |   -         | Yes          | The robot will align towards the goal by performing an in-place rotation (vel_x=0, vel_y=0) until its heading is within this threshold.  | An holonomic robot with max_beta_angle=360 is allowed to move towards the goal, with no requirements on its heading. |  
 | ROBOT_TRAJECTORY   |  ang_speed_gain    | double      | (deg/s)/deg  |   -         | Yes          | The P gain of the controller which tracks the heading of the robot                  | - |  
 | ROBOT_TRAJECTORY   |  lin_speed_gain    | double      | (m/s)/m  |   -         | Yes          | The P gain of the controller which regulates the velocity of the robot depending on its distance to the goal   | - |  
 | ROBOT_TRAJECTORY   |  max_lin_speed    | double      | m/s  |   -         | No          | maximum linear speed for robot velocity commands sent to baseControl                   | - |  
 | ROBOT_TRAJECTORY   |  max_ang_speed    | double      | deg/s  |   -         | No          | maximum angular speed for robot angular velocity commands sent to baseControl                   | - |  
 | ROBOT_TRAJECTORY   |  min_lin_speed    | double      | m/s  |   0        | Yes          | minimum linear speed for robot velocity commands sent to baseControl    | - |  
 | ROBOT_TRAJECTORY   |  min_ang_speed    | double      | deg/s  |   0         | Yes          | minimum angular speed for robot angular velocity commands sent to baseControl   | - |  
 | ROBOT_TRAJECTORY   |  goal_tolerance_lin    | double      | m  |   -         | Yes          | The navigation task is complete when robot position respect to the goal is within this threshold.   | - |  
 | ROBOT_TRAJECTORY   |  goal_tolerance_ang    | double      | deg/s  |   -         | Yes          | The navigation task is complete when robot orientation repesct to the goal is within this threshold.   | - |  
 | OBSTACLES_EMERGENCY_STOP   |  enable_obstacles_emergency_stop    | bool      | -  |   -         | Yes          | If set to true, the robot will stop if an obstacle is detected inside the specified threshold.   | The navigation status wil be put in *navigation_status_waiting_obstacle* until the obstacle is removed.  |  
 | OBSTACLES_EMERGENCY_STOP   |  max_detection_distance    | double      | m  |   -         | Yes          | Obstacle which are more distant than this threshold are ignored.   | - |  
 | OBSTACLES_EMERGENCY_STOP   |  min_detection_distance    | double      | m  |   -         | Yes          | Obstacle which are nearer than this threshold are ignored.   | - |  
 | OBSTACLES_EMERGENCY_STOP   |  max_waiting_time    | double      | s  |   -         | Yes          | The navigation will be aborted if the obstacle is not removed within the specified amount of seconds  | If the obstacle is not removed in time, the navigation status is set to *navigation_status_failing*. |  
 | OBSTACLES_AVOIDANCE   |  enable_obstacles_avoidance    | bool      | -  |   -         | Yes          | If set to true, the obstacle avoidance behavior is enabled.  | - |  
 | OBSTACLES_AVOIDANCE   |  speed_reduction_factor    | double      | 0.0-1.0  |   -         | Yes          | The robot speed is reduced when the robot is avoiding the obstacle.  | - |  
