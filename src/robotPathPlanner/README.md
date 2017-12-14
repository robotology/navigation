# robotPathPlanner
 This module performs global path-planning by generating a sequence of waypoints to be tracked by a local navigation algorithm. It receives a goal from the user either via RPC command or via yarp iNavigation2D interface and it computes a sequence of waypoints which are sent one by one to a local navigator such as   [**robotGoto**](https://github.com/robotology/navigation/tree/master/src/robotGoto). If the local navigation module fails to reach one of these waypoints, the global navigation is aborted too.   
 
## YARP Connections
* **/roboPathPlanner/rpc**   Standard rpc port
* **/roboPathPlanner/localizationClient** The input port used by the internally opened *localization2DClient* to receive the position of the robot in the environment.
* **/roboPathPlanner/laser:i** The input port used by the internally opened *Rangefinder2DClient* to receive laser scan data.
* **/roboPathPlanner/map2DClient:i** The input port used by the internally opened *Map2DClient* to receive map data from a *map2DServer*. 
* **/roboPathPlanner/navigationStatus:i** This port receives the navigation status broadcasted by the local navigation module.
* **/roboPathPlanner/plannerStatus:o** This port broadcasts the internal status of the module (*yarp::dev::INavigation2D::NavigationStatusEnum*), expressed as a string.
* **/roboPathPlanner/yarpviewTarget:i** Input port used to receive goal commands from the mouse click performed on a yarpview module (e.g. the map broadcasted by */roboPathPlanner/map:o* port). The mouse click is processed and the target, expressed in the current map reference frame, is sent to */roboPathPlanner/yarpviewTarget:o*.
* **/roboPathPlanner/yarpviewTarget:o** The port broadcasts the current goal received by */roboPathPlanner/yarpviewTarget:i* port in a human readable form. This port is typically connected to */roboPathPlanner/rpc* to set a new target to the robotPathPlanner.
* **/roboPathPlanner/map:o** This port broadcasts a yarp image showing the map, the position of the robot, the obstacles etc.
* **/roboPathPlanner/commands:o** This port has to be connected to the rpc port of the local navigation algorithm. It is used to send waypoints and control command.

## Rpc commands
The module accepts string rpc commands (meant to be issued from a user) or Vocab rpc commands (meant to be used from a software module):
### String commands
* **goto `<location_name>`** Starts a navigation task to reach a location previously stored into the map server.
* **gotoAbs `<x> <y> <angle in degrees>`** Starts a navigation task to reach a goal defined in the map reference frame.
* **gotoRel `<x> <y> <angle in degrees>`** Starts a navigation task to reach a goal defined in the robot reference frame.
* **stop** Stops the current navigation task. It is also used to clear a previously aborted navigation task.
* **pause** Pauses the current navigation task until a resume command is issued. 
* **resume** Resumes the current navigation params.
* **quit** Terminates the module execution.
* **store_current_location `<location_name>`**  Stores the current position onto the map server, with the specified location name.
* **delete_location `<location_name>`** Asks the map server to remove a previously saved location.
* **draw_locations `<0/1>`** Enables the visualization of all locations stored into the mapServer on the image broadcasted onto the */roboPathPlanner/map:o* port.


### Vocab commands
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GOTOABS] `<x> <y> <angle in degrees>`** Starts a navigation task to reach a goal defined in the map reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GOTOREL] `<x> <y> <angle in degrees>`** Starts a navigation task to reach a goal defined in the robot reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_STATUS]** Returns the current navigation status, expressed as a yarp::dev::INavigation2D::NavigationStatusEnum.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_STOP]** stops the current navigation task. It is also used to clear a previously aborted navigation task.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_SUSPEND]** Pauses the current navigation task until a resume command is issued. 
* **[VOCAB_INAVIGATION] [VOCAB_NAV_RESUME]** Resumes the current navigation params.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_CURRENT_POS]** Returns the current robot position in map reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_ABS_TARGET]** Returns the current navigation target expressed in the map reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_REL_TARGET]** Returns the current navigation target expressed in the robot reference frame.

 ## Parameters
   Parameters required by this module are:
   
  | Parameter name | SubParameter   | Type    | Units          | Default Value      | Required     | Description                                                       | Notes    |
  |:--------------:|:--------------:|:-------:|:--------------:|:------------------:|:-----------: |:-----------:|:---------------------------:|
  | GENERAL    |  publish_map_image_Hz     | double  | Hz              | 15                | Yes          | The map is broadcasted on */roboPathPlanner/map:o* with the specified  frequency. | -   |
  | ROBOT_GEOMETRY    |  robot_radius     | double  | m              | -                | Yes          | The robot is approximated by a circle with the specified radius. | -   |
| NAVIGATION   |  min_waypoint_distance    | double      | m  |   -         | Yes          | Minimum distance in meters between consecutive waypoints.  | - |
| NAVIGATION   |  use_optimized_path    | bool      | -  |   -         | Yes          | If set, an optimization is performed on the computed path, removing unnecessary waypoints that are aligned on the robot's path. | - |
| NAVIGATION   |  enable_try_recovery    | bool      | -  |   -         | Yes          | If enabled, the global planner is allowed to generate a new path if the local navigation module is blocked by an obstacle and the internal timeout is expired.  | - |
| NAVIGATION   |  goal_ang_speed_gain    | double      | (deg/s)/deg  |   -         | Yes          | The P gain of the controller which tracks the heading of the robot  when robot is reaching robot its final goal                  | - |
| NAVIGATION   |  goal_lin_speed_gain    | double      | (m/s)/m  |   -         | Yes          | The P gain of the controller which tracks the heading of the robot when robot is reaching robot its final goal        | - |
| NAVIGATION   |  goal_max_lin_speed    | double      | m/s  |   -         | No          | Maximum linear speed for robot velocity commands sent to baseControl when robot is reaching robot its final goal                   | - |
| NAVIGATION   | goal_max_ang_speed    | double      | deg/s  |   -         | No          | Maximum angular speed for robot angular velocity commands sent to baseControl when robot is reaching robot its final goal                   | - |
| NAVIGATION   |  goal_min_lin_speed    | double      | m/s  |   0        | Yes          | minimum linear speed for robot velocity commands sent to baseControl when robot is reaching robot its final goal   | - |
| NAVIGATION   |  goal_min_ang_speed    | double      | deg/s  |   0         | Yes          | minimum angular speed for robot angular velocity commands sent to baseControl when robot is reaching robot its final goal  | - |
| NAVIGATION   |  goal_tolerance_lin    | double      | m  |   -         | Yes          | The navigation task is complete when robot position respect to the goal is within this threshold.   | - |
| NAVIGATION   |  goal_tolerance_ang    | double      | deg/s  |   -         | Yes          | The navigation task is complete when robot orientation respect to the goal is within this threshold.   | - |
| NAVIGATION   |  waypoint_ang_speed_gain    | double      | (deg/s)/deg  |   -         | Yes          | The P gain of the controller which tracks the heading of the robot  during waypoint tracking                   | - |
| NAVIGATION   |  waypoint_lin_speed_gain    | double      | (m/s)/m  |   -         | Yes          | The P gain of the controller which tracks the heading of the robot during waypoint tracking       | - |
| NAVIGATION   |  waypoint_max_lin_speed    | double      | m/s  |   -         | No          | Maximum linear speed for robot linear velocity to be used by local navigation module during waypoint tracking    | - |
| NAVIGATION   | waypoint_max_ang_speed    | double      | deg/s  |   -         | No          |  Maximum angular speed for robot angular velocity to be used by local navigation module during waypoint tracking  | - |
| NAVIGATION   |  waypoint_min_lin_speed    | double      | m/s  |   0        | Yes          | Minimum linear speed for robot linear velocity to be used by local navigation module during waypoint tracking    | - |
| NAVIGATION   |  waypoint_min_ang_speed    | double      | deg/s  |   0         | Yes          | Minimum angular speed for robot angular velocity to be used by local navigation module during waypoint tracking  | - |
| NAVIGATION   |  waypoint_tolerance_lin    | double      | m  |   -         | Yes          | The waypoint is reached when robot position respect to the goal is within this threshold.   | - |
| NAVIGATION   |  waypoint_tolerance_ang    | double      | deg/s  |   -         | Yes          | The waypoint is reached when robot orientation respect to the goal is within this threshold.   | - |

