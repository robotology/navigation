# LocalizationServer
  A module which acts as the server side for a **Localization2DClient**. 
  The module can be configured to receive localization data either from a yarp port (e.g. /localizationServer/odometry:i) or through a tfClient (e.g. /localizationServer/TfClient). A user module which instantiates a Localization2DClient device can use RPC commands to ask to the LocalizationServer module the current position of the robot or set a new estimate intial position. 
  
 NB: This module does not run any localization algorithm. As a server, it can be attached to a ROS node performing localization such as [AMCL](http://wiki.ros.org/amcl). 
 
## YARP Connections
* **/localizationServer/rpc**   standard rpc port
* **/localizationServer/odometry:i** a port used to receive odometry data from the robot (e.g. baseControl module)
* **/localizationServer/TfClient** service port use to communicate with tfServer, if requested by configuration options.
* **/localizationServer/map2DClient** service port use to communicate with map2DServer, if requested by configuration options.

## ROS Connections
* **ROS::initialpose_topic** The name of this topic is defined by module configuration options. The module publishes on this topic the initial position received thorush an RPC command.
* **ROS::occupancygrid_topic** The name of this topic is defined by module configuration options. The module publishes on this topic the occupancy grid of the map currently in use.

## Rpc commands
### Strings
* **getLoc**  returns a string containing the current position of the robot in the format `<map_name> <x> <y> <theta>`
* **initLoc `<map_name> <x> <y> <theta>`** initialize the localization system with the specified coordinates.
### Vocabs
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_CURRENT_POS]** identical to getLoc string command. Returned bottle is in the format: **[VOCAB_OK] <map_name> `<x> <y> <theta>`**
* **[VOCAB_INAVIGATION] [VOCAB_NAV_SET_INITIAL_POS] `<map_name> <x> <y> <theta>`** identical to initLoc string command.

 ## Parameters
   Parameters required by this device are:
   
  | Parameter name | SubParameter   | Type    | Units          | Default Value      | Required     | Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;        | Notes |  
  |:--------------:|:--------------:|:-------:|:--------------:|:------------------:|:-----------: |:------------------:|:-----:|  
  | GENERAL        |  module_name   | string  | -              | localizationServer | Yes          | The name of the module   | It can be changed to provide to multiple localizationServer modules unique port names.    |   
  | GENERAL        |  enable_ros    | int     | 0/1            | -                  | Yes          | If set to 1, the module will open the ROS topic specified by ROS::initialpose_topic parameter     | -      |
  | INITIAL_POS    |  initial_x     | double  | m              | 0.0                | Yes          | Initial guess for estimated robot position                              | -     |
  | INITIAL_POS    |  initial_y     | double  | m              | 0.0                | Yes          | Initial guess for estimated robot position                              | -     |
 | INITIAL_POS    |  initial_theta | double  | deg            | 0.0                | Yes          | Initial guess for estimated robot position                              | -     |
 | INITIAL_POS    |  initial_map   | string  | -              |   -                | Yes          | Name of the map on which localization is performed                | -     |
 | MAP            |  connect_to_yarp_mapserver   | int  | 0/1     |   -              | Yes          | If set to 1, LocalizationServer will ask maps to yarp map server when initial pose is updated | -     |
  | ROS            |  initialpose_topic   | string     | -         |   -              | No           | Name of the topic which will be used to publish the initial pose  | -     |
  | ROS            |  occupancygrid_topic | string     | -         |   -              | No           | Name of the topic which will be used to publish map data when initial pose is updated         | -     |
 | TF             |  map_frame_id      | string     | -         |   -                | Yes          | Name of the map reference frame                                   | e.g. /map    |
 | TF             |  robot_frame_id    | string     | -         |   -                | Yes          | Name of the robot reference frame                                 | e.g. /mobile_base    |
  | LOCALIZATION   |  use_localization_from_odometry_port    | int      | 0/1  |   -         | Yes          | If set to 1, the module will use a port to receive localization data                         | Incompatible with 'use_localization_from_tf=1'  |
  | LOCALIZATION   |  use_localization_from_tf      | int      | 0/1  |   -         | Yes          | If set to 1, the module will use a tfClient to receive localization data                     | Incompatible with 'use_localization_from_odometry_port=1 |
  | ODOMETRY       |  odometry_broadcast_port       | string   |  -   |   -         | Yes          | Full name of port broadcasting the localization data. The server will connect to this port.  | -    |



