
# Navigation

[![DOI](https://zenodo.org/badge/60769866.svg)](https://zenodo.org/badge/latestdoi/60769866)

A collection of modules to perform 2D navigation with a YARP-based robot.

* [Introduction](#introduction)
* [Installation](#installation)
* [Main Modules](#mainModules)
* [Examples](#examples)
  - [robotGotoExample1](#robotGotoExample1)
  - [robotGotoExample2](#robotGotoExample2)
  - [robotPathPlannerExample1](#robotPathPlannerExample1)
  - [robotPathPlannerExample2](#robotPathPlannerExample2)
  - [rosExample1](#rosExample1)
  - [rosExample2](#rosExample2)
* [Additional Material](#additional)

<a name='introduction'></a>
## Introduction

This repository contains various modules and tools to perform 2D navigation.
In particular, it contains the implementation of the **server** side of most methods of YARP interface [*yarp::dev::INavigation2D*](https://github.com/robotology/yarp/blob/master/src/libYARP_dev/include/yarp/dev/INavigation2D.h),
 such as *gotoTargetByLocationName()*, *gotoTargetByAbsoluteLocation()*, *getCurrentPosition()*, etc.
 In order to perform a navigation task, user needs to launch one of the server modules described below, instantiate in its own module a [*navigation2DClient*](https://github.com/robotology/yarp/blob/master/src/libYARP_dev/include/yarp/dev/INavigation2D.h) device and perform a view of its *yarp::dev::INavigation2D* interface.
 The client device automatically performs all the connections to the server modules, which actually execute the navigation task. Depending on the task, different server modules can be chosen. For example, *robotGoto* performs a simple point-to-point navigation, integrating local obstacle detection and avoidance, while *robotPathPlanner* can compute complex waypoint-based paths. See [Main Modules](#mainModules) for a more detailed description.


<a name='installation'></a>
## Installation

This following packages are required dependencies to successfully compile all the modules included in this repository:

* YARP (provided by https://github.com/robotology/yarp, version >=2.3.71 )
* ICUB (provided by https://github.com/robotology/icub-main version >=1.9.0)
* ICUBcontrib (provided by https://github.com/robotology/icub-contrib-common)
* OpenCV (tested version 2.4.9 )
* GSL (tested 1.14 )
* SDL (tested 1.2.13 )

<a name='mainModules'></a>
## Main Modules

### baseControl
This module behaves as the main interface to control a mobile robot. It computes wheels velocity commands and odometry data, given user cartesian commands, either from a joystick device or a user module. It currently supports iKart and R1 robot kinematics. See [here](https://github.com/robotology/navigation/tree/master/src/baseControl) for a description of available parameters and options.  

### robotGoto
This module, connected to *baseControl*, computes the cartesian velocities required by the robot to perform a point-to-point navigation. The module requires an external localization system and it allows to perform local obstacle avoidance. 
See [here](https://github.com/robotology/navigation/tree/master/src/robotGoto) for a description of available parameters and options.

### robotPathPlanner
This module performs a global navigation by computing a sequence of waypoints to be tracked by a local navigation module. It requires to be attached to an external localization system and to a server providing map information. See [here](https://github.com/robotology/navigation/tree/master/src/robotPathPlanner) for a description of available parameters and options.

### localizationServer
This module acts as the servers side for a *yarp::dev::Localization2DClient*, by publishing the robot position on a yarp port or on a frame transform server. It is also responsible of providing an initial guess of robot estimated position, acting as a bridge for a yarp or ROS localization algorithm (e.g. AMCL).
See [here](https://github.com/robotology/navigation/tree/master/src/localizationServer) for a description of available parameters and options.  

### fakeMobileBaseTest
This module simulates a virtual robot, opening the same ports and interfaces opened by *baseControl*. The user can thus tests its own navigation modules without connecting to a real robot.
See [here](https://github.com/robotology/navigation/tree/master/src/fakeMobileBaseTest) for a description of available parameters and options.  

<a name='development'></a>
## Modules under development
The following modules are currently under development/testing and should be used by developers only.
* mapper2D
* forceGuidance
* iKartNav

<a name='examples'></a>
## Examples

Navigation examples are provided by the following YARP applications:

* robotGotoExamples [(link to the repo)](https://github.com/robotology/navigation/tree/master/app/robotGotoExamples/scripts)
* robotPathPlannerExamples  [(link to the repo)](https://github.com/robotology/navigation/tree/master/app/robotaPathPlannerExamples/scripts)
* rosExamples  [(link to the repo)](https://github.com/robotology/navigation/tree/master/app/rosExamples/scripts)

These examples involve the execution of several, interconnected yarp modules. It is thus recommended to launch the xml files through [yarpmanager](http://www.yarp.it/yarpmanager.html) GUI.
It's also recommended not to start the modules all at once. Simply start each module manually, giving at least a second of waiting time before starting the next one. Yarp ports can be safely connected as a last step.
The following examples are sorted in order of increasing complexity.
Since the interaction between multiple YARP-ROS modules is objectively complex, we encourage users to extensively hack the provided examples, trying to connect, disconnect and modify modules configuration, to better grasp all provided functionalities.
We also encourage users to inspect interconnections between all application modules using [yarpviz](https://github.com/robotology/yarpviz). 

<a name='robotGotoExample1'></a>
### robotGotoExample1
The module demonstrates how to start robotGoto module. 
First of all, the applications starts two yarpdev devices, **transformServer** and **map2DServer**. The first one is the server which keeps track of the frame transforms used for localization. The second one loads a collection of simulated 2D environment maps.
**fakeMobileBaseTest** is then started. The module creates a simulated robot exposing the same interfaces of **baseControl**. **Rangefinder2DWrapper** and **fakeLaser** are used to generate a laser scan of the virtual robot in the simulated environment. **localizationServer** reads the position of the robot from the **fakeMobileBaseTest** odometry port and registers the corresponding frame transforms onto the transformServer device. Finally, robotGoto retrieves the position of the simulated robot using the yarp **yarp::dev::ILocalization2D** interface, and the current laser scan with ***yarp::dev::IRangefinder2D**. The user can send commands to the robot via rpc port */robotGoto/rpc*. For example, user can send a **gotoAbs 1.0 0.0** command to move the robot to position 1.0,0.0 respect to the origin of the current map. Send a **help** command to get complete list of available commands.

<a name='robotGotoExample2'></a>
### robotGotoExample2
This example demonstrates how to use yarp navigation interfaces (**yarp::dev::INavigation2D**) to control robotGoto behavior via user application. The module **robotGotoExample** connects to robotGoto and sends navigation commands to the virtual robot, asking it to reach different locations in the environment (specified via x,y,theta coordinates respect to the origin of the map).

<a name='robotPathPlannerExample1'></a>
### robotPathPlannerExample1
This example is similar to robotGotoExample1. In particular, it launches the same modules used by robotGotoExample1 plus **robotPathPlanner**. RobotPathPlanner connects to robotGoto in order to provide more advanced functionalities, i.e. it generates a sequence of waypoints which are tracked by robotGoto. The example also opens a **yarpview** module on which the map of the environment is displayed, as well as the path computed by robotPathPlanner. The user can send navigation commands by clicking on yarpview (or performing a drag&drop operation to give also the direction command). It is also possible to control robotPathPlanner via rpc (*/robotPathPlanner/rpc*). Send a **help** command to get complete list of available commands.

<a name='robotPathPlannerExample2'></a>
### robotPathPlannerExample2
Similarly to robotGotoExample2, this application demonstrates how to use yarp navigation interfaces (yarp::dev::INavigation2D) in a user module. **robotPathPlannerExample** module sends navigation commands to robotPathPlanner, asking the virtual robot to reach, alternatively, a location named **office** and a location named **living_room**. These locations are stored offline in file *navigation\app\mapsExample\locations.ini*

<a name='rosExample1'></a>
### rosExample1
This application demonstrate YARP-ROS interoperation. In all previous examples,  **localizationServer** retrieves robot's position data from **fakeMobileBaseTest** (either from YARP ports or through transformServer) which computes perfect robot odometry. However, in this example, **fakeMobileBaseTest.ini** is configured to introduce some error in the computation to simulate wheels slippage and make the simulation more realistic. In this case, in order to complete the navigation task, an absolute localization system is required. This example employs ROS's [**AMCL module**](http://wiki.ros.org/amcl) to correct this error with laser localization: fakeMobileBaseTest wrong odometry and laser scan data are sent to AMCL module, which computes the robot pose estimates and publishes it as a frame transform on **/tf** topic. This information is then read back by **transformServer**, broadcasted to **localizationServer** and made available to yarp navigation modules.

<a name='rosExample2'></a>
### rosExample2
This application is similar to the previous one, with the only exception that the map is not loaded from a .map yarp file. Instead, **map2DServer** receives it from ROS **map_server** node, through ROS topics */map* and */map_metadata*. The map is stored into YARP map server with the default name *ros_map*.

<a name='additional'></a>
## AdditionalMaterial
We recommend YARP users who want to learn more about ROS, its core functionalities and its navigation stack to check the following links:
* [**ROS topics**](http://wiki.ros.org/Topics): basic explanation of ROS topics.
* [**TF package**](http://wiki.ros.org/tf2): a library which allows users to keep track of multiple coordinate frames over time. 
* [**AMCL**](http://wiki.ros.org/amcl): a module to perform localization in a previosuly generated map.
* [**Navigation Stack**](http://wiki.ros.org/navigation): ROS navigation stack.
* [**ROS Map server**](http://wiki.ros.org/map_server): a module to broadcast previosuly saved maps on a ROS topic.
* [**gmapping**](http://wiki.ros.org/gmapping): a module to perform laser-based SLAM.




