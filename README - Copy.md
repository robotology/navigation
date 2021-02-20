# Navigation

A collection of modules to perform 2D navigation with a YARP-based robot

* [Introduction](#introduction)
* [Installation](#installation)
* [Main Modules](#main modules)
* [Examples](#examples)
  - [robotGotoExample1](#robotGotoExample1)
  - [robotGotoExample2](#robotGotoExample2)
  - [robotPathPlannerExample1](#robotPathPlannerExample1)
  - [robotPathPlannerExample2](#robotPathPlannerExample2)

<a name='introduction'></a>
## Introduction

This library has 

<a name='installation'></a>
## Installation

This following packages are required dependencies for Navigation repo:

* YARP (provided by https://github.com/robotology/yarp)
* ICUB (provided by https://github.com/robotology/icub-main)
* ICUBcontrib (provided by https://github.com/robotology/icub-contrib-common)
* OpenCV 
* GSL
* SDL

<a name='main modules'></a>
## Main Modules

<a name='examples'></a>
## Examples

Navigatiton examples are provided by the following YARP applications:

* robotGotoExample [(link to the repo)](https://github.com/robotology/navigation/tree/master/app/robotGotoExample/scripts)
* robotPathPlannerExample  [(link to the repo)](https://github.com/robotology/navigation/tree/master/app/robotaPathPlannerExample/scripts)

Since these examples involve the execution of several, interconnected yarp modules, it's recommended to launch the xml files through [yarpmanager](http://www.yarp.it/yarpmanager.html) GUI.

<a name='robotGotoExample1'></a>
### robotGotoExample1
The module demonstrates how to start robotGoto module. 
First of all, the applications starts two yarpdev devices, **transformServer** and **map2DServer**. The first one is the server which keeps track of the frame transforms used
for localization. The second one loads a collection of simulated 2D enviroment maps.
**fakeMobileBaseTest** is then started. The module creates a simulated robot exposing the same interfaces of **baseControl**. **Rangefinder2DWrapper** and **fakeLaser** are used to
generare a laserScan of the virtual robot in the simulated enviroment. **localizationServer** reads the position of the robot from the **fakeMobileBaseTest** odometry port and
registers the correspoding frame transforms onto the transformServer device. Finally, robotGoto retreives the position of the simulated robot using the yarp **yarp::dev::** interface,
and the current laser scan with **...**. The user can send commands to the robot via rpc port **/robotGoto/rpc**. For example, you can use **gotoAbs 1.0 0.0** to move the robot to position
1.0,0.0 respect to the origin of the current map. Send a **help** command to get complete list of available commands.

<a name='robotGotoExample2'></a>
### robotGotoExample2
This example demonstrates how to use yarp navigation interfaces (**yarp::dev::INavigation2D**) to control robotGoto behavior via user application. The module **robotGotoExample** connects to
robotGoto and sends navigation commands to the virtual robot, asking it to reach different locations in the environment (specified via x,y,theta coordinates respect to the origin of the
current map).

<a name='robotPathPlannerExample1'></a>
### robotPathPlannerExample1
This example is similar to robotGotoExample1. In particular, it launches the same modules used by robotGotoExample1 plus **robotPathPlanner**. RobotPathPlanner connects to robotGoto 
in order to provide more advanced functionalities, i.e. it generates the waypoints which are tracked by robotGoto. The example also opens a yarpview module on which the map of the enviroment is displayed,
as well as the path computed by robotPathPlanner. The user can send navigation commands by clickin on yarpview (or performing a drag&drop operation to give also the direction command). It is also possibile
to control robotPathPlanner via rpc (/robotPathPlanner/rpc). Send a **help** command to get complete list of available commands.

<a name='robotPathPlannerExample2'></a>
### robotPathPlannerExample2
Similary to robotGotoExample2, this application demonstrates how to use yarp navigation interfaces (yarp::dev::INavigation2D) in a user module. **robotPathPlannerExample** module sends
navigation commands to robotPathPlanner, asking the virtual robot to reach, alternatively, a location named **office** and a location named **living room**.



