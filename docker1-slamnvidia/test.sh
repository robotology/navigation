roscore &
sleep 1;
yarp server --ros --write&
sleep 1;
gazebo --verbose ./r1world.world &
sleep 3;
/usr/local/src/robot/navigation/build/bin/baseControl --from /usr/local/src/robot/navigation/app/baseControl_SIM/conf/baseCtrl_cer_sim.ini --skip_robot_interface_check &
sleep 2;
cd /usr/local/src/robot/cer/app/robots/CER03
roslaunch robotStatePublisher.launch &
sleep 1;
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

