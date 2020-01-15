roscore &
sleep 1;
yarp server --ros &
sleep 1;
gazebo --verbose /workspace/r1world.world &
sleep 3;
baseControl --silent --from /home/user1/navigation/app/baseControl_SIM/conf/baseCtrl_cer_sim.ini --skip_robot_interface_check &
sleep 1;
#roslaunch /usr/local/src/robot/cer/app/robots/CER03/robotStatePublisher.launch model:=/usr/local/src/robot/cer/app/robots/CER03/cer.urdf &
roslaunch /home/user1/cer/app/robots/CER03/robotStatePublisher.launch model:=/home/user1/cer/app/robots/CER03/cer.urdf &
sleep 2;
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

