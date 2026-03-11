echo 'nv' | sudo -S chmod 777 /dev/tty* & sleep 1;
roslaunch mavros px4.launch & sleep 6;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
roslaunch faster_lio mapping_mid360.launch & sleep 6;
roslaunch l_ctrl run_ctrl.launch & sleep 2;
roslaunch ego_planner run_in_exp.launch & sleep 2;
roslaunch ego_planner exp_rviz.launch & sleep 1;
wait;
