roslaunch faster_lio mapping_mid360.launch & sleep 6;
roslaunch l_ctrl run_ctrl.launch & sleep 2;
roslaunch ego_planner run_in_exp.launch & sleep 2;
roslaunch multipoint multipointplan_sim.launch & sleep 2;
wait;
