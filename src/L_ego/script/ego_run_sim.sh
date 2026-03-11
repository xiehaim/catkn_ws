roslaunch ego_planner single_drone_interactive.launch & sleep 4;
roslaunch ego_planner rviz.launch & sleep 2;
roslaunch multipoint multipointplan_sim.launch & sleep 2;
wait;
