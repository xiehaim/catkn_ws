echo 'nv' | sudo -S chmod 777 /dev/tty* & sleep 1;
roslaunch mavros px4.launch & sleep 6;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
roslaunch faster_lio mapping_mid360.launch & sleep 3;

roslaunch ego_planner all_run.launch & sleep 1;

sh script/record.sh
wait;
#更新sys_run,添加自动删bag和录bag功能
