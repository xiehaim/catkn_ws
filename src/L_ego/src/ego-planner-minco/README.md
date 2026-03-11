# ego-planner-minco
This version uses the MINCO algorithm to optimize the trajectory.

The improvements from the original version are as follows:

- Make the ego-planner a compiled library.
- Add some easy-to-use interfaces to make the ego-planner callable.
- The simple demo of the ego-planner library is:
Note: This demo can not be compiled, just for reference.

```c++

#include <plan_manage/ego_replan_fsm.h>
int main()
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");
  ros::Rate r(20);  // 20hz
  EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);

  while(ros::ok())
  {
    // 这是第一种触发方式，接近目标点一定范围后，规划下一个点
    if(rebo_replan.ifCloseToTarget(0.5))  // 靠近目标点0.5米范围内
    {
        Eigen::Vector3d target_pos(it->head<3>());
        ref_yaw_publish((*it)(3));
        if(rebo_replan.trigger_by_one_waypoint(target_pos)) // 可以调用这个接口来输入单个点位进行ego-planner规划
        {
          // 规划成功
        }
    }
    // 这是第二种触发方式，规划完成后，开始新一轮的规划
    if(!rebo_replan.have_target()) // 
    {
        ros::Duration(2.0).sleep();
        Eigen::Vector3d target_pos(it->head<3>());
        ref_yaw_publish((*it)(3));
        if(rebo_replan.trigger_by_one_waypoint(target_pos))
        {
          // 规划成功
        }
    }
  }
```
