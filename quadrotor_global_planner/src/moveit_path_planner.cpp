#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>

#include <string>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "moveit_path_planner");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    ros::WallDuration time(20.0);
    time.sleep();
    
    move_group_interface::MoveGroup base_group("base");
    //base_group.setPositionTarget(0, 0, 5.0);
    base_group.setRandomTarget();
    moveit::planning_interface::MoveGroup::Plan plan;
    bool ret = base_group.plan(plan);
    if(!ret) ROS_ERROR("plan was not found");
    
    time.sleep();

    return 0;
}
