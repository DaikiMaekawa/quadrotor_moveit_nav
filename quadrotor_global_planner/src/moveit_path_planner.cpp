#include <ros/ros.h>
//#include <moveit/robot_state/joint_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>

#include <string>
#include <vector>
#include <map>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "moveit_path_planner");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    ros::WallDuration time(20.0);
    time.sleep();
    
    move_group_interface::MoveGroup base_group("base");
    
    base_group.getJointValueTarget().printStateInfo();
    base_group.getJointValueTarget().printStatePositions();
    ROS_INFO("get variable counts");
    ROS_INFO_STREAM("var = " << base_group.getJointValueTarget().getVariablePositions()[6]);
    std::vector<std::string> names = base_group.getActiveJoints();
    
    for(int i=0; i < names.size(); i++){
        ROS_INFO_STREAM("name: " << names[i]);
    }
    
    moveit::core::RobotState tmp = base_group.getJointValueTarget();
    const double pos[7] = {5.0, -2.0, 3.0, 0, 0, 0, 1};
    tmp.setVariablePositions(pos);
    base_group.setJointValueTarget(tmp);
    
    //base_group.setPositionTarget(0, 0, 5.0);
    //base_group.setRandomTarget();
    moveit::planning_interface::MoveGroup::Plan plan;
    bool ret = base_group.plan(plan);
    if(!ret) ROS_ERROR("plan was not found");
    
    time.sleep();

    return 0;
}
