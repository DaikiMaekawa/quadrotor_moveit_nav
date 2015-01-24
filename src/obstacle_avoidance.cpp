#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "quadrotor_obstacle_avoidance");
    
    ros::NodeHandle node;
    ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    ros::Duration(1).sleep();
    
    geometry_msgs::Twist cmd;
    
    cmd.linear.z = 1;
    cmd_pub.publish(cmd);
    ros::Duration(5).sleep();
    
    cmd.linear.z = 0.0;
    cmd_pub.publish(cmd);
    ros::Duration(5).sleep();

    return 0;
}

