#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit_msgs/MoveGroupActionResult.h>

#include <string>
#include <math.h>
#include <vector>

#include "dmath/geometry.h"

class ArtificialPotentialField{
public:
    ArtificialPotentialField(ros::NodeHandle &node) : 
        base_link_("base_link"),
        cmd_pub_(node.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
        obs_sub_(node.subscribe("octomap_full", 10, &ArtificialPotentialField::obstacleCallback, this)),
        goal_sub_(node.subscribe("clicked_point", 10, &ArtificialPotentialField::goalCallback, this)),
        path_sub_(node.subscribe("/move_group/result", 10, &ArtificialPotentialField::globalPathCallback, this))

    {
        collision_map_.header.stamp = ros::Time(0);
    }

    void spin(){
        ros::Rate r(10);
        
        ros::Duration(1).sleep();
        geometry_msgs::Twist cmd;
        cmd.linear.z = 0.15;
        cmd_pub_.publish(cmd);
        ros::Duration(3).sleep();
        
        cmd.linear.z = 0;
        cmd_pub_.publish(cmd);
        ros::Duration(3).sleep();
        
        const double force = 0.025;
        
        while(ros::ok()){
            if(collision_map_.header.stamp != ros::Time(0)){
                std::string map_frame = collision_map_.header.frame_id;
                octomap::OcTree *tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(collision_map_));
                octomap::OcTree::leaf_iterator const end_it = tree->end_leafs();
                
                double min_dist = 99999999;
                dmath::Vector3D min_obs;
                
                ros::Time now = ros::Time::now();
                tf_listener_.waitForTransform(map_frame, base_link_, now, ros::Duration(1));
                for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(0); it != end_it; it++){
                    if(it->getOccupancy() < tree->getOccupancyThres()) continue;
                    
                    geometry_msgs::PointStamped p_in, p_out;
                    p_in.header.frame_id = map_frame;
                    p_in.point.x = it.getX();
                    p_in.point.y = it.getY();
                    p_in.point.z = it.getZ();
                    
                    try{
                        p_in.header.stamp = now;
                        tf_listener_.transformPoint(base_link_, p_in, p_out);
                        dmath::Vector3D obs(p_out.point.x, p_out.point.y, p_out.point.z);
                        double dist = magnitude(obs);
                        if(min_dist > dist){
                            min_dist = dist;
                            min_obs = -obs;
                        }
                        
                    }catch(tf::TransformException &ex){
                        ROS_ERROR_STREAM("Exception trying to transform octomap: " << ex.what());
                    }
                }

                dmath::Vector3D Fs;
                Fs += get_potential_force(min_obs, 0, 3.0, 1.0, 4.0);
                
                double min_goal_dist = 99999999;
                dmath::Vector3D goal_lc;
                for(int i=0; i < path_msg_gl_.result.planned_trajectory.joint_trajectory.points.size(); i++){
                    geometry_msgs::PoseStamped pose_in, pose_out;
                    pose_in.header.frame_id = path_msg_gl_.header.frame_id;
                    pose_in.pose.position.x = path_msg_gl_.result.planned_trajectory.joint_trajectory.points[i].positions[0];
                    pose_in.pose.position.y = path_msg_gl_.result.planned_trajectory.joint_trajectory.points[i].positions[1];
                    pose_in.pose.position.z = path_msg_gl_.result.planned_trajectory.joint_trajectory.points[i].positions[2];
                    pose_in.pose.orientation.x = path_msg_gl_.result.planned_trajectory.joint_trajectory.points[i].positions[3];
                    pose_in.pose.orientation.y = path_msg_gl_.result.planned_trajectory.joint_trajectory.points[i].positions[4];
                    pose_in.pose.orientation.z = path_msg_gl_.result.planned_trajectory.joint_trajectory.points[i].positions[5];
                    pose_in.pose.orientation.w = path_msg_gl_.result.planned_trajectory.joint_trajectory.points[i].positions[6];
                    try{
                        tf_listener_.waitForTransform(pose_in.header.frame_id, base_link_, now, ros::Duration(1));
                        pose_in.header.stamp = now;
                        tf_listener_.transformPose(base_link_, pose_in, pose_out);
                        dmath::Vector3D pos_lc(pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);
                        double dist = magnitude(pos_lc);
                        if(min_goal_dist > dist){
                            min_goal_dist = dist;
                            goal_lc = -pos_lc;
                        }
                    }catch(tf::TransformException &ex){
                        ROS_ERROR_STREAM("Exception trying to transform goal position: " << ex.what());
                    }
                }
                
                /*
                geometry_msgs::PointStamped goal_msg_lc;
                dmath::Vector3D goal_lc;
                try{
                    tf_listener_.waitForTransform(goal_msg_gl_.header.frame_id, base_link_, now, ros::Duration(1));
                    goal_msg_gl_.header.stamp = now;
                    tf_listener_.transformPoint(base_link_, goal_msg_gl_, goal_msg_lc);
                    goal_lc = -dmath::Vector3D(goal_msg_lc.point.x, goal_msg_lc.point.y, goal_msg_lc.point.z);
                }catch(tf::TransformException &ex){
                    ROS_ERROR_STREAM("Exception trying to transform goal position: " << ex.what());
                    goal_lc = dmath::Vector3D();
                }
                */

                Fs += get_potential_force(goal_lc, 50, 0, 1, 1);
                
                dmath::Vector3D vel = Fs * force;

                if(vel.x > 0.5) vel.x = 0.5;
                if(vel.x < -0.5) vel.x = -0.5;
                if(vel.y > 0.5) vel.y = 0.5;
                if(vel.y < -0.5) vel.y = -0.5;
                if(vel.z > 0.5) vel.z = 0.5;
                if(vel.z < -0.5) vel.z = -0.5;
                cmd.linear.x = vel.x;
                cmd.linear.y = vel.y;
                //cmd.linear.z = vel.z;
                
                cmd_pub_.publish(cmd);
            }
            r.sleep();
            ros::spinOnce();
        }
    }

private:
    dmath::Vector3D get_potential_force(const dmath::Vector3D &dest_lc, double A = 1, double B = 1, double n = 1, double m = 1){
        dmath::Vector3D u = dest_lc;
        u = normalize(u);

        const double d = magnitude(dest_lc);
        double U = 0;
        if(fabs(d) > dmath::tol){
            U = -A/pow(d, n) + B/pow(d, m);
        }
        
        return U * u;
    }

    void globalPathCallback(const moveit_msgs::MoveGroupActionResult &path_msg){
        path_msg_gl_ = path_msg;
        ROS_INFO_STREAM("path = " << path_msg_gl_);
    }
    
    void obstacleCallback(const octomap_msgs::OctomapPtr &obs_msg){
        collision_map_ = *obs_msg;
    }

    void goalCallback(const geometry_msgs::PointStamped &goal_msg){
        goal_msg_gl_ = goal_msg;
    }
    
    octomap_msgs::Octomap collision_map_;
    ros::Publisher cmd_pub_;
    ros::Subscriber obs_sub_, goal_sub_, path_sub_;
    tf::TransformListener tf_listener_;
    std::string base_link_;
    geometry_msgs::PointStamped goal_msg_gl_;
    moveit_msgs::MoveGroupActionResult path_msg_gl_;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "apf_planner");
    
    ros::NodeHandle node;
    ArtificialPotentialField apf(node);
    apf.spin();
    
    return 0;
}

