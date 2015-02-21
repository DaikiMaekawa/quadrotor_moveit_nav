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
        goal_sub_(node.subscribe("clicked_point", 10, &ArtificialPotentialField::goalCallback, this))

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
                std::vector<dmath::Vector3D> obstacles_lc;
                std::string map_frame = collision_map_.header.frame_id;
                octomap::OcTree *tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(collision_map_));
                octomap::OcTree::leaf_iterator const end_it = tree->end_leafs();
                for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(0); it != end_it; it++){
                    geometry_msgs::PointStamped p_in, p_out;
                    p_in.header.frame_id = map_frame;
                    p_in.point.x = it.getX();
                    p_in.point.y = it.getY();
                    p_in.point.z = it.getZ();
                    
                    try{
                        tf_listener_.transformPoint(base_link_, p_in, p_out);
                        dmath::Vector3D obs(p_out.point.x, p_out.point.y, p_out.point.z);
                        if(magnitude(obs) < 500){
                            obstacles_lc.push_back(obs);
                        }
                    }catch(tf::TransformException &ex){
                        ROS_ERROR_STREAM("Exception trying to transform octomap: " << ex.what());
                    }
                }
                
                dmath::Vector3D Fs;
                //for(int i=0; i < obstacles_lc.size(); i++){
                //    Fs += get_potential_force(obstacles_lc[i], 0, 1.0, 1.0, 1.5);
                //}

                geometry_msgs::PointStamped goal_msg_lc;
                dmath::Vector3D goal_lc;
                try{
                    tf_listener_.waitForTransform(goal_msg_gl_.header.frame_id, base_link_, ros::Time(0), ros::Duration(1));
                    goal_msg_gl_.header.stamp = ros::Time(0);
                    tf_listener_.transformPoint(base_link_, goal_msg_gl_, goal_msg_lc);
                    goal_lc = dmath::Vector3D(goal_msg_lc.point.x, goal_msg_lc.point.y, goal_msg_lc.point.z);
                }catch(tf::TransformException &ex){
                    ROS_ERROR_STREAM("Exception trying to transform goal position: " << ex.what());
                    goal_lc = dmath::Vector3D();
                }
                
                Fs += get_potential_force(goal_lc, 100, 0, 1, 1);
                
                dmath::Vector3D vel = Fs * force;
                cmd.linear.x = -vel.x;
                cmd.linear.y = -vel.y;
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
        ROS_INFO_STREAM("dist = " << d);
        double U = 0;
        if(fabs(d) > dmath::tol){
            U = -A/pow(d, n) + B/pow(d, m);
        }
        
        return U * u;
    }

    void obstacleCallback(const octomap_msgs::OctomapPtr &obs_msg){
        collision_map_ = *obs_msg;
    }

    void goalCallback(const geometry_msgs::PointStamped &goal_msg){
        goal_msg_gl_ = goal_msg;
    }
    
    octomap_msgs::Octomap collision_map_;
    ros::Publisher cmd_pub_;
    ros::Subscriber obs_sub_, goal_sub_;
    tf::TransformListener tf_listener_;
    std::string base_link_;
    geometry_msgs::PointStamped goal_msg_gl_;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "apf_planner");
    
    ros::NodeHandle node;
    ArtificialPotentialField apf(node);
    apf.spin();
    
    return 0;
}

