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

#include <string>
#include <math.h>
#include <vector>

#include "dmath/geometry.h"

class ArtificialPotentialField{
public:
    ArtificialPotentialField(ros::NodeHandle &node) : 
        base_link_("base_link"),
        cmd_pub_(node.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
        obs_sub_(node.subscribe("/camera/depth/points", 10, &ArtificialPotentialField::obstacleCallback, this))

    {
        collision_map_.stamp = ros::Duration(0);
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
        
        const double force = 0.0025;
        
        while(ros::ok()){
            if(collision_map_.stamp != ros::Duration(0)){
                std::vector<dmath::Vector3D> obstacles;
                octomap::OcTree tree = octomap_msgs::msgToMap(collision_map_);
                octomap::OcTree::leaf_iterator const end_it = tree->end_leafs();
                for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(0); it != end_it; it++){
                    obstacles.push_back(dmath::Vector3D(it.getX(), it.getY(), it.getZ()));
                }
                
                dmath::Vector3D Fs;
                for(int i=0; i < obstacles.size(); i++){
                    Fs += get_potential_force(obstacles[i], 0, 0.005, 1.0, 1.5);
                }

                //dmath::Vector3D g;
                //Fs += get_potential_force(g, 2, 0, 1.5, 1);
                
                dmath::Vector3D vel = Fs * force;
                cmd.linear.x = vel.y;
                cmd.linear.y = vel.x;
                cmd.linear.z = vel.z;
                
                ROS_INFO_STREAM("cmd = " << cmd);
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

    void obstacleCallback(const octomap_msgs::OctomapPtr &obs_msg){
        collision_map_ = *obs_msg;
    }
    
    octomap_msgs::Octomap collision_map_;
    ros::Publisher cmd_pub_;
    ros::Subscriber obs_sub_;
    tf::TransformListener tf_listener_;
    std::string base_link_;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "apf_planner");
    
    ros::NodeHandle node;
    ArtificialPotentialField apf(node);
    apf.spin();
    
    return 0;
}

