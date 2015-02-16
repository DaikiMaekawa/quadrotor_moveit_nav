#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

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
            dmath::Vector3D Fs;
            for(int i=0; i < obstacles_.size(); i++){
                Fs += get_potential_force(obstacles_[i], 0, 3, 1, 2.0);
            }

            dmath::Vector3D g;
            Fs += get_potential_force(g, 2, 0, 1.5, 1);
            
            dmath::Vector3D vel = Fs * force;
            cmd.linear.x = Fs.y * force;
            cmd.linear.y = Fs.x * force;
            
            //ROS_INFO("obs = (%f, %f)", obs_.x, obs_.y);
            ROS_INFO_STREAM("cmd = " << cmd);
            cmd_pub_.publish(cmd);
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

    void obstacleCallback(const sensor_msgs::PointCloud2Ptr &obs_msg){
        sensor_msgs::PointCloud obs_lsr, obs_base;
        sensor_msgs::convertPointCloud2ToPointCloud(*obs_msg, obs_lsr);
        tf_listener_.transformPointCloud(obs_lsr.header.frame_id, obs_lsr.header.stamp, obs_lsr, base_link_, obs_base);

        if(obs_base.points.size() == 0){
            obstacles_.clear();
            return;
        }
        
        std::vector<dmath::Vector3D> obs_vec;
        for(int i=0; i < obs_base.points.size(); i++){
            dmath::Vector3D obs;
            obs.x = obs_base.points[i].x;
            obs.y = obs_base.points[i].y;
            obs.z = obs_base.points[i].z;
            
            obs_vec.push_back(obs);
        }

        obstacles_ = obs_vec;

        /*
        dmath::Vector3D min_obs;
        min_obs.x = obs_base.points[0].x;
        min_obs.y = obs_base.points[0].y;
        min_obs.z = obs_base.points[0].z;

        float min_dist = magnitude(min_obs);

        for(int i=1; i < obs_base.points.size(); i++){
            dmath::Vector3D obs;
            obs.x = obs_base.points[i].x;
            obs.y = obs_base.points[i].y;
            obs.z = obs_base.points[i].z;
            
            //ROS_INFO("(%f, %f)", obs[0], obs[1]);

            double dist = magnitude(obs);
            if(dist < min_dist){
                min_obs.x = obs.x;
                min_obs.y = obs.y;
                min_obs.z = obs.z;
                min_dist = dist;
            }
        }

        obs_.x = min_obs.x;
        obs_.y = min_obs.y;
        obs_.z = min_obs.z;
        */
    }
    
    std::vector<dmath::Vector3D> obstacles_;
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

