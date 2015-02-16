#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

#include <string>
#include <math.h>

static const float tol = 0.000000000000001f;

double magnitude(double vec[3]){
    return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

void normalize(double vec[3]){
    float m = magnitude(vec);
    if(tol >= m) m = 1;
    vec[0] /= m;
    vec[1] /= m;
    vec[2] /= m;

    if(fabs(vec[0]) < tol) vec[0] = 0.0f;
    if(fabs(vec[1]) < tol) vec[1] = 0.0f;
    if(fabs(vec[2]) < tol) vec[2] = 0.0f;
}

class ArtificialPotentialField{
public:
    ArtificialPotentialField(ros::NodeHandle &node) : 
        base_link_("base_frame"),
        cmd_pub_(node.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
        obs_sub_(node.subscribe("/camera/depth/points", 10, &ArtificialPotentialField::obstacleCallback, this))

    {
        for(int i=0; i < 3; i++) obs_[i] = 0;
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
            double Fs[3];
            Fs[0] = Fs[1] = Fs[2] = 0;
            
            double f_in[3];
            get_potential_force(obs_, f_in, 0, 2, 1, 1.5);
            
            Fs[0] += f_in[0];
            Fs[1] += f_in[1];
            Fs[2] += f_in[2];

            double g[3];
            g[0] = 0;
            g[1] = 0;
            g[2] = 0;

            get_potential_force(g, f_in, 2, 0, 1.5, 1);

            Fs[0] += f_in[0];
            Fs[1] += f_in[1];
            Fs[2] += f_in[2];

            cmd.linear.x = Fs[1] * force;
            //cmd.linear.y = Fs[1] * force;
            
            ROS_INFO("obs = (%f, %f)", obs_[0], obs_[1]);
            ROS_INFO_STREAM("cmd = " << cmd);
            cmd_pub_.publish(cmd);
            r.sleep();
            ros::spinOnce();
        }
    }

private:
    void get_potential_force(double dest_lc[3], double f_out[3], double A = 1, double B = 1, double n = 1, double m = 1){
        double u[3];
        u[0] = dest_lc[0];
        u[1] = dest_lc[1];
        u[2] = dest_lc[2];
        normalize(u);

        const double d = magnitude(dest_lc);
        double U = 0;
        if(fabs(d) > tol){
            U = -A/pow(d, n) + B/pow(d, m);
        }

        f_out[0] = U * u[0];
        f_out[1] = U * u[1];
        f_out[2] = U * u[2];
    }

    void obstacleCallback(const sensor_msgs::PointCloud2Ptr &obs_msg){
        sensor_msgs::PointCloud obs_lsr, obs_base;
        sensor_msgs::convertPointCloud2ToPointCloud(*obs_msg, obs_lsr);
        tf_listener_.transformPointCloud(obs_lsr.header.frame_id, obs_lsr.header.stamp, obs_lsr, base_link_, obs_base);

        if(obs_base.points.size() == 0){
            obs_[0] = 0;
            obs_[1] = 0;
            obs_[2] = 0;
            return;
        }
        
        double min_obs[3];
        min_obs[0] = obs_base.points[0].x;
        min_obs[1] = obs_base.points[0].y;
        min_obs[2] = obs_base.points[0].z;

        float min_dist = magnitude(min_obs);

        for(int i=1; i < obs_base.points.size(); i++){
            double obs[3];
            obs[0] = obs_base.points[i].x;
            obs[1] = obs_base.points[i].y;
            obs[2] = obs_base.points[i].z;
            
            //ROS_INFO("(%f, %f)", obs[0], obs[1]);

            double dist = magnitude(obs);
            if(dist < min_dist){
                min_obs[0] = obs[0];
                min_obs[1] = obs[1];
                min_obs[2] = obs[2];
                min_dist = dist;
            }
        }

        obs_[0] = min_obs[0];
        obs_[1] = min_obs[1];
        obs_[2] = min_obs[2];
    }
    
    double obs_[3];
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

