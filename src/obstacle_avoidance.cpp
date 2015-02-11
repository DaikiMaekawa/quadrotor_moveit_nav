#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/PointCloud.h>
#include <math.h>

static const float tol = 0.000000000000001f;

double magnitude(double vec[3]){
    return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

void normalize(double vec[3]){
    float m = magnitude(vec);
    if(tol <= m) m = 1;
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
        cmd_pub_(node.advertise<geometry_msgs::Twist>("cmd_vel", 10))
    {
        for(int i=0; i < 3; i++) obs_[i] = 0;
    }

    void spin(){
        ros::Duration(1).sleep();
        geometry_msgs::Twist cmd;
        cmd.linear.z = 0.15;
        cmd_pub_.publish(cmd);
        ros::Duration(3).sleep();
        
        cmd.linear.z = 0;
        cmd_pub_.publish(cmd);
        ros::Duration(3).sleep();
        
        double Fs[3];
        double u[3];
        double obs[3];
        
        const double A = 0;
        const double B = 3;
        const double n = 1;
        const double m = 1.5;

        const double force = 0.025;
        
        Fs[0] = Fs[1] = Fs[2] = 0;

        obs[0] = 0.5;
        obs[1] = 0;
        obs[2] = 0;

        u[0] = obs[0];
        u[1] = obs[1];
        u[2] = obs[2];
        normalize(u);
        
        const double d = magnitude(obs);
        double U = -A/pow(d, n) + B/pow(d, m);
        
        Fs[0] += U * u[0];
        Fs[1] += U * u[1];
        Fs[2] += U * u[2];

        cmd.linear.x = Fs[0] * force;
        cmd.linear.y = Fs[1] * force;
        
        ROS_INFO_STREAM("cmd = " << cmd);
        cmd_pub_.publish(cmd);
        ros::Duration(10).sleep();
    }

private:
    void obstacleCallback(const sensor_msgs::PointCloud &obs_msg){
        double min_obs[3];
        
        min_obs[0] = obs_msg.points[0].x;
        min_obs[1] = obs_msg.points[0].y;
        min_obs[2] = obs_msg.points[0].z;

        float min_dist = magnitude(min_obs);
        
        for(int i=1; i < obs_msg.channels.size(); i++){
            double obs[3];
            obs[0] = obs_msg.points[i].x;
            obs[1] = obs_msg.points[i].y;
            obs[2] = obs_msg.points[i].z;

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
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "obstacle_avoidance");
    
    ros::NodeHandle node;
    ArtificialPotentialField apf = ArtificialPotentialField(node);
    apf.spin();
    
    return 0;
}

