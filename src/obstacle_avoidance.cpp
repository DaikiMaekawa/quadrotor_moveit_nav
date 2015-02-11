#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> 
#include <math.h>

static const float tol = 0.000000000000001f;

void normalize(double vec[2]){
    float m = sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
    if(tol <= m) m = 1;
    vec[0] /= m;
    vec[1] /= m;

    if(fabs(vec[0]) < tol) vec[0] = 0.0f;
    if(fabs(vec[1]) < tol) vec[1] = 0.0f;
}

class ArtificialPotentialField{
public:
    ArtificialPotentialField(){
        for(int i=0; i < 3; i++){
            obs_[i] = 0;
        }
    }

    void spin(){
        double Fs[2];
        double u[2];
        double obs[2];
        
        const double A = 0;
        const double B = 3;
        const double n = 1;
        const double m = 1.5;

        const double force = 0.025;
        
        Fs[0] = Fs[1] = 0;
        obs[0] = 0.5;
        obs[1] = 0;

        u[0] = obs[0];
        u[1] = obs[1];
        normalize(u);
        
        const double d = sqrt(obs[0]*obs[0] + obs[1]*obs[1]);
        double U = -A/pow(d, n) + B/pow(d, m);
        
        Fs[0] += U * u[0];
        Fs[1] += U * u[1];

    }

private:
    void obstacleCallback(const sensor_msgs::ConstPtr &obs_msg){
        int max = obs_msg->get_points_size();
        double obs1[3];
        
        double obs1[0] = obs_msg->points[0].x;
        double obs1[1] = obs_msg->points[0].y;
        double obs1[2] = obs_msg->points[0].z;

        float min = sqrt(pow(x1,2))

    }
    
    double obs_[3];
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "obstacle_avoidance");
    
    ros::NodeHandle node;
    ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    ros::Duration(1).sleep();
    geometry_msgs::Twist cmd;
    cmd.linear.z = 0.15;
    cmd_pub.publish(cmd);
    ros::Duration(3).sleep();
    
    cmd.linear.z = -0.13;
    cmd_pub.publish(cmd);
    ros::Duration(3).sleep();

    cmd.linear.z = 0;
    cmd_pub.publish(cmd);
    ros::Duration(3).sleep();
     
    /*
    cmd.linear.x = Fs[0] * force;
    cmd.linear.y = Fs[1] * force;
    
    ROS_INFO_STREAM("cmd = " << cmd);
    cmd_pub.publish(cmd);
    ros::Duration(10).sleep();
    */

    return 0;
}

