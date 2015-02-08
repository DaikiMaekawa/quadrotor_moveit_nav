#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> 
#include <math.h>

static const float tol = 0.000000000000001f;

void normalize(double &vec[2]){
    float m = sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
    if(tol <= m) m = 1;
    vec[0] /= m;
    vec[1] /= m;

    if(fabs(vec[0]) < tol) vec[0] = 0.0f;
    if(fabs(vec[1]) < tol) vec[1] = 0.0f;
}

int main(int argc, char *argv[]){
    ros::NodeHandle node;
    ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    ros::Duration(1).sleep();
    geometry_msgs::Twist cmd;
    cmd.linear.z = 0.3;
    cmd_pub.publish(cmd);
    ros::Duration(1).sleep();
    
    cmd.linear.z = 0;
    cmd_pub.publish(cmd);
    ros::Duration(1).sleep();
     
    /*
    double Fs[2];
    double r[2], u[2];
    double obs[2];
    double U = 0;
    
    const double A = 0;
    const double B = 13000;
    const double n = 1;
    const double m = 2.5;

    const double force = 0.025;
    
    Fs[0] = Fs[1] = 0;
    obs[0] = 0.5;
    obs[1] = 0;

    r[0] -= obs[0];
    r[1] -= obs[1];
    u = r;
    normalize(u);
    
    //const double length = 4.0; //TODO: fix param
    //const double d = sqrt(r[0]*r[0] + r[1]*r[1]) / length;
    const double d = sqrt(r[0]*r[0] + r[1]*r[1]);
    U = -A/pow(d, n) + B/pow(d, m);
    
    Fs[0] += U * u[0];
    Fs[1] += U * u[1];
    */

    return 0;
}

