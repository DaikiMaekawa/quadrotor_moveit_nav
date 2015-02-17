#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "local_map_2d_publisher");
    ros::NodeHandle node;
    ros::Publisher map_lc_pub = node.advertise<nav_msgs::OccupancyGrid>("local_map", 1);

    const int size = 5;

    nav_msgs::OccupancyGrid map_lc;
    map_lc.resolution = 2;
    map_lc.frame_id = "/world";
    map_lc.info.origin.position.x = -size/2;
    map_lc.info.origin.position.y = -size/2;
    map_lc.header.stamp = ros::Time::now();
    map_lc.info.width = size;
    map_lc.info.height = size;
    map_lc.data.resize(size * size);
    
    for(int row=0; row < size; row++){
        for(int col=0; col < size; col++){
            map_lc.data[(row*map_lc.info.width)+col] = 0;
        }
    }

    map_lc_pub.publish(map_lc);
    ros::spin();
}
