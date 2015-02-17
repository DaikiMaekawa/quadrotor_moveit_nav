#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

/*
void globalMapCallback(const octomap_msgs::OctomapPtr &map_msg){
    
}
*/

int main(int argc, char *argv[]){
    ros::init(argc, argv, "local_map_publisher");

    octomap::OcTree tree(0.1);

    for (int x=-20; x<20; x++) {
        for (int y=-20; y<20; y++) {
            for (int z=-20; z<20; z++) {
            octomap::point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true); // integrate 'occupied' measurement
            }
        }
    }

    for (int x=-30; x<30; x++) {
        for (int y=-30; y<30; y++) {
            for (int z=-30; z<30; z++) {
            octomap::point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
            tree.updateNode(endpoint, false); // integrate 'free' measurement
            }
        }
    }
    
    ros::NodeHandle node;
    ros::Publisher map_lc_pub = node.advertise<octomap_msgs::Octomap>("/local_map", 1, true);
    octomap_msgs::Octomap map_lc_msg;
    map_lc_msg.header.stamp = ros::Time::now();
    map_lc_msg.header.frame_id = "/map";
    octomap_msgs::fullMapToMsg(tree, map_lc_msg);
    map_lc_pub.publish(map_lc_msg);
    //ros::Subscriber map_gl_sub = node.subscribe("", 10, &globalMapCallback);
    
    ros::spin();

    return 0;
}
