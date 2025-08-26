#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <frtree_local_map/local_map.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frtree_local_map_node");
    ros::NodeHandle nh;

    GridMap grid_map;

    grid_map.initMap(nh);
    ros::Duration(1.0).sleep();
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
