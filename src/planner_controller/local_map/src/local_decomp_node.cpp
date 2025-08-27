#include <frtree_local_map/local_decomp.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_decomp_node");
    ros::NodeHandle nh("~");

    LocalDecomp local_decomp;
    local_decomp.init(nh);
    ros::spin();
    return 0;
}