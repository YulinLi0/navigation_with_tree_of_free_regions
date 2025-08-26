#include <frtree_planner_manage/global_graph.h>

void GlobalGraph::setStartNode(Eigen::Vector3d start_pos)
{
    start_node_ = new GraphNode();
    start_node_->replan_pos_ = start_pos;
    start_node_->parent_ = nullptr;
}

void GlobalGraph::addVisitedPoint(Eigen::Vector3d robot_center)
{
    // inflate the robot center
    double inflate_xy = 1; // the radius_xy id depend on the robot size, it should be larger than the robot size
    double inflate_z = 0.5; // for robot on the ground, this parameter is less important, but for arieal robot, it should be larger than the robot size
    Eigen::Vector3d center = robot_center;

    // add point within the radius_xy and radius_z
    for (double x = center[0] - inflate_xy; x <= center[0] + inflate_xy; x += 0.1)
    {
        for (double y = center[1] - inflate_xy; y <= center[1] + inflate_xy; y += 0.1)
        {
            for (double z = center[2] - inflate_z; z <= center[2] + inflate_z; z += 0.1)
            {
                Eigen::Vector3d pt;
                pt << x, y, z;
                visited_points_.push_back(pt);
            }
        }
    }
}

bool GlobalGraph::isVisited(Eigen::Vector3d center)
{
    // inflate the center of the polyhedron and find out if it intersect with the visited points
    for (int i = 0; i < visited_points_.size(); i++)
    {
        // if the distacne between the center and the visited point is less than 3, return true
        if ((center - visited_points_[i]).norm() < 3)
            return true;
    }
    return false;
}

bool GlobalGraph::isBackToParent(Eigen::Vector3d ray_dir, GraphNode *parent)
{
    // compare the direction of the ray and the direction of the leading ray from parent node
    // if in any plane, the angle between the two ray is less than 5 degree, return true
    // else return false
    double angle_xy, angle_xz, angle_yz;
    Eigen::Vector3d leading_ray = parent->leading_ray_;
    angle_xy = std::acos(ray_dir.head<2>().dot(leading_ray.head<2>()) / (ray_dir.head<2>().norm() * leading_ray.head<2>().norm()));
    if(angle_xy < 5.0 / 180.0 * M_PI)
        return true;
    angle_xz = std::acos(ray_dir.head<2>().dot(leading_ray.head<2>()) / (ray_dir.head<2>().norm() * leading_ray.head<2>().norm()));
    if(angle_xz < 5.0 / 180.0 * M_PI)
        return true;
    angle_yz = std::acos(ray_dir.head<2>().dot(leading_ray.head<2>()) / (ray_dir.head<2>().norm() * leading_ray.head<2>().norm()));
    if(angle_yz < 5.0 / 180.0 * M_PI)
        return true;
    return false;
}
