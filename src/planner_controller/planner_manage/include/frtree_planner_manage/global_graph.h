#ifndef _GLOBAL_GRAPH_H_
#define _GLOBAL_GRAPH_H_

#include <Eigen/Eigen>
#include <vector>
#include <memory>

#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_ros_msgs/EllipsoidArray.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_util/seed_decomp.h>
#include <decomp_util/line_segment.h>
#include <decomp_util/ellipsoid_decomp.h>

struct GraphNode
{
    Eigen::Vector3d replan_pos_;

    // a vector to store the representation of the generated polynomial along the leading ray
    vec_E<Polyhedron3D> polys_;

    // a vector that store the center of the generated polyhedron (also store the center of the intersection between the first and the second polyhedron)
    std::vector<Eigen::Vector3d> center_list_;

    // cost to goal is the cost from the last polynoimal's center to the goal
    double cost_to_goal_;

    // if this branch is a dead end
    bool dead_end_ = false;

    // if this branch has been visited
    bool visited_ = false;

    // if dectect obstacle and start replan woithout reaching the replan_pos
    bool obstacle_ = false;
    Eigen::Vector3d original_replan_pos_;
    bool root = false;

    // leading ray is in global frame, which is the direction from the parent node to the current node
    Eigen::Vector3d leading_ray_;

    GraphNode *parent_;
    std::vector<GraphNode*> child_;
};


class GlobalGraph
{
    public:
        GlobalGraph(){}
        ~GlobalGraph(){}

        typedef std::shared_ptr<GlobalGraph> Ptr;

        void setStartNode(Eigen::Vector3d start_pos);
        GraphNode* getStartNode(){return start_node_;}
        GraphNode* getParentNode(GraphNode *node){return node->parent_;}
        void setGoal(Eigen::Vector3d goal){goal_ = goal;}
        void getGoal(Eigen::Vector3d &goal){goal = goal_;}

        // add visited points according to the robot center
        void addVisitedPoint(Eigen::Vector3d robot_center);
        // check if the region around the center of the second generated polyhedron is visited
        bool isVisited(Eigen::Vector3d center);
        // check if the leading ray will lead back to the parent node
        bool isBackToParent(Eigen::Vector3d ray_dir, GraphNode *parent);

        void nodeDeadEnd(GraphNode *node){node->dead_end_ = true;};

        std::vector<Eigen::Vector3d> points_for_backtrack_;
        std::vector<GraphNode *> nodes_for_backtrack_;

    private:
        GraphNode *start_node_;                             // the start node of the graph, only store the start pos
        std::vector<Eigen::Vector3d> visited_points_;       // the points is all in golbal frame
        Eigen::Vector3d goal_;
        // Eigen::Vector3d subgoal_;
};

#endif