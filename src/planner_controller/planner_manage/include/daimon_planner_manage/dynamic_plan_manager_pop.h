#ifndef _DYNAMIC_PLAN_MANAGER_POP_H_
#define _DYNAMIC_PLAN_MANAGER_POP_H_

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <string.h>
#include <Eigen/Eigen>

// msgs or srvs
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h> // visulization

#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

// mapping
#include <daimon_local_map/local_map.h>


// global planner
#include <daimon_path_search/jump_point.h>
// #include "jps_planner/jps_planner/jps_planner.h"

// data container
#include <daimon_planner_manage/dynamic_plan_container_pop.hpp>

// graph node
#include <daimon_planner_manage/global_graph.h>

#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_ros_msgs/EllipsoidArray.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_util/seed_decomp.h>
#include <decomp_util/line_segment.h>
#include <decomp_util/ellipsoid_decomp.h>

#include "geo_utils.hpp"
#include "quickhull.hpp"
#include "sdlp.hpp"

class TimeScaling
{
public:
    // start time and end time of local traj
    double last_local_start_time_, last_local_end_time_;
    // start time and end time of global traj
    double global_start_time_, global_end_time_;
    // start time and end time of local traj in global frame
    double start_process_time_, end_process_time_;
    // time ratio used to convert local time to global time
    double last_time_ratio_;
    UniformBspline last_local_traj_, last_local_vel_, last_local_acc_;
    GlobalData global_traj_;

    TimeScaling()
    {
        is_local_valid_ = false;
        is_global_valid_ = false;
    }

    TimeScaling(UniformBspline &local_traj, const double &ts,
                const double &start_time, const int planning_horizon)
    {
        is_local_valid_ = false;
        is_global_valid_ = false;
        reset(local_traj, ts, start_time, planning_horizon);
    }

    void setGlobalTraj(GlobalData &global_trajectory)
    {
        global_traj_ = global_trajectory;
        global_start_time_ = global_trajectory.getStartTime();
        global_end_time_ = global_trajectory.getEndTime();
        is_local_valid_ = false;
        is_global_valid_ = true;
    }

    void getStartAndEndTime(double &start_time, double &end_time)
    {
        start_time = start_process_time_;
        end_time = end_process_time_;
    }

    void reset(UniformBspline &local_traj, const double &ts,
               const double &start_time, const double &end_time)
    {
        /*
         * Note: Reset the TimeScaling, including the last local trajectory,
         *
         */
        is_local_valid_ = true;
        last_local_traj_ = local_traj;
        last_local_vel_ = local_traj.getDerivative();
        last_local_acc_ = last_local_vel_.getDerivative();
        last_local_traj_.getTimeSpan(last_local_start_time_, last_local_end_time_);
        last_time_ratio_ = (end_time - start_time) / (last_local_end_time_ - last_local_start_time_);
        start_process_time_ = start_time;
        end_process_time_ = end_time; // toGlobalTime(last_local_end_time_);
    }

    bool getTrajInfoFromFused(const double &global_time, Eigen::Vector3d &pos, Eigen::Vector3d &vel,
                              Eigen::Vector3d &acc, bool is_global = false)
    /*
     * Note: Global_time is the time from the start of the global trajectory,
     *       TimeScaling will help to give the point based on local and global trajectory.
     *       If the time is out of range, it will return false.
     *       Otherwise, calculating the point on the local trajectory and return true.
     */
    {
        if (!is_global_valid_)
        {
            ROS_ERROR("[TimeScaling]: Global trajectory is not valid!");
            return false;
        }

        if (!is_local_valid_ || is_global)
        {
            pos = global_traj_.getPosition(global_time);
            vel = global_traj_.getVelocity(global_time);
            acc = global_traj_.getAcceleration(global_time);
        }
        else
        {
            if (global_time < end_process_time_ && global_time >= start_process_time_)
            {
                double local_time = toLocalTime(global_time);
                pos = last_local_traj_.evaluateDeBoor(local_time);
                vel = last_local_vel_.evaluateDeBoor(local_time);
                acc = last_local_acc_.evaluateDeBoor(local_time);
            }
            else
            {
                pos = global_traj_.getPosition(global_time);
                vel = global_traj_.getVelocity(global_time);
                acc = global_traj_.getAcceleration(global_time);
            }
        }

        return true;
    }

    bool getTrajInHorizon(double &global_time, const int &horizon_index, double &time_step,
                          Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc)
    {
        /*
         * This will change the global time based on the time ratio
         */
        if (!is_global_valid_)
        {
            ROS_ERROR("[TimeScaling]: Global trajectory is not valid!");
            return false;
        }

        if (!is_local_valid_)
        {
            global_time += horizon_index * time_step;
            pos = global_traj_.getPosition(global_time);
            vel = global_traj_.getVelocity(global_time);
            acc = global_traj_.getAcceleration(global_time);
        }
        else
        {
            double local_time = toLocalTime(global_time);
            // std::cout << global_time << " " << start_process_time_ << " " << end_process_time_ << std::endl;
            if (local_time != -1)
            {
                local_time += horizon_index * time_step;
                if (local_time < last_local_end_time_ && local_time >= last_local_start_time_)
                {
                    global_time += horizon_index * time_step * last_time_ratio_;
                    pos = last_local_traj_.evaluateDeBoor(local_time);
                    vel = last_local_vel_.evaluateDeBoor(local_time);
                    acc = last_local_acc_.evaluateDeBoor(local_time);
                    return true;
                }
            }
            global_time += horizon_index * time_step;
            pos = global_traj_.getPosition(global_time);
            vel = global_traj_.getVelocity(global_time);
            acc = global_traj_.getAcceleration(global_time);
            if (horizon_index == 0 && global_time >= start_process_time_)
                is_local_valid_ = false;
        }
        return true;
    }

    bool getTrajInfoFromGlobal(const double &global_time, Eigen::Vector3d &pos, Eigen::Vector3d &vel,
                               Eigen::Vector3d &acc)
    {
        if (!is_global_valid_)
        {
            ROS_ERROR("[TimeScaling]: Global trajectory is not valid!");
            return false;
        }
        pos = global_traj_.getPosition(global_time);
        vel = global_traj_.getVelocity(global_time);
        acc = global_traj_.getAcceleration(global_time);
        return true;
    }

    bool getTrajInfoFromLocal(const double &local_time, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc)
    {
        double g_time = toGlobalTime(local_time);
        return getTrajInfoFromFused(g_time, pos, vel, acc);
    }

    double toLocalTime(const double &global_time)
    {
        if (global_time <= end_process_time_ && global_time >= start_process_time_)
            return (global_time - start_process_time_) / last_time_ratio_ + last_local_start_time_;
        else
            return -1;
    }

    double toGlobalTime(const double &local_time)
    {
        if (local_time <= last_local_end_time_)
            return (local_time - last_local_start_time_) * last_time_ratio_ + start_process_time_;
        else
            return -1;
    }

    UniformBspline getLocalTraj() { return last_local_traj_; }

    bool checkLocalValid()
    {
        return is_local_valid_;
    }

private:
    bool is_local_valid_ = false;
    bool is_global_valid_ = false;
};

class DynamicPlanManagerPOP
{
private:
    ros::NodeHandle node_;

    // local planner
    // JumpPoint::Ptr planner_jps_;
    // std::unique_ptr<JPSPlanner3D> planner_ptr_;
    
    // local traj optimizer

    bool adjustStartAndTargetPoint(Eigen::Vector3d &start_pt, Eigen::Vector3d &target_pt);

public:
    DynamicPlanManagerPOP() {}
    ~DynamicPlanManagerPOP() {}

    PlanParameters pp_;
    GridMap::Ptr grid_map_;

    GlobalData global_data_;
    LocalTrajData local_traj_data_;
    TimeScaling local_time_scaling_;
    // dynamic obstacle info
    std::string str_dynamic_obs_;
    // ros publisher
    ros::Publisher pub_global_traj_, pub_local_traj_, pub_local_traj_opt_,
        pub_jps_point_, pub_occ_point_, pub_local_goal_;
    // ros::Subscriber sub_laser_scan_;

    ros::Timer poly_test_timer_;
    ros::Timer odom_timer_;
    ros::Publisher poly_test_pub_;
    ros::Publisher point_test_pub_;
    ros::Publisher poly_pub_;
    ros::Publisher interesting_rays_for_replan_pub_;
    ros::Subscriber velodyne_sub_;
    ros::Subscriber interesting_points_sub_;

    void initPlanModules(ros::NodeHandle &nh);

    bool planFromCurrentTraj(const double &planning_distance, Eigen::Vector3d &start_pos);

    // global path when first get the goal
    bool planGlobalPath(GraphNode *node, Eigen::Vector3d &start_pos, Eigen::Vector3d &end_pos);

    bool optimizeLocalTraj(double ts, std::vector<Eigen::Vector3d> point_set, std::vector<Eigen::Vector3d> start_end_derivatives, UniformBspline &bspline_traj);

    // bool ReplanLocalTraj(const int &start_collision_index, 
									//    const Eigen::Vector3d &odom_pos);

    bool ReplanWithInit(double ts);

    bool checkCollision(const Eigen::Vector3d &pos);

    bool checkLocalCollision(const Eigen::Vector3d &pos);

    double checkSDFDistance(const Eigen::Vector3d &pos);

    bool checkTrajCollision(double &distance, int &start_collision_index);

    /* function for polys_ opreate */
    vec_Vec3f obs3d_odom_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_pop_;
    tf2_ros::Buffer tfBuffer_pop_;
    geometry_msgs::TransformStamped::Ptr base_to_odom_ptr_, velodyne_to_base_ptr_;
    vec_E<Polyhedron3D> polys_test_;
    vec_E<Polyhedron3D> polys_;
    std::vector<Eigen::Vector3d> points_for_poly_;
    std::vector<Eigen::Vector3d> interest_points_;
    std::vector<Eigen::Vector3d> points_for_decompose_;
    std::vector<Eigen::Vector3d> points_for_test_1, points_for_test_2, points_for_test_3;
    void polyTestTimerCallback(const ros::TimerEvent &e);
    void odomTimerCallback(const ros::TimerEvent &e);
    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void interestPointsCallback(const visualization_msgs::MarkerPtr &msg);
    std::vector<Eigen::Vector3d> local_waypoints_;
    bool goalInFirstTwoPoly_ = false;
    bool backtrackFlag_ = false;
    bool firstreplanafterbacktrack_ = false;
    vec_E<Polyhedron3D> polys_deadend_;
    vec_E<Polyhedron3D> polys_visited_;

    // graph and node
    GlobalGraph::Ptr global_graph_ptr_;
    GraphNode *current_node_;
    GraphNode *backtrack_node_;

    void setLinearConstraintsandVertices(vec_E<Polyhedron3D> &polys_, std::vector<Eigen::Vector3d> &points_for_poly_);
    void sortPolyhedronArray(vec_E<Polyhedron3D> &polys_, std::vector<Eigen::Vector3d> &points_for_poly_, vec_E<Polyhedron3D> &polys_sorted_, Eigen::Vector3d &goal_pos, std::vector<double> &cost_to_goal_sorted);
    void generatePolyhedronAtIntersection(vec_E<Polyhedron3D> &polys_, GraphNode *node, std::vector<double> &cost_to_goal_sorte);
    bool checkIfNewPolyhedronNeeded(Eigen::Vector3d center, std::vector<std::vector<Eigen::Vector3d>> vertices_for_each_surfaces);
    vec_E<Polyhedron3D> estimateIntersectionVolumeAndGetNewPolyhedron(Polyhedron3D &poly1, Polyhedron3D &poly2);
    bool ifCanPassThroughShortestLine(Polyhedron3D &poly1, Polyhedron3D &poly2, Eigen::Vector3d odom_pos);
    bool ifGoBackToParentCanGetLowerCost(GraphNode *node, int mode);
    void updateCurrentNode(GraphNode *&node, std::vector<Eigen::Vector3d> &points_for_backtrack, std::vector<GraphNode *> &nodes_for_backtrack);
    void getPolysForBackTrack(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos, vec_E<Polyhedron3D> &polys_);
    void getCenterListForNode(GraphNode *&node);
    void getDeadEndPoly(Polyhedron3D &poly, Eigen::Vector3d pos);
    void getVisitedPoly(Polyhedron3D &poly, Eigen::Vector3d pos);

    void decomposePolys();

    /*
     * @brief refine the angle to be continuous
     * @param angle: current angle
     * @param last_angle: last angle
     * @return: refined angle
     * @note: calcute the angle difference with continuous form, because uncontious form will cause occilation
     */
    double refineAngle(double &angle, double &last_angle);

    void displayPointList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                          Eigen::Vector4d color, int id);

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id);

    void debugDisplayTraj(ros::Publisher &pub, const Eigen::MatrixXd &point_set, double scale,
                          Eigen::Vector4d color, int id);

    typedef std::unique_ptr<DynamicPlanManagerPOP> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif