#ifndef _DYNAMIC_PLAN_FSM_POP_H_
#define _DYNAMIC_PLAN_FSM_POP_H_

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <vector>
#include <cstdlib>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>

#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_ros_msgs/EllipsoidArray.h>
#include <decomp_ros_msgs/PolyhedronArray.h>

#include <daimon_planner_manage/dynamic_plan_manager_pop.h>

#include <Altro/altro_model.h>
#include <Altro/altro_problem.h>
#include <Altro/altro_constraints.h>
#include <Altro/altro_cost.h>

static ros::Publisher cmd_vel;

class DynamicReplanFSMPOP
{
private:
    enum FSM_EXEC_STATE
    {
        INIT,
        WAIT_GOAL,
        GEN_NEW_GLOBAL,
        REPLAN_TRAJ,
        EXEC_LOCAL
    };
    enum MODE_TYPE
    {
        TRAIN = 1,
        TEST = 2
    };
    int mode_; // 1 train , 2 test

    /* planning utils */
    DynamicPlanManagerPOP::Ptr planner_manager_;

    /* planning data */
    bool have_goal_, have_odom_;
    bool doing_tracking_;
    bool stop_publishing_cmd_vel_;
    FSM_EXEC_STATE exec_state_;

    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Quaterniond odom_orient_;
    Eigen::Quaterniond goal_orient_;
    double goal_roll_, goal_pitch_, goal_yaw_;
    double odom_roll_, odom_pitch_, odom_yaw_;

    Eigen::Vector3d start_pos_, start_vel_;
    Eigen::Vector3d end_pos_, end_vel_;

    std::vector<Eigen::Vector3d> landmark_wps_;
    std::vector<Eigen::Vector3d> sample_wps_;
    std::vector<Eigen::Vector3d> local_traj_list_, local_pred_list_;
    std::vector<Eigen::Vector3d> whole_trajectory;
    std::vector<Eigen::Vector3d> interest_rays_;

    size_t curr_wp_index_;

    /* MPC controller */
    Eigen::Vector3d update_state_;
    double delta_time_, planning_time_, start_collision_time_;
    int start_collision_index_;
    int planning_horizon_;
    double check_time_horizon_;

    /* parameters */
    double local_planning_len_, planning_horizen_time_, planning_distance_;
    double goal_tolerance_, goal_yaw_tolerance_; // meter
    double t_replan_thresh_;                     // sec
    int subgoal_drl_mode_;                       // 0: spacial horizon, 1:time_astar
    double subgoal_pub_period_;
    int mid_replan_count_ = 0;
    int track_fut_pts_num_ = 0;

    int replan_count_ = 0;

    bool early_replan_flag_ = false;

    bool if_no_interesting_rays_1_ = false;
    bool if_no_interesting_rays_2_ = false;
    bool if_no_interesting_rays_ahead_1_ = false;
    bool if_no_interesting_rays_ahead_2_ = false;
    bool if_no_interesting_rays_ahead_3_ = false;
    bool if_no_interesting_rays_ahead_4_ = false;

    std::vector<Eigen::Vector3d> replan_points_;    // every two points are the start and end of a line segment, to visualize the graph


    /* ROS utils */
    ros::NodeHandle node_;

    ros::Timer exec_timer_, safety_timer_; // vis_timer_;
    ros::Timer traj_tracker_timer_;
    ros::Timer subgoal_DRL_timer_;
    ros::Timer collision_check_timer_;
    ros::Timer replan_check_timer_;
    ros::Timer global_graph_timer_;
    ros::Timer cmd_timer_;

    // subscriber
    ros::Subscriber goal_sub_, odom_sub_, interesting_rays_sub_;
    ros::Subscriber cmd_vel_sub_debug_;

    // publisher
    ros::Publisher cmd_vel_pub_;
    ros::Publisher subgoal_DRL_pub_;
    ros::Publisher vis_global_plan_pub_, vis_local_plan_pub_, whole_traj_pub_;
    ros::Publisher local_wp_pub_;

    // vis publisher
    ros::Publisher vis_goal_pub_;
    ros::Publisher vis_sample_path_pub_, vis_landmark_pub_;
    ros::Publisher vis_triangle_pub_, vis_local_wp_pub_;
    ros::Publisher vis_local_path_pub_, vis_pred_wp_pub_,  global_graph_pub_, interesting_point_at_decomp_pub_;
    ros::Publisher current_polys_pub_, traj_pose_pub_;

    // paramters for polyhedron decomposition
    vec_E<Polyhedron3D> polys_;
    vec_E<Polyhedron3D> polys_sorted_;
    std::vector<Eigen::Vector3d> points_for_poly_;
    std::vector<Eigen::Vector3d> points_for_poly_sorted_;
    std::vector<double> cost_to_goal_sorted_;
    std::vector<Eigen::Vector3d> back_track_points_;

    /* ros related callback*/
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStampedPtr &msg);

    void interestRaysCallback(const visualization_msgs::MarkerPtr &msg);
    // void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
    void execFSMCallback(const ros::TimerEvent &e);
    void trackTrajCallback(const ros::TimerEvent &e);
    void replanCheckCallback(const ros::TimerEvent &e);
    void publishGlobalGraph(const ros::TimerEvent &e);
    void publishCmdCallback(const ros::TimerEvent &e);

    // bool planFromCurrentTraj(const int trial_times=1 /*=1*/);
    bool achieve_goal();

    /* helper functions */
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    void printFSMExecState();

    /* viusalization*/
    void visualizePath(const std::vector<Eigen::Vector3d> &path, const ros::Publisher &pub);
    void visualizeTrajList(const vector<Eigen::Vector3d> &list, ros::Publisher &pub);
    void getTrajByALTRO();

public:
    DynamicReplanFSMPOP(/* args */) {}
    ~DynamicReplanFSMPOP() {}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Define NStates and NControls
    static constexpr int NStates = 6; // Replace with the actual number of states
    static constexpr int NControls = 6; // Replace with the actual number of controls
    // build a unique pointer to store the MobileManipulatorProblem
    std::unique_ptr<altro::problems::QuadrupedalProblem> QuadrupedalProblem_ptr_;
    // build a unique pointer to alsolver
    std::unique_ptr<altro::augmented_lagrangian::AugmentedLagrangianiLQR<NStates, NControls>> solver_al_ptr_;
    std::shared_ptr<altro::Trajectory<NStates, NControls>> traj_ptr_;
    std::unique_ptr<altro::problem::Problem> probPtr_;

    std::vector<Eigen::Vector3d> traj_from_altro_;
    std::vector<double> yaw_traj_from_altro_;
    double plan_time_;
};

#endif