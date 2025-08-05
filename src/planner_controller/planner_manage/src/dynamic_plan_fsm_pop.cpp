#include <daimon_planner_manage/dynamic_plan_fsm_pop.h>

using std::cout;
using std::endl;

#define DEBUG

// pos and yaw to 4 by 4 transformation matrix
static void toTransformationMatrix(Eigen::Matrix4d &T, const Eigen::Vector3d &pos, const double &yaw)
{
    T.setIdentity();
    T.block<3, 1>(0, 3) = pos;
    T.block<2, 2>(0, 0) = Eigen::Rotation2Dd(yaw).matrix();
}

// quaternion to euler angle
static void toEulerAngle(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}

static void stopRobot(){
    ROS_INFO("[Plan FSM]: Stop robot.");
    geometry_msgs::Twist twist_cmd;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;

    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    cmd_vel.publish(twist_cmd);
}

void DynamicReplanFSMPOP::init(ros::NodeHandle &nh)
{

    exec_state_ = FSM_EXEC_STATE::INIT;
    have_goal_ = false;
    have_odom_ = false;

    /*  fsm param  */
    node_ = nh;
    node_.param("fsm/goal_tolerance", goal_tolerance_, 0.15); // sim1/plan_manager/fsm/goal_tolerance
    node_.param("fsm/goal_yaw_tolerance", goal_yaw_tolerance_, 0.1);
    node_.param("fsm/check_horizon", check_time_horizon_, 5.0);
    node_.param("fsm/planner/planning_distance", planning_distance_, 3.0);
    node_.param("fsm/controller/delta_time", delta_time_, 0.1);
    node_.param("fsm/controller/track_future_pts_num", track_fut_pts_num_, 10);

    // planning_horizon_ = int(planning_time_ / delta_time_);
    // ROS_INFO("[Plan FSM]: planning_horizon: %d", planning_horizon_);
// std::cout << "fsm param loaded" << endl;
    /* initialize main modules */
    planner_manager_.reset(new DynamicPlanManagerPOP());
    planner_manager_->initPlanModules(node_);
    planner_manager_->local_traj_data_ = LocalTrajData();



    /* callback */
    exec_timer_            = node_.createTimer(ros::Duration(0.2), &DynamicReplanFSMPOP::execFSMCallback, this);
    // traj_tracker_timer_    = node_.createTimer(ros::Duration(delta_time_), &DynamicReplanFSMPOP::trackTrajCallback, this);
    replan_check_timer_    = node_.createTimer(ros::Duration(0.2), &DynamicReplanFSMPOP::replanCheckCallback, this);
    global_graph_timer_    = node_.createTimer(ros::Duration(0.2), &DynamicReplanFSMPOP::publishGlobalGraph, this);
    cmd_timer_             = node_.createTimer(ros::Duration(0.2), &DynamicReplanFSMPOP::publishCmdCallback, this);

    /* ros communication with public node */
    ros::NodeHandle public_nh; // sim1/goal
    goal_sub_ = public_nh.subscribe("goal", 1, &DynamicReplanFSMPOP::goalCallback, this);
    odom_sub_ = public_nh.subscribe("odom", 1, &DynamicReplanFSMPOP::odomCallback, this);
    // poly_sub_ = public_nh.subscribe("/local_decomp_node/polyhedron_array", 1, &DynamicReplanFSMPOP::polyDecomposeCallback, this);
    // points_for_poly_sub_ = public_nh.subscribe("/local_decomp_node/points_for_polyhedron_array", 1, &DynamicReplanFSMPOP::pointsForPolyCallback, this);
    interesting_rays_sub_ = node_.subscribe("interesting_rays_for_replan", 1, &DynamicReplanFSMPOP::interestRaysCallback, this);
    global_graph_pub_ = public_nh.advertise<visualization_msgs::Marker>("global_graph", 10);
    interesting_point_at_decomp_pub_ = public_nh.advertise<visualization_msgs::Marker>("interesting_point_at_decomp", 10);


#ifdef DEBUG
    stop_publishing_cmd_vel_ = false;
    // cmd_vel_sub_debug_ = public_nh.subscribe("cmd_vel", 1, &DynamicReplanFSMPOP::cmdVelCallback, this);
#endif

    cmd_vel_pub_ = public_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    local_wp_pub_ = public_nh.advertise<geometry_msgs::PoseStamped>("local_wp", 10);

    /* visualization */
    vis_global_plan_pub_ = public_nh.advertise<nav_msgs::Path>("globalPlan", 10); // /ns/globalPlan
    vis_local_plan_pub_ = public_nh.advertise<nav_msgs::Path>("localPlan", 10);   // /ns/localPlan
    whole_traj_pub_ = public_nh.advertise<nav_msgs::Path>("wholeTraj", 10);       // /ns/wholeTraj
    vis_landmark_pub_ = public_nh.advertise<visualization_msgs::Marker>("vis_landmarks_", 10);
    vis_goal_pub_ = public_nh.advertise<geometry_msgs::PoseStamped>("vis_goal", 10);
    vis_local_wp_pub_ = public_nh.advertise<geometry_msgs::PoseArray>("vis_local_path", 10);
    vis_pred_wp_pub_ = public_nh.advertise<geometry_msgs::PoseArray>("vis_pred_path", 10);

    current_polys_pub_ = public_nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_for_current_node", 1, true);
    traj_pose_pub_ = public_nh.advertise<geometry_msgs::PoseArray>("traj_pose_vis", 1, true);
}

// void DynamicReplanFSMPOP::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
// {
//     if (cmd_vel_sub_debug_.getNumPublishers() > 1)
//     {
//         // ROS_WARN("cmd_vel_sub_debug has more than one publisher");
//         stop_publishing_cmd_vel_ = true;
//     }
//     else
//     {
//         stop_publishing_cmd_vel_ = false;
//     }
// }

void DynamicReplanFSMPOP::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    // TODO update to 3D space
    odom_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z);
// std::cout << "odom pos: " << odom_pos_.transpose() << endl;
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    toEulerAngle(odom_orient_, odom_roll_, odom_pitch_, odom_yaw_);
    update_state_(0) = odom_pos_(0);
    update_state_(1) = odom_pos_(1);
    update_state_(2) = odom_yaw_;

    have_odom_ = true;
}

void DynamicReplanFSMPOP::goalCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    if (have_odom_ == false)
        return;

    // end pt without orientation
    end_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    end_pos_(2) = odom_pos_(2);
    end_vel_ = Eigen::Vector3d::Zero();

    goal_orient_.w() = msg->pose.orientation.w;
    goal_orient_.x() = msg->pose.orientation.x;
    goal_orient_.y() = msg->pose.orientation.y;
    goal_orient_.z() = msg->pose.orientation.z;
    toEulerAngle(goal_orient_, goal_roll_, goal_pitch_, goal_yaw_);
    
    planner_manager_->local_traj_data_.reset();
    stopRobot();
    ROS_INFO("[Plan FSM]: Goal set!");
    changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
}

void DynamicReplanFSMPOP::execFSMCallback(const ros::TimerEvent &e)
{
    /* init phase wating for odom & goal */
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 2000)
    {
        if (!have_odom_)
        {
            ROS_INFO("[Plan FSM]: no odom.");
        }
        if (!have_goal_)
        {
            ROS_INFO("[Plan FSM]: wait for goal.");
        }
        printFSMExecState();
        fsm_num = 0;
    }

    /* FSM state-> action */
    switch (exec_state_)
    {

    /* --------------Case1:INIT ---------------*/
    case INIT:
    {
        if (!have_odom_)
        {
            return;
        }

        Eigen::Vector3d start_pos = odom_pos_;
        planner_manager_->global_graph_ptr_->setStartNode(start_pos);
        planner_manager_->current_node_ = planner_manager_->global_graph_ptr_->getStartNode();
        planner_manager_->current_node_->root = true;
        Eigen::Vector3d ray;
        ray << 0, 0, 0;
        planner_manager_->current_node_->leading_ray_ = ray;
        replan_points_.clear();
        Polyhedron3D visited_poly;
        planner_manager_->getVisitedPoly(visited_poly, planner_manager_->current_node_->replan_pos_);
        planner_manager_->polys_visited_.push_back(visited_poly);

        if (!have_goal_)
        {
            return;
        }
        changeFSMExecState(WAIT_GOAL, "FSM");
        break;
    }

    /* --------------Case2:WAIT_GOAL ---------------*/
    case WAIT_GOAL:
    {
        if (!have_goal_)
            return;
        else
        {
            changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
        }
        break;
    }

    /* --------------Case3:GEN_NEW_GLOBAL ---------------*/
    case GEN_NEW_GLOBAL:
    {
        if (odom_vel_.norm() > 0.1)
        {
            ROS_INFO("[Plan FSM]: wait for odom stop.");
            stopRobot();
            return;
        }

        // plan the global traj
        start_pos_ = odom_pos_;
        {
            // based on recived polyhedron array, first sort the polyhedron array based on the distance between the center of the last polyhedron and the goal
            cost_to_goal_sorted_.clear();
            polys_sorted_.clear();

            planner_manager_->decomposePolys();
            // planner_manager_->setLinearConstraintsandVertices(planner_manager_->polys_, planner_manager_->points_for_poly_);

            planner_manager_->sortPolyhedronArray(planner_manager_->polys_, planner_manager_->points_for_poly_, polys_sorted_, end_pos_, cost_to_goal_sorted_);            
            // check if the intersection is good enough
            planner_manager_->generatePolyhedronAtIntersection(polys_sorted_, planner_manager_->current_node_, cost_to_goal_sorted_);
            
            for (int i = 0; i < planner_manager_->current_node_->child_.size(); i++)
            {
                replan_points_.push_back(planner_manager_->current_node_->replan_pos_);
                replan_points_.push_back(planner_manager_->current_node_->child_[i]->replan_pos_);
            }
std::cout << "current node child size: " << planner_manager_->current_node_->child_.size() << endl;
            // get the best choose, if not feasible use second best. if all path is not feasible, use parent node's second
            planner_manager_->global_graph_ptr_->points_for_backtrack_.clear();
            planner_manager_->updateCurrentNode(planner_manager_->current_node_, planner_manager_->global_graph_ptr_->points_for_backtrack_, planner_manager_->global_graph_ptr_->nodes_for_backtrack_);        
            if(!planner_manager_->backtrackFlag_){
                Polyhedron3D visited_poly;
                planner_manager_->getVisitedPoly(visited_poly, planner_manager_->current_node_->replan_pos_);
                planner_manager_->polys_visited_.push_back(visited_poly);
            }
            planner_manager_->current_node_->visited_ = true;
            if(planner_manager_->backtrackFlag_){
std::cout << "backtrack flag is true" << endl;
                    planner_manager_->backtrack_node_ = new GraphNode();
                    planner_manager_->backtrack_node_->replan_pos_ = odom_pos_;
                    planner_manager_->backtrack_node_->parent_ = nullptr;
                    planner_manager_->backtrack_node_->root = true;
            }
            bool success = false;
            if(!planner_manager_->backtrackFlag_){
                success = planner_manager_->planGlobalPath(planner_manager_->current_node_, start_pos_, end_pos_);
                // only need first two polyhedrons in current node
                vec_E<Polyhedron3D> first_two_polys;
                first_two_polys.push_back(planner_manager_->current_node_->polys_[0]);
                first_two_polys.push_back(planner_manager_->current_node_->polys_[1]);
                decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(first_two_polys);
                poly_msg.header.frame_id = "odom";
	            current_polys_pub_.publish(poly_msg);
            }

            if (planner_manager_->backtrackFlag_){
//                 polys_sorted_.clear();
//                 planner_manager_->getPolysForBackTrack(odom_pos_, planner_manager_->global_graph_ptr_->points_for_backtrack_[0], polys_sorted_);
//                 cost_to_goal_sorted_.push_back(1);
//                 planner_manager_->generatePolyhedronAtIntersection(polys_sorted_, planner_manager_->backtrack_node_, cost_to_goal_sorted_);
// std::cout << "size of points for backtrack: " << planner_manager_->global_graph_ptr_->points_for_backtrack_.size() << endl;
// for (int i = 0; i < planner_manager_->global_graph_ptr_->points_for_backtrack_.size(); i++){
//     std::cout << "point " << i << ": " << planner_manager_->global_graph_ptr_->points_for_backtrack_[i].transpose() << endl;
// }
// std::cout << "size of backtrack node child: " << planner_manager_->backtrack_node_->child_.size() << endl;
// std::cout << "size of backtrack node child's polys_: " << planner_manager_->backtrack_node_->child_[0]->polys_.size() << std::endl;
// std::cout << "backtrack_node_ replan pos: " << planner_manager_->backtrack_node_->replan_pos_.transpose() << endl;
// std::cout << "current subgoal: " << planner_manager_->global_graph_ptr_->points_for_backtrack_[0].transpose() << endl;
//                 planner_manager_->backtrack_node_ = planner_manager_->backtrack_node_->child_[0];
// std::cout << "backtrack_node_ replan pos after updates: " << planner_manager_->backtrack_node_->replan_pos_.transpose() << endl;
// std::cout << "size of backtrack node's polys_: " << planner_manager_->backtrack_node_->polys_.size() << std::endl;


//             success = planner_manager_->planGlobalPath(planner_manager_->backtrack_node_, start_pos_, planner_manager_->global_graph_ptr_->points_for_backtrack_[0]);


                if(planner_manager_->global_graph_ptr_->nodes_for_backtrack_.size() < 2){
                    // the last must be the same direction, from parent to child
                    planner_manager_->backtrack_node_ = planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0];
                }
                else{
                    // still in reverse direction, from child to parent
                    GraphNode *tmp_node;
                    tmp_node->replan_pos_ = planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->center_list_[0];
                    tmp_node->parent_ = planner_manager_->backtrack_node_;
                    tmp_node->center_list_.push_back(planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->center_list_[2]);
                    tmp_node->center_list_.push_back(planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->center_list_[1]);
                    tmp_node->center_list_.push_back(planner_manager_->global_graph_ptr_->points_for_backtrack_[0]);
                    tmp_node->leading_ray_ = -planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->leading_ray_;
                    tmp_node->polys_.push_back(planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->polys_[1]);
                    tmp_node->polys_.push_back(planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->polys_[0]);
                    planner_manager_->backtrack_node_->child_.push_back(tmp_node);
                    planner_manager_->backtrack_node_ = planner_manager_->backtrack_node_->child_[0];
                }
std::cout << "points for back track[0]: " << planner_manager_->global_graph_ptr_->points_for_backtrack_[0].transpose() << endl;
std::cout << "back track nodes's center list[2]: " << planner_manager_->backtrack_node_->center_list_[2].transpose() << endl;

            success = planner_manager_->planGlobalPath(planner_manager_->backtrack_node_, start_pos_, planner_manager_->global_graph_ptr_->points_for_backtrack_[0]);
            }

            if (success)
            {
                ROS_INFO("\033[1;32m[Plan Manager]: GLOBAL_PLAN Success\033[0m");
                getTrajByALTRO();
                visualizePath(planner_manager_->global_data_.getGlobalPath(), vis_global_plan_pub_); // visualize global path
                have_goal_ = true;
                changeFSMExecState(EXEC_LOCAL, "FSM");
            }
            else
            {
                ROS_ERROR("[Plan FSM]: Failed to generate the global plan!");
                have_goal_ = false;
                changeFSMExecState(WAIT_GOAL, "FSM");
                return ;
            }
        }
        break;
    }

    /* --------------Case4:REPLAN_TRAJ ---------------*/
    case REPLAN_TRAJ:
    {
        if(achieve_goal()) return;

        ROS_INFO("\033[1;32m[Plan Manager]: Enter Replan\033[0m");

        start_pos_ = odom_pos_;

        if (early_replan_flag_)
        {
            if (!planner_manager_->backtrackFlag_)
            {
                planner_manager_->current_node_ ->original_replan_pos_ = planner_manager_->current_node_ ->replan_pos_;
                planner_manager_->current_node_ ->replan_pos_ = odom_pos_;
                planner_manager_->current_node_ ->obstacle_ = true;
            }
            early_replan_flag_ = false;
        }
        cost_to_goal_sorted_.clear();
        polys_sorted_.clear();
        planner_manager_->decomposePolys();
std::cout << "size of polys: " << planner_manager_->polys_.size() << endl;
        // planner_manager_->setLinearConstraintsandVertices(planner_manager_->polys_, planner_manager_->points_for_poly_);
        bool success = false;           
        // check if the intersection is good enough
        if (!planner_manager_->backtrackFlag_ && !planner_manager_->firstreplanafterbacktrack_){
            planner_manager_->sortPolyhedronArray(planner_manager_->polys_, planner_manager_->points_for_poly_, polys_sorted_, end_pos_, cost_to_goal_sorted_); 
            planner_manager_->generatePolyhedronAtIntersection(polys_sorted_, planner_manager_->current_node_, cost_to_goal_sorted_);
// std::cout << "111" << endl;
std::cout << "current node replan pos: " << planner_manager_->current_node_->replan_pos_.transpose() << endl;
std::cout << "current node child size: " << planner_manager_->current_node_->child_.size() << endl;
for (int i = 0; i < planner_manager_->current_node_->child_.size(); i++){
    std::cout << "child " << i << " replan pos: " << planner_manager_->current_node_->child_[i]->replan_pos_.transpose() << endl;
}
            for (int i = 0; i < planner_manager_->current_node_->child_.size(); i++)
            {
                replan_points_.push_back(planner_manager_->current_node_->replan_pos_);
                replan_points_.push_back(planner_manager_->current_node_->child_[i]->replan_pos_);
            }
            
            // get the best choose, if not feasible use second best. if all path is not feasible, use parent node's second
            if(planner_manager_->ifGoBackToParentCanGetLowerCost(planner_manager_->current_node_, 0)){
                planner_manager_->global_graph_ptr_->points_for_backtrack_.clear();
                planner_manager_->global_graph_ptr_->points_for_backtrack_.push_back(planner_manager_->current_node_->parent_->replan_pos_);
                planner_manager_->global_graph_ptr_->points_for_backtrack_.push_back(planner_manager_->current_node_->parent_->child_[1]->replan_pos_);
                planner_manager_->backtrackFlag_ = true;
                planner_manager_->backtrack_node_ = new GraphNode();
                planner_manager_->backtrack_node_->replan_pos_ = odom_pos_;
                planner_manager_->backtrack_node_->parent_ = nullptr;
                planner_manager_->backtrack_node_->root = true;
                planner_manager_->global_graph_ptr_->nodes_for_backtrack_.push_back(planner_manager_->current_node_);
                planner_manager_->current_node_ = planner_manager_->current_node_->parent_;
            }
            else{
                planner_manager_->global_graph_ptr_->points_for_backtrack_.clear();
                // planner_manager_->polys_deadend_.clear();
std::cout << "start to update current node" << endl;
                planner_manager_->updateCurrentNode(planner_manager_->current_node_, planner_manager_->global_graph_ptr_->points_for_backtrack_, planner_manager_->global_graph_ptr_->nodes_for_backtrack_);
                if(!planner_manager_->backtrackFlag_){
                    Polyhedron3D visited_poly;
                    planner_manager_->getVisitedPoly(visited_poly, planner_manager_->current_node_->replan_pos_);
                    planner_manager_->polys_visited_.push_back(visited_poly);
                }   
                planner_manager_->current_node_->visited_ = true;
                if(planner_manager_->backtrackFlag_){
std::cout << "planner_manager_->current_node_ replan pos: " << planner_manager_->current_node_->replan_pos_.transpose() << endl;
std::cout << "backtrack flag is true" << endl;
                    planner_manager_->backtrack_node_ = new GraphNode();
                    planner_manager_->backtrack_node_->replan_pos_ = odom_pos_;
                    planner_manager_->backtrack_node_->parent_ = nullptr;
                    planner_manager_->backtrack_node_->root = true;
                    for (int i = 0; i < planner_manager_->global_graph_ptr_->nodes_for_backtrack_.size(); i++){
                        if (planner_manager_->global_graph_ptr_->nodes_for_backtrack_[i]->dead_end_){
                            Polyhedron3D deadend_poly;
                            planner_manager_->getDeadEndPoly(deadend_poly, planner_manager_->global_graph_ptr_->nodes_for_backtrack_[i]->replan_pos_);
                            planner_manager_->polys_deadend_.push_back(deadend_poly);
                        }
                    }
                }
            }
std::cout << "back check done" << endl;
            if(!planner_manager_->backtrackFlag_){
                success = planner_manager_->planGlobalPath(planner_manager_->current_node_, start_pos_, end_pos_);
std::cout << "size of polys_deadend 111: " << planner_manager_->polys_deadend_.size() << endl;
                // only need first two polyhedrons in current node
                vec_E<Polyhedron3D> first_two_polys;
                first_two_polys.push_back(planner_manager_->current_node_->polys_[0]);
                first_two_polys.push_back(planner_manager_->current_node_->polys_[1]);
                decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(first_two_polys);
                poly_msg.header.frame_id = "odom";
	            current_polys_pub_.publish(poly_msg);
            }
        }
        if (!planner_manager_->backtrackFlag_ && planner_manager_->firstreplanafterbacktrack_){
std::cout << "first replan after backtrack" << endl;
            if(planner_manager_->ifGoBackToParentCanGetLowerCost(planner_manager_->current_node_, 1)){
                planner_manager_->global_graph_ptr_->points_for_backtrack_.clear();
                planner_manager_->global_graph_ptr_->points_for_backtrack_.push_back(planner_manager_->current_node_->parent_->replan_pos_);
                // planner_manager_->global_graph_ptr_->points_for_backtrack_.push_back(planner_manager_->current_node_->parent_->child_[1]->replan_pos_);
                planner_manager_->backtrackFlag_ = true;
                planner_manager_->backtrack_node_ = new GraphNode();
                planner_manager_->backtrack_node_->replan_pos_ = odom_pos_;
                planner_manager_->backtrack_node_->parent_ = nullptr;
                planner_manager_->backtrack_node_->root = true;
                planner_manager_->global_graph_ptr_->nodes_for_backtrack_.push_back(planner_manager_->current_node_);
                planner_manager_->current_node_ = planner_manager_->current_node_->parent_;
            }
            else{
                planner_manager_->firstreplanafterbacktrack_ = false;
                planner_manager_->global_graph_ptr_->points_for_backtrack_.clear();
                planner_manager_->global_graph_ptr_->nodes_for_backtrack_.clear();
                // planner_manager_->polys_deadend_.clear();
                
std::cout << "start to update current node" << endl;
                planner_manager_->updateCurrentNode(planner_manager_->current_node_, planner_manager_->global_graph_ptr_->points_for_backtrack_, planner_manager_->global_graph_ptr_->nodes_for_backtrack_);
                if(!planner_manager_->backtrackFlag_){
                    Polyhedron3D visited_poly;
                    planner_manager_->getVisitedPoly(visited_poly, planner_manager_->current_node_->replan_pos_);
                    planner_manager_->polys_visited_.push_back(visited_poly);
                }

                planner_manager_->current_node_->visited_ = true;

for (int i = 0; i < planner_manager_->global_graph_ptr_->nodes_for_backtrack_.size(); i++){
    std::cout << "size of backtrack node's center list: " << planner_manager_->global_graph_ptr_->nodes_for_backtrack_[i]->center_list_.size() << std::endl;
    std::cout << "if node's is root node: " << planner_manager_->global_graph_ptr_->nodes_for_backtrack_[i]->root << std::endl;
}


                if(planner_manager_->backtrackFlag_){
std::cout << "planner_manager_->current_node_ replan pos: " << planner_manager_->current_node_->replan_pos_.transpose() << endl;

std::cout << "backtrack flag is true" << endl;
                    planner_manager_->backtrack_node_ = new GraphNode();
                    planner_manager_->backtrack_node_->replan_pos_ = odom_pos_;
                    planner_manager_->backtrack_node_->parent_ = nullptr;
                    planner_manager_->backtrack_node_->root = true;
                    for (int i = 0; i < planner_manager_->global_graph_ptr_->nodes_for_backtrack_.size(); i++){
                        if (planner_manager_->global_graph_ptr_->nodes_for_backtrack_[i]->dead_end_){
                            Polyhedron3D deadend_poly;
                            planner_manager_->getDeadEndPoly(deadend_poly, planner_manager_->global_graph_ptr_->nodes_for_backtrack_[i]->replan_pos_);
                            planner_manager_->polys_deadend_.push_back(deadend_poly);
                        }
                    }
                }
                else{
                    // planner_manager_->polys_deadend_.clear();
                }
            }
            if(!planner_manager_->backtrackFlag_){
                success = planner_manager_->planGlobalPath(planner_manager_->current_node_, start_pos_, end_pos_);
                
std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!1size of polys_deadend!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: " << planner_manager_->polys_deadend_.size() << endl;
                // only need first two polyhedrons in current node
                vec_E<Polyhedron3D> first_two_polys;
                first_two_polys.push_back(planner_manager_->current_node_->polys_[0]);
                first_two_polys.push_back(planner_manager_->current_node_->polys_[1]);
                decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(first_two_polys);
                poly_msg.header.frame_id = "odom";
	            current_polys_pub_.publish(poly_msg);
            }
        }


        if (planner_manager_->backtrackFlag_){
//             polys_sorted_.clear();
std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!size of polys_deadend!!!!!!!!!!!!!!!!!!!!!!!!!!!!: " << planner_manager_->polys_deadend_.size() << endl;
//             planner_manager_->getPolysForBackTrack(odom_pos_, planner_manager_->global_graph_ptr_->points_for_backtrack_[0], polys_sorted_);
//             cost_to_goal_sorted_.push_back(1);
//             planner_manager_->generatePolyhedronAtIntersection(polys_sorted_, planner_manager_->backtrack_node_, cost_to_goal_sorted_);
// std::cout << "size of points for backtrack: " << planner_manager_->global_graph_ptr_->points_for_backtrack_.size() << endl;
// for (int i = 0; i < planner_manager_->global_graph_ptr_->points_for_backtrack_.size(); i++){
//     std::cout << "point " << i << ": " << planner_manager_->global_graph_ptr_->points_for_backtrack_[i].transpose() << endl;
// }
// std::cout << "current subgoal: " << planner_manager_->global_graph_ptr_->points_for_backtrack_[0].transpose() << endl;
// std::cout << "size of backtrack node child: " << planner_manager_->backtrack_node_->child_.size() << endl;
// std::cout << "size of backtrack node child's polys_: " << planner_manager_->backtrack_node_->child_[0]->polys_.size() << std::endl;
// std::cout << "backtrack_node_ replan pos: " << planner_manager_->backtrack_node_->replan_pos_.transpose() << endl;
//             planner_manager_->backtrack_node_ = planner_manager_->backtrack_node_->child_[0];
// std::cout << "backtrack_node_ replan pos after updates: " << planner_manager_->backtrack_node_->replan_pos_.transpose() << endl;
// std::cout << "size of backtrack node's polys_: " << planner_manager_->backtrack_node_->polys_.size() << std::endl;

//             success = planner_manager_->planGlobalPath(planner_manager_->backtrack_node_, start_pos_, planner_manager_->global_graph_ptr_->points_for_backtrack_[0]);

std::cout << "size of backtrack node: " << planner_manager_->global_graph_ptr_->nodes_for_backtrack_.size() << std::endl;
std::cout << "size of backtrack node's center list: " << planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->center_list_.size() << std::endl;
            // still in reverse direction, from child to parent
            GraphNode *tmp_node;
            tmp_node = new GraphNode();
            tmp_node->replan_pos_ = planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->center_list_[0];
            tmp_node->parent_ = planner_manager_->backtrack_node_;
            tmp_node->center_list_.push_back(planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->center_list_[2]);
            tmp_node->center_list_.push_back(planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->center_list_[1]);
            tmp_node->center_list_.push_back(planner_manager_->global_graph_ptr_->points_for_backtrack_[0]);
            tmp_node->leading_ray_ = -planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->leading_ray_;
            tmp_node->polys_.push_back(planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->polys_[1]);
            tmp_node->polys_.push_back(planner_manager_->global_graph_ptr_->nodes_for_backtrack_[0]->polys_[0]);
            planner_manager_->backtrack_node_->child_.push_back(tmp_node);
            planner_manager_->backtrack_node_ = planner_manager_->backtrack_node_->child_[0];
            
std::cout << "points for back track[0]: " << planner_manager_->global_graph_ptr_->points_for_backtrack_[0].transpose() << endl;
std::cout << "back track nodes's center list[2]: " << planner_manager_->backtrack_node_->center_list_[2].transpose() << endl;

            success = planner_manager_->planGlobalPath(planner_manager_->backtrack_node_, start_pos_, planner_manager_->global_graph_ptr_->points_for_backtrack_[0]);


            vec_E<Polyhedron3D> first_two_polys;
            first_two_polys.push_back(planner_manager_->backtrack_node_->polys_[0]);
            first_two_polys.push_back(planner_manager_->backtrack_node_->polys_[1]);
            decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(first_two_polys);
            poly_msg.header.frame_id = "odom";
            current_polys_pub_.publish(poly_msg);
        }
    

        if (success){
            getTrajByALTRO();
std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!size of polys_deadend!!!!!!!!!!!!!!!!!!!!!!!!!!!!: " << planner_manager_->polys_deadend_.size() << endl;
            // planner_manager_->planFromCurrentTraj(planning_distance_, odom_pos_);
            // planner_manager_->local_traj_data_.getRefTraj(local_traj_list_);
            visualizePath(planner_manager_->global_data_.getGlobalPath(), vis_global_plan_pub_); // visualize global path
            // visualizePath(planner_manager_->local_traj_data_.local_path_, vis_local_plan_pub_); // visualize local path
            // visualizeTrajList(local_traj_list_, vis_local_wp_pub_);
            changeFSMExecState(EXEC_LOCAL, "FSM");
        }else{
            ROS_ERROR("[Plan FSM]: Replan fail, wait for goal");
            changeFSMExecState(WAIT_GOAL, "FSM");
        }


        break;
    }

    /* --------------Case5:EXEC_LOCAL ---------------*/
    case EXEC_LOCAL:
    {
        if(achieve_goal()) return;
        // check time and if time greater than threshold, replan
        if(have_goal_)
        { 
            planner_manager_->planFromCurrentTraj(planning_distance_, odom_pos_);
            planner_manager_->local_traj_data_.getRefTraj(local_traj_list_);

            // visualizePath(planner_manager_->local_traj_data_.local_path_, vis_local_plan_pub_); // visualize local path
            // int index = 0;
            // double process_time = planner_manager_->local_traj_data_.nearestPointTime(odom_pos_, index);
            // // if(planner_manager_->global_data_.getLastProcessTime() < process_time){
            // planner_manager_->global_data_.setLastProcessTime(process_time);
// std::cout << "EXEC_LOCAL" << std::endl;
        }
    }
    
    } // End of Switch
}

void DynamicReplanFSMPOP::replanCheckCallback(const ros::TimerEvent &e)
{
    if (exec_state_ == EXEC_LOCAL)
    {
        // first check if there's obstacle in the front
        double dist;
        // bool   safe = planner_manager_->checkTrajCollision(dist, start_collision_index_);
        // check collision
        // if (!safe) {
        //     ROS_WARN("[Plan FSM]: Local trajectory is not safe, replan!");
        //     early_replan_flag_ = true;
        //     changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        // }

        
//         // if in backtracking
        if (planner_manager_->backtrackFlag_){
//             Eigen::Vector3d sub_goal = planner_manager_->backtrack_node_->center_list_[2];
//             if (sub_goal[2] > 0.32)
//             {
//                 sub_goal[2] = 0.3;
//             }
//             double dist_to_sub_goal = (odom_pos_ - sub_goal).norm();
// // std::cout << "dist to sub goal: " << dist_to_sub_goal << endl;

//             if (dist_to_sub_goal < 0.25)
//             {
//                 early_replan_flag_ = false;
//                 ROS_INFO("[Plan FSM]: Robot is close to the backtrack sub_goal, replan!");
//                 changeFSMExecState(REPLAN_TRAJ, "SAFETY");
//             }
        }
        else{
            //firt check if there's direction that worth exploring along current direction
            if(interest_rays_.size() == 0){
                if (!if_no_interesting_rays_1_){
                    if_no_interesting_rays_1_ = true;
                }
                else if (if_no_interesting_rays_1_ && !if_no_interesting_rays_2_){
                    if_no_interesting_rays_2_ = true;
                }
                else if (if_no_interesting_rays_1_ && if_no_interesting_rays_2_){
                    ROS_INFO("[Plan FSM]: No interesting direction, replan!");
                    if_no_interesting_rays_1_ = false;
                    if_no_interesting_rays_2_ = false;
                    // planner_manager_->backtrackFlag_ = true;
                    changeFSMExecState(REPLAN_TRAJ, "REPLAN");
                }
            }
            else{
                if_no_interesting_rays_1_ = false;
                if_no_interesting_rays_2_ = false;
            }
            // if interesting rays is not empty, check the angle between leading ray of the current node with the interesting rays, if all the angle is larger than 45 degree, replan
            bool all_angle_larger_than_theta = true;
            double theta = 100.0 / 180.0 * M_PI;
            for (int i = 0; i < interest_rays_.size()/2; i++){
                Eigen::Vector3d ray = interest_rays_[2*i] - interest_rays_[2*i+1];
                double angle = acos(planner_manager_->current_node_->leading_ray_.normalized().dot(ray.normalized()));
                if (angle < theta){
                    all_angle_larger_than_theta = false;
                    break;
                }
            }
            if (all_angle_larger_than_theta){
                if (!if_no_interesting_rays_ahead_1_){
                    if_no_interesting_rays_ahead_1_ = true;
                }
                else if (if_no_interesting_rays_ahead_1_ && !if_no_interesting_rays_ahead_2_){
                    if_no_interesting_rays_ahead_2_ = true;
                }
                else if (if_no_interesting_rays_ahead_1_ && if_no_interesting_rays_ahead_2_ && !if_no_interesting_rays_ahead_3_){
                    if_no_interesting_rays_ahead_3_ = true;
                }
                else if (if_no_interesting_rays_ahead_1_ && if_no_interesting_rays_ahead_2_ && if_no_interesting_rays_ahead_3_ && !if_no_interesting_rays_ahead_4_){
                    if_no_interesting_rays_ahead_4_ = true;
                }
                else if (if_no_interesting_rays_ahead_1_ && if_no_interesting_rays_ahead_2_ && if_no_interesting_rays_ahead_3_ && if_no_interesting_rays_ahead_4_){
                    ROS_INFO("[Plan FSM]: No interesting direction ahead, replan!");
                    if_no_interesting_rays_ahead_1_ = false;
                    if_no_interesting_rays_ahead_2_ = false;
                    if_no_interesting_rays_ahead_3_ = false;
                    if_no_interesting_rays_ahead_4_ = false;
                    // Polyhedron3D deadend_poly;
                    // planner_manager_->getDeadEndPoly(deadend_poly, planner_manager_->current_node_->replan_pos_);
                    // planner_manager_->polys_deadend_.push_back(deadend_poly);
                    // planner_manager_->backtrackFlag_ = true;
                    early_replan_flag_ = false;
                    
                    changeFSMExecState(REPLAN_TRAJ, "REPLAN");
                }
            }
            else{
                if_no_interesting_rays_ahead_1_ = false;
                if_no_interesting_rays_ahead_2_ = false;
                if_no_interesting_rays_ahead_3_ = false;
                if_no_interesting_rays_ahead_4_ = false;
            }

            // second check if the robot is close to the sub_goal, which is the center of the second polyhedron
            // also if the robot is close to the goal, do not replan
// std::cout << "replan check" << endl;
// std::cout << "current node center list size: " << planner_manager_->current_node_->center_list_.size() << endl;
            Eigen::Vector3d sub_goal = planner_manager_->current_node_->center_list_[2];
            double sub_goal_yaw_ = atan2(planner_manager_->current_node_->center_list_[2](1) - planner_manager_->current_node_->center_list_[1](1), planner_manager_->current_node_->center_list_[2](0) - planner_manager_->current_node_->center_list_[1](0));
            if (sub_goal[2] > 0.32)
            {
                sub_goal[2] = 0.3;
            }
            // distacne only consider x and y
            double dist_to_sub_goal = (odom_pos_.head(2) - sub_goal.head(2)).norm();
            // double dist_to_sub_goal = (odom_pos_ - sub_goal).norm();
            double dist_to_sub_goal_yaw = abs(odom_yaw_ - sub_goal_yaw_);
            dist_to_sub_goal_yaw = 0;
// std::cout << "replan dist to sub goal: " << dist_to_sub_goal << endl;
// std::cout << "replan dist to sub goal yaw: " << dist_to_sub_goal_yaw << endl;
            if (dist_to_sub_goal < 0.15 && !planner_manager_->goalInFirstTwoPoly_ && dist_to_sub_goal_yaw < 0.15)
            {
                early_replan_flag_ = false;
                ROS_INFO("[Plan FSM]: Robot is close to the sub_goal, replan!");
                changeFSMExecState(REPLAN_TRAJ, "REPLAN");
            }
        }

    }
}

void DynamicReplanFSMPOP::interestRaysCallback(const visualization_msgs::MarkerPtr &msg){
    interest_rays_.clear();
    if (msg->points.size() == 0)
        return;
    for (unsigned int i = 0; i < msg->points.size(); i++){
        Eigen::Vector3d ray;
        ray << msg->points[i].x, msg->points[i].y, msg->points[i].z;
        interest_rays_.push_back(ray);
    }
}

void DynamicReplanFSMPOP::trackTrajCallback(const ros::TimerEvent &e)
{
    if (exec_state_ == WAIT_GOAL || exec_state_ == INIT || exec_state_ == GEN_NEW_GLOBAL)
        return;

    if (planner_manager_->local_traj_data_.isEmpty())
    {
        ROS_WARN("[Plan FSM]: Local trajectory is empty, wait for replan!");
        return;
    }
    // get waypoint from local traj to track via publisher local_wp_pub_
    planner_manager_->local_traj_data_.getRefTraj(local_traj_list_);
    // if near global path start point, track_fut_pts_num_ = 0
    int track_index = track_fut_pts_num_;
    Eigen::Vector3d start_pos = planner_manager_->global_data_.getStartPos();
    if ((odom_pos_- start_pos).norm() < 0.1){
        track_index = 3;
    }

    // track waypoint 
    if (local_traj_list_.size() > 0)
    {
        geometry_msgs::PoseStamped local_wp;
        local_wp.header.frame_id = "map";
        local_wp.header.stamp = ros::Time::now();
        local_wp.pose.position.x = local_traj_list_[track_index](0);
        local_wp.pose.position.y = local_traj_list_[track_index](1);
        local_wp.pose.position.z = 0.0;
        // orientation from yaw angle
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(local_traj_list_[track_index](2), Eigen::Vector3d::UnitZ());
        local_wp.pose.orientation.w = q.w();
        local_wp.pose.orientation.x = q.x();
        local_wp.pose.orientation.y = q.y();
        local_wp.pose.orientation.z = q.z();
        local_wp_pub_.publish(local_wp);
    }
}

void DynamicReplanFSMPOP::publishCmdCallback(const ros::TimerEvent &e){
    if (exec_state_ == WAIT_GOAL || exec_state_ == INIT || exec_state_ == GEN_NEW_GLOBAL)
        return;

    // find the nearest point in traj_from_altro_
    int index = 0;
    double min_dist = 1000;
    for (int i = 1; i < traj_from_altro_.size(); i++){
// std::cout << "traj from altro " << i << ": " << traj_from_altro_[i].transpose() << endl;

        double dist = (odom_pos_ - traj_from_altro_[i]).norm();
        if (dist < min_dist){
            min_dist = dist;
            index = i;
        }
    }
    double dt = plan_time_ / 10;
    Eigen::Vector3d current_desired_vel;
    if (index + 3 > traj_from_altro_.size()){
        current_desired_vel = (traj_from_altro_[traj_from_altro_.size()] -  traj_from_altro_[traj_from_altro_.size() - 1]) / dt;
    }
    else{
        current_desired_vel = (traj_from_altro_[index+1] -  traj_from_altro_[index]) / dt;
    }
    // double current_desired_yaw_vel = (yaw_traj_from_altro_[index] - yaw_traj_from_altro_[index-1]) / dt;

// std::cout << "index: " << index << endl;
    geometry_msgs::Twist twist_cmd;
    if (index + 3 > traj_from_altro_.size()){
        twist_cmd.linear.x = (traj_from_altro_[traj_from_altro_.size() - 1](0) - odom_pos_(0)) * 1.5 + ( - odom_vel_(0)) * 0.1;
        twist_cmd.linear.y = (traj_from_altro_[traj_from_altro_.size() - 1](1) - odom_pos_(1)) * 1.2 + ( - odom_vel_(1)) * 0.1;
        twist_cmd.linear.z = (traj_from_altro_[traj_from_altro_.size() - 1](2) - odom_pos_(2)) * 0.2;
        twist_cmd.angular.z = (yaw_traj_from_altro_[traj_from_altro_.size() - 1] - odom_yaw_) * 1.2;
// std::cout << "pos error: " << (traj_from_altro_[traj_from_altro_.size() - 1] - odom_pos_).transpose() << endl;
// std::cout << "pos error norm: " << (traj_from_altro_[traj_from_altro_.size() - 1] - odom_pos_).norm() << endl;
// std::cout << "yaw error: " << (yaw_traj_from_altro_[traj_from_altro_.size() - 1] - odom_yaw_) << endl;
// std::cout << "odom_pos: " << odom_pos_.transpose() << endl;
// std::cout << "target pos: " << traj_from_altro_[traj_from_altro_.size() - 1].transpose() << endl;
// std::cout << "odom_yaw: " << odom_yaw_ << " target yaw: " << yaw_traj_from_altro_[traj_from_altro_.size() - 1] << endl;
    }
    else{       
        // twist_cmd.linear.x = (traj_from_altro_[index+1](0) - odom_pos_(0)) * 1.5 + (current_desired_vel(0) - odom_vel_(0)) * 0.5;
        // twist_cmd.linear.y = (traj_from_altro_[index+1](1) - odom_pos_(1)) * 0.8 + (current_desired_vel(1) - odom_vel_(1)) * 0.2;
        // twist_cmd.linear.z = (traj_from_altro_[index+1](2) - odom_pos_(2)) * 0.2;
        // twist_cmd.angular.z = (yaw_traj_from_altro_[index+1] - odom_yaw_) * 0.8;
        twist_cmd.linear.x = (traj_from_altro_[index+1](0) - odom_pos_(0)) * 1.5 + ( - odom_vel_(0)) * 0.1;
        twist_cmd.linear.y = (traj_from_altro_[index+1](1) - odom_pos_(1)) * 1.2 + ( - odom_vel_(1)) * 0.1;
        twist_cmd.linear.z = (traj_from_altro_[index+1](2) - odom_pos_(2)) * 0.2;
        twist_cmd.angular.z = (yaw_traj_from_altro_[index+1] - odom_yaw_) * 1.2;
// std::cout << "pos error: " << (traj_from_altro_[index+1] - odom_pos_).transpose()    << endl;
// std::cout << "pos error norm: " << (traj_from_altro_[index+1] - odom_pos_).norm() << endl;
// std::cout << "yaw error: " << (yaw_traj_from_altro_[index+1] - odom_yaw_) << endl;
// std::cout << "odom_pos: " << odom_pos_.transpose() << endl;
// std::cout << "target pos: " << traj_from_altro_[index+1].transpose() << endl;
// std::cout << "odom_yaw: " << odom_yaw_ << " target yaw: " << yaw_traj_from_altro_[index+1] << endl;
    }
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    // the linear velocity is now in world frame, need to transform to robot frame
    Eigen::Vector3d linear_vel(twist_cmd.linear.x, twist_cmd.linear.y, twist_cmd.linear.z);
    Eigen::Vector3d linear_vel_robot = Eigen::AngleAxisd(-odom_yaw_, Eigen::Vector3d::UnitZ()) * linear_vel;

    twist_cmd.linear.x = linear_vel_robot(0);
    twist_cmd.linear.y = linear_vel_robot(1);
    twist_cmd.linear.z = linear_vel_robot(2);

    // linear velocity has a limit in three directions
    if (twist_cmd.linear.x > 0.15)
        twist_cmd.linear.x = 0.15;
    if (twist_cmd.linear.x < -0.15)
        twist_cmd.linear.x = -0.15;
    if (twist_cmd.linear.y > 0.15)
        twist_cmd.linear.y = 0.15;
    if (twist_cmd.linear.y < -0.15)
        twist_cmd.linear.y = -0.15;
    if (twist_cmd.linear.z > 0.1)
        twist_cmd.linear.z = 0.1;
    if (twist_cmd.linear.z < -0.1)
        twist_cmd.linear.z = -0.1; 
    if(twist_cmd.angular.z > 0.5)
        twist_cmd.angular.z = 0.5;
    if(twist_cmd.angular.z < -0.5)
        twist_cmd.angular.z = -0.5;

std::cout << "twist cmd: " << twist_cmd.linear.x << " " << twist_cmd.linear.y << " " << twist_cmd.angular.z << endl;
    whole_trajectory.push_back(odom_pos_);
    visualizePath(whole_trajectory, whole_traj_pub_);
    cmd_vel_pub_.publish(twist_cmd);

}

void DynamicReplanFSMPOP::getTrajByALTRO(){
    // first get a ref path from center of first polyhedron to the center of the intersection polyhedron and to the second polyhedron
std::cout << "in getTrajByALTRO" << endl;
    GraphNode* node;
    Eigen::Vector3d end_pos_for_altro;
    if (planner_manager_->backtrackFlag_){
        node = planner_manager_->backtrack_node_;
        end_pos_for_altro = planner_manager_->global_graph_ptr_->points_for_backtrack_[0];
    }
    else{
        node = planner_manager_->current_node_;
        end_pos_for_altro = end_pos_;
    }
    std::vector<Eigen::Vector3d> center_list_;
    double dist1, dist2, goal_yaw;
    center_list_.push_back(odom_pos_);

    // if (planner_manager_->backtrackFlag_){
    //     center_list_.push_back(node->center_list_[1]); 
    //     dist1 = (odom_pos_ - node->center_list_[1]).norm();
    //     center_list_.push_back(end_pos_for_altro);
    //     dist2 = (node->center_list_[1] - end_pos_for_altro).norm();
    //     goal_yaw = atan2((end_pos_for_altro(1) - node->center_list_[1](1)), (end_pos_for_altro(0) - node->center_list_[1](0)));
    // }
    // else{
        if (node->polys_[0].inside(end_pos_for_altro) && node->polys_[1].inside(end_pos_for_altro)){
std::cout << "altro in both poly" << endl;
            center_list_.push_back(end_pos_for_altro);
            dist1 = (odom_pos_ - end_pos_for_altro).norm();
            dist2 = 0;
            goal_yaw = goal_yaw_;
        }
        else if (node->polys_[0].inside(odom_pos_) && node->polys_[1].inside(odom_pos_)){
std::cout << "odom in both poly" << endl;
            center_list_.push_back(node->center_list_[2]);
            dist1 = 0;
            dist2 = (odom_pos_ - node->center_list_[2]).norm();
            goal_yaw = atan2((node->center_list_[2](1) - odom_pos_(1)), (node->center_list_[2](0) - odom_pos_(0)));
        }
        else if (node->polys_[0].inside(end_pos_for_altro) && !node->polys_[1].inside(end_pos_for_altro)){
std::cout << "altro in first poly" << endl;
            center_list_.push_back(end_pos_for_altro);
            dist1 = (odom_pos_ - end_pos_for_altro).norm();
            dist2 = 0;
            goal_yaw = goal_yaw_;
        }
        else if (node->polys_[1].inside(end_pos_for_altro) && ((node->center_list_[2] - end_pos_for_altro).norm() < 0.5)){
std::cout << "altro in second poly" << endl;
            center_list_.push_back(node->center_list_[1]);
            center_list_.push_back(end_pos_for_altro);
            dist1 = (odom_pos_ - node->center_list_[1]).norm();
            dist2 = (node->center_list_[1] - end_pos_for_altro).norm();
            goal_yaw = goal_yaw_;
        }
        else if(node->polys_[0].inside(node->center_list_[2]) && ((node->center_list_[1] - node->center_list_[0]).normalized().dot((node->center_list_[2] - node->center_list_[1])) < 0)){
std::cout << "center of second poly in first poly and center of intersection is ahead of second polyhedron" << endl;
            // directly go to the center of the second polyhedron
            center_list_.push_back(node->center_list_[2]);
            dist1 = (odom_pos_ - node->center_list_[2]).norm();
            dist2 = 0;
            goal_yaw = atan2((node->center_list_[2](1) - odom_pos_(1)), (node->center_list_[2](0) - odom_pos_(0)));
        }
        else{
std::cout << "altro not in first two poly" << endl;
            center_list_.push_back(node->center_list_[1]);
            center_list_.push_back(node->center_list_[2]);
            dist1 = (center_list_[0] - center_list_[1]).norm();
            dist2 = (center_list_[1] - center_list_[2]).norm();
            // base on the direction between the first and last point in center list ,get the yaw angle
            goal_yaw = atan2((center_list_[2](1) - center_list_[0](1)), (center_list_[2](0) - center_list_[0](0)));
        }
std::cout << "dist1: " << dist1 << " dist2: " << dist2 << endl;
    // }


    for (int i = 0; i < center_list_.size(); i++){
        if(center_list_[i](2) > 0.32){
            center_list_[i](2) = 0.3;
        }
    }

    std::vector<double> segment_dist_list;
    segment_dist_list.resize(2);
    segment_dist_list[0] = 0.15;
    segment_dist_list[1] = 0.15;
    // make sure the number of segment is greater or equal to 1
    int num_segments1, num_segments2;
    std::vector<Eigen::Vector3d> dir_list;
    if (dist2 == 0){
        num_segments1 = 10;
        num_segments2 = 0;
        dir_list.push_back((center_list_[1] - center_list_[0]).normalized());
        segment_dist_list[0] = dist1 / 10;
    }
    else if (dist1 == 0){
        num_segments1 = 0;
        num_segments2 = 10;
        dir_list.push_back((center_list_[1] - center_list_[0]).normalized());
        segment_dist_list[1] = dist2 / 10;
    }
    else{
        num_segments1 = std::max(int(dist1 / segment_dist_list[0]), 1);
        num_segments2 = std::max(int(dist2 / segment_dist_list[1]), 1);
        if (num_segments1 + num_segments2 > 10 || num_segments1 + num_segments2 < 10){
            // reset the segment_dist to make sure the number of segments is 10
            // allocate the num of waypoints to two segments based on the length of the two segments
            num_segments1 = std::max(int (10 * dist1 / (dist1 + dist2)), 1);
            num_segments2 = 10 - num_segments1;
            segment_dist_list[0] = dist1 / num_segments1;
            segment_dist_list[1] = dist2 / num_segments2;
        }
        dir_list.push_back((center_list_[1] - center_list_[0]).normalized());
        dir_list.push_back((center_list_[2] - center_list_[1]).normalized());
    }
std::cout << "size of num 1: " << num_segments1 << " size of num 2: " << num_segments2 << endl;
std::cout << "size of dir list: " << dir_list.size() << endl;
std::cout << "size of segment dist list: " << segment_dist_list.size() << endl;
std::cout << "size of center list: " << center_list_.size() << endl;
    // interpolate the waypoints
    std::vector<Eigen::Vector3d> ref_path;
    std::vector<double> ref_yaw;
    // base on the direction of the first line segment, get the yaw angle
    double yaw_line1 = atan2(dir_list[0](1), dir_list[0](0));
    // yaw line1 means the heading direction of the robot, if the angle between the heading direction and the direction of the first line segment is greater than 90 degree, then the yaw_line1 should be the opposite direction
    if (fabs(yaw_line1 - odom_yaw_) > M_PI / 2){
        if (yaw_line1 > 0){
            yaw_line1 = yaw_line1 - M_PI;
        }
        else{
            yaw_line1 = yaw_line1 + M_PI;
        }
    }

    double yaw_segment1 = (yaw_line1 - odom_yaw_) / num_segments1;
    for (int i = 0; i < num_segments1; i++){
        ref_path.push_back(center_list_[0] + dir_list[0] * i * segment_dist_list[0]);
        ref_yaw.push_back(odom_yaw_ + yaw_segment1 * i);
    }
    if (num_segments2 == 0){
        ref_path.push_back(center_list_[1]);
        ref_yaw.push_back(yaw_line1);
    }
    else if(num_segments1 == 0){
        double yaw_line2 = atan2(dir_list[0](1), dir_list[0](0));
        if (fabs(yaw_line2 - yaw_line1) > M_PI / 2){
            if (yaw_line2 > 0){
                yaw_line2 = yaw_line2 - M_PI;
            }
            else{
                yaw_line2 = yaw_line2 + M_PI;
            }
        }
        double yaw_segment2 = (yaw_line2 - yaw_line1) / num_segments2;
        for (int i = 0; i < num_segments2; i++){
            ref_path.push_back(center_list_[0] + dir_list[0] * i * segment_dist_list[1]);
            ref_yaw.push_back(yaw_line2 + yaw_segment2 * i);
        }
        ref_path.push_back(center_list_[1]);
        ref_yaw.push_back(yaw_line2);
    }
    else{
        // double yaw_line2 = atan2(dir_list[1](1), dir_list[1](0));
        double yaw_line2 = goal_yaw;
        if (fabs(yaw_line2 - yaw_line1) > M_PI / 2){
            if (yaw_line2 > 0){
                yaw_line2 = yaw_line2 - M_PI;
            }
            else{
                yaw_line2 = yaw_line2 + M_PI;
            }
        }
        double yaw_segment2 = (yaw_line2 - yaw_line1) / num_segments2;
        for (int i = 0; i < num_segments2; i++){
            ref_path.push_back(center_list_[1] + dir_list[1] * i * segment_dist_list[1]);
            ref_yaw.push_back(yaw_line1 + yaw_segment2 * i);
        }
        ref_path.push_back(center_list_[2]);
        ref_yaw.push_back(yaw_line2);
    }

for (int i = 0; i < center_list_.size(); i++){
    std::cout << "center list " << i << ": " << center_list_[i].transpose() << endl;
}
for (int i = 0; i < ref_path.size(); i++){
    std::cout << "ref path " << i << ": " << ref_path[i].transpose() << endl;
}
for (int i = 0; i < ref_yaw.size(); i++){
    std::cout << "ref yaw " << i << ": " << ref_yaw[i] << endl;
}
    plan_time_ = (dist1 + dist2) / 0.15; // 0.2 represents the speed of the robot

 //////////////////////////////////////////////////ALTRO SOLVER////////////////////////////////////////////////////

        ROS_WARN("Start to solve the ALTRO problem");
        Eigen::VectorXd currState_; 
        currState_ = Eigen::VectorXd::Zero(6);
        // currState_ << odom_pos_(0), odom_pos_(1), odom_pos_(2), odom_roll_, odom_pitch_, odom_yaw_;
        currState_ << center_list_[0](0), center_list_[0](1), 0.3, 0, 0, odom_yaw_;
// ROS_WARN("Start to solve the ALTRO problem11");        
        /////////////build the problem////////////////////////////////////////////////////
        QuadrupedalProblem_ptr_ = std::make_unique<altro::problems::QuadrupedalProblem>();
        if (num_segments2 == 0 || num_segments1 == 0){
            QuadrupedalProblem_ptr_->xf << center_list_[1](0), center_list_[1](1), 0.3, 0, 0, ref_yaw[10];
        }
        else{
            QuadrupedalProblem_ptr_->xf << center_list_[2](0), center_list_[2](1), 0.3, 0, 0, ref_yaw[10];

        }
        QuadrupedalProblem_ptr_->tf = plan_time_;
        QuadrupedalProblem_ptr_->x0 = currState_;
        QuadrupedalProblem_ptr_->xref.resize(10);
// ROS_WARN("Start to solve the ALTRO problem22");
        for (int i = 0; i < 10; ++i)
        {
            QuadrupedalProblem_ptr_->xref[i] = Eigen::VectorXd::Zero(6);
            QuadrupedalProblem_ptr_->xref[i] << ref_path[i](0), ref_path[i](1), 0.3, 0, 0, ref_yaw[i];
        }

// ROS_WARN("Start to solve the ALTRO problem1");
        /////////////////////////////////////////////////////////////////////
        probPtr_.reset(new altro::problem::Problem(QuadrupedalProblem_ptr_->MakeProblem(true)));
        traj_ptr_ = std::make_shared<altro::Trajectory<NStates, NControls>>(QuadrupedalProblem_ptr_->InitialTrajectory());
        solver_al_ptr_ = std::make_unique<altro::augmented_lagrangian::AugmentedLagrangianiLQR<6, 6>>(*probPtr_);

// ROS_WARN("Start to solve the ALTRO problem2");
        solver_al_ptr_->SetPenalty(10);
        solver_al_ptr_->GetOptions().verbose = altro::LogLevel::kSilent;
        solver_al_ptr_->GetOptions().verbose = altro::LogLevel::kDebug;
        solver_al_ptr_->GetOptions().max_iterations_total = 30;
        solver_al_ptr_->GetOptions().max_iterations_outer = 5;
        solver_al_ptr_->GetOptions().max_iterations_inner = 6;
        solver_al_ptr_->GetOptions().line_search_max_iterations = 6;
        solver_al_ptr_->GetOptions().nthreads = 12;
        solver_al_ptr_->GetOptions().initial_penalty = 10;
        // solver_al_ptr_->GetOptions().profiler_enable = true;
        // solver_al_ptr_->GetOptions().profiler_output_to_file = true;
        // solver_al_ptr_->GetOptions().log_directory = "~";
        solver_al_ptr_->GetOptions().line_search_lower_bound = 1e-4;
        solver_al_ptr_->GetOptions().line_search_decrease_factor = 3;
        solver_al_ptr_->GetOptions().cost_tolerance = 8e-4;
        solver_al_ptr_->GetOptions().constraint_tolerance = 1e-3;
        // solver_al_ptr_->GetOptions().l
        // solver_al_ptr_->GetOptions().l
        


        //////////////////////////////////////////////////////////////////////////////////////

        for (int i = 0; i < 11; ++i)
        {
            // traj_ptr_->State(i) = currState_;
            traj_ptr_->State(i) = QuadrupedalProblem_ptr_->xref[0];
        }
    

        // probPtr_->SetInitialState(BaseM_TR.transpose());   // TODO: magic words
        solver_al_ptr_->SetTrajectory(traj_ptr_);
    
// ROS_WARN("Start to solve the ALTRO problem3");
        //////////////////////////////////////////////////IMPLICIT CONSTRAINTS////////////////////////////////////////////////////
        int numk = solver_al_ptr_->NumSegments();
        for (int i = 0; i < numk+1; ++i)
        {   
            int ineq_cnt = solver_al_ptr_->GetALCost(i)->GetInequalityConstraints().size();
            //find the label of the constraint, if it is "SDP Constraint", then update the A, b, F, g, c
            for (int j = 0; j < ineq_cnt; ++j)
            {
                auto sdp_ineqconstraint = solver_al_ptr_->GetALCost(i)->GetInequalityConstraints()[j]->GetConstraint();
                if (sdp_ineqconstraint->GetLabel()== "SDP Constraint")
                {
                    auto sdp_constraint = std::dynamic_pointer_cast<altro::constraints::SDPConstraint>(solver_al_ptr_->GetALCost(i)->GetInequalityConstraints()[j]->GetConstraint());

                    if (i<num_segments1)
                    {
                        sdp_constraint->F_ = node->polys_[0].lc_.A_;
                        sdp_constraint->g_ = node->polys_[0].lc_.b_;
                        sdp_constraint->c_ = node->polys_[0].vertices_.rowwise().mean();
// ROS_INFO("IMP CONSTRAINT: %d", i);
                    }
                    else
                    {
                        sdp_constraint->F_ = node->polys_[1].lc_.A_;
                        sdp_constraint->g_ = node->polys_[1].lc_.b_;
                        sdp_constraint->c_ = node->polys_[1].vertices_.rowwise().mean();
// ROS_INFO("IMP CONSTRAINT: %d", i);
                    }
                }
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// RcOS_WARN("Start to solve the ALTRO problem4");
// cnt time here
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        solver_al_ptr_->Solve();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // print the time
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
// ROS_WARN("Start to solve the ALTRO problem5");
        Eigen::VectorXd uoutput = Eigen::VectorXd::Zero(10);
        std::vector<Eigen::Vector3d> ref_angle;
        
        for (int k = 0; k < 10; ++k) {
             uoutput[k] = traj_ptr_->Control(k)[0];
        // std::cout<<"traj_ptr_: "<<traj_ptr_->State(k).transpose()<<std::endl;
        }
        uoutput = traj_ptr_->Control(0);
        ref_path.clear();
        yaw_traj_from_altro_.clear();
for (int k = 0; k < 11; ++k) {
    // std::cout<<"traj_ptr_: "<<traj_ptr_->State(k).transpose()<<std::endl;
    // std::cout<<"trajectory control: "<<traj_ptr_->Control(k).transpose()<<std::endl;
    ref_path.push_back(traj_ptr_->State(k).head(3));
    yaw_traj_from_altro_.push_back(traj_ptr_->State(k)(5));
    ref_angle.push_back(traj_ptr_->State(k).tail(3));
}
for (int i = 0; i < ref_path.size(); i++){
    std::cout << "ref path " << i << ": " << ref_path[i].transpose() << endl;
}
for (int i = 0; i < yaw_traj_from_altro_.size(); i++){
    std::cout << "yaw traj from altro " << i << ": " << yaw_traj_from_altro_[i] << endl;
}

        //////////////////////////////////////////END OF ALTRO SOLVER/////////////////////////////////////////////////////

    visualizePath(ref_path, vis_local_plan_pub_);
    traj_from_altro_ = ref_path;

    // put the ref path and ref angle into pose array and publish
    geometry_msgs::PoseArray path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "odom";
    for (int i = 0; i < ref_path.size(); i++){
        geometry_msgs::Pose traj_pose;
        traj_pose.position.x = ref_path[i](0);
        traj_pose.position.y = ref_path[i](1);
        traj_pose.position.z = ref_path[i](2);
// std::cout << "pose.pose.position: " << traj_pose.position.x << " " << traj_pose.position.y << " " << traj_pose.position.z << endl;
        Eigen::Quaterniond q;
        // get the quaternion from the euler angle
        // q = Eigen::AngleAxisd(ref_angle[i](2), Eigen::Vector3d::UnitZ());
        q = Eigen::AngleAxisd(ref_angle[i](2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(ref_angle[i](1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(ref_angle[i](0), Eigen::Vector3d::UnitX());
        traj_pose.orientation.w = q.w();
        traj_pose.orientation.x = q.x();
        traj_pose.orientation.y = q.y();
        traj_pose.orientation.z = q.z();
        path_msg.poses.push_back(traj_pose);
    }
// for (int i = 0; i < path_msg.poses.size(); i++){
//     std::cout << "pose " << i << ": " << path_msg.poses[i].position.x << " " << path_msg.poses[i].position.y << " " << path_msg.poses[i].position.z << endl;
// }
    traj_pose_pub_.publish(path_msg);
}

/* --------------------achieve gaol-------------------------------- */
bool DynamicReplanFSMPOP::achieve_goal(){
    // distance to goal
    if (planner_manager_->backtrackFlag_){
        double dist_to_goal, ang_to_goal;
        Eigen::Vector3d sub_goal = planner_manager_->global_graph_ptr_->points_for_backtrack_[0];
        if(sub_goal[2] > 0.32){
            sub_goal[2] = 0.3;
        }
        dist_to_goal = (odom_pos_ - sub_goal).norm();
// std::cout << "dist to goal: " << dist_to_goal << endl;
        if (dist_to_goal < goal_tolerance_ * 3)
        {
std::cout << "achieve subgoal" << endl;
            planner_manager_->local_traj_data_.reset();
            // delete the first element in the backtrack points
std::cout << "size of points for backtrack before erase: " << planner_manager_->global_graph_ptr_->points_for_backtrack_.size() << endl;
std::cout << "size of nodes for backtrack before erase: " << planner_manager_->global_graph_ptr_->nodes_for_backtrack_.size() << endl;
            planner_manager_->global_graph_ptr_->points_for_backtrack_.erase(planner_manager_->global_graph_ptr_->points_for_backtrack_.begin());
            planner_manager_->global_graph_ptr_->nodes_for_backtrack_.erase(planner_manager_->global_graph_ptr_->nodes_for_backtrack_.begin());
std::cout << "size of nodes for backtrack after erase: " << planner_manager_->global_graph_ptr_->nodes_for_backtrack_.size() << endl;
            if(planner_manager_->global_graph_ptr_->points_for_backtrack_.size() == 0 || planner_manager_->global_graph_ptr_->nodes_for_backtrack_.size() == 0){
                // delete all the node in the whole backtrack process
std::cout << "all backtrack done" << endl;
                for (int i = 0; i < planner_manager_->backtrack_node_->child_.size(); i++){
                    delete planner_manager_->backtrack_node_->child_[i];
                }
std::cout << "current node's child already deleted" << endl;
                while(!planner_manager_->backtrack_node_->root){
std::cout << "whether current node is root: " << planner_manager_->backtrack_node_->root << endl;
                    GraphNode* temp = planner_manager_->backtrack_node_;
                    planner_manager_->backtrack_node_ = planner_manager_->backtrack_node_->parent_;
                    delete temp;
                }
                planner_manager_->backtrackFlag_ = false;
                planner_manager_->firstreplanafterbacktrack_ = true;
                ROS_INFO("[Plan FSM]: Backtrack Done");
            }
            // delete all the node in the whole backtrack process

            stopRobot();
            early_replan_flag_ = false;
            ROS_INFO("[Plan FSM]: Reach backtrack Subgoal");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            return true;
        }
    }
    else{
        double dist_to_goal, ang_to_goal;
        dist_to_goal = (odom_pos_ - end_pos_).norm();
        // ang_to_goal = std::abs(odom_yaw_ - goal_yaw_);
std::cout << "dist to goal: " << dist_to_goal << endl;
        // reached goal
        if (dist_to_goal < goal_tolerance_ * 3)
        {
            planner_manager_->local_traj_data_.reset();
            have_goal_ = false;
            stopRobot();
            ROS_INFO("[Plan FSM]: Reach Goal, wait goal");
            changeFSMExecState(WAIT_GOAL, "FSM");
            return true;
        }
    }

    return false;
}


/* --------------------helper functions---------------------------- */
void DynamicReplanFSMPOP::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call)
{
    string state_str[5] = {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_TRAJ", "EXEC_LOCAL"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    ROS_INFO("\033[37m[%s]: State: %s -> %s\033[0m", pos_call.c_str(), state_str[pre_s].c_str(), state_str[int(new_state)].c_str());
}

void DynamicReplanFSMPOP::printFSMExecState()
{
    string state_str[5] = {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_TRAJ", "EXEC_LOCAL"};
    ROS_INFO("\033[36m[Plan FSM]: State: %s\033[0m", state_str[int(exec_state_)].c_str());
}

/* --------------------visualization---------------------------- */

void DynamicReplanFSMPOP::visualizePath(const std::vector<Eigen::Vector3d> &path, const ros::Publisher &pub)
{

    // create a path message
    ros::Time plan_time = {}; // ros::Time::now();
    std::string global_frame = "/map";

    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = global_frame;
    gui_path.header.stamp = plan_time;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = path[i](0); // world_x;
        pose.pose.position.y = path[i](1); // world_y;
        pose.pose.position.z = path[i](2);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        gui_path.poses[i] = pose; // plan.push_back(pose);
    }
    pub.publish(gui_path);
}

void DynamicReplanFSMPOP::visualizeTrajList(const vector<Eigen::Vector3d> &list, ros::Publisher &pub)
{
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = ros::Time::now();
    // ROS_INFO("[Plan FSM]: VisualizeTrajList, size: %ld", list.size());
    for (int i = 0; i < list.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = list[i](0);
        pose.position.y = list[i](1);
        pose.position.z = 0.0;
        pose.orientation = tf::createQuaternionMsgFromYaw(list[i](2));
        pose_array.poses.push_back(pose);
    }
    pub.publish(pose_array);
}

void DynamicReplanFSMPOP::publishGlobalGraph(const ros::TimerEvent &e)
{
    visualization_msgs::Marker ray_list;
    ray_list.header.frame_id = "odom";
    ray_list.header.stamp = ros::Time::now();
    ray_list.ns = "global_graph";
    ray_list.action = visualization_msgs::Marker::ADD;
    ray_list.pose.orientation.w = 1.0;
    ray_list.id = 0;
    ray_list.type = visualization_msgs::Marker::LINE_LIST;
    ray_list.scale.x = 0.02;
    ray_list.color.b = 1.0;
    ray_list.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    for (int i = 0; i < replan_points_.size(); i += 2)
    {
        p1.x = replan_points_[i](0);
        p1.y = replan_points_[i](1);
        p1.z = replan_points_[i](2);
        p2.x = replan_points_[i + 1](0);
        p2.y = replan_points_[i + 1](1);
        p2.z = replan_points_[i + 1](2);
        ray_list.points.push_back(p1);
        ray_list.points.push_back(p2);
    }

    global_graph_pub_.publish(ray_list);


    // pub interesting point 
    visualization_msgs::Marker ray_list2;
    ray_list2.header.frame_id = "odom";
    ray_list2.header.stamp = ros::Time::now();
    ray_list2.ns = "interesting_points";
    ray_list2.action = visualization_msgs::Marker::ADD;
    ray_list2.pose.orientation.w = 1.0;
    ray_list2.id = 0;
    ray_list2.type = visualization_msgs::Marker::LINE_LIST;
    ray_list2.scale.x = 0.02;
    ray_list2.color.r = 1.0;
    ray_list2.color.a = 1.0;

    for (int i = 0; i < planner_manager_->points_for_decompose_.size(); i += 2)
    {
        p1.x = planner_manager_->points_for_decompose_[i](0);
        p1.y = planner_manager_->points_for_decompose_[i](1);
        p1.z = planner_manager_->points_for_decompose_[i](2);
        p2.x = planner_manager_->points_for_decompose_[i + 1](0);
        p2.y = planner_manager_->points_for_decompose_[i + 1](1);
        p2.z = planner_manager_->points_for_decompose_[i + 1](2);
        ray_list2.points.push_back(p1);
        ray_list2.points.push_back(p2);
    }
    interesting_point_at_decomp_pub_.publish(ray_list2);
}

int main(int argc, char **argv)
{
    ROS_INFO("[FSM]: Plan manager node start");
    ros::init(argc, argv, "plan_manager");

    // ros::NodeHandle node_handle("~"); every topic will be with namespace
    // ros::NodeHandle nh("");
    ros::NodeHandle nh("~");
    cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    DynamicReplanFSMPOP dynamic_replan;
    dynamic_replan.init(nh);

    std::string ns = ros::this_node::getNamespace();
    ROS_INFO("[FSM node]: Plan manager loaded namespace %s", ns.c_str());

    ros::spin();
}
