#include <daimon_planner_manage/dynamic_plan_manager_pop.h>

using std::cout;
using std::endl;

void DynamicPlanManagerPOP::initPlanModules(ros::NodeHandle &nh)
{

    node_ = nh;
// std::cout << "initPlanModules" << std::endl;
    // ros param
	node_.param("plan_manager/max_vel", pp_.max_vel_, 0.6); // TODO should be same to MPC controller
	node_.param("plan_manager/max_acc", pp_.max_acc_, 0.1);
	node_.param("plan_manager/max_jerk", pp_.max_jerk_, 2.0);
	// node_.param("plan_manager/time_resolution",  pp_.time_resolution_, 1.0);
	node_.param("plan_manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.05);
	node_.param("plan_manager/global_points_distance", pp_.global_pt_dist_, 0.05);
	node_.param("plan_manager/use_distinctive_trajs", pp_.use_distinctive_trajs_, true);
	node_.param("plan_manager/local_time_horizon", pp_.local_time_horizon_, 2.0);
	pub_global_traj_ = node_.advertise<visualization_msgs::Marker>("global_raw_traj", 10);
	pub_local_traj_ = node_.advertise<visualization_msgs::Marker>("local_raw_traj", 10);
	pub_local_traj_opt_ = node_.advertise<visualization_msgs::Marker>("local_opt_traj", 10);
	// pub_jps_point_ = node_.advertise<visualization_msgs::Marker>("local_jps_point", 10);
	pub_occ_point_ = node_.advertise<visualization_msgs::Marker>("local_occ_point", 10);
	pub_local_goal_ = node_.advertise<visualization_msgs::Marker>("local_goal", 10);
	// sub_laser_scan_ = node_.subscribe("/scan", 10, &DynamicPlanManagerPOP::laserScanCallback, this);
	poly_test_timer_ = node_.createTimer(ros::Duration(0.05), &DynamicPlanManagerPOP::polyTestTimerCallback, this);
	odom_timer_ = node_.createTimer(ros::Duration(0.05), &DynamicPlanManagerPOP::odomTimerCallback, this);
	poly_test_pub_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("poly_test", 1, true);
	point_test_pub_ = node_.advertise<visualization_msgs::Marker>("point_test", 1, true);
	velodyne_sub_ = node_.subscribe("/velodyne_points", 1, &DynamicPlanManagerPOP::velodyneCallback, this);
	interesting_points_sub_ = node_.subscribe("/local_map/visualizeLeadingRayAfterPruneForDecompose", 1, &DynamicPlanManagerPOP::interestPointsCallback, this);
	poly_pub_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
	interesting_rays_for_replan_pub_ = node_.advertise<visualization_msgs::Marker>("interesting_rays_for_replan", 10);

	tf_listener_pop_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_pop_);
	base_to_odom_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
	velodyne_to_base_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
	tf2_ros::Buffer tfBuffer_velodyne;
	tf2_ros::TransformListener tfListener_velodyne(tfBuffer_velodyne);
	try{
		*(velodyne_to_base_ptr_) = tfBuffer_velodyne.lookupTransform("unitree_scan", "base", ros::Time::now(), ros::Duration(2.0));
	}
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

	grid_map_.reset(new GridMap);
	grid_map_->initMap(node_);
	// planner_ptr_.reset(new JPSPlanner3D(true));

	global_graph_ptr_.reset(new GlobalGraph);

}

bool DynamicPlanManagerPOP::adjustStartAndTargetPoint(Eigen::Vector3d &start_pt, Eigen::Vector3d &target_pt)
{
	if (checkCollision(start_pt) == 1) // 1 means obstalce, others means free
	{
		ROS_ERROR("[Plan manage]: This start point is insdide an obstacle.");
		return false;
	}
	if (checkCollision(target_pt) == 1)
	{
		ROS_ERROR("[Plan manage]: This target point is insdide an obstacle.");
		return false;
	}
	return true;
}

void DynamicPlanManagerPOP::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
											Eigen::Vector4d color, int id)
{
	visualization_msgs::Marker sphere, line_strip;
	sphere.header.frame_id = line_strip.header.frame_id = "map";
	sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
	sphere.type = visualization_msgs::Marker::SPHERE_LIST;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
	sphere.id = id;
	line_strip.id = id + 1000;

	sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
	sphere.color.r = line_strip.color.r = color(0);
	sphere.color.g = line_strip.color.g = color(1);
	sphere.color.b = line_strip.color.b = color(2);
	sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
	sphere.scale.x = scale;
	sphere.scale.y = scale;
	sphere.scale.z = scale;
	line_strip.scale.x = scale / 2;
	geometry_msgs::Point pt;
	for (int i = 0; i < int(list.size()); i++)
	{
		pt.x = list[i](0);
		pt.y = list[i](1);
		pt.z = list[i](2);
		sphere.points.push_back(pt);
		line_strip.points.push_back(pt);
	}
	pub.publish(sphere);
	pub.publish(line_strip);
}

bool DynamicPlanManagerPOP::planGlobalPath(GraphNode *node, Eigen::Vector3d &start_pos, Eigen::Vector3d &end_pos){
	// get all the geometry center of the polygedron
	// need to find out if the end_pos is inside the first two polyhedrons
std::cout << "in planGlobalPath" << std::endl;
	std::vector<Eigen::Vector3d> center_list;

	if (node->polys_[0].inside(end_pos) && node->polys_[1].inside(end_pos)){
		goalInFirstTwoPoly_ = true;
std::cout << "goal in intersection" << std::endl;
		center_list.push_back(start_pos);
		center_list.push_back(end_pos);
	}

	else if (node->polys_[0].inside(end_pos) && (node->center_list_[0] - end_pos).norm() < 0.5){
		goalInFirstTwoPoly_ = true;
std::cout << "goal in first poly" << std::endl;
		center_list.push_back(start_pos);
		center_list.push_back(end_pos);
	}
	else if (node->polys_[1].inside(end_pos) && (node->center_list_[2] - end_pos).norm() < 0.5){
		goalInFirstTwoPoly_ = true;
std::cout << "goal in second poly" << std::endl;
		center_list.push_back(start_pos);
		center_list.push_back(node->center_list_[1]);
		center_list.push_back(end_pos);
	}

	else{
std::cout << "not in first two poly" << std::endl;
		goalInFirstTwoPoly_ = false;
		int num_polys = node->polys_.size();
		// int num_polys = 2;
		std::vector<Eigen::Vector3d> center_list_temp;
		center_list_temp.push_back(start_pos);
		// consider the center of the first polyhedron is similar to the start_pos
		for (int i = 1; i < num_polys+1; i++){
			// center_list_temp.push_back(node->polys_[i].vertices_.rowwise().mean());
			center_list_temp.push_back(node->center_list_[i]);
		}

// for (int i = 0; i < node->center_list_.size(); i++){
// 	std::cout << "node center list: " << node->center_list_[i].transpose() << std::endl;
// }
for (int i = 0; i < center_list_temp.size(); i++){
	std::cout << "temp center list: " << center_list_temp[i].transpose() << std::endl;
}

		bool is_end = false;
		Eigen::Vector3d temp_center1, temp_center2;
		for (int i = 0; i < center_list_temp.size() - 1; i++){
			temp_center1 = center_list_temp[i];
			temp_center2 = center_list_temp[i+1];
			if ((end_pos - temp_center1).norm() < (temp_center2 - temp_center1).norm()){
	// std::cout << "early break" << std::endl;
				center_list.push_back(temp_center1);
				center_list.push_back(end_pos);
				is_end = true;
				break;
			}
			else{
				center_list.push_back(temp_center1);
			}
		}
		if(!is_end){
			center_list.push_back(center_list_temp[center_list_temp.size()-1]);
			center_list.push_back(end_pos);
		}
	}
	
std::cout << "center_list size: " << center_list.size() << std::endl;
for (int i = 0; i < center_list.size(); i++){
	std::cout << center_list[i].transpose() << std::endl;
}
// std::cout << "end_pos: " << end_pos << std::endl;
	// the global path will be current position -> center of the polyhedrons -> goal position
	std::vector<Eigen::Vector3d> point_set;
	double step_size = pp_.global_pt_dist_;

	Eigen::Vector3d temp_vec;
	int seg_num;
	for (int i = 0; i < center_list.size() - 1; i++){
		temp_vec = center_list[i+1] - center_list[i];
		seg_num = int(temp_vec.norm() / step_size);
		for (int j = 0; j < seg_num; j++){
			point_set.push_back(center_list[i] + temp_vec * j / seg_num);
		}
	}

	point_set.push_back(end_pos);

	local_waypoints_.clear();
	local_waypoints_ = point_set;

	displayMarkerList(pub_global_traj_, point_set, 0.08, Eigen::Vector4d(0, 1, 0, 1), 0);

	ROS_INFO("\033[1;32m[Plan manage]: Global Trajectory generated successful\033[0m");

	global_data_.setGlobalPath(point_set);

	return true;
}

bool DynamicPlanManagerPOP::checkCollision(const Eigen::Vector3d &pos)
{
	bool is_occ = grid_map_->getStaticInflateOccupancy(pos);
	return is_occ;
}

bool DynamicPlanManagerPOP::checkLocalCollision(const Eigen::Vector3d &pos)
{
	bool is_occ = grid_map_->getFusedDynamicInflateOccupancy(pos);
	return is_occ;
}

bool DynamicPlanManagerPOP::checkTrajCollision(double &distance, int &start_collision_index)
{
	// get local trajetory list
	std::vector<Eigen::Vector3d> local_path_list = local_traj_data_.local_path_;
	Eigen::Vector3d fut_pt;
	double     		check_distance = 1.5; // m
	distance = 0.0;
	bool            is_occ_occur = false;
	bool            is_occ, is_last_occ;
	std::vector<Eigen::Vector3d> safe_point_list;
	is_occ = is_last_occ = false;
	// check collision along the local trajectory for check_distance, obtain collision distance and start collision index
	// if collision, return true, else return false
	for (int i = 1; i < local_path_list.size() && distance < check_distance; i++)
	{
		fut_pt = local_path_list[i];
		safe_point_list.push_back(fut_pt);
		is_occ = checkLocalCollision(fut_pt);
		distance += (fut_pt - local_path_list[i-1]).norm();
		if (is_occ)
		{
			start_collision_index = i;
			is_occ_occur = true;
			break;
		}
	}
	if (is_occ_occur && safe_point_list.size() > 0)
	{
		displayMarkerList(pub_occ_point_, safe_point_list, 0.1, Eigen::Vector4d(1.0, 1.0, 0.8, 1), 0);
		return false;
	}
	return true;

}

bool DynamicPlanManagerPOP::planFromCurrentTraj(const double &planning_distance, Eigen::Vector3d &start_pos)
{
	/**
	 * @brief plan a local trajectory from start pos based on global path list
	 * @param planning_distance: the distance of local trajectory
	 * @param start_pos: the start position of local trajectory
	 * @return true if success
	 */

	int near_goal_pts = 5;	// calculate the angle based on the disturb point
	Eigen::Vector3d point;
	std::vector<Eigen::Vector3d> pos_list;
	std::vector<Eigen::Vector3d> rpy_list;
	// get global path list
	std::vector<Eigen::Vector3d> global_path_list = global_data_.getGlobalPath();

	// get nearest point index
	int index = 0;
	global_data_.getClosetIndex(start_pos, index);
	int index_start = index;
	// plan from index for planning_distance
	double angle, last_angle, roll, pitch, yaw;
	double distance = 0.0;
	while(distance < planning_distance && index < global_path_list.size()-1)
	{	
		pos_list.push_back(global_path_list[index]);
		// give value to yaw angle, if near the end, use the goal yaw
		if (index < global_path_list.size()-near_goal_pts)
			angle = atan2(global_path_list[index+1](1) - global_path_list[index](1), global_path_list[index+1](0) - global_path_list[index](0));
		else
			global_data_.getGoalRPY(roll, pitch, angle);
		// refine angle
		if (index != index_start){
			angle = refineAngle(angle, last_angle);
		}
		last_angle = angle;
		rpy_list.push_back(Eigen::Vector3d(roll, pitch, angle));
		distance += (global_path_list[index+1] - global_path_list[index]).norm();
		index++;
	}

	// assign local trajectory
	local_traj_data_.resetDataPOP(pos_list, rpy_list, start_pos);
	return true;
}

// bool DynamicPlanManagerPOP::ReplanLocalTraj(const int &start_collision_index, 
// 									   const Eigen::Vector3d &odom_pos)
// {
// 	/**
// 	 * @brief replan a local trajectory from start pos using jps and update global trajectory
// 	 * @param start_collision_index: the start collision index in current local trajectory
// 	 * @param odom_pos: the current position of robot
// 	 * @return true if success
// 	*/
// 	Eigen::Vector3d start_pos, end_pos;	
// 	start_pos = odom_pos;
// 	bool find_end_pos = false;
// 	bool   is_local_plan_success = false;
// 	bool current_safe, last_safe;
// 	current_safe = last_safe = false;

// 	// check from start_collision_index along local trajectory to find a "safe" end pos
// 	int index = start_collision_index;
// 	std::vector<Eigen::Vector3d> local_path_list = local_traj_data_.local_path_;
// 	int end_index = local_path_list.size()-1;
// 	// local_traj_data_.getDistanceIndex(start_pos, end_index, 0.3, index);

// 	while (end_index > index)
// 	{
// 		if (checkLocalCollision(local_path_list[end_index]))
// 		{
// 			end_index--;
// 		}
// 		else
// 		{	
// 			find_end_pos = true;
// 			end_pos = local_path_list[end_index];
// 			displayPointList(pub_local_goal_, {end_pos}, 0.1, Eigen::Vector4d(0, 1, 0.5, 1), 0);
// 			break;
// 		}
// 		last_safe = current_safe;

// 	}
// 	// if no "safe" end pos, return false
// 	if (end_index == index && !find_end_pos)
// 	{
// 		ROS_ERROR("[Plan manage]: No safe end pos found.");
// 		return false;
// 	}
// 	// plan a new local trajectory from start pos to end pos using jps->search and replace it in global_data
// 	std::vector<Eigen::Vector3d> new_local_path_list;
// 	auto start_time = std::chrono::steady_clock::now();
// 	int iteration = 20;
// 	int search_iter_num = 0;

// 	std::vector<Eigen::Matrix<double, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1>>> path_buffer;

// 	// planner_ptr_->setMapUtil(grid_map_->getMapUtilForJPS3d()); // Set collision checking function
// 	// planner_ptr_->updateMap();
// 	while(search_iter_num < iteration) {
// 		is_local_plan_success = planner_ptr_->plan(start_pos, end_pos, 1, true);

// 		if (is_local_plan_success){
// 			path_buffer = planner_ptr_->getRawPath();
// 			new_local_path_list.reserve(path_buffer.size());
// 			for(const auto& item : path_buffer)
// 				new_local_path_list.push_back(item);
// 			ROS_INFO("[Plan manage]: Local path search successful.");
// 			break;
// 		}
// 		else {
// 			search_iter_num++;
// 			if(end_index <= index){
// 				ROS_ERROR("[Plan manage]: The goal is not reachable.");
// 				return false;
// 			}
// 			end_index--;
// 			end_pos = local_path_list[end_index];
// 		}
// 	}


// 	if (search_iter_num >= iteration)
// 	{
// 		ROS_ERROR("[Plan manage]: Local path search failed.");
// 		return false;
// 	} 

// 	ROS_INFO("\033[1;32m[Plan manage]: Local path search time: %f ms\033[0m", 
// 		std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count() / 1000.0);
// 	// interpolate the new_local_path_list with pp_.global_pt_dist_ to get a smooth local trajectory
// 	std::vector<Eigen::Vector3d> intepolate_local_path_list;
// 	double step_size = pp_.global_pt_dist_;
// 	for (int i = 0; i<new_local_path_list.size()-1; i++)
// 	{
// 		Eigen::Vector3d temp_vec = new_local_path_list[i+1] - new_local_path_list[i];
// 		int seg_num = int(temp_vec.norm() / step_size);
// 		for (int j = 0; j < seg_num; j++)
// 		{
// 			intepolate_local_path_list.push_back(new_local_path_list[i] + temp_vec * j / seg_num);
// 		}
		
// 	}
// 	displayMarkerList(pub_local_traj_, intepolate_local_path_list, 0.05, Eigen::Vector4d(0, 1, 0.5, 1), 0);

// 	// erase the segment of global trajectory from start pos index to end index and insert new local trajectory
// 	std::vector<Eigen::Vector3d> global_path_list = global_data_.getGlobalPath();
// 	std::vector<Eigen::Vector3d> new_global_path_list;
// 	int start_index_global;

// 	global_data_.getClosetIndex(start_pos, start_index_global);

// 	int end_index_global = end_index + start_index_global;
// 	for (int i = 0; i < start_index_global; i++)
// 	{
// 		new_global_path_list.push_back(global_path_list[i]);
// 	}
// 	for (int i = 0; i < intepolate_local_path_list.size(); i++)
// 	{
// 		new_global_path_list.push_back(intepolate_local_path_list[i]);
// 	}
// 	for (int i = end_index_global; i < global_path_list.size(); i++)
// 	{
// 		new_global_path_list.push_back(global_path_list[i]);
// 	}
// 	// update global trajectory

// 	global_data_.setGlobalPath(new_global_path_list);
// 	return true;
// }

double DynamicPlanManagerPOP::refineAngle(double &angle, double &last_angle)
{
	double diff_angle;
	// calcute the angle difference with continuous form
	// uncontious form will cause occilation		
	if (angle * last_angle < 0)
	{
		if (angle >= 0)
		{
			if (angle - last_angle > M_PI)
				diff_angle = angle - last_angle - 2 * M_PI;
			else
				diff_angle = angle - last_angle;
		}
		else
		{
			if (last_angle - angle > M_PI)
				diff_angle = angle - last_angle + 2 * M_PI;
			else
				diff_angle = angle - last_angle;
		}
	}
	else
	{
		diff_angle = angle - last_angle;
	}

	return last_angle + diff_angle;
}

void DynamicPlanManagerPOP::displayPointList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, 
										Eigen::Vector4d color, int id)
{
	visualization_msgs::Marker sphere;
	sphere.header.frame_id  = "map";
	sphere.header.stamp = ros::Time::now();
	sphere.type = visualization_msgs::Marker::SPHERE_LIST;
	sphere.action = visualization_msgs::Marker::ADD;
	sphere.id = id;

	sphere.pose.orientation.w = 1.0;
	sphere.color.r = color(0);
	sphere.color.g = color(1);
	sphere.color.b = color(2);
	sphere.color.a = color(3) > 1e-5 ? color(3) : 1.0;
	sphere.scale.x = scale;
	sphere.scale.y = scale;
	sphere.scale.z = scale;

	geometry_msgs::Point pt;
	for (int i = 0; i < int(list.size()); i++)
	{
		pt.x = list[i](0);
		pt.y = list[i](1);
		pt.z = list[i](2);
		sphere.points.push_back(pt);
	}
	pub.publish(sphere);
}

void DynamicPlanManagerPOP::odomTimerCallback(const ros::TimerEvent &e)
{

	try
	{
		*base_to_odom_ptr_ = tfBuffer_pop_.lookupTransform("odom", "base", ros::Time(0));
		// ROS_INFO("odomTimerCallback: %f, %f, %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);

	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
	}
}

void DynamicPlanManagerPOP::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if (msg->data.size() == 0)
			return;

	sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);

    vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);

    vec_Vec3f obs3d;
    // only take the point that x is within the range [-3.0, 3.0], y is within the range [-3.0, 3.0], z is within the range [-0.8, 0.8], and the resolution is 0.05
   for (unsigned int i = 0; i < obs.size(); i++){
        if (obs[i][0] >= -3.0 && obs[i][0] <= 3.0 && obs[i][1] >= -3.0 && obs[i][1] <= 3.0 && obs[i][2] >= -0.8 && obs[i][2] <= 0.8)
            obs3d.push_back(obs[i]);
    }

    // to eliminate the points that casued by the robot itself
    // all the point that within a rectangle of 0.55m * 0.4m will be eliminated
	// for (int i = 0; i < obs3d.size(); i++){
	// 	if (obs3d[i][0] < 0.275 && obs3d[i][0] > -0.275 && obs3d[i][1] < 0.2 && obs3d[i][1] > -0.2){
	// 		obs3d.erase(obs3d.begin() + i);
	// 		i--;
	// 	}
	// }
    // for(const auto& it: obs)
    //     obs3d.push_back(it.topRows<3>());  

    for (double x = -5; x <= 5;x += 0.05){
        for (double y = -5; y <= 5; y += 0.05){
            for (double z = -0.5; z <= 0.3; z += 0.8){
                Vec3f pt;
                pt << x, y, z;
                obs3d.push_back(pt);
            }
        }
    }
    for (double x = -5; x <= 5; x += 0.05){
        for (double y = -5; y <= 5; y +=10){
            for (double z = -0.5; z <= 0.5; z += 0.05){
                Vec3f pt;
                pt << x, y, z;
                obs3d.push_back(pt);
            }
        }
    }
    for (double x = -5; x <= 5; x += 10){
        for (double y = -5; y <= 5; y += 0.05){
            for (double z = -0.5; z <= 0.5; z += 0.05){
                Vec3f pt;
                pt << x, y, z;
                obs3d.push_back(pt);
            }
        }
    }

    // the points now are in velodyne frame, we need to transform the points to the odom frame
	const Eigen::Affine3d T_velodyne_base = tf2::transformToEigen(*(velodyne_to_base_ptr_));
    const Eigen::Matrix4f T_velodyne_base_matrix = T_velodyne_base.matrix().cast<float>();
    const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*(base_to_odom_ptr_));
    const Eigen::Matrix4f T_base_odom_matrix = T_base_odom.matrix().cast<float>();
    vec_Vec3f obs3d_odom;
    for(unsigned int i = 0; i < obs3d.size(); i++){
        Eigen::Vector4f pt;
        Vec3f pt_odom;
        pt << obs3d[i].cast<float>(), 1.0;
        Eigen::Vector4f pt_transformed = T_base_odom_matrix * T_velodyne_base_matrix.inverse() * pt;
        pt_odom << pt_transformed[0], pt_transformed[1], pt_transformed[2];
        obs3d_odom.push_back(pt_odom);
    }
	obs3d_odom_ = obs3d_odom;
}

void DynamicPlanManagerPOP::polyTestTimerCallback(const ros::TimerEvent &e)
{
// std::cout << "size of polys_test_: " << polys_test_.size() << std::endl;
	if(!polys_test_.empty()){
		decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_test_);
		poly_msg.header.frame_id = "odom";
		poly_test_pub_.publish(poly_msg);
	}
// std::cout << "size of points_for_test_1: " << points_for_test_1.size() << std::endl;
// std::cout << "size of points_for_test_2: " << points_for_test_2.size() << std::endl;
// std::cout << "size of points_for_test_3: " << points_for_test_3.size() << std::endl;
	if(!points_for_test_1.empty()){
		visualization_msgs::Marker polytope_list;
		polytope_list.header.frame_id = "odom";
		polytope_list.header.stamp = ros::Time::now();
		polytope_list.ns = "polytope_list";
		polytope_list.action = visualization_msgs::Marker::ADD;
		polytope_list.pose.orientation.w = 1.0;
		polytope_list.id = 0;
		polytope_list.type = visualization_msgs::Marker::LINE_LIST;
		polytope_list.scale.x = 0.01;
		polytope_list.color.b = 1.0;
		polytope_list.color.g = 1.0;
		polytope_list.color.a = 1.0;
		geometry_msgs::Point pt1, pt2;
		for(int i = 0; i < points_for_test_1.size()-1; i=i+2){
			pt1.x = points_for_test_1[i](0);
			pt1.y = points_for_test_1[i](1);
			pt1.z = points_for_test_1[i](2);
			pt2.x = points_for_test_1[i+1](0);
			pt2.y = points_for_test_1[i+1](1);
			pt2.z = points_for_test_1[i+1](2);
			polytope_list.points.push_back(pt1);
			polytope_list.points.push_back(pt2);
		}
		// pt1.x = points_for_test_1[0](0);
		// pt1.y = points_for_test_1[0](1);
		// pt1.z = points_for_test_1[0](2);
		// pt2.x = points_for_test_1[points_for_test_1.size()-1](0);
		// pt2.y = points_for_test_1[points_for_test_1.size()-1](1);
		// pt2.z = points_for_test_1[points_for_test_1.size()-1](2);
		// polytope_list.points.push_back(pt1);
		// polytope_list.points.push_back(pt2);
		// for (int i = 0; i < points_for_test_2.size()-1; i++){
		// 	pt1.x = points_for_test_2[i](0);
		// 	pt1.y = points_for_test_2[i](1);
		// 	pt1.z = points_for_test_2[i](2);
		// 	pt2.x = points_for_test_2[i+1](0);
		// 	pt2.y = points_for_test_2[i+1](1);
		// 	pt2.z = points_for_test_2[i+1](2);
		// 	polytope_list.points.push_back(pt1);
		// 	polytope_list.points.push_back(pt2);
		// }
		// pt1.x = points_for_test_2[0](0);
		// pt1.y = points_for_test_2[0](1);
		// pt1.z = points_for_test_2[0](2);
		// pt2.x = points_for_test_2[points_for_test_2.size()-1](0);
		// pt2.y = points_for_test_2[points_for_test_2.size()-1](1);
		// pt2.z = points_for_test_2[points_for_test_2.size()-1](2);
		// polytope_list.points.push_back(pt1);
		// polytope_list.points.push_back(pt2);
		// for (int i = 0; i < points_for_test_3.size()-1; i++){
		// 	pt1.x = points_for_test_3[i](0);
		// 	pt1.y = points_for_test_3[i](1);
		// 	pt1.z = points_for_test_3[i](2);
		// 	pt2.x = points_for_test_3[i+1](0);
		// 	pt2.y = points_for_test_3[i+1](1);
		// 	pt2.z = points_for_test_3[i+1](2);
		// 	polytope_list.points.push_back(pt1);
		// 	polytope_list.points.push_back(pt2);
		// }
		// pt1.x = points_for_test_3[0](0);
		// pt1.y = points_for_test_3[0](1);
		// pt1.z = points_for_test_3[0](2);
		// pt2.x = points_for_test_3[points_for_test_3.size()-1](0);
		// pt2.y = points_for_test_3[points_for_test_3.size()-1](1);
		// pt2.z = points_for_test_3[points_for_test_3.size()-1](2);
		// polytope_list.points.push_back(pt1);
		// polytope_list.points.push_back(pt2);
		point_test_pub_.publish(polytope_list);

	}
	// polys_test_.clear();
	points_for_test_1.clear();
	points_for_test_2.clear();
	points_for_test_3.clear();
}

void DynamicPlanManagerPOP::setLinearConstraintsandVertices(vec_E<Polyhedron3D> &polys_, std::vector<Eigen::Vector3d> &points_for_poly_){
	// set LinearConstraints and vertices for each polyhedron
	for (unsigned int i = 0; i < polys_.size(); ++i){
		auto vs_ = polys_[i].hyperplanes();
		int poly_num_facets = vs_.size();
		Eigen::MatrixX4d hPoly;
		hPoly.resize(poly_num_facets, 4);
		Eigen::Matrix3Xd vPoly;

		LinearConstraint3D cs(points_for_poly_[i], vs_);
		polys_[i].set_lc(cs);
		auto A = cs.A();
		auto b = cs.b();

		for (int j = 0; j < poly_num_facets; ++j)
		{
			hPoly.row(j) << A.row(j), -b(j);
		}
		// convert the H-representation to V-representation
		// current expression is in the form of Ax < b
		bool flag = false;
		flag =  geo_utils::enumerateVs(hPoly, vPoly);
		if (!flag)
		{
			ROS_ERROR("[Plan manage]: Failed to convert polyhedron to vertices.");
			return;
		}
		else{
			polys_[i].set_vertices(vPoly);
		}
	}
}

void DynamicPlanManagerPOP::sortPolyhedronArray(vec_E<Polyhedron3D> &polys_, std::vector<Eigen::Vector3d> &points_for_poly, vec_E<Polyhedron3D> &polys_sorted_, Eigen::Vector3d &goal_pos, std::vector<double> &cost_to_goal_sorted){
	// sort the polyhedron array based on the distance to the goal position
	// each sequence contain 3 polyhedrons
	Eigen::MatrixX4d hPoly;
	std::vector<double> distance_to_goal;

	for (unsigned int i = 0; i < polys_.size() / 3; ++i){
		// calculate the distance to the goal position
		double cost_to_goal = 0.0;
		cost_to_goal += (polys_[3*i].vertices_.rowwise().mean() -polys_[3*i+1].vertices_.rowwise().mean()).norm();
		cost_to_goal += (polys_[3*i+1].vertices_.rowwise().mean() -polys_[3*i+2].vertices_.rowwise().mean()).norm();
		cost_to_goal += (polys_[3*i+2].vertices_.rowwise().mean() - goal_pos).norm();
		// Eigen::Vector3d center = polys_[3 * i + 2].vertices_.rowwise().mean();
		distance_to_goal.push_back(cost_to_goal);
	}

	// based on the distance to the goal position, sort the polyhedron array
	std::vector<size_t> idx(distance_to_goal.size());
	std::iota(idx.begin(), idx.end(), 0);
	std::sort(idx.begin(), idx.end(), [&distance_to_goal](size_t i1, size_t i2) {return distance_to_goal[i1] < distance_to_goal[i2];});

	// polys_sorted_.push_back(polys_[0]);
	for (unsigned int i = 0; i < polys_.size() / 3; ++i){
		polys_sorted_.push_back(polys_[3*idx[i]]);
		polys_sorted_.push_back(polys_[3*idx[i]+1]);
		polys_sorted_.push_back(polys_[3*idx[i]+2]);
		cost_to_goal_sorted.push_back(distance_to_goal[idx[i]]);
	}
}

vec_E<Polyhedron3D> DynamicPlanManagerPOP::estimateIntersectionVolumeAndGetNewPolyhedron(Polyhedron3D &poly1_, Polyhedron3D &poly2_){
	// get the H-representation of the two polyhedrons
	// polys_test_.clear();
std::cout << "in estimateIntersectionVolumeAndGetNewPolyhedron" << std::endl;
	vec_E<Polyhedron3D> polys_tmp_;
	auto vs1 = poly1_.hyperplanes();
	auto vs2 = poly2_.hyperplanes();
	int poly1_num_facets = vs1.size();
	int poly2_num_facets = vs2.size();
	Eigen::MatrixX4d hPoly;
	hPoly.resize(poly1_num_facets + poly2_num_facets, 4);
	Eigen::Matrix3Xd vPoly;
// std::cout << "hPoly already resized" << std::endl;
	auto A1 = poly1_.lc_.A();
	auto b1 = poly1_.lc_.b();
	auto A2 = poly2_.lc_.A();
	auto b2 = poly2_.lc_.b();
// std::cout << "A and b already got" << std::endl;
	// put A1 and A2 b1 and b2 into hPoly
	for (int j = 0; j < poly1_num_facets; ++j)
	{
		hPoly.row(j) << A1.row(j), -b1(j);

	}
	for (int j = 0; j < poly2_num_facets; ++j)
	{
		hPoly.row(j + poly1_num_facets) << A2.row(j), -b2(j);
	}
	geo_utils::enumerateVs(hPoly, vPoly);
// for(int i = 0; i < vPoly.cols(); i++){
// 	std::cout << vPoly.col(i).transpose() << std::endl;
// }
std::cout << "vPoly size: " << vPoly.cols() << std::endl;
// std::cout << std::endl;
Polyhedron3D poly_new = poly1_;
for (int i = 0; i < poly2_num_facets; i++){
	poly_new.add(vs2[i]);
}
auto vertices_test = cal_vertices(poly_new);
std::vector<std::vector<Eigen::Vector3d>> vertices_for_each_surfaces;
vertices_for_each_surfaces.resize(vertices_test.size());
for (int i = 0; i < vertices_test.size(); i++){
	vertices_for_each_surfaces[i].resize(0);
	for (int j = 0; j < vertices_test[i].size(); j++){
		// check if there is any repeated vertices, only push the unique vertices
		if (std::find(vertices_for_each_surfaces[i].begin(), vertices_for_each_surfaces[i].end(), vertices_test[i][j]) == vertices_for_each_surfaces[i].end()){
			vertices_for_each_surfaces[i].push_back(vertices_test[i][j]);
		}
	}
}
// for(int i = 0; i < vertices_for_each_surfaces.size(); i++){
// 	std::cout << "surface " << i << std::endl;
// 	for (int j = 0; j < vertices_for_each_surfaces[i].size(); j++){
// 		std::cout << vertices_for_each_surfaces[i][j].transpose() << std::endl;
// 	}
// }

	if(checkIfNewPolyhedronNeeded(vPoly.rowwise().mean(), vertices_for_each_surfaces)){
		Eigen::Vector3d center = vPoly.rowwise().mean();
		// std::cout << "need new polyhedron in point " << center.transpose() << std::endl;
		const Vec3f pos(center(0), center(1), center(2));
		SeedDecomp3D sd(pos);
		sd.set_obs(obs3d_odom_);
		sd.set_local_bbox(Vec3f(0.6, 0.6, 0.5));
		sd.dilate(0.2);
		Polyhedron3D poly_new = sd.get_polyhedron();
		poly_new.set_vertices(vPoly);
		auto vs_new = poly_new.hyperplanes();
		LinearConstraint3D cs_new(center, vs_new);
		poly_new.set_lc(cs_new);

		polys_tmp_.push_back(poly1_);
		polys_tmp_.push_back(poly_new);
		polys_tmp_.push_back(poly2_);

		// polys_test_.push_back(poly_new);
		return polys_tmp_;
	}
	else{
		polys_tmp_.push_back(poly1_);
		polys_tmp_.push_back(poly2_);
		return polys_tmp_;
	}

}

bool DynamicPlanManagerPOP::ifCanPassThroughShortestLine(Polyhedron3D &poly1, Polyhedron3D &poly2, Eigen::Vector3d odom){
	// add a hyperplane to the given two polyhedrons
std::cout << "in ifCanPassThroughShortestLine" << std::endl;
	auto vs1 = poly1.hyperplanes();
	auto vs2 = poly2.hyperplanes();
	int poly1_num_facets = vs1.size();
	int poly2_num_facets = vs2.size();
	Eigen::MatrixX4d hPoly1, hPoly2;
	Eigen::Matrix3Xd vPoly1, vPoly2;
	hPoly1.resize(poly1_num_facets + 1, 4);
	hPoly2.resize(poly2_num_facets + 1, 4);
	// hPoly1.resize(poly1_num_facets, 4);
	// hPoly2.resize(poly2_num_facets, 4);

	auto A1 = poly1.lc_.A();
	auto b1 = poly1.lc_.b();
	auto A2 = poly2.lc_.A();
	auto b2 = poly2.lc_.b();

	Eigen::MatrixX4d hPoly_intersect_;
	Eigen::Matrix3Xd vPoly_intersect_;
	hPoly_intersect_.resize(poly1_num_facets + poly2_num_facets + 1, 4);
	// put A1 and A2 b1 and b2 into hPoly
	for (int j = 0; j < poly1_num_facets; ++j)
	{
		hPoly1.row(j) << A1.row(j), -b1(j);
		hPoly_intersect_.row(j) << A1.row(j), -b1(j);
	}
	hPoly1.row(poly1_num_facets) << 0, 0, 1, -0.25;
	for (int j = 0; j < poly2_num_facets; ++j)
	{
		hPoly2.row(j) << A2.row(j), -b2(j);
		hPoly_intersect_.row(j + poly1_num_facets) << A2.row(j), -b2(j);
	}
	hPoly2.row(poly2_num_facets) << 0, 0, 1, -0.25;
	hPoly_intersect_.row(poly1_num_facets + poly2_num_facets) << 0, 0, 1, -0.25;
std::cout << "Prepare to enumerateVs" << std::endl;
	bool flag1 = false;
	flag1 = geo_utils::enumerateVs(hPoly1, vPoly1);
	flag1 = geo_utils::enumerateVs(hPoly2, vPoly2);
	flag1 = geo_utils::enumerateVs(hPoly_intersect_, vPoly_intersect_);
std::cout << "if Vpoly successfully got: " << flag1 << std::endl;

// std::cout << "vPoly_intersect_ size: " << vPoly_intersect_.cols() << std::endl;
// for (int i = 0; i < vPoly_intersect_.cols(); i++){
// 	std::cout << vPoly_intersect_.col(i).transpose() << std::endl;
// }
	if(!flag1){
		ROS_ERROR("[Plan manage]: Failed to convert polyhedron to vertices.");
		return false;
	}

	std::vector<Eigen::Vector3d> vPoly_intersect_at_robot_height;
	std::vector<Eigen::Vector3d> vPoly1_at_robot_height;
	std::vector<Eigen::Vector3d> vPoly2_at_robot_height;
	for (int i = 0; i < vPoly_intersect_.cols(); i++){
		if (vPoly_intersect_.col(i)(2) < 0.25 + 0.00001 && vPoly_intersect_.col(i)(2) > 0.25 - 0.00001){
			vPoly_intersect_at_robot_height.push_back(vPoly_intersect_.col(i));
		}
	}
	for (int i = 0; i < vPoly1.cols(); i++){
		if (vPoly1.col(i)(2) < 0.25 + 0.00001 && vPoly1.col(i)(2) > 0.25 - 0.00001){
			vPoly1_at_robot_height.push_back(vPoly1.col(i));
		}
	}
	for (int i = 0; i < vPoly2.cols(); i++){
		if (vPoly2.col(i)(2) < 0.25 + 0.00001 && vPoly2.col(i)(2) > 0.25 - 0.00001){
			vPoly2_at_robot_height.push_back(vPoly2.col(i));
		}
	}
std::cout << "vPoly_intersect_at_robot_height size: " << vPoly_intersect_at_robot_height.size() << std::endl;
for ( int i = 0; i < vPoly_intersect_at_robot_height.size(); i++){
	std::cout << vPoly_intersect_at_robot_height[i].transpose() << std::endl;
}
std::cout << "vPoly1_at_robot_height size: " << vPoly1_at_robot_height.size() << std::endl;
for ( int i = 0; i < vPoly1_at_robot_height.size(); i++){
	std::cout << vPoly1_at_robot_height[i].transpose() << std::endl;
}
std::cout << "vPoly2_at_robot_height size: " << vPoly2_at_robot_height.size() << std::endl;
for ( int i = 0; i < vPoly2_at_robot_height.size(); i++){
	std::cout << vPoly2_at_robot_height[i].transpose() << std::endl;
}

	std::vector<Eigen::Vector3d> vPoly_final_list;
	// among the vertices of vPoly_intersect_, find the vertices that are exactly on the boundary of poly1 and poly2
	for (int i = 0; i < vPoly_intersect_at_robot_height.size(); i++){
		bool on_boundary1 = false;
		bool on_boundary2 = false;
		// check if the vertex of vPoly_intersect_ is on the boundary of poly1
		// use all the hyperplanes of poly1 to check
		for (int k = 0; k < poly1_num_facets; k++){
// std::cout << "if on boundary1: " << fabs(A1.row(k) * vPoly_intersect_at_robot_height[i] - b1(k)) << std::endl;
			fabs(A1.row(k) * vPoly_intersect_at_robot_height[i] - b1(k)) < 0.01 ? on_boundary1 = true : on_boundary1 = false;
			if (on_boundary1){
				break;
			}
		}
		if (on_boundary1){
			// check if the vertex of vPoly_intersect_ is on the boundary of poly2
			// use all the hyperplanes of poly2 to check
			for (int k = 0; k < poly2_num_facets; k++){
// std::cout << "if on boundary2: " << fabs(A2.row(k) * vPoly_intersect_at_robot_height[i] - b2(k)) << std::endl;
				fabs(A2.row(k) * vPoly_intersect_at_robot_height[i] - b2(k)) < 0.01 ? on_boundary2 = true : on_boundary2 = false;
				if (on_boundary2){
					break;
				}
			}
		}
		// if the vertex of vPoly_intersect_ is on the boundary of both poly1 and poly2, we add it to the vPoly_final_list
		if (on_boundary1 && on_boundary2){
			vPoly_final_list.push_back(vPoly_intersect_at_robot_height[i]);
		}
	}
std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! show vPoly_final_list" << std::endl;
std::cout << "vPoly_final_list size: " << vPoly_final_list.size() << std::endl;
for (int i = 0; i < vPoly_final_list.size(); i++){
	std::cout << vPoly_final_list[i].transpose() << std::endl;
}
	// base on the points from vPoly_final_list, find the closet point that the line generate from them will seperate the start_point and end_point
	Eigen::Vector3d start_point, end_point;
	Eigen::Vector3d points1, points2;
	points1 << 0, 0, 0;
	points2 << 3, 4, 0;
	start_point = odom;
	end_point = poly2.vertices_.rowwise().mean();
	double min_distance = 1e10;
	for(int i = 0 ; i < vPoly_final_list.size() - 1; i++){
		for (int j = i+1; j < vPoly_final_list.size(); j++){
			double slope = (vPoly_final_list[j](1) - vPoly_final_list[i](1)) / (vPoly_final_list[j](0) - vPoly_final_list[i](0));
			double intersect = vPoly_final_list[i](1) - slope * vPoly_final_list[i](0);
			// check if the start_point and end_point are on the different side of the line
			if ((start_point(1) - slope * start_point(0) - intersect) * (end_point(1) - slope * end_point(0) - intersect) < 0){
				if (( vPoly_final_list[i] - vPoly_final_list[j]).norm() < min_distance){
					min_distance = (vPoly_final_list[i] - vPoly_final_list[j]).norm();
					points1 = vPoly_final_list[i];
					points2 = vPoly_final_list[j];
				}
			}
		}
	}

	points_for_test_1.push_back(points1);
	points_for_test_1.push_back(points2);
std::cout << points1.transpose() << std::endl;
std::cout << points2.transpose() << std::endl;
std::cout << "min_distance: " << min_distance << std::endl;
	// set 0.32m as the shortest edge of the robot
	if (min_distance > 0.32 && min_distance < 20){
		return true;
	}
	else{
		return false;
	}
}

bool DynamicPlanManagerPOP::checkIfNewPolyhedronNeeded(Eigen::Vector3d center, std::vector<std::vector<Eigen::Vector3d>> vertices_for_each_surfaces){
	// check if the volume of the intersection polyhedron is larger than the threshold
	// if not, generate a new polyhedron at the center of the intersection polyhedron
	// the threshold is set to be the volume of robot times 2
	// robot is considered as a box with size 0.6m * 0.3m * 0.3m
std::cout << "in checkIfNewPolyhedronNeeded" << std::endl;
	double threshold = 0.8 * 0.4 * 0.3 * 6;
	double volume_sum = 0.0;
	for (int i = 0; i < vertices_for_each_surfaces.size(); i++){
		// get tetrahedron volume for each surface
		//first get the center of the surface
		Eigen::Vector3d center_surface = Eigen::Vector3d::Zero();
		for (int j = 0; j < vertices_for_each_surfaces[i].size(); j++){
			center_surface += vertices_for_each_surfaces[i][j];
		}
		center_surface /= vertices_for_each_surfaces[i].size();
		// then sort all the points clockwise
		// first get the vector from center to each point
		std::vector<Eigen::Vector3d> vec_to_center;
		vec_to_center.resize(vertices_for_each_surfaces[i].size());
		for (int j = 0; j < vertices_for_each_surfaces[i].size(); j++){
			vec_to_center[j] = vertices_for_each_surfaces[i][j] - center_surface;
		}
		// then get the angle of each vector and sort the point based on the angle
		std::vector<double> angle;
		angle.resize(vertices_for_each_surfaces[i].size());
		for (int j = 0; j < vertices_for_each_surfaces[i].size(); j++){
			angle[j] = atan2(vec_to_center[j](1), vec_to_center[j](0));
		}
		std::vector<size_t> idx(angle.size());
		std::iota(idx.begin(), idx.end(), 0);
		std::sort(idx.begin(), idx.end(), [&angle](size_t i1, size_t i2) {return angle[i1] < angle[i2];});	
		std::vector<Eigen::Vector3d> vertices_sorted;
		vertices_sorted.resize(vertices_for_each_surfaces[i].size());
		for (int j = 0; j < vertices_for_each_surfaces[i].size(); j++){
			vertices_sorted[j] = vertices_for_each_surfaces[i][idx[j]];
		}
		// then calculate the volume of all the tetrahedrons by connecting adjacent points to the center and the geometry center
		for (int i = 0; i < vertices_sorted.size(); i++){
			Eigen::Vector3d p1 = vertices_sorted[i];
			Eigen::Vector3d p2 = vertices_sorted[(i+1) % vertices_sorted.size()];
			Eigen::Vector3d p3 = center_surface;
			Eigen::Vector3d p4 = center;
			double volume = (p1 - p4).dot((p2 - p4).cross(p3 - p4)) / 6;
			if (volume < 0){
				volume = -volume;
			}
			volume_sum += volume;
		}
	}

	

	// double average_distance = 0.0;
	// double dist_sum = 0.0;
	// average_distance = dist_sum / vPol.cols();
std::cout << "volume_sum: " << volume_sum << std::endl;
	if (volume_sum > threshold){
		// no need for more polyhedrons
		return false;
	}
	else{
		// generate one more polyhedron at the center of the intersection polyhedron
std::cout << "need new polyhedron" << std::endl;
		return true;
	}
}

void DynamicPlanManagerPOP::generatePolyhedronAtIntersection(vec_E<Polyhedron3D> &polys_, GraphNode *node, std::vector<double> &cost_to_goal_sorted){
std::cout << "in generatePolyhedronAtIntersection" << std::endl;
	std::vector<vec_E<Polyhedron3D>> polys_after_generation_;
	int sequence_of_polys;
	if(polys_.size() % 3 == 0){
		sequence_of_polys = polys_.size() / 3;
	}
	else{
		sequence_of_polys = (polys_.size()+1) / 3;
	}
	polys_after_generation_.resize(sequence_of_polys);
std::cout << "size of polys_: " << polys_.size() << std::endl;

	for (unsigned int i = 0; i < sequence_of_polys; ++i){
		// if need to generate a new polyhedron, estimateIntersectionVolume will return a new polyhedron, else return nothing
		// only check first two polyhedrons
// std::cout << i << "th polyhedron before generation" << std::endl;
		polys_after_generation_[i] = estimateIntersectionVolumeAndGetNewPolyhedron(polys_[3*i], polys_[3*i+1]);
		if(3*i+2 < polys_.size())
			polys_after_generation_[i].push_back(polys_[3*i+2]);
	}
std::cout << "size of polys_after_generation_: " << polys_after_generation_.size() << std::endl;
	Eigen::Vector3d robot_pos_velodyne(0,0,0);
	const Eigen::Affine3d T_velodyne_base = tf2::transformToEigen(*(velodyne_to_base_ptr_));
    const Eigen::Matrix4f T_velodyne_base_matrix = T_velodyne_base.matrix().cast<float>();
    const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*(base_to_odom_ptr_));
    const Eigen::Matrix4f T_base_odom_matrix = T_base_odom.matrix().cast<float>();
	Eigen::Vector4f robot_pos_transformed =  T_base_odom_matrix * T_velodyne_base_matrix.inverse() * Eigen::Vector4f(robot_pos_velodyne(0), robot_pos_velodyne(1), robot_pos_velodyne(2), 1.0);
	Eigen::Vector3d robot_pos_odom;
	robot_pos_odom << robot_pos_transformed[0], robot_pos_transformed[1], robot_pos_transformed[2];
	// for all direction, check first 3 polyhedrons
	bool dead_end = false;
	Eigen::Vector3d start_pos = node->replan_pos_;
	for (unsigned int i = 0; i < sequence_of_polys ; ++i){
std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!i: " << i << std::endl;
		dead_end = !ifCanPassThroughShortestLine(polys_after_generation_[i][0], polys_after_generation_[i][1], robot_pos_odom);
std::cout << "dead_end: " << dead_end << std::endl;
		if(!dead_end){
			// compare the leading ray of the polyhedron with the direction from the start point to the end point
			Eigen::Vector3d end_pos = polys_after_generation_[i][1].vertices_.rowwise().mean();
			// Eigen::Vector3d end_pos = interest_points_[2*i];

			Eigen::Vector3d leading_ray_unit = (end_pos - start_pos).normalized();
			Eigen::Vector3d leading_ray_unit_xy, node_leading_ray_unit_xy;
			leading_ray_unit_xy << leading_ray_unit(0), leading_ray_unit(1), 0;
			node_leading_ray_unit_xy << node->leading_ray_.normalized()(0), node->leading_ray_.normalized()(1), 0;
std::cout << "start_pos: " << start_pos.transpose() << std::endl;
std::cout << "end_pos: " << end_pos.transpose() << std::endl;
// if(!node->root){
// std::cout << "parent node leading ray start pos: " << node->polys_[0].vertices_.rowwise().mean().transpose() << std::endl;
// std::cout << "parent node leading ray end pos: " << node->polys_[1].vertices_.rowwise().mean().transpose() << std::endl;
// }
std::cout << "leading_ray_unit: " << leading_ray_unit_xy.transpose() << std::endl;
std::cout << "node_leading_ray_unit: " << node_leading_ray_unit_xy.transpose() << std::endl;
std::cout << "two rays' direction: " << leading_ray_unit_xy.dot(node_leading_ray_unit_xy) << std::endl;
			if (node->root){
				GraphNode *new_node = new GraphNode();
				Eigen::Vector3d replan_pts = polys_after_generation_[i][1].vertices_.rowwise().mean();
				new_node->replan_pos_ = replan_pts;																// the geometry center of the second polyhedron, and this is the replan position
				new_node->polys_ = polys_after_generation_[i];
				new_node->cost_to_goal_ = cost_to_goal_sorted[i];
				new_node->dead_end_ = dead_end;
				new_node->leading_ray_ = end_pos - start_pos;	// the leading ray is the direction from the start point to the end point

				bool too_close = false;
std::cout << "size of child nodes: " << node->child_.size() << std::endl;
				for ( int n = 0; n < node->child_.size(); n++){
					// check leading_ray with existing child nodes, if too close do not add
					Eigen::Vector3d child_leading_ray_unit_xy;
					child_leading_ray_unit_xy << node->child_[n]->leading_ray_.normalized()(0), node->child_[n]->leading_ray_.normalized()(1), 0;
					Eigen::Vector3d new_node_leading_ray_unit_xy;
					new_node_leading_ray_unit_xy << new_node->leading_ray_.normalized()(0), new_node->leading_ray_.normalized()(1), 0;
std::cout << "child leading ray: " << child_leading_ray_unit_xy.transpose() << std::endl;
std::cout << "new node leading ray: " << new_node_leading_ray_unit_xy.transpose() << std::endl;
					// get the angle between the leading_ray and the leading_ray of the child node
std::cout << "cos of two rays: " << new_node_leading_ray_unit_xy.dot(child_leading_ray_unit_xy) << std::endl;
					if (new_node_leading_ray_unit_xy.dot(child_leading_ray_unit_xy) > 0.75){
						too_close = true;
						break;
					}
				}

				if (!too_close){
					new_node->parent_ = node;
					getCenterListForNode(new_node);
std::cout << "dist to travel: " << (new_node->center_list_[2] - new_node->center_list_[0]).norm() << std::endl;
					if((new_node->center_list_[2] - new_node->center_list_[0]).norm() > 0.049){
						node->child_.push_back(new_node);
std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! node added" << std::endl;
						}
				}

			}
			// compare the yaw angle between the leading_ray_unit and the leading_ray of the parent node
			// if those two vector is almost parallel, do not add the node
			else if (leading_ray_unit_xy.dot(node_leading_ray_unit_xy) > -0.71 && !backtrackFlag_){
std::cout << " trying to add node " << std::endl;
				bool if_in_dead_polys = false;
				bool if_in_visited_polys = false;
				for (int j = 0; j < polys_deadend_.size(); j++){
					if (polys_deadend_[j].inside(polys_after_generation_[i][1].vertices_.rowwise().mean())){
						if_in_dead_polys = true;
						break;
					}
				}
				for (int j = 0; j < polys_visited_.size(); j++){
					if (polys_visited_[j].inside(polys_after_generation_[i][1].vertices_.rowwise().mean())){
						if_in_visited_polys = true;
						break;
					}
				}
std::cout << "if in dead polys: " << if_in_dead_polys << std::endl;
std::cout << "if in visited polys: " << if_in_visited_polys << std::endl;
				if (!if_in_dead_polys && !if_in_visited_polys){
					GraphNode *new_node = new GraphNode();
					Eigen::Vector3d replan_pts = polys_after_generation_[i][1].vertices_.rowwise().mean();
					new_node->replan_pos_ = replan_pts;																// the geometry center of the second polyhedron, and this is the replan position
					new_node->polys_ = polys_after_generation_[i];
					new_node->cost_to_goal_ = cost_to_goal_sorted[i];
					new_node->dead_end_ = dead_end;
					new_node->leading_ray_ = end_pos - start_pos;	// the leading ray is the direction from the start point to the end point
					bool too_close = false;
std::cout << "size of child nodes: " << node->child_.size() << std::endl;
					for ( int n = 0; n < node->child_.size(); n++){
						// check leading_ray with existing child nodes, if too close do not add
						Eigen::Vector3d child_leading_ray_unit_xy;
						child_leading_ray_unit_xy << node->child_[n]->leading_ray_.normalized()(0), node->child_[n]->leading_ray_.normalized()(1), 0;
std::cout << "cos of two rays: " << leading_ray_unit_xy.dot(child_leading_ray_unit_xy) << std::endl;
						// get the angle between the leading_ray and the leading_ray of the child node
						if (leading_ray_unit_xy.dot(child_leading_ray_unit_xy) > 0.75){
							too_close = true;
							break;
						}
					}

					if (!too_close){
						new_node->parent_ = node;
						getCenterListForNode(new_node);
std::cout << "dist to travel: " << (new_node->center_list_[2] - new_node->center_list_[0]).norm() << std::endl;
					if((new_node->center_list_[2] - new_node->center_list_[0]).norm() > 0.049){
						node->child_.push_back(new_node);
std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! node added" << std::endl;
						}
				}
				}

			}
			else if (backtrackFlag_){
				GraphNode *new_node = new GraphNode();
				Eigen::Vector3d replan_pts = polys_after_generation_[i][1].vertices_.rowwise().mean();
				new_node->replan_pos_ = replan_pts;																// the geometry center of the second polyhedron, and this is the replan position
				new_node->polys_ = polys_after_generation_[i];
				new_node->cost_to_goal_ = cost_to_goal_sorted[i];
				new_node->dead_end_ = dead_end;
				new_node->leading_ray_ = end_pos - start_pos;	// the leading ray is the direction from the start point to the end point
				new_node->parent_ = node;
				getCenterListForNode(new_node);
				node->child_.push_back(new_node);
std::cout << "node added" << std::endl;
			}
		}
		else{
// std::cout << "can't pass shortest line" << std::endl;
		}
	}

std::cout << "end of generatePolyhedronAtIntersection" << std::endl;
}

bool DynamicPlanManagerPOP::ifGoBackToParentCanGetLowerCost(GraphNode *node, int mode){
	// check if go back to parent can get lower cost
	// if the cost of the parent is larger than the cost of the current node, return true
	// else return false
std::cout << "in ifGoBackToParentCanGetLowerCost" << std::endl;
	int index;
	if(mode ==0){
		index = 0;
	}
	else{
		index = 1;
	}

	if (node->root == true){
std::cout << "node is root" << std::endl;
		return false;
	}
	else{
		if (node->parent_->child_.size() < 2 || node->child_.size() < 1){
std::cout << "node->parent_->child_.size() < 2 || node->child_.size() < 1" << std::endl;
			return false;
		}

std::cout << "node parent child_[1] is dead end or not: " << node->parent_->child_[1]->dead_end_ << std::endl;
std::cout << "node child_[index] is dead end or not: " << node->child_[index]->dead_end_ << std::endl;
		if(!node->child_[index]->dead_end_ && !node->parent_->child_[1]->dead_end_){
			double current_cost = node->child_[index]->cost_to_goal_;
			double back_cost = node->parent_->child_[1]->cost_to_goal_ + (node->replan_pos_ - node->parent_->replan_pos_).norm();
std::cout << "current_cost: " << current_cost << "and back_cost: " << back_cost << "  and the difference is: " << current_cost - back_cost << std::endl;
			if (current_cost > back_cost + 0.55){
std::cout << "go back to parent" << endl;
				return true;
			}
			else{
				return false;
			}
		}
		else{
std::cout << "one of the node is dead end" << std::endl;
			return false;
		}
	}
}

void DynamicPlanManagerPOP::updateCurrentNode(GraphNode *&node, std::vector<Eigen::Vector3d> &points_for_backtrack, std::vector<GraphNode *> &nodes_for_backtrack){
	// update the current node
	// if child of node is empty, call updateCurrentNode, and the node will be updated to the parent of the node
	// if child of node is not empty, but all child of node is dead end, call updateCurrentNode, and the node will be updated to the parent of the node
	if (node->child_.size() == 0){
		if (node->parent_ != NULL){
			node->dead_end_ = true;
			backtrackFlag_ = true;
			firstreplanafterbacktrack_ = true;
			points_for_backtrack.push_back(node->parent_->replan_pos_);
			nodes_for_backtrack.push_back(node);
			node = node->parent_;
			updateCurrentNode(node, points_for_backtrack, nodes_for_backtrack);
		}
	}
	else{
		bool all_dead_end = true;
		bool all_visited = true;
		for (unsigned int i = 0; i < node->child_.size(); ++i){
			if (!node->child_[i]->dead_end_){
				GraphNode *childNode = node->child_[i];
				all_dead_end = false;
				if (!backtrackFlag_){
					if (!childNode->visited_){
std::cout << i << "th child is choose" << std::endl;
						node = childNode;
						all_visited = false;
						break;
					}
					else{
						continue;
					}
				}
				else{
					break;
				}	
			}
		}
		if (all_visited){
std::cout << "all visited" << std::endl;
			for (unsigned int i = 0; i < node->child_.size(); ++i){
				if (!node->child_[i]->dead_end_){
					GraphNode *childNode = node->child_[i];
					if (!backtrackFlag_){
						node = childNode;
					}
				}
				break;
			}
		}
		if (all_dead_end){
			if (node->parent_ != NULL){
				node->dead_end_ = true;
				backtrackFlag_ = true;
				firstreplanafterbacktrack_ = true;
				points_for_backtrack.push_back(node->parent_->replan_pos_);
				nodes_for_backtrack.push_back(node);
				node = node->parent_;
				updateCurrentNode(node, points_for_backtrack, nodes_for_backtrack);
			}
		}
	}
}

void DynamicPlanManagerPOP::getPolysForBackTrack(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos, vec_E<Polyhedron3D> &polys_){
std::cout << "in getPolysForBackTrack" << std::endl;
	Eigen::Vector3d pos1, pos2, pos3, pos4;
std::cout << "start_pos: " << start_pos.transpose() << std::endl;
std::cout << "end_pos: " << end_pos.transpose() << std::endl;
	Eigen::Vector3d dir = end_pos - start_pos;
std::cout << "dir: " << dir.transpose() << std::endl;
	pos1 = start_pos + 0.1 * dir;
	pos2 = start_pos + 0.6 * dir;
	pos3 = start_pos + 0.5 * dir;
	pos4 = start_pos + 1.0 * dir;
	const Vec3f pos1_f(pos1(0), pos1(1), pos1(2));
	const Vec3f pos2_f(pos2(0), pos2(1), pos2(2));
	const Vec3f pos3_f(pos3(0), pos3(1), pos3(2));
	const Vec3f pos4_f(pos4(0), pos4(1), pos4(2));

	LineSegment3D line1(pos1_f, pos2_f);
	LineSegment3D line2(pos3_f, pos4_f);
	line1.set_obs(obs3d_odom_);
	line2.set_obs(obs3d_odom_);
	line1.set_local_bbox(Vec3f(0.8, 2, 0.5));
	line2.set_local_bbox(Vec3f(0.8, 2, 0.5));
	line1.dilate(0.1);
	line2.dilate(0.1);
	Polyhedron3D poly1 = line1.get_polyhedron();
	Polyhedron3D poly2 = line2.get_polyhedron();

	auto vs1 = poly1.hyperplanes();
	auto vs2 = poly2.hyperplanes();
	int poly1_num_facets = vs1.size();
	int poly2_num_facets = vs2.size();
	Eigen::MatrixX4d hPoly1, hPoly2;
	hPoly1.resize(poly1_num_facets, 4);
	hPoly2.resize(poly2_num_facets, 4);
	Eigen::Matrix3Xd vPoly1, vPoly2;

	LinearConstraint3D cs1((pos1_f + pos2_f) / 2, vs1);
	LinearConstraint3D cs2((pos3_f + pos4_f) / 2, vs2);
	poly1.set_lc(cs1);
	poly2.set_lc(cs2);
	auto A1 = cs1.A();
	auto b1 = cs1.b();
	auto A2 = cs2.A();
	auto b2 = cs2.b();
	for (int j = 0; j < poly1_num_facets; ++j)
	{
		hPoly1.row(j) << A1.row(j), -b1(j);
	}
	for (int j = 0; j < poly2_num_facets; ++j)
	{
		hPoly2.row(j) << A2.row(j), -b2(j);
	}
	geo_utils::enumerateVs(hPoly1, vPoly1);
	geo_utils::enumerateVs(hPoly2, vPoly2);
	poly1.set_vertices(vPoly1);
	poly2.set_vertices(vPoly2);

	polys_.push_back(poly1);
	polys_.push_back(poly2);

	polys_test_.push_back(poly1);
	polys_test_.push_back(poly2);
std::cout << "size of polys_test_: " << polys_test_.size() << std::endl;
	decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_test_);
	poly_msg.header.frame_id = "odom";
	poly_pub_.publish(poly_msg);
}

void DynamicPlanManagerPOP::decomposePolys(){
	polys_.clear();
    points_for_poly_.clear();
	points_for_decompose_.clear();
// std::cout << "size of interest_points_: " << interest_points_.size() << std::endl;
	if (interest_points_.size() > 1){

        for (unsigned int i = 0; i < interest_points_.size()/2; ++i){
			points_for_decompose_.push_back(interest_points_[2*i]);
			points_for_decompose_.push_back(interest_points_[2*i+1]);

            Eigen::Vector3d dir = interest_points_[2*i] - interest_points_[2*i+1];
            double dist = dir.norm();
            if (dist > 3)
                dist = 3;
            dir.normalize();

            Eigen::Vector3d pos1, pos2, pos3, pos4, pos5, pos6;
            pos1 = interest_points_[2*i+1] + 0.05*dir * dist;
            pos2 = interest_points_[2*i+1] + 0.45*dir * dist;

            pos3 = interest_points_[2*i+1] + 0.4*dir * dist;
            pos4 = interest_points_[2*i+1] + 0.7*dir * dist;

            pos5 = interest_points_[2*i+1] + 0.65*dir * dist;
            pos6 = interest_points_[2*i+1] + 0.9*dir * dist;

            const Vec3f pos1_v(pos1[0], pos1[1], pos1[2]);
            const Vec3f pos2_v(pos2[0], pos2[1], pos2[2]);
            const Vec3f pos3_v(pos3[0], pos3[1], pos3[2]);
            const Vec3f pos4_v(pos4[0], pos4[1], pos4[2]);
            const Vec3f pos5_v(pos5[0], pos5[1], pos5[2]);
            const Vec3f pos6_v(pos6[0], pos6[1], pos6[2]);
    // std::cout << "pos1: " << pos1.transpose() << " pos2: " << pos2.transpose() << std::endl;
    // std::cout << "pos3: " << pos3.transpose() << " pos4: " << pos4.transpose() << std::endl;
    // std::cout << "pos5: " << pos5_v.transpose() << " pos6: " << pos6_v.transpose() << std::endl;
            LineSegment3D decomp_util1(pos1_v,pos2_v);
            decomp_util1.set_obs(obs3d_odom_);
            decomp_util1.set_local_bbox(Vec3f(0.7, 0.4, 0.5));
            decomp_util1.dilate(0.1);

            LineSegment3D decomp_util2(pos3_v,pos4_v);
            decomp_util2.set_obs(obs3d_odom_);
            decomp_util2.set_local_bbox(Vec3f(0.7, 0.4, 0.5));
            decomp_util2.dilate(0.1);

            LineSegment3D decomp_util3(pos5_v,pos6_v);
            decomp_util3.set_obs(obs3d_odom_);
            decomp_util3.set_local_bbox(Vec3f(1, 1, 0.5));
            decomp_util3.dilate(0.1);

            // es_.push_back(decomp_util1.get_ellipsoid());
            polys_.push_back(decomp_util1.get_polyhedron());

            // es_.push_back(decomp_util2.get_ellipsoid());
            polys_.push_back(decomp_util2.get_polyhedron());

            // es_.push_back(decomp_util3.get_ellipsoid());
            polys_.push_back(decomp_util3.get_polyhedron());

            points_for_poly_.push_back((pos1_v + pos2_v) / 2);
            points_for_poly_.push_back((pos3_v + pos4_v) / 2);
            points_for_poly_.push_back((pos5_v + pos6_v) / 2);
        }
	}
	setLinearConstraintsandVertices(polys_, points_for_poly_);
	decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_);
	poly_msg.header.frame_id = "odom";
	poly_pub_.publish(poly_msg);
}

void DynamicPlanManagerPOP::interestPointsCallback(const visualization_msgs::MarkerPtr &msg){
	if (msg->points.size() == 0)
        return;
    interest_points_.clear();
    for (unsigned int i = 0; i < msg->points.size(); i++){
        Eigen::Vector3d pt;
        pt << msg->points[i].x, msg->points[i].y, msg->points[i].z;
        interest_points_.push_back(pt);
    }
	// all the points are in the base frame, we need to transform the points to the odom frame
    const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*(base_to_odom_ptr_));
    const Eigen::Matrix4f T_base_odom_matrix = T_base_odom.matrix().cast<float>();
    for (unsigned int i = 0; i < interest_points_.size(); i++){
        Eigen::Vector4f pt;
        pt << interest_points_[i].cast<float>(), 1.0;
        Eigen::Vector4f pt_transformed = T_base_odom_matrix * pt;
        interest_points_[i] = pt_transformed.head<3>().cast<double>();
    }
	// After getting the interest points, publish the points so that the replan module can get the points
	visualization_msgs::Marker rays_for_replan;
	rays_for_replan.header.frame_id = "odom";
	rays_for_replan.header.stamp = ros::Time::now();
	rays_for_replan.ns = "rays_for_replan";
	rays_for_replan.action = visualization_msgs::Marker::ADD;
	rays_for_replan.pose.orientation.w = 1.0;
	rays_for_replan.id = 0;
	rays_for_replan.type = visualization_msgs::Marker::LINE_LIST;
	rays_for_replan.scale.x = 0.05;
	rays_for_replan.color.r = 1.0;
	rays_for_replan.color.a = 1.0;

	geometry_msgs::Point p1, p2;
	for (unsigned int i = 0; i < interest_points_.size(); i+=2){
		p1.x = interest_points_[i](0);
		p1.y = interest_points_[i](1);
		p1.z = interest_points_[i](2);
		p2.x = interest_points_[i+1](0);
		p2.y = interest_points_[i+1](1);
		p2.z = interest_points_[i+1](2);
		rays_for_replan.points.push_back(p1);
		rays_for_replan.points.push_back(p2);
	}
	interesting_rays_for_replan_pub_.publish(rays_for_replan);
}

void DynamicPlanManagerPOP::getCenterListForNode(GraphNode *&node){
	auto vs1 = node->polys_[0].hyperplanes();
	auto vs2 = node->polys_[1].hyperplanes();
	int poly1_num_facets = vs1.size();
	int poly2_num_facets = vs2.size();
	Eigen::MatrixX4d hPoly;
	hPoly.resize(poly1_num_facets + poly2_num_facets, 4);
	Eigen::Matrix3Xd vPoly;
// std::cout << "hPoly already resized" << std::endl;
	auto A1 = node->polys_[0].lc_.A();
	auto b1 = node->polys_[0].lc_.b();
	auto A2 = node->polys_[1].lc_.A();
	auto b2 = node->polys_[1].lc_.b();
// std::cout << "A and b already got" << std::endl;
	// put A1 and A2 b1 and b2 into hPoly
	for (int j = 0; j < poly1_num_facets; ++j)
	{
		hPoly.row(j) << A1.row(j), -b1(j);

	}
	for (int j = 0; j < poly2_num_facets; ++j)
	{
		hPoly.row(j + poly1_num_facets) << A2.row(j), -b2(j);
	}
	geo_utils::enumerateVs(hPoly, vPoly);
	// get the center of the polyhedron
	Eigen::Vector3d center = vPoly.rowwise().mean();
	node->center_list_.push_back(node->polys_[0].vertices_.rowwise().mean());
	node->center_list_.push_back(center);
	node->center_list_.push_back(node->polys_[1].vertices_.rowwise().mean());
	for (int i = 2; i < node->polys_.size(); ++i){
		node->center_list_.push_back(node->polys_[i].vertices_.rowwise().mean());
	}
std::cout << "center list size: " << node->center_list_.size() << std::endl;
}

void DynamicPlanManagerPOP::getDeadEndPoly(Polyhedron3D &poly, Eigen::Vector3d pos){
	// generate a polyhedron at the position of the robot
	// the polyhedron is a box with size 0.8m * 0.8m * 0.8m
	const Vec3f center(pos(0), pos(1), pos(2));
	SeedDecomp3D sd(center);
	sd.set_obs(obs3d_odom_);
	sd.set_local_bbox(Vec3f(1.0, 1.0, 0.5));
	sd.dilate(0.2);
	Polyhedron3D poly_new = sd.get_polyhedron();
	auto vs_new = poly_new.hyperplanes();
	LinearConstraint3D cs_new(center, vs_new);
	poly_new.set_lc(cs_new);
	poly = poly_new;
	polys_test_.push_back(poly);
}

void DynamicPlanManagerPOP::getVisitedPoly(Polyhedron3D &poly, Eigen::Vector3d pos){
	const Vec3f center(pos(0), pos(1), pos(2));
	SeedDecomp3D sd(center);
	sd.set_obs(obs3d_odom_);
	sd.set_local_bbox(Vec3f(0.5, 0.5, 0.2));
	sd.dilate(0.2);
	Polyhedron3D poly_new = sd.get_polyhedron();
	auto vs_new = poly_new.hyperplanes();
	LinearConstraint3D cs_new(center, vs_new);
	poly_new.set_lc(cs_new);
	poly = poly_new;
	polys_test_.push_back(poly);
}