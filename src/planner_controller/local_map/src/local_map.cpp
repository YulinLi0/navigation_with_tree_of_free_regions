#include "daimon_local_map/local_map.h"

void GridMap::initMap(ros::NodeHandle &nh)
{
    node_ = nh;
    double x_size, y_size, z_size;      
    // size in meter
    node_.param("local_map/map_size_x", x_size, 7.0);
    node_.param("local_map/map_size_y", y_size, 7.0);
    node_.param("local_map/map_size_z", z_size, 1.6);

    // local map size
    node_.param("local_map/local_update_range_x", mp_.local_update_range_(0), 5.5);
    node_.param("local_map/local_update_range_y", mp_.local_update_range_(1), 5.5);
    node_.param("local_map/local_update_range_z", mp_.local_update_range_(2), 5.5);
    node_.param("local_map/local_update_range_x", mp_.local_x_range_(0), -3.5);
    node_.param("local_map/local_update_range_x", mp_.local_x_range_(1), 3.5);
    node_.param("local_map/local_update_range_y", mp_.local_y_range_(0), -3.5);
    node_.param("local_map/local_update_range_y", mp_.local_y_range_(1), 3.5);
    node_.param("local_map/local_update_range_z", mp_.local_z_range_(0), -0.8);
    node_.param("local_map/local_update_range_z", mp_.local_z_range_(1), 0.8);

    // occupancy map
    node_.param("local_map/p_hit", mp_.p_hit_, 0.55);
    node_.param("local_map/p_miss", mp_.p_miss_, 0.2);
    node_.param("local_map/p_min", mp_.p_min_, 0.12);
    node_.param("local_map/p_max", mp_.p_max_, 0.80);
    node_.param("local_map/p_occ", mp_.p_occ_, 0.70);

    // show time
    node_.param("local_map/show_occ_time", mp_.show_occ_time_, false);

    node_.param("local_map/frame_id", mp_.frame_id_, std::string("map"));
    node_.param("local_map/base_id", mp_.base_id_, std::string("base"));
    node_.param("local_map/pointcloud_sensor_frame_id", mp_.pointcloud_sensor_frame_id_, std::string("camera_depth_optical_frame"));
    node_.param("local_map/scan_sensor_frame_id", mp_.scan_sensor_frame_id_, std::string("unitree_scan"));
    // local map
    node_.param("local_map/obstacles_inflation", mp_.obstacles_inflation_, 0.15);
    node_.param("local_map/local_bound_inflate", mp_.local_bound_inflate_, 0.0);
    node_.param("local_map/local_map_margin", mp_.local_map_margin_, 50);

    /* map size*/
    // resolution
    node_.param("local_map/resolution", mp_.resolution_, 0.05);
    mp_.resolution_inv_ = 1 / mp_.resolution_;

    // size in meter
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, -z_size / 2.0);     // left bottom corner

    // size in pixel of global map
    for (int i = 0; i < 3; ++i)
        mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) * mp_.resolution_inv_);

    // global map boundary in pixel/index
    mp_.map_min_idx_ = Eigen::Vector3i::Zero();                      // min pixel idx  (x_voxel_min,y_voxel_min, z_voxel_min)
    mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones(); // max pixel idx  (x_voxel_max,y_voxel_max, z_voxel_max)

    // global map boundary in meter
    mp_.map_min_boundary_ = mp_.map_origin_; // map boundary (x_min,y_min) in meter
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

    // local map boundary in voxel/index
    mp_.local_map_origin_ = Eigen::Vector3d(mp_.local_x_range_(0), mp_.local_y_range_(0), mp_.local_z_range_(0)); // local map origin (x_min,y_min,z_min) in meter
    mp_.local_index_range_ = Eigen::Vector3i(ceil((mp_.local_x_range_(1) - mp_.local_x_range_(0)) * mp_.resolution_inv_),
                                             ceil((mp_.local_y_range_(1) - mp_.local_y_range_(0)) * mp_.resolution_inv_),
                                             ceil((mp_.local_z_range_(1) - mp_.local_z_range_(0)) * mp_.resolution_inv_));
   
    mp_.local_map_voxel_num_ = mp_.local_index_range_;
    mp_.local_min_idx_ = Eigen::Vector3i::Zero();                            // min voxel idx  (x_voxel_min,y_voxel_min,z_voxel_min)
    mp_.local_max_idx_ = mp_.local_map_voxel_num_ - Eigen::Vector3i::Ones(); // max pixel idx  (x_voxel_max,y_voxel_max,z_voxel_max)

    // local bound inflate
    mp_.local_bound_inflate_ = std::max(mp_.resolution_, mp_.local_bound_inflate_); // esdf inflation

    // occupancy map probability param                    // odd(s+)=odd(s-)+prob_hit_log_ or odd(s+)=odd(s-)+likelihood
    mp_.prob_hit_log_ = logit(mp_.p_hit_);      // log likelihood log[p(z=hit|s=occ)/p(m=hit|s=free)]
    mp_.prob_miss_log_ = logit(mp_.p_miss_);    // log likelihood log[p(z=miss|s=occ)/p(m=miss|s=free)]
    mp_.clamp_min_log_ = logit(mp_.p_min_);     // log min state prob  log[p(s=occ)/p(s=free)]
    mp_.clamp_max_log_ = logit(mp_.p_max_);     // log max state prob  log[p(s=occ)/p(s=free)]
    mp_.min_occupancy_log_ = logit(mp_.p_occ_); // log of occupancy determined prob
    mp_.unknown_flag_ = 0.01;

    /* map data param */
    md_.occ_need_update_ = false; // scan_odom_callback
    md_.local_updated_ = false;   // raycast process

    md_.has_odom_ = false; // odom callback [not much use]

    md_.fuse_time_ = 0.0;
    md_.update_num_ = 0;
    md_.max_fuse_time_ = 0.0;

    /* initialize data buffers*/
    // global map buffer size
    md_.buffer_size_ = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);                   // buffer size
    md_.local_buffer_size_ = mp_.local_map_voxel_num_(0) * mp_.local_map_voxel_num_(1) * mp_.local_map_voxel_num_(2); // local buffer size

    // global occupancy map buffer
    // save state_occu probability [mp_.clamp_min_log_,mp_.clamp_max_log_]
    // md_.occupancy_buffer_ = std::vector<double>(md_.buffer_size_, mp_.clamp_min_log_ - mp_.unknown_flag_); (not use)
    // md_.occupancy_buffer_neg_ = std::vector<char>(md_.buffer_size_, 0);
    md_.local_occupancy_buffer_ = std::vector<double>(md_.local_buffer_size_, mp_.clamp_min_log_ - mp_.unknown_flag_);
    md_.local_occupancy_buffer_neg_ = std::vector<char>(md_.local_buffer_size_, 0);
    md_.local_occupancy_buffer_inflate_ = std::vector<char>(md_.local_buffer_size_, 0);
    md_.local_occupancy_buffer_inflate_2D_slice_ = std::vector<char>(md_.local_buffer_size_, 0);
    // md_.local_occupancy_buffer_inflate_for_JPS_3D = std::vector<signed char, std::allocator<signed char>>(md_.local_buffer_size_, 0);

    md_.occupancy_buffer_static_inflate_ = std::vector<char>(md_.buffer_size_, 0); // static map buffer

    // curvatures
    md_.cloud_curvature_ = std::vector<double>(md_.local_buffer_size_ - 10, 0);
    md_.cloud_neighbor_picked_ = std::vector<double>(md_.local_buffer_size_ - 10, 0);
    md_.cloud_sort_ind_ = std::vector<int>(md_.local_buffer_size_ - 10, 0);
    md_.length_of_cloud_curvature_ = 0;
    md_.num_of_interest_pts_xy_ = 30;
    md_.num_of_interest_pts_xz_ = 5;

    // initialize the interest points
    md_.interest_pts_xy_.resize(md_.num_of_interest_pts_xy_);

    // global distance map buffer
    md_.distance_buffer_static_all_ = std::vector<double>(md_.buffer_size_, 10000);

    // local distance map buffer
    md_.local_distance_buffer_ = std::vector<double>(md_.local_buffer_size_, 0);
    md_.local_distance_buffer_neg_ = std::vector<double>(md_.local_buffer_size_, 0);
    md_.local_distance_buffer_all_ = std::vector<double>(md_.local_buffer_size_, 0);
    md_.local_tmp_buffer_ = std::vector<double>(md_.local_buffer_size_, 0);

    // local occupancy point cloud
    md_.fused_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // init static occ & ESDF buffers
    get_static_buffer(md_.occupancy_buffer_static_inflate_);


    // global occ log probabilty buffer
    md_.count_hit_and_miss_ = std::vector<short>(md_.buffer_size_, 0);
    md_.count_hit_ = std::vector<short>(md_.buffer_size_, 0);
    md_.flag_rayend_ = std::vector<char>(md_.buffer_size_, -1);
    md_.flag_traverse_ = std::vector<char>(md_.buffer_size_, -1);

    /* show map param */
    ROS_INFO("[Local map]: X_size: %.2f", x_size);
    ROS_INFO("[Local map]: Y_size: %.2f", y_size);
    ROS_INFO("[Local map]: Z_size: %.2f", z_size);
    ROS_INFO("[Local map]: X_origin: %.2f", mp_.map_origin_[0]);
    ROS_INFO("[Local map]: Y_origin: %.2f", mp_.map_origin_[1]);
    ROS_INFO("[Local map]: Z_origin: %.2f", mp_.map_origin_[2]);
    ROS_INFO("[Local map]: X_size_vox: %d", mp_.map_voxel_num_(0));
    ROS_INFO("[Local map]: Y_size_vox: %d", mp_.map_voxel_num_(1));
    ROS_INFO("[Local map]: Z_size_vox: %d", mp_.map_voxel_num_(2));
    ROS_INFO("[Local map]: X_map_max_idx: %d", mp_.map_max_idx_(0));
    ROS_INFO("[Local map]: Y_map_max_idx: %d", mp_.map_max_idx_(1));
    ROS_INFO("[Local map]: Z_map_max_idx: %d", mp_.map_max_idx_(2));
    ROS_INFO("[Local map]: X_local_update_range: %.2f", mp_.local_update_range_(0));
    ROS_INFO("[Local map]: Y_local_update_range: %.2f", mp_.local_update_range_(1));
    ROS_INFO("[Local map]: Z_local_update_range: %.2f", mp_.local_update_range_(2));
    ROS_INFO("[Local map]: Hit: %.2f", mp_.prob_hit_log_);
    ROS_INFO("[Local map]: Miss: %.2f", mp_.prob_miss_log_);
    ROS_INFO("[Local map]: Min_log: %.2f", mp_.clamp_min_log_);
    ROS_INFO("[Local map]: Max_log: %.2f", mp_.clamp_max_log_);
    ROS_INFO("[Local map]: Threshold log: %.2f", mp_.min_occupancy_log_);

    ros::NodeHandle public_nh("");

    // sensor sub
    // async_scan_sub_ = public_nh.subscribe<sensor_msgs::LaserScan>("local_map/scan", 10, &GridMap::scanCallback, this);
    async_scan_sub_ = public_nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &GridMap::scanCallback, this);
    async_odom_sub_ = public_nh.subscribe<nav_msgs::Odometry>("/odom", 10, &GridMap::odomCallback, this);
    async_pointcloud_sub_ = public_nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points_downsampled", 10, &GridMap::pointcloudCallback, this);
    goal_sub_ =  public_nh.subscribe("goal", 1, &GridMap::goalCallback, this);

    // timer callbacks
    occ_timer_ = public_nh.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this); // raycasting & setCacheOccupancy is the key
    vis_timer_ = public_nh.createTimer(ros::Duration(0.05), &GridMap::visCallback, this);

    // debug
    debug_pub_      = public_nh.advertise<sensor_msgs::PointCloud2>("debug/points", 1);
    debug_grad_pub_ = public_nh.advertise<geometry_msgs::PoseArray>("debug/grad", 1);

    // Publishers
    // local map and esdf map pub
    // local_map_pub_ = public_nh.advertise<nav_msgs::OccupancyGrid>("local_map/localOccupancyMap", 10);
    local_map_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("local_map/localOccupancyMap", 10);
rotated_map_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("local_map/rotatedOccupancyMap", 10);
local_map_interest_pts_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("local_map/localOccupancyMapInterestPoints", 10);
visualize_leading_ray_for_decompose_pub_ = public_nh.advertise<visualization_msgs::Marker>("local_map/visualizeLeadingRayForDecompose", 10);
visualize_leading_ray_after_prune_for_decompose_pub_= public_nh.advertise<visualization_msgs::Marker>("local_map/visualizeLeadingRayAfterPruneForDecompose", 10);
visualize_leading_ray_xyz_pub_ = public_nh.advertise<visualization_msgs::Marker>("local_map/visualizeLeadingRayXYZ", 10);
    // global map and esdf map pub
    // static_map_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("local_map/globalOccupancyMap", 10);

    mp_.TF_base_to_sensor_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    mp_.TF_base_to_scan_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    mp_.odom_transform_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);

    tf2_ros::Buffer tfBuffer_scan, tfBuffer_pc;
    tf2_ros::TransformListener tfListener_scan(tfBuffer_scan), tfListener_pc(tfBuffer_pc);

    try
    {
        *(mp_.TF_base_to_scan_ptr_) = tfBuffer_scan.lookupTransform(mp_.base_id_, mp_.scan_sensor_frame_id_, ros::Time::now(), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("fail to listen the transform of %s->%s. Error message: %s", mp_.base_id_.c_str(), mp_.scan_sensor_frame_id_.c_str(), ex.what());
        return;
    }

    try
    {
        *(mp_.TF_base_to_sensor_ptr_) = tfBuffer_pc.lookupTransform(mp_.base_id_, mp_.pointcloud_sensor_frame_id_, ros::Time::now(), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("fail to listen the transform of %s->%s. Error message: %s", mp_.base_id_.c_str(), mp_.pointcloud_sensor_frame_id_.c_str(), ex.what());
        return;
    }
}


/* Map utils */
void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
    ori = mp_.map_origin_;
    size = mp_.map_size_;
}

double GridMap::getResolution()
{
    return mp_.resolution_;
}

/*Time event callback: occupancy update*/
void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
// std::cout << "updateOccupancyCallback" << std::endl;
    if (md_.fused_cloud_ptr_->points.size() == 0)
    {
// std::cout << "no pointcloud" << std::endl;
        return;
    }
    std::fill(md_.local_occupancy_buffer_.begin(), md_.local_occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
    std::fill(md_.local_occupancy_buffer_inflate_.begin(), md_.local_occupancy_buffer_inflate_.end(), 0);
    std::fill(md_.local_occupancy_buffer_neg_.begin(), md_.local_occupancy_buffer_neg_.end(), 0);
    std::fill(md_.local_occupancy_buffer_inflate_2D_slice_.begin(), md_.local_occupancy_buffer_inflate_2D_slice_.end(), 0);
    // std::fill(md_.local_occupancy_buffer_inflate_for_JPS_3D.begin(), md_.local_occupancy_buffer_inflate_for_JPS_3D.end(), 0); 
    std::fill(md_.cloud_sort_ind_.begin(), md_.cloud_sort_ind_.end(), 0);
    std::fill(md_.cloud_curvature_.begin(), md_.cloud_curvature_.end(), 0);
    std::fill(md_.cloud_neighbor_picked_.begin(), md_.cloud_neighbor_picked_.end(), 0);
    md_.interest_pts_xy_.clear();
    md_.interest_pts_xy_pruned_.clear();
    md_.interest_pts_xyz_.clear();
    md_.test_pts_for_3D_.clear();
    md_.interest_pts_xz_unpruned_.clear();
    md_.pts_for_xz_.clear();

    // md_.interest_pts_xy_.resize(md_.num_of_interest_pts_);
    // ros::WallTime t1, t2;
    // t1 = ros::WallTime::now();
    // At start, we need to transfer the point cloud to the orgin point. After processing, we need to transfer back.
    const Eigen::Matrix4f odom_to_base_matrix = tf2::transformToEigen(*mp_.odom_transform_ptr_).matrix().cast<float>();
    local_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*md_.fused_cloud_ptr_, *local_pointcloud_, odom_to_base_matrix.inverse());

    projectPointCloud(local_pointcloud_);
    slicePointCloudInxyPlane(local_pointcloud_);

    // put different layer into one layer
    for (int i = 0; i < local_pointcloud_->points.size(); i++){
        local_pointcloud_->points[i].z = 0;
    }
    // to eliminate the points that casued by the robot itself
    // all the point that within a rectangle of 0.5m * 0.4m will be eliminated
    // for (int i = 0; i < local_pointcloud_->points.size(); i++){
    //     if (local_pointcloud_->points[i].x < 0.275 && local_pointcloud_->points[i].x > -0.275 && local_pointcloud_->points[i].y < 0.2 && local_pointcloud_->points[i].y > -0.2){
    //         local_pointcloud_->points.erase(local_pointcloud_->points.begin() + i);
    //         i--;
    //     }
    // }

    // since projection has changed the order of the points, we need to sort the points
    // sort the points based on the angle of the point to the origin
    // first get the angle of each point to the origin
    std::vector<double> angles;
    for (int i = 0; i < local_pointcloud_->points.size(); i++){
        angles.push_back(atan2(local_pointcloud_->points[i].y, local_pointcloud_->points[i].x));
    }
    // based on the angles, sort the points
    std::vector<int> indices(local_pointcloud_->points.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&angles](int i1, int i2) {return angles[i1] < angles[i2];});
    // reorder the points
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < local_pointcloud_->points.size(); i++){
        temp_cloud->points.push_back(local_pointcloud_->points[indices[i]]);
    }
    local_pointcloud_->points.clear();
    for (int i = 0; i < temp_cloud->points.size(); i++){
        local_pointcloud_->points.push_back(temp_cloud->points[i]);
    }   

    // uniform sample the points
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    // uniform_sampling.setInputCloud(local_pointcloud_);
    // uniform_sampling.setRadiusSearch(0.001f);
    // pcl::PointCloud<int> sampled_indices;
    // // uniform_sampling.compute(sampled_indices);
    // // pcl::copyPointCloud(*local_pointcloud_, sampled_indices.points, *temp_cloud2);
    // uniform_sampling.filter(*temp_cloud2);
    // local_pointcloud_->points.clear();
    // for (int i = 0; i < temp_cloud2->points.size(); i++){
    //     local_pointcloud_->points.push_back(temp_cloud2->points[i]);
    // }

    // t2 = ros::WallTime::now();
    // ROS_INFO("[Local map]: local OccMap process time: %f ms", (t2 - t1).toSec() * 1000);

    // first get the interest points in xy plane and prune the similar rays
    getInterestingPointsFromPointCloud(local_pointcloud_, md_.interest_pts_xy_, md_.num_of_interest_pts_xy_);
    // find out if the line segment between odom and goal will intersect with obstacles, if not add it to the interest points
    // all the points are in the base frame, we need to get the goal_pos_ in the base frame
    Eigen::Vector3d goal_pos_base;
    Eigen::Vector4f goal_pos_odom, goal_pos_base_homogeneous;
    goal_pos_odom(0) = goal_pos_(0);
    goal_pos_odom(1) = goal_pos_(1);
    goal_pos_odom(2) = goal_pos_(2);
    goal_pos_odom(3) = 1;
    goal_pos_base_homogeneous = odom_to_base_matrix.inverse() * goal_pos_odom;
    goal_pos_base(0) = goal_pos_base_homogeneous(0);
    goal_pos_base(1) = goal_pos_base_homogeneous(1);
    goal_pos_base(2) = goal_pos_base_homogeneous(2);

    if (!isRayIntersectWithObstacle(Eigen::Vector3d(0, 0, 0), goal_pos_base)){
// std::cout << "current to goal is clear" << std::endl;
// std::cout << "goal_pos_base: " << goal_pos_base.transpose() << std::endl;
        if(goal_pos_base.head(2).norm() < 2){
            md_.interest_pts_xy_.push_back(goal_pos_base);
        }
    }

    for (int i = 0; i < md_.interest_pts_xy_.size(); i++){
        pcl::PointXYZ pt;
        pt.x = md_.interest_pts_xy_[i](0);
        pt.y = md_.interest_pts_xy_[i](1);
        pt.z = md_.interest_pts_xy_[i](2);
        md_.test_pts_for_3D_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }
// std::cout << "size of test_pts_for_3D_: " << md_.test_pts_for_3D_.size() << std::endl;

    // if no interest points, return
    if (md_.interest_pts_xy_.size() == 0)
    {
// std::cout << "no interest points" << std::endl;
        return;
    }

// std::cout << "interest points size: " << md_.interest_pts_xy_.size() << std::endl;
    pruneSimilarRaysInxyPlane(md_.interest_pts_xy_, md_.interest_pts_xy_pruned_, 3.13, md_.pts_for_xz_);
// std::cout << "interest points pruned size: " << md_.interest_pts_xy_pruned_.size() << std::endl;
// std::cout << "pts_for_xz_ size: " << md_.pts_for_xz_.size() << std::endl;
    // if (md_.interest_pts_xy_pruned_.size() == 0 && !isRayIntersectWithObstacle(Eigen::Vector3d(0, 0, 0), goal_pos_base))
    // {
    //     md_.interest_pts_xy_pruned_.push_back(goal_pos_base);
    // }

    if (!isRayIntersectWithObstacle(Eigen::Vector3d(0, 0, 0), goal_pos_base))
    {
        if(goal_pos_base.head(2).norm() < 2){
            md_.interest_pts_xy_pruned_.push_back(goal_pos_base);
        }
    }

    // second along each ray, we get a plane that is perpendicular to the xy plane and get the interest points again
    // md_.interest_pts_xz_ground_check_.clear();
    // md_.interest_pts_xz_pruned_.clear();
    // for (int i = 0; i < md_.pts_for_xz_.size(); i++){
    //     md_.interest_pts_xz_.clear();

    //     // transfer the point cloud to a frame that the x axis is the ray direction
    //     double angle = atan2(md_.pts_for_xz_[i](1), md_.pts_for_xz_[i](0));
    //     Eigen::Matrix3f rot_matrix;
    //     rot_matrix << cos(angle), -sin(angle), 0,
    //                   sin(angle), cos(angle), 0,
    //                   0, 0, 1;
    //     Eigen::Matrix4f rot_matrix_4f = Eigen::Matrix4f::Identity();
    //     rot_matrix_4f.block<3, 3>(0, 0) = rot_matrix;
    //     // transfer the point cloud to base frame and then to the rotated frame
    //     rotated_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    //     // pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::transformPointCloud(*md_.fused_cloud_ptr_, *temp_cloud, odom_to_base_matrix.inverse());
    //     pcl::transformPointCloud(*temp_cloud, *rotated_pointcloud_, rot_matrix_4f.inverse());
    //     slicePointCloudInxzPlane(rotated_pointcloud_);
        
    //     //put different layer into one layer

    //     for (int j = 0; j < rotated_pointcloud_->points.size(); j++){
    //         rotated_pointcloud_->points[j].y = 0;
    //     }
    //     // since projection has changed the order of the points, we need to sort the points
    //     // sort the points based on the angle of the point to the origin
    //     // first get the angle of each point to the origin
    //     std::vector<double> angles;
    //     for (int j = 0; j < rotated_pointcloud_->points.size(); j++){
    //         angles.push_back(atan2(rotated_pointcloud_->points[j].z, rotated_pointcloud_->points[j].x));
    //     }
    //     // based on the angles, sort the points
    //     std::vector<int> indices(rotated_pointcloud_->points.size());
    //     std::iota(indices.begin(), indices.end(), 0);
    //     std::sort(indices.begin(), indices.end(), [&angles](int i1, int i2) {return angles[i1] < angles[i2];});
    //     // reorder the points
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    //     for (int j = 0; j < rotated_pointcloud_->points.size(); j++){
    //         temp_cloud2->points.push_back(rotated_pointcloud_->points[indices[j]]);
    //     }
    //     rotated_pointcloud_->points.clear();
    //     for (int j = 0; j < temp_cloud2->points.size(); j++){
    //         rotated_pointcloud_->points.push_back(temp_cloud2->points[j]);
    //     }

    //     // get the interest points in the rotated frame
    //     getInterestingPointsFromPointCloud(rotated_pointcloud_, md_.interest_pts_xz_, md_.num_of_interest_pts_xz_);

    //     // // transform the rotated_cloud to the base frame
    //     pcl::transformPointCloud(*rotated_pointcloud_, *filter_cloud, rot_matrix_4f);
    //     rotated_pointcloud_->points.clear();
    //     for (int j = 0; j < filter_cloud->points.size(); j++){
    //         rotated_pointcloud_->points.push_back(filter_cloud->points[j]);
    //     }

    //     // the point in md_.interest_pts_xz_ is in the rotated frame, first change it back to base frame
    //     for (int j = 0; j < md_.interest_pts_xz_.size(); j++){
    //         Eigen::Vector3d pt = md_.interest_pts_xz_[j];
    //         Eigen::Vector4f pt_base;
    //         pt_base(0) = pt(0) * cos(angle) - pt(1) * sin(angle);
    //         pt_base(1) = pt(0) * sin(angle) + pt(1) * cos(angle);
    //         pt_base(2) = pt(2);
    //         pt_base(3) = 1;
    //         // change the pt_base to the odom frame
    //         Eigen::Vector4f pt_odom;
    //         pt_odom = odom_to_base_matrix * pt_base;
    //         Eigen::Vector3d pt_odom_3d;
    //         pt_odom_3d(0) = pt_odom(0);
    //         pt_odom_3d(1) = pt_odom(1);
    //         pt_odom_3d(2) = pt_odom(2);
    //         if(pt_odom(2) > 0.1){
    //             md_.interest_pts_xz_ground_check_.push_back(pt_odom_3d);
    //         }
    //     }
        // change the md_.interest_pts_xy_pruned_[i] from the base frame to the rotated frame
        // Eigen::Vector4f interest_pts_rotated;
        // interest_pts_rotated = rot_matrix_4f.inverse() * Eigen::Vector4f(md_.interest_pts_xy_pruned_[i](0), md_.interest_pts_xy_pruned_[i](1), md_.interest_pts_xy_pruned_[i](2), 1);
        // md_.interest_pts_xz_ground_check_.push_back(Eigen::Vector3d(interest_pts_rotated(0), interest_pts_rotated(1), interest_pts_rotated(2)));
        
        // // for (int j = 0; j < md_.interest_pts_xz_ground_check_.size(); j++){
        // //     // change the point from rotate frame to base frame
        // //     Eigen::Vector3d pt = md_.interest_pts_xz_ground_check_[j];
        // //     Eigen::Vector3d pt_base;
        // //     pt_base(0) = pt(0) * cos(angle) - pt(1) * sin(angle);
        // //     pt_base(1) = pt(0) * sin(angle) + pt(1) * cos(angle);
        // //     pt_base(2) = pt(2);
        // //     md_.interest_pts_xz_unpruned_.push_back(pt_base);
        // // }
        // // for (int j = 0; j < md_.interest_pts_xz_ground_check_.size(); j++){
        // //     md_.interest_pts_xz_pruned_.push_back(md_.interest_pts_xz_ground_check_[j]);
        // // }
        // // md_.interest_pts_xz_pruned_.clear();
        // pruneSimilarRaysInxzPlane(md_.interest_pts_xz_ground_check_, md_.interest_pts_xz_pruned_, 1.57);

        // // the interest points in the rotated frame are transferred back to the base frame
        // for (int j = 0; j < md_.interest_pts_xz_pruned_.size(); j++){
        //     Eigen::Vector3d pt = md_.interest_pts_xz_pruned_[j];
        //     Eigen::Vector3d pt_rotated;
        //     pt_rotated(0) = pt(0) * cos(angle) - pt(1) * sin(angle);
        //     pt_rotated(1) = pt(0) * sin(angle) + pt(1) * cos(angle);
        //     pt_rotated(2) = pt(2);
        //     md_.interest_pts_xyz_.push_back(pt_rotated);
        // }
    // }


#ifdef DEBUG
    // After processing, we need to transfer back.
    pcl::transformPointCloud(*filter_cloud, *filter_cloud, odom_to_base_matrix);
    sensor_msgs::PointCloud2 debug_cloud_msg;
    filter_cloud->header.frame_id = "map";
    filter_cloud->width = filter_cloud->points.size();
    filter_cloud->height = 1;
    filter_cloud->is_dense = true;
    pcl::toROSMsg(*filter_cloud, debug_cloud_msg);
    debug_pub_.publish(debug_cloud_msg);
#endif
}

void GridMap::slicePointCloudInxyPlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud)
{
    // apply a passthrough filter to the point cloud
// std::cout << "size of point cloud before filter: " << point_cloud->points.size() << std::endl;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(point_cloud);
    // filter the point cloud at a height
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.2, 0.2);
    pass.filter(*point_cloud);
    // filter the point cloud at a distance in x
    pass.setFilterFieldName("x");
    pass.setFilterLimits(mp_.local_x_range_(0), mp_.local_x_range_(1));
    pass.filter(*point_cloud);
    // filter the point cloud at a distance in y
    pass.setFilterFieldName("y");
    pass.setFilterLimits(mp_.local_y_range_(0), mp_.local_y_range_(1));
    pass.filter(*point_cloud);

// std::cout << "size of point cloud after filter: " << point_cloud->points.size() << std::endl;
}

void GridMap::slicePointCloudInxzPlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud)
{
    // apply a passthrough filter to the point cloud
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(point_cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(mp_.local_z_range_(0), mp_.local_z_range_(1));
    pass.filter(*point_cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.1, 0.1);
    pass.filter(*point_cloud);
    // filter the point cloud, since the ray has a direction, only half plane is needed
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, mp_.local_x_range_(1));
    pass.filter(*point_cloud);
}

void GridMap::projectPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud)
{
    pcl::PointXYZ pt;
    Eigen::Vector3d p2d, p2d_inf;
    Eigen::Vector3i inf_pt, pt_2D_slice_;

    // double inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    double inf_step = 0;

// std::cout << "point_cloud->points.size(): " << point_cloud->points.size() << std::endl;
    for (size_t i = 0; i < point_cloud->points.size(); ++i)
    {
        // current point
        pt = point_cloud->points[i];
        p2d(0) = pt.x, p2d(1) = pt.y, p2d(2) = pt.z;
        // convert to index

        if (!isInLocalMap(p2d))
            continue;

        for (int x = -inf_step; x <= inf_step; ++x)
        {
            for (int y = -inf_step; y <= inf_step; ++y)
            {    
                for (int z = -inf_step; z <= inf_step; ++z)
                {
                    p2d_inf(0) = p2d(0) + x * mp_.resolution_;
                    p2d_inf(1) = p2d(1) + y * mp_.resolution_;
                    p2d_inf(2) = p2d(2) + z * mp_.resolution_;
                    if (!isInLocalMap(p2d_inf))
                        continue;

                    localPosToIndex(p2d_inf, inf_pt);
                    md_.local_occupancy_buffer_[toLocalAddress(inf_pt)] = mp_.clamp_max_log_;
                    md_.local_occupancy_buffer_inflate_[toLocalAddress(inf_pt)] = 1; // is used to construct the local distance map

                    // obtain a slice of the point cloud at the height of the robot
                    if (p2d_inf(2) > -0.025 && p2d_inf(2) < 0.025)
                    {
                        localPosToIndex(p2d_inf, pt_2D_slice_);
                        md_.local_occupancy_buffer_inflate_2D_slice_[toLocalAddress(pt_2D_slice_)] = 1;
                    }
                }
            }
        }
    }
    // int numOfOne = 0;
    // update map
    // for(size_t i = 0; i < md_.local_occupancy_buffer_inflate_.size(); i++){
    //     md_.local_occupancy_buffer_inflate_for_JPS_3D[i] = md_.local_occupancy_buffer_inflate_[i]*100;
    //     // if(md_.local_occupancy_buffer_inflate_[i] == 1){
    //     //     numOfOne++;
    //     // }
    // }

    // Eigen::Matrix<decimal_t, 3, 1> origin_test;
    // Eigen::Vector3i map_size_test;
    // origin_test = mp_.map_origin_.cast<decimal_t>(); 
    // origin_test[0] += mp_.odom_transform_ptr_->transform.translation.x;
    // origin_test[1] += mp_.odom_transform_ptr_->transform.translation.y;
    // origin_test[2] += mp_.odom_transform_ptr_->transform.translation.z;
    // map_size_test = mp_.local_map_voxel_num_.cast<int>();
    // md_.map_util->setMap(origin_test, map_size_test, md_.local_occupancy_buffer_inflate_for_JPS_3D, mp_.resolution_);

}

void GridMap::getInterestingPointsFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<Eigen::Vector3d> &pts, int num_of_interest_pts){
    // get cloud curvature of the point cloud based on 10 neghbors
    md_.cloud_curvature_.clear();
    md_.cloud_neighbor_picked_.clear();
    md_.cloud_sort_ind_.clear();
    computeCurvature(cloud, md_.cloud_curvature_);
    sortCloudBasedOnCurvature(cloud, md_.cloud_curvature_, md_.cloud_sort_ind_);
    pickCornerPoints(cloud, pts, num_of_interest_pts);
}

void GridMap::computeCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<double> &curvature){
    // compute the curvature of the point cloud (10 to the left and 10 to the right)
    // for (int i = 10; i < cloud->points.size() - 10; i++)
    // {
    //     float diffX = cloud->points[i - 10].x + cloud->points[i - 9].x + cloud->points[i - 8].x + cloud->points[i - 7].x + cloud->points[i - 6].x +\
    //                     cloud->points[i - 5].x + cloud->points[i - 4].x + cloud->points[i - 3].x + cloud->points[i - 2].x + cloud->points[i - 1].x - 20*cloud->points[i].x +\
    //                     cloud->points[i + 1].x + cloud->points[i + 2].x + cloud->points[i + 3].x + cloud->points[i + 4].x + cloud->points[i + 5].x + cloud->points[i + 6].x +\
    //                     cloud->points[i + 7].x + cloud->points[i + 8].x + cloud->points[i + 9].x + cloud->points[i + 10].x;
    //     float diffY = cloud->points[i - 10].y + cloud->points[i - 9].y + cloud->points[i - 8].y + cloud->points[i - 7].y + cloud->points[i - 6].y +\
    //                     cloud->points[i - 5].y + cloud->points[i - 4].y + cloud->points[i - 3].y + cloud->points[i - 2].y + cloud->points[i - 1].y - 20*cloud->points[i].y +\
    //                     cloud->points[i + 1].y + cloud->points[i + 2].y + cloud->points[i + 3].y + cloud->points[i + 4].y + cloud->points[i + 5].y + cloud->points[i + 6].y +\
    //                     cloud->points[i + 7].y + cloud->points[i + 8].y + cloud->points[i + 9].y + cloud->points[i + 10].y;
    //     float diffZ = cloud->points[i - 10].z + cloud->points[i - 9].z + cloud->points[i - 8].z + cloud->points[i - 7].z + cloud->points[i - 6].z +\
    //                     cloud->points[i - 5].z + cloud->points[i - 4].z + cloud->points[i - 3].z + cloud->points[i - 2].z + cloud->points[i - 1].z - 20*cloud->points[i].z +\
    //                     cloud->points[i + 1].z + cloud->points[i + 2].z + cloud->points[i + 3].z + cloud->points[i + 4].z + cloud->points[i + 5].z + cloud->points[i + 6].z +\
    //                     cloud->points[i + 7].z + cloud->points[i + 8].z + cloud->points[i + 9].z + cloud->points[i + 10].z;
    //     md_.cloud_curvature_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    //     md_.cloud_neighbor_picked_[i] = 0;
    //     md_.cloud_sort_ind_[i] = i;
    // }
    // // get the curvature of the first 10 points and the last 10 points
    // // consider the sequence of the points into a loop
    // for (int i = 0; i < 10; i++){
    //     float diffX = cloud->points[i].x + cloud->points[i + 1].x + cloud->points[i + 2].x + cloud->points[i + 3].x + cloud->points[i + 4].x +\
    //                     cloud->points[i + 5].x + cloud->points[i + 6].x + cloud->points[i + 7].x + cloud->points[i + 8].x + cloud->points[i + 9].x - 20*cloud->points[i].x ;
    //     float diffY = cloud->points[i].y + cloud->points[i + 1].y + cloud->points[i + 2].y + cloud->points[i + 3].y + cloud->points[i + 4].y +\
    //                     cloud->points[i + 5].y + cloud->points[i + 6].y + cloud->points[i + 7].y + cloud->points[i + 8].y + cloud->points[i + 9].y - 20*cloud->points[i].y ;
    //     float diffZ = cloud->points[i].z + cloud->points[i + 1].z + cloud->points[i + 2].z + cloud->points[i + 3].z + cloud->points[i + 4].z +\
    //                     cloud->points[i + 5].z + cloud->points[i + 6].z + cloud->points[i + 7].z + cloud->points[i + 8].z + cloud->points[i + 9].z - 20*cloud->points[i].z ;
    //     // base on the index of the point, add certain points from the end of the point cloud to make sure 10 points are considered
    //     for (int j = 0; j < 10 - i; j++){
    //         diffX += cloud->points[cloud->points.size() - 1 - j].x;
    //         diffY += cloud->points[cloud->points.size() - 1 - j].y;
    //         diffZ += cloud->points[cloud->points.size() - 1 - j].z;
    //     }
    //     for (int j = 0; j < i; j++){
    //         diffX += cloud->points[j].x;
    //         diffY += cloud->points[j].y;
    //         diffZ += cloud->points[j].z;
    //     }
    //     md_.cloud_curvature_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    //     md_.cloud_neighbor_picked_[i] = 0;
    //     md_.cloud_sort_ind_[i] = i;
    // }
    // for(int i = 0; i < 10; i++){
    //     float diffX = cloud->points[cloud->points.size() - 1 - i].x + cloud->points[cloud->points.size() - 2 - i].x + cloud->points[cloud->points.size() - 3 - i].x + cloud->points[cloud->points.size() - 4 - i].x + cloud->points[cloud->points.size() - 5 - i].x +\
    //                     cloud->points[cloud->points.size() - 6 - i].x + cloud->points[cloud->points.size() - 7 - i].x + cloud->points[cloud->points.size() - 8 - i].x + cloud->points[cloud->points.size() - 9 - i].x + cloud->points[cloud->points.size() - 10 - i].x - 20*cloud->points[cloud->points.size() - 1 - i].x ;
    //     float diffY = cloud->points[cloud->points.size() - 1 - i].y + cloud->points[cloud->points.size() - 2 - i].y + cloud->points[cloud->points.size() - 3 - i].y + cloud->points[cloud->points.size() - 4 - i].y + cloud->points[cloud->points.size() - 5 - i].y +\
    //                     cloud->points[cloud->points.size() - 6 - i].y + cloud->points[cloud->points.size() - 7 - i].y + cloud->points[cloud->points.size() - 8 - i].y + cloud->points[cloud->points.size() - 9 - i].y + cloud->points[cloud->points.size() - 10 - i].y - 20*cloud->points[cloud->points.size() - 1 - i].y ;
    //     float diffZ = cloud->points[cloud->points.size() - 1 - i].z + cloud->points[cloud->points.size() - 2 - i].z + cloud->points[cloud->points.size() - 3 - i].z + cloud->points[cloud->points.size() - 4 - i].z + cloud->points[cloud->points.size() - 5 - i].z +\
    //                     cloud->points[cloud->points.size() - 6 -i].z + cloud->points[cloud->points.size() - 7 - i].z + cloud->points[cloud->points.size() - 8 - i].z + cloud->points[cloud->points.size() - 9 - i].z + cloud->points[cloud->points.size() - 10 - i].z - 20*cloud->points[cloud->points.size() - 1 - i].z ;
    //     // base on the index of the point, add certain points from the begin of the point cloud to make sure 10 points are considered
    //     for (int j = 0; j < 10 - i; j++){
    //         diffX += cloud->points[j].x;
    //         diffY += cloud->points[j].y;
    //         diffZ += cloud->points[j].z;
    //     }
    //     for (int j = 0; j < i; j++){
    //         diffX += cloud->points[cloud->points.size() - 1 - j].x;
    //         diffY += cloud->points[cloud->points.size() - 1 - j].y;
    //         diffZ += cloud->points[cloud->points.size() - 1 - j].z;
    //     }
    //     md_.cloud_curvature_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    //     md_.cloud_neighbor_picked_[i] = 0;
    //     md_.cloud_sort_ind_[i] = i;        
    // }

    for (int i = 5; i < cloud->points.size() - 5; i++)
    {
        float diffX = cloud->points[i - 5].x + cloud->points[i - 4].x + cloud->points[i - 3].x + cloud->points[i - 2].x + cloud->points[i - 1].x - 10*cloud->points[i].x + cloud->points[i + 1].x + cloud->points[i + 2].x + cloud->points[i + 3].x + cloud->points[i + 4].x + cloud->points[i + 5].x;
        float diffY = cloud->points[i - 5].y + cloud->points[i - 4].y + cloud->points[i - 3].y + cloud->points[i - 2].y + cloud->points[i - 1].y - 10*cloud->points[i].y + cloud->points[i + 1].y + cloud->points[i + 2].y + cloud->points[i + 3].y + cloud->points[i + 4].y + cloud->points[i + 5].y;
        float diffZ = cloud->points[i - 5].z + cloud->points[i - 4].z + cloud->points[i - 3].z + cloud->points[i - 2].z + cloud->points[i - 1].z - 10*cloud->points[i].z + cloud->points[i + 1].z + cloud->points[i + 2].z + cloud->points[i + 3].z + cloud->points[i + 4].z + cloud->points[i + 5].z;

        md_.cloud_curvature_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        md_.cloud_neighbor_picked_[i] = 0;
        md_.cloud_sort_ind_[i] = i;
    }
    // get the curvature of the first 5 points and the last 5 points
    // consider the sequence of the points into a loop
    for (int i = 0; i < 5; i++){
        float diffX = cloud->points[i].x + cloud->points[i + 1].x + cloud->points[i + 2].x + cloud->points[i + 3].x + cloud->points[i + 4].x - 10*cloud->points[i].x ;
        float diffY = cloud->points[i].y + cloud->points[i + 1].y + cloud->points[i + 2].y + cloud->points[i + 3].y + cloud->points[i + 4].y - 10*cloud->points[i].y ;
        float diffZ = cloud->points[i].z + cloud->points[i + 1].z + cloud->points[i + 2].z + cloud->points[i + 3].z + cloud->points[i + 4].z - 10*cloud->points[i].z ;
        // base on the index of the point, add certain points from the end of the point cloud to make sure 5 points are considered
        for (int j = 0; j < 5 - i; j++){
            diffX += cloud->points[cloud->points.size() - 1 - j].x;
            diffY += cloud->points[cloud->points.size() - 1 - j].y;
            diffZ += cloud->points[cloud->points.size() - 1 - j].z;
        }
        for (int j = 0; j < i; j++){
            diffX += cloud->points[j].x;
            diffY += cloud->points[j].y;
            diffZ += cloud->points[j].z;
        }
        md_.cloud_curvature_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        md_.cloud_neighbor_picked_[i] = 0;
        md_.cloud_sort_ind_[i] = i;
    }
    for(int i = 0; i < 5; i++){
        float diffX = cloud->points[cloud->points.size() - 1 - i].x + cloud->points[cloud->points.size() - 2 - i].x + cloud->points[cloud->points.size() - 3 - i].x + cloud->points[cloud->points.size() - 4 - i].x + cloud->points[cloud->points.size() - 5 - i].x - 10*cloud->points[cloud->points.size() - 1 - i].x ;
        float diffY = cloud->points[cloud->points.size() - 1 - i].y + cloud->points[cloud->points.size() - 2 - i].y + cloud->points[cloud->points.size() - 3 - i].y + cloud->points[cloud->points.size() - 4 - i].y + cloud->points[cloud->points.size() - 5 - i].y - 10*cloud->points[cloud->points.size() - 1 - i].y ;
        float diffZ = cloud->points[cloud->points.size() - 1 - i].z + cloud->points[cloud->points.size() - 2 - i].z + cloud->points[cloud->points.size() - 3 - i].z + cloud->points[cloud->points.size() - 4 - i].z + cloud->points[cloud->points.size() - 5 - i].z - 10*cloud->points[cloud->points.size() - 1 - i].z ;
        // base on the index of the point, add certain points from the begin of the point cloud to make sure 5 points are considered
        for (int j = 0; j < 5 - i; j++){
            diffX += cloud->points[j].x;
            diffY += cloud->points[j].y;
            diffZ += cloud->points[j].z;
        }
        for (int j = 0; j < i; j++){
            diffX += cloud->points[cloud->points.size() - 1 - j].x;
            diffY += cloud->points[cloud->points.size() - 1 - j].y;
            diffZ += cloud->points[cloud->points.size() - 1 - j].z;
        }
        md_.cloud_curvature_[cloud->points.size() - 1 - i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        md_.cloud_neighbor_picked_[cloud->points.size() - 1 - i] = 0;
        md_.cloud_sort_ind_[cloud->points.size() - 1 - i] = cloud->points.size() - 1 - i;
    }
    md_.length_of_cloud_curvature_ = cloud->points.size();
// std::cout << "length_of_cloud_curvature_: " << md_.length_of_cloud_curvature_ << std::endl;
}

void GridMap::sortCloudBasedOnCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<double> &curvature, std::vector<int> &sort_ind){
    // sort cloud_sort_ind_ based on cloud_curvature_
    std::sort(sort_ind.begin(), sort_ind.end(), [&](const int &ind1, const int &ind2) { return curvature[ind1] > curvature[ind2]; });
}

void GridMap::pickCornerPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<Eigen::Vector3d> &pts, int count){
    int largestPickedNum = 0;
// std::cout << "largest cloud curvature: " << md_.cloud_curvature_[md_.cloud_sort_ind_[0]] << std::endl;
    for (int k =  0; k <= md_.length_of_cloud_curvature_; k++){
        int ind = md_.cloud_sort_ind_[k];
        if (md_.cloud_neighbor_picked_[ind] == 0 && md_.cloud_curvature_[ind] > 0.85){
            largestPickedNum++;
            if (largestPickedNum <= count){
                Eigen::Vector3d pt;
                pt(0) = cloud->points[ind].x;
                pt(1) = cloud->points[ind].y;
                pt(2) = cloud->points[ind].z;
                pts.push_back(pt);
            }
            else{
                break;
            }
            md_.cloud_neighbor_picked_[ind] = 1;

            for (int l = 1; l <= 50; l++){
                float diffX = cloud->points[ind + l].x - cloud->points[ind + l - 1].x;
                float diffY = cloud->points[ind + l].y - cloud->points[ind + l - 1].y;
                float diffZ = cloud->points[ind + l].z - cloud->points[ind + l - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.0025)
                {
                    break;
                }
                md_.cloud_neighbor_picked_[ind + l] = 1;
            }
            for (int l = -1; l >= -50; l--)
            {
                float diffX = cloud->points[ind + l].x - cloud->points[ind + l + 1].x;
                float diffY = cloud->points[ind + l].y - cloud->points[ind + l + 1].y;
                float diffZ = cloud->points[ind + l].z - cloud->points[ind + l + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.0025)
                {
                    break;
                }

                md_.cloud_neighbor_picked_[ind + l] = 1;
            }
        }
    }
}

void GridMap::pruneSimilarRaysInxyPlane(const std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &pruned_pts, double angle_threshold, std::vector<Eigen::Vector3d> &pts_for_xy){
    int num_of_sampled_pts = 50;
    // based on the  x, y and z range of the local map, the farest obstacle that can be detected can be calculated
    double max_dist = sqrt(pow(mp_.map_size_(0)/2, 2) + pow(mp_.map_size_(1)/2, 2) + pow(mp_.map_size_(2)/2, 2));
    bool obstacle_flag = false;
    Eigen::Vector3d start;
    start(0) = 0;
    start(1) = 0;
    start(2) = 0;
    // first compute the angle between every ray with x aixs
    std::vector<double> angles;
    std::vector<int> angle_ind;
    for (int i = 0; i < pts.size(); i++){
        Eigen::Vector3d pt = pts[i];
        double angle = atan2(pt(1), pt(0));
        angles.push_back(angle);
        angle_ind.push_back(i);
    }
    // sort the angle_ind base on angles so that the ray is in right order
    std::sort(angle_ind.begin(), angle_ind.end(), [&](const int &ind1, const int &ind2) { return angles[ind1] > angles[ind2]; });

// for (int i = 0; i < angle_ind.size(); i++){
//     std::cout << "angle[" << i << "]: " << angles[angle_ind[i]] << std::endl;
// }

    std::vector<double> angle_for_pruned_pts;
    // when push back ray, make sure its angle between itself and last ray is bigger than a threshold(3 degree)
    // equally sample num_of_sampled_pts points between each ray
    for (int i = 0; i < pts.size() - 1; ++i){
        double angle_err = angles[angle_ind[i]] - angles[angle_ind[i+1]];
        // angle between two rays is less than 0.04(about 5 degree), means ray pass two obstacles
        if (angle_err < 0.05){
            // push back the ray with longer distance
            // add some bias, so that the ray can be used to generate polygon
            // the distance of the new point must equal to the distance of longer ray
// std::cout << "angle err too small" << std::endl;
            double angle_bias = 0.15;
            double angle;
            Eigen::Vector3d pt;
            if (pts[angle_ind[i]].norm() > pts[angle_ind[i+1]].norm()){
                angle = angles[angle_ind[i]] + angle_bias * (angle_err)/abs(angle_err);
                pt(0) = (pts[angle_ind[i]].norm() + pts[angle_ind[i+1]].norm()) * cos(angle) / 2;
                pt(1) = (pts[angle_ind[i]].norm() + pts[angle_ind[i+1]].norm()) * sin(angle) / 2;
                pt(2) = pts[angle_ind[i]](2);
            }
            else{
                angle = angles[angle_ind[i+1]] - angle_bias * (angle_err)/abs(angle_err);
                pt(0) = (pts[angle_ind[i]].norm() + pts[angle_ind[i+1]].norm()) * cos(angle) / 2;
                pt(1) = (pts[angle_ind[i]].norm() + pts[angle_ind[i+1]].norm()) * sin(angle) / 2;
                pt(2) = pts[angle_ind[i+1]](2);
            }
            if (angle_for_pruned_pts.size() > 0){
                if(angle_for_pruned_pts[angle_for_pruned_pts.size() - 1] - angle > 0.5){
                    angle_for_pruned_pts.push_back(angle);
                    // Eigen::Vector3d pt1;
                    // pt1 = (pt + pruned_pts[pruned_pts.size() - 1]) / 2;
                    pruned_pts.push_back(pt);
                }
            }
            else{
                angle_for_pruned_pts.push_back(angle);
                pruned_pts.push_back(pt);
            }
            continue;
        }
        // if (angle_err > M_PI){
        //     angle_err = 2 * M_PI - angle_err;
        // }
        // find num_of_sampled_pts points between two rays
        if (angle_err < 0.18){
            num_of_sampled_pts = 6;
        }
        else{
            num_of_sampled_pts = 10;
        }
        for (int j = 2; j < num_of_sampled_pts -  1; ++j){
            double angle = angles[angle_ind[i]] - j * angle_err / num_of_sampled_pts;
            Eigen::Vector3d pt;
            pt(0) = max_dist * cos(angle);
            pt(1) = max_dist * sin(angle);
            pt(2) = pts[angle_ind[i]](2);

            if(isRayIntersectWithObstacle(start, pt)){
                obstacle_flag = true;
// std::cout << "obstacle" << std::endl;
                break;
                // obstacle between two rays
            }
        }
        // free region between two rays
        if (!obstacle_flag && angle_err < angle_threshold && (pts[angle_ind[i]] - pts[angle_ind[i+1]]).norm() > 0.2){
// std::cout << "no obstacle" << std::endl;
            Eigen::Vector3d pt;
            double angle = angles[angle_ind[i]] - angle_err / 2;
            pt(0) = max_dist * cos(angle);
            pt(1) = max_dist * sin(angle);
            pt(2) = pts[angle_ind[i]](2);
            if (angle_for_pruned_pts.size() > 0){
                if(angle_for_pruned_pts[angle_for_pruned_pts.size() - 1] - angle > 0.5){
                    angle_for_pruned_pts.push_back(angle);
                    // Eigen::Vector3d pt1;
                    // pt1 = (pt + pruned_pts[pruned_pts.size() - 1]) / 2;
                    // pruned_pts.push_back(pt1);                    
                    pruned_pts.push_back(pt);
                }
            }
            else{
                angle_for_pruned_pts.push_back(angle);
                pruned_pts.push_back(pt);
            }
        }
        else if (!obstacle_flag && angle_err >= angle_threshold){
            // add some bias, so that the the two rays do not intersect with the obstacle
// std::cout << "no obstacle and big error" << std::endl;
            double bias_angle = angle_err / 10;
            Eigen::Vector3d pt;
            double angle = angles[angle_ind[i]] - bias_angle;
            pt(0) = max_dist * cos(angle);
            pt(1) = max_dist * sin(angle);
            pt(2) = pts[angle_ind[i]](2);
            pruned_pts.push_back(pt);
            angle_for_pruned_pts.push_back(angle);
            angle = angles[angle_ind[i+1]] + bias_angle;
            pt(0) = max_dist * cos(angle);
            pt(1) = max_dist * sin(angle);
            pt(2) = pts[angle_ind[i+1]](2);
            pruned_pts.push_back(pt);
            angle_for_pruned_pts.push_back(angle);
        }
        else if(obstacle_flag){
            // obstalce between two rays, check if can go up or down to overcome the obstalce
            // record the ray that lie between the two rays
            Eigen::Vector3d pt;
            double angle = angles[angle_ind[i]] - angle_err / 2;
            pt(0) = max_dist * cos(angle);
            pt(1) = max_dist * sin(angle);
            pt(2) = pts[angle_ind[i]](2);
            pts_for_xy.push_back(pt);
        }

        obstacle_flag = false;
    }
    // compare first and last ray
    // find num_of_sampled_pts points between two rays in reverse direction

    double angle_err = angles[angle_ind[0]] - angles[angle_ind[angles.size() - 1]];
    angle_err = 2 * M_PI - angle_err;
    for (int j = 1; j < num_of_sampled_pts; ++j){
        double angle = angles[angle_ind[0]] + j * angle_err / num_of_sampled_pts;
        Eigen::Vector3d pt;
        pt(0) = max_dist * cos(angle);
        pt(1) = max_dist * sin(angle);
        pt(2) = pts[angle_ind[0]](2);

        if(isRayIntersectWithObstacle(start, pt)){
// std::cout << "last and first obstacle" << std::endl;
            obstacle_flag = true;
            break;
            // obstacle between two rays
        }
    }

    if(!obstacle_flag){
        if (angle_err < angle_threshold){
// std::cout << "last and first free" << std::endl; 
            Eigen::Vector3d pt;
            double angle = angles[angle_ind[0]] + angle_err / 2;
            pt(0) = max_dist * cos(angle);
            pt(1) = max_dist * sin(angle);
            pt(2) = pts[angle_ind[0]](2);
            pruned_pts.push_back(pt);
        }
        else if(angle_err >= angle_threshold){
// std::cout << "last and first free big error" << std::endl;
            // add some bias, so that the the last and first ray do not intersect with the obstacle
            double bias_angle = angle_err / 10;
            Eigen::Vector3d pt;
            double angle = angles[angle_ind[0]] + bias_angle;
            pt(0) = max_dist * cos(angle);
            pt(1) = max_dist * sin(angle);
            pt(2) = pts[angle_ind[0]](2);
            pruned_pts.push_back(pt);
            
            angle = angles[angle_ind[angles.size() - 1]] - bias_angle;
            pt(0) = max_dist * cos(angle);
            pt(1) = max_dist * sin(angle);
            pt(2) = pts[angle_ind[angles.size() - 1]](2);
            pruned_pts.push_back(pt);
        }
    }
    obstacle_flag = false;
}

void GridMap::pruneSimilarRaysInxzPlane(const std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &pruned_pts, double angle_threshold){
    int num_of_sampled_pts = 20;
    // based on the  x, y and z range of the local map, the farest obstacle that can be detected can be calculated
    double max_dist = sqrt(pow(mp_.map_size_(0)/2, 2) + pow(mp_.map_size_(1)/2, 2) + pow(mp_.map_size_(2)/2, 2));
    bool obstacle_flag = false;
    Eigen::Vector3d start;
    start(0) = 0;
    start(1) = 0;
    start(2) = 0;
    // first compute the angle between every ray with x aixs
    std::vector<double> angles;
    std::vector<int> angle_ind;
    for (int i = 0; i < pts.size(); i++){
        Eigen::Vector3d pt = pts[i];
        double angle = atan2(pt(2), pt(0));
        angles.push_back(angle);
        angle_ind.push_back(i);
    }
// std::cout << "start pruneSimilarRaysInxzPlane" << std::endl;
    // sort the angle_ind base on angles so that the ray is in right order
    std::sort(angle_ind.begin(), angle_ind.end(), [&](const int &ind1, const int &ind2) { return angles[ind1] > angles[ind2]; });
// for (int i = 0; i < angle_ind.size(); i++){
//     std::cout << "angle: " << angles[angle_ind[i]] << std::endl;
// }

    std::vector<double> angle_for_pruned_pts;
    // equally sample num_of_sampled_pts points between each ray
    for(int i = 0; i < pts.size() - 1; ++i){
        double angle_err = angles[angle_ind[i]] - angles[angle_ind[i+1]];
        // angle between two rays is less than 0.02, means ray pass two obstacles
        if (angle_err < 0.05){
            // push back the ray with longer distance
            // add some bias, so that the ray can be used to generate polygon
            // the distance of the new point must equal to the distance of longer ray
            double angle_bias = 0.15;
            double angle;
            Eigen::Vector3d pt;
            if (pts[angle_ind[i]].norm() > pts[angle_ind[i+1]].norm()){
                angle = angles[angle_ind[i]] + angle_bias * (angle_err)/abs(angle_err);
                pt(0) = (pts[angle_ind[i]].norm() + pts[angle_ind[i+1]].norm()) * cos(angle) / 2;
                pt(1) = pts[angle_ind[i]](1);
                pt(2) = (pts[angle_ind[i]].norm() + pts[angle_ind[i+1]].norm()) * sin(angle) / 2;
            }
            else{
                angle = angles[angle_ind[i+1]] - angle_bias * (angle_err)/abs(angle_err);
                pt(0) = (pts[angle_ind[i]].norm() + pts[angle_ind[i+1]].norm()) * cos(angle) / 2;
                pt(1) = pts[angle_ind[i+1]](1);
                pt(2) = (pts[angle_ind[i]].norm() + pts[angle_ind[i+1]].norm()) * sin(angle) / 2;
            }

            if (angle_for_pruned_pts.size() > 0){
                if(angle_for_pruned_pts[angle_for_pruned_pts.size() - 1] - angle > 0.5){
                    angle_for_pruned_pts.push_back(angle);
                    pruned_pts.push_back(pt);
                }
            }
            else{
                angle_for_pruned_pts.push_back(angle);
                pruned_pts.push_back(pt);
            }
        }
        // if (angle_err > M_PI){
        //     angle_err = 2 * M_PI - angle_err;
        // }
        // find num_of_sampled_pts points between two rays
        if (angle_err < 0.18){
            num_of_sampled_pts = 6;
        }
        else{
            num_of_sampled_pts = 10;
        }
        int num_of_contact = 0;
        for (int j = 2; j < num_of_sampled_pts - 1; ++j){
            double angle = angles[angle_ind[i]] - j * angle_err / num_of_sampled_pts;
            Eigen::Vector3d pt;
            pt(0) = max_dist * cos(angle);
            pt(1) = pts[angle_ind[i]](1);
            pt(2) = max_dist * sin(angle);

            if(isRayIntersectWithObstacle(start, pt)){
                num_of_contact++;
                // avoid point cloud's uncertainty which may cause the ray intersect with obstacle
                if (num_of_contact > 1){
                    obstacle_flag = true;
                    break;
                }
                // obstacle between two rays
            }
        }
        // free region between two rays
        if (!obstacle_flag && angle_err < angle_threshold){
            Eigen::Vector3d pt;
            double angle = angles[angle_ind[i]] - angle_err / 2;
            pt(0) = max_dist * cos(angle);
            pt(1) = pts[angle_ind[i]](1);
            pt(2) = max_dist * sin(angle);
            if (angle_for_pruned_pts.size() > 0){
                if(angle_for_pruned_pts[angle_for_pruned_pts.size() - 1] - angle > 0.5){
                    angle_for_pruned_pts.push_back(angle);                   
                    pruned_pts.push_back(pt);
                }
            }
            else{
                angle_for_pruned_pts.push_back(angle);
                pruned_pts.push_back(pt);
            }
        }
        else if (!obstacle_flag && angle_err >= angle_threshold){
            // add some bias, so that the the two rays do not intersect with the obstacle
            double bias_angle = angle_err / 10;
            Eigen::Vector3d pt;
            double angle = angles[angle_ind[i]] - bias_angle;
            pt(0) = max_dist * cos(angle);
            pt(1) = pts[angle_ind[i]](1);
            pt(2) = max_dist * sin(angle);
            pruned_pts.push_back(pt);
            angle_for_pruned_pts.push_back(angle);
            angle = angles[angle_ind[i+1]] + bias_angle;
            pt(0) = max_dist * cos(angle);
            pt(1) = pts[angle_ind[i+1]](1);
            pt(2) = max_dist * sin(angle);
            pruned_pts.push_back(pt);
            angle_for_pruned_pts.push_back(angle);
        }

        obstacle_flag = false;
    }

    if (pruned_pts.size() == 0){
        pruned_pts.push_back(pts[0]);
    }
}

bool GridMap::isRayIntersectWithObstacle(const Eigen::Vector3d &start, const Eigen::Vector3d &end){
    bool first_check = false;
    bool second_check = false;
    bool third_check = false;
    Eigen::Vector3d start1 = start;
    Eigen::Vector3d dir = end - start1;
    double dist = dir.norm();
    int num_of_sampled_pts = int (floor(dist / mp_.resolution_));
    dir.normalize();
    for (int i = 1; i < num_of_sampled_pts; ++i){
        Eigen::Vector3d pt = start1 + i * dist / num_of_sampled_pts * dir;
        if (getFusedDynamicInflateOccupancyInLocalMap(pt)){
            first_check = true;
            break;
        }
    }
    start1(0) += 0.1 * dir(0);
    start1(1) += 0.1 * dir(1);
    start1(2) += 0.1 * dir(2);
    for (int i = 1; i < num_of_sampled_pts; ++i){
        Eigen::Vector3d pt = start1 + i * dist / num_of_sampled_pts * dir;
        if (getFusedDynamicInflateOccupancyInLocalMap(pt)){
            second_check = true;
            break;
        }
    }
    start1(0) += 0.1 * dir(0);
    start1(1) += 0.1 * dir(1);
    start1(2) += 0.1 * dir(2);
    for (int i = 1; i < num_of_sampled_pts; ++i){
        Eigen::Vector3d pt = start1 + i * dist / num_of_sampled_pts * dir;
        if (getFusedDynamicInflateOccupancyInLocalMap(pt)){
            third_check = true;
            break;
        }
    }
    return first_check && second_check && third_check;


    // Eigen::Vector3d dir = end - start;
    // double dist = dir.norm();
    // int num_of_sampled_pts = int (floor(dist / mp_.resolution_));
    // dir.normalize();
    // for (int i = 1; i < num_of_sampled_pts; ++i){
    //     Eigen::Vector3d pt = start + i * dist / num_of_sampled_pts * dir;
    //     if (getFusedDynamicInflateOccupancyInLocalMap(pt)){
    //         return true;
    //     }
    // }
    // return false;
}

// std::shared_ptr<JPS::VoxelMapUtil> GridMap::getMapUtilForJPS3d()
// {
//     return md_.map_util;
// }

void GridMap::goalCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    goal_pos_(0) = msg->pose.position.x;
    goal_pos_(1) = msg->pose.position.y;
    // goal_pos_(2) = goal->pose.position.z;
    goal_pos_(2) = md_.robot_pos_(2);
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
    md_.robot_pos_(0) = odom->pose.pose.position.x;
    md_.robot_pos_(1) = odom->pose.pose.position.y;
    md_.robot_pos_(2) = odom->pose.pose.position.z;
    md_.robot_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

    mp_.odom_transform_ptr_->transform.translation.x = odom->pose.pose.position.x;
    mp_.odom_transform_ptr_->transform.translation.y = odom->pose.pose.position.y;
    mp_.odom_transform_ptr_->transform.translation.z = odom->pose.pose.position.z;
    mp_.odom_transform_ptr_->transform.rotation.x = odom->pose.pose.orientation.x;
    mp_.odom_transform_ptr_->transform.rotation.y = odom->pose.pose.orientation.y;
    mp_.odom_transform_ptr_->transform.rotation.z = odom->pose.pose.orientation.z;
    mp_.odom_transform_ptr_->transform.rotation.w = odom->pose.pose.orientation.w;

    md_.has_odom_ = true;
}

void GridMap::scanCallback(const sensor_msgs::PointCloud2ConstPtr &scan)
{
    // the buffer is refresh based on the slow sensor
    md_.fused_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    /* scan to pcl cloud -----------------------------------*/
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> latest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// std::cout << "In scancallback, the size of point cloud2 is: " << scan->data.size() << std::endl;
    pcl::fromROSMsg(*scan, *latest_cloud);
// std::cout << "In scancallback, after pcl::fromROSMsg the size of point cloud is: " << latest_cloud->points.size() << std::endl;
    if (latest_cloud->points.size() == 0)
        return;

    if (!md_.has_odom_)
    {
        ROS_INFO("[Local map]: waiting for odom");
        return;
    }

    if (isnan(md_.robot_pos_(0)) || isnan(md_.robot_pos_(1)) || isnan(md_.robot_pos_(2)))
        return;

    // transfer the point cloud to the base_link frame
    const Eigen::Affine3d base_to_scan_affine = tf2::transformToEigen(*(mp_.TF_base_to_scan_ptr_));
    const Eigen::Matrix4f base_to_scan_matrix = base_to_scan_affine.matrix().cast<float>();
    pcl::transformPointCloud(*latest_cloud, *latest_cloud, base_to_scan_matrix);

    // transform the point cloud to the map frame
    const Eigen::Matrix4f odom_to_base_matrix = tf2::transformToEigen(*mp_.odom_transform_ptr_).matrix().cast<float>();
    pcl::transformPointCloud(*latest_cloud, *latest_cloud, odom_to_base_matrix);

    *md_.fused_cloud_ptr_ += *latest_cloud;
// std::cout << "In scancallback, in the end the size of point cloud is: " << md_.fused_cloud_ptr_->points.size() << std::endl;
}

void GridMap::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> latest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *latest_cloud);
    // wait for transform from robot_base to laser scan

    if (latest_cloud->points.size() == 0)
        return;

    if (!md_.has_odom_)
        return;

    if (isnan(md_.robot_pos_(0)) || isnan(md_.robot_pos_(1)) || isnan(md_.robot_pos_(2)))
        return;

    // transfer the point cloud to the base_link frame
    const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*(mp_.TF_base_to_sensor_ptr_));
    const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();
    pcl::transformPointCloud(*latest_cloud, *latest_cloud, base_to_sensor_matrix);

    // transform the point cloud to the map frame
    const Eigen::Matrix4f odom_to_base_matrix = tf2::transformToEigen(*mp_.odom_transform_ptr_).matrix().cast<float>();
    pcl::transformPointCloud(*latest_cloud, *latest_cloud, odom_to_base_matrix);

    *md_.fused_cloud_ptr_ += *latest_cloud;

#ifdef DEBUG
    std::cout << min_z_point << " " << max_z_point << std::endl;
    sensor_msgs::PointCloud2 debug_cloud_msg;
    latest_cloud->header.frame_id = "map";
    latest_cloud->width = latest_cloud->points.size();
    latest_cloud->height = 1;
    latest_cloud->is_dense = true;
    pcl::toROSMsg(*latest_cloud, debug_cloud_msg);
    debug_pub_.publish(debug_cloud_msg);
    latest_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
#endif
}

void GridMap::evaluateEDTBoth(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad)
{
    Eigen::Vector3d global_grad, local_grad;
    double global_dist, local_dist;

    evaluateEDTWithGrad(pos, global_dist, global_grad);
    evaluateLocalEDT(pos, local_dist, local_grad);

    if(global_dist < local_dist)
    {
        dist = global_dist;
        grad = global_grad;
    }
    else
    {
        dist = local_dist;
        grad = local_grad;
    }
}

void GridMap::evaluateLocalEDT(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad)
{
    /*
     * @brief Generate the distance and gradient of the distance at the given position in local.
     */
    // transform pos point to local point
    Eigen::Matrix4f odom_to_base_matrix = tf2::transformToEigen(*(mp_.odom_transform_ptr_)).matrix().cast<float>();
    Eigen::Vector4f transform_point = Eigen::Vector4f::Zero();
    transform_point.head<3>() = pos.cast<float>();
    transform_point(3) = 1.0;
    transform_point =  odom_to_base_matrix.inverse() * transform_point;
    Eigen::Vector3d local_pos = transform_point.head<3>().cast<double>();

    Eigen::Vector3d diff;
    Eigen::Vector3d sur_pts[2][2][2];
    getLocalSurroundPts(local_pos, sur_pts, diff);
    
    Eigen::Vector3i local_id;
    localPosToIndex(local_pos, local_id);

    if(!isInLocalMap(local_id))
    {
        dist = 0;
        grad = Eigen::Vector3d::Zero();
        return;
    }

#ifdef DEBUG
    std::cout << local_pos.transpose() << " " << local_id.transpose() << std::endl;
#endif

    // get distances of the surround pts
    double dists[2][2][2];
    getLocalSurroundDistance(sur_pts, dists);

    // do interpolate to get distance gradient
    interpolateTrilinear(dists, diff, dist, grad);

    transform_point.head<3>() = grad.cast<float>();
    // let odom_to_base_matrix translation to be zero
    odom_to_base_matrix.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();
    transform_point = odom_to_base_matrix * transform_point;
    grad = transform_point.head<3>().cast<double>();
}

void GridMap::debugEvalue(const std::vector<Eigen::Vector3d> &pos, const std::vector<double> &dist, 
                          const std::vector<Eigen::Vector3d> &grad)
{
    grad_pos_arr_.poses.clear();
    grad_pos_arr_.header.frame_id = "map";
    grad_pos_arr_.header.stamp = ros::Time::now();
    
    assert(pos.size() == dist.size() && pos.size() == grad.size());
    for(size_t i=0; i < pos.size(); i++){
        geometry_msgs::Pose pose;
        pose.position.x = pos[i](0);
        pose.position.y = pos[i](1);
        pose.position.z = pos[i](2);
        double angle = atan2(grad[i](1), grad[i](0));
        pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        grad_pos_arr_.poses.push_back(pose);
    }
    
    debug_grad_pub_.publish(grad_pos_arr_);
}

void GridMap::getLocalSurroundPts(const Eigen::Vector3d &pos, Eigen::Vector3d pts[2][2][2],
                             Eigen::Vector3d &diff)
{
    Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
    Eigen::Vector3i idx;
    Eigen::Vector3d idx_pos;

    localPosToIndex(pos_m, idx);
    localIndexToPos(idx, idx_pos);
    diff = (pos - idx_pos) * mp_.resolution_inv_;

    for (int x = 0; x < 2; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            for (int z = 0; z < 2; z++)
            {
                Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y,z);
                Eigen::Vector3d current_pos;
                localIndexToPos(current_idx, current_pos);
                pts[x][y][z] = current_pos;
            }
        }
    }
}

void GridMap::getLocalSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2])
{
    for (int x = 0; x < 2; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            for (int z = 0; z < 2; z++)
            {
                dists[x][y][z] = getDistanceDynamic(pts[x][y][z]);
            }
        }
    }
}

void GridMap::evaluateEDTWithGrad(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad)
{
    /*
     * @brief Generate the distance and gradient of the distance at the given position.
     */

    // get diff & surround pts
    Eigen::Vector3d diff;
    Eigen::Vector3d sur_pts[2][2][2];
    getSurroundPts(pos, sur_pts, diff);

    // get distances of the surround pts
    double dists[2][2][2];
    getSurroundDistance(sur_pts, dists);

    // do interpolate to get distance gradient
    interpolateTrilinear(dists, diff, dist, grad);
}

void GridMap::getSurroundPts(const Eigen::Vector3d &pos, Eigen::Vector3d pts[2][2][2],
                             Eigen::Vector3d &diff)
{
    // if (!isInMap(pos)) { std::cout << "pos invalid for interpolation." << std::endl; }

    /* interpolation position */
    Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
    Eigen::Vector3i idx;
    Eigen::Vector3d idx_pos;

    posToIndex(pos_m, idx);
    indexToPos(idx, idx_pos);
    diff = (pos - idx_pos) * mp_.resolution_inv_; // (p-p0)/ (p1-p0)

    for (int x = 0; x < 2; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            for (int z = 0; z < 2; z++)
            {
                Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y,z);
                Eigen::Vector3d current_pos;
                indexToPos(current_idx, current_pos);
                pts[x][y][z] = current_pos;
            }
        }
    }
}

void GridMap::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2])
{
    for (int x = 0; x < 2; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            for (int z = 0; z < 2; z++)
                dists[x][y][z] = getDistance(pts[x][y][z]);
        }
    }
}

void GridMap::interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d &diff, double &value, Eigen::Vector3d &grad)
{
    // Trilinear interpolation
    double v000 = values[0][0][0]; // f(x0,y0,z0)
    double v100 = values[1][0][0]; // f(x1,y0,z0)
    double v010 = values[0][1][0]; // f(x0,y1,z0)
    double v110 = values[1][1][0]; // f(x1,y1,z0)
    double v001 = values[0][0][1]; // f(x0,y0,z1)
    double v101 = values[1][0][1]; // f(x1,y0,z1)
    double v011 = values[0][1][1]; // f(x0,y1,z1)
    double v111 = values[1][1][1]; // f(x1,y1,z1)

    double v00 = (1 - diff(0)) * v000 + diff(0) * v100;
    double v01 = (1 - diff(0)) * v001 + diff(0) * v101;
    double v10 = (1 - diff(0)) * v010 + diff(0) * v110;
    double v11 = (1 - diff(0)) * v011 + diff(0) * v111;

    double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
    double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

    value = (1 - diff(2)) * v0 + diff(2) * v1;

    // calculate gradient
    grad[2] = (v1 - v0) * mp_.resolution_inv_;
    grad[1] = ((1 - diff(2)) * (v10 - v00) + diff(2) * (v11 - v01)) * mp_.resolution_inv_;
    grad[0] = ((1 - diff(2)) * ((1 - diff(1)) * (v100 - v000) + diff(1) * (v110 - v010)) 
            + diff(2) * ((1 - diff(1)) * (v101 - v001) + diff(1) * (v111 - v011))) 
           * mp_.resolution_inv_;
}

void GridMap::get_static_buffer(std::vector<char> &static_buffer_inflate)
{
    int idx_nav_occ;
    int data;
    double value;

    /* inflate the point */
    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);

    for (int id_x = 0; id_x < mp_.map_voxel_num_(0); id_x++)
    {
        for (int id_y = 0; id_y < mp_.map_voxel_num_(1); id_y++)
        {           
            for (int id_z = 0; id_z < mp_.map_voxel_num_(2); id_z++)
            {
                // [*] addr in nav_msg::OccupancyGrid.data
                idx_nav_occ = id_x + id_y * mp_.map_voxel_num_(0) + id_z * mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);
                // [*] cast data from int8 to int[important]
                // data = (int)static_map_.data[idx_nav_occ];
                // Check data not be 0 or 100
                if (data != 0 && data != 100)
                {
                    // std::cout<<"data probability="<<value<<std::endl;
                }
                value = double(data) / 100.0;
                // save data to buffer
                if (value > mp_.p_occ_)
                {
                    // [*] addr in buffer
                    Eigen::Vector3i idx;
                    Eigen::Vector3d idx_pos;

                    idx(0) = id_x;
                    idx(1) = id_y;
                    idx(2) = id_z;
                    indexToPos(idx, idx_pos);
                    // idx_inf=toAddress(idx);
                    // md_.occupancy_buffer_static_inflate_[idx_inf]=1;

                    /* inflate the point */
                    Eigen::Vector3i inf_pt;
                    Eigen::Vector3d p2d_inf;

                    // Determine local occupandcy boundary for current location
                    double max_x, max_y, max_z, min_x, min_y, min_z;

                    min_x = mp_.map_max_boundary_(0);
                    min_y = mp_.map_max_boundary_(1);
                    min_z = mp_.map_max_boundary_(2);
                    max_x = mp_.map_min_boundary_(0);
                    max_y = mp_.map_min_boundary_(1);
                    max_z = mp_.map_min_boundary_(2);

                    for (int x = -inf_step; x <= inf_step; ++x)
                    {
                        for (int y = -inf_step; y <= inf_step; ++y)
                        {
                            for (int z = -inf_step; z <= inf_step; ++z)
                            {
                                p2d_inf(0) = idx_pos(0) + x * mp_.resolution_;
                                p2d_inf(1) = idx_pos(1) + y * mp_.resolution_;
                                p2d_inf(2) = idx_pos(2) + z * mp_.resolution_;
                            }


                            max_x = std::max(max_x, p2d_inf(0));
                            max_y = std::max(max_y, p2d_inf(1));
                            max_z = std::max(max_z, p2d_inf(2));
                            min_x = std::min(min_x, p2d_inf(0));
                            min_y = std::min(min_y, p2d_inf(1));
                            min_z = std::min(min_z, p2d_inf(2));

                            posToIndex(p2d_inf, inf_pt);
                            if (!isInMap(inf_pt))
                                continue;
                            int idx_inf = toAddress(inf_pt);
                            if (idx_inf > md_.buffer_size_)
                                std::cout << "idx_inf=" << idx_inf << std::endl;
                            static_buffer_inflate[idx_inf] = 1;
                        }
                    }
                }       
            }
        } 
    }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &robot_pos)
{
    Eigen::Vector3d diff = pt - robot_pos;
    Eigen::Vector3d max_tc = mp_.map_max_boundary_ - robot_pos;
    Eigen::Vector3d min_tc = mp_.map_min_boundary_ - robot_pos;

    double min_t = 1000000;

    for (int i = 0; i < 3; ++i)
    {
        if (fabs(diff[i]) > 0)
        {

            double t1 = max_tc[i] / diff[i];
            if (t1 > 0 && t1 < min_t)
                min_t = t1;

            double t2 = min_tc[i] / diff[i];
            if (t2 > 0 && t2 < min_t)
                min_t = t2;
        }
    }

    return robot_pos + (min_t - 1e-3) * diff;
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
    if (occ != 1 && occ != 0)
        return INVALID_IDX;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    int idx_ctns = toAddress(id);

    md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1) md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::publishDynamicMap()
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // get all the popint in 

    // for (int x = 0; x < mp_.map_voxel_num_[0]; x++){
    //     for (int y = 0; y < mp_.map_voxel_num_[1]; y++){
    //         for (int z = 0; z < mp_.map_voxel_num_[2]; z++){
    //             if (md_.local_occupancy_buffer_inflate_[toLocalAddress(x, y, z)] == 0) continue;
    //             Eigen::Vector3d pos;
    //             localIndexToPos(Eigen::Vector3i(x, y, z), pos);

    //             pt.x = pos(0);
    //             pt.y = pos(1);
    //             pt.z = pos(2);
    //             cloud.push_back(pt);
    //         }
    //     }
    // }
    // extract all the point in local_pointcloud_
    if (local_pointcloud_ != nullptr){
        // std::cout << "In publishDynamicMap, the size of local_pointcloud_ is: " << local_pointcloud_->points.size() << std::endl;
        for (int i = 0; i < local_pointcloud_->points.size(); i++)
        {
            pt.x = local_pointcloud_->points[i].x;
            pt.y = local_pointcloud_->points[i].y;
            pt.z = local_pointcloud_->points[i].z;
            cloud.push_back(pt);
        }
    }
    else {
        return;
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;

    cloud.is_dense = true;
    cloud.header.frame_id = mp_.base_id_;



    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    local_map_pub_.publish(cloud_msg);
}

void GridMap::publishRotatedMap(){
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (rotated_pointcloud_ != nullptr){
// std::cout << "111" << std::endl;
// std::cout << "rotated_pointcloud_ size: " << rotated_pointcloud_->points.size() << std::endl;
        for (int i = 0; i < rotated_pointcloud_->points.size(); i++)
        {
            pt.x = rotated_pointcloud_->points[i].x;
            pt.y = rotated_pointcloud_->points[i].y;
            pt.z = rotated_pointcloud_->points[i].z;
            cloud.push_back(pt);
        }
    }
    else {
        return;
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.base_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    rotated_map_pub_.publish(cloud_msg);
}

void GridMap::publishDynamicInterestPointMap()
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // put point cloud from local_pointcloud_sliced_ to cloud
    for (int i = 0; i < md_.test_pts_for_3D_.size(); i++)
    {
        pt.x = md_.test_pts_for_3D_[i](0);
        pt.y = md_.test_pts_for_3D_[i](1);
        pt.z = md_.test_pts_for_3D_[i](2);
        cloud.push_back(pt);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;

    cloud.is_dense = true;
    cloud.header.frame_id = mp_.base_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    local_map_interest_pts_pub_.publish(cloud_msg);
}

void GridMap::publishLeadingRayForDecompose()
{
    visualization_msgs::Marker ray_list;
    ray_list.header.frame_id = mp_.base_id_;
    ray_list.header.stamp = ros::Time::now();
    ray_list.ns = "leading_ray";
    ray_list.action = visualization_msgs::Marker::ADD;
    ray_list.pose.orientation.w = 1.0;
    ray_list.id = 0;
    ray_list.type = visualization_msgs::Marker::LINE_LIST;
    ray_list.scale.x = 0.01;
    ray_list.color.r = 1.0;
    ray_list.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    // p2 under world frame
    p2.x = md_.robot_pos_(0);
    p2.y = md_.robot_pos_(1);
    p2.z = md_.robot_pos_(2);
    // convert to base frame
    const Eigen::Matrix4f odom_to_base_matrix = tf2::transformToEigen(*mp_.odom_transform_ptr_).matrix().cast<float>();
    Eigen::Vector4f p2_base = odom_to_base_matrix.inverse() * Eigen::Vector4f(p2.x, p2.y, p2.z, 1.0);
    p2.x = p2_base(0);
    p2.y = p2_base(1);
    p2.z = p2_base(2);

    for (int i = 0; i < md_.interest_pts_xy_.size(); ++i){
        p1.x = md_.interest_pts_xy_[i](0);
        p1.y = md_.interest_pts_xy_[i](1);
        p1.z = md_.interest_pts_xy_[i](2);

        ray_list.points.push_back(p1);
        ray_list.points.push_back(p2);
    }
    visualize_leading_ray_for_decompose_pub_.publish(ray_list);
}

void GridMap::publishLeadingRayAfterPruneForDecompose(){
    visualization_msgs::Marker ray_list;
    ray_list.header.frame_id = mp_.base_id_;
    ray_list.header.stamp = ros::Time::now();
    ray_list.ns = "leading_ray_after_pruned";
    ray_list.action = visualization_msgs::Marker::ADD;
    ray_list.pose.orientation.w = 1.0;
    ray_list.id = 0;
    ray_list.type = visualization_msgs::Marker::LINE_LIST;
    ray_list.scale.x = 0.02;
    ray_list.color.g = 1.0;
    ray_list.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    // p2 under world frame
    p2.x = md_.robot_pos_(0);
    p2.y = md_.robot_pos_(1);
    p2.z = md_.robot_pos_(2);
    // convert to base frame
    const Eigen::Matrix4f odom_to_base_matrix = tf2::transformToEigen(*mp_.odom_transform_ptr_).matrix().cast<float>();
    Eigen::Vector4f p2_base = odom_to_base_matrix.inverse() * Eigen::Vector4f(p2.x, p2.y, p2.z, 1.0);
    p2.x = p2_base(0);
    p2.y = p2_base(1);
    p2.z = p2_base(2);

    for (int i = 0; i < md_.interest_pts_xy_pruned_.size(); ++i){
        p1.x = md_.interest_pts_xy_pruned_[i](0);
        p1.y = md_.interest_pts_xy_pruned_[i](1);
        p1.z = md_.interest_pts_xy_pruned_[i](2);

        ray_list.points.push_back(p1);
        ray_list.points.push_back(p2);
    }
    visualize_leading_ray_after_prune_for_decompose_pub_.publish(ray_list);
}

void GridMap::publishLeadingRayXYZ(){
    visualization_msgs::Marker ray_list;
    ray_list.header.frame_id = mp_.base_id_;
    ray_list.header.stamp = ros::Time::now();
    ray_list.ns = "leading_ray_xyz";
    ray_list.action = visualization_msgs::Marker::ADD;
    ray_list.pose.orientation.w = 1.0;
    ray_list.id = 0;
    ray_list.type = visualization_msgs::Marker::LINE_LIST;
    ray_list.scale.x = 0.02;
    ray_list.color.b = 1.0;
    ray_list.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    // p2 under world frame
    p2.x = md_.robot_pos_(0);
    p2.y = md_.robot_pos_(1);
    p2.z = md_.robot_pos_(2);
    // convert to base frame
    const Eigen::Matrix4f odom_to_base_matrix = tf2::transformToEigen(*mp_.odom_transform_ptr_).matrix().cast<float>();
    Eigen::Vector4f p2_base = odom_to_base_matrix.inverse() * Eigen::Vector4f(p2.x, p2.y, p2.z, 1.0);
    p2.x = p2_base(0);
    p2.y = p2_base(1);
    p2.z = p2_base(2);

    for (int i = 0; i < md_.interest_pts_xz_ground_check_.size(); ++i){
        p1.x = md_.interest_pts_xz_ground_check_[i](0);
        p1.y = md_.interest_pts_xz_ground_check_[i](1);
        p1.z = md_.interest_pts_xz_ground_check_[i](2);

        ray_list.points.push_back(p1);
        ray_list.points.push_back(p2);
    }
    visualize_leading_ray_xyz_pub_.publish(ray_list);
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{
    // if (map_pub_.getNumSubscribers() > 0)
    //     publishMap();
    // if (static_map_pub_.getNumSubscribers() > 0)
    //     publishStaticMap();
    if (local_map_pub_.getNumSubscribers() > 0)
        publishDynamicMap();
    if (local_map_interest_pts_pub_.getNumSubscribers() > 0)
        publishDynamicInterestPointMap();
    if (visualize_leading_ray_for_decompose_pub_.getNumSubscribers() > 0)
        publishLeadingRayForDecompose();
    if (visualize_leading_ray_after_prune_for_decompose_pub_.getNumSubscribers() > 0)
        publishLeadingRayAfterPruneForDecompose();
    if (rotated_map_pub_.getNumSubscribers() > 0)
        publishRotatedMap();
    if (visualize_leading_ray_xyz_pub_.getNumSubscribers() > 0)
        publishLeadingRayXYZ();
}
