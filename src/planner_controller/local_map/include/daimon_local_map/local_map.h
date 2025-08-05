#ifndef MAPPING_H
#define MAPPING_H

//
#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>

// data structure
#include <queue>
#include <vector>
#include <unordered_map>

// #include <memory>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <algorithm>

// tf
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// Laser & PCL
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>

// Odom & Pose
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

// Map
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>

// message filter
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

// visual
#include <visualization_msgs/Marker.h>

// raycaster
#include <daimon_local_map/raycast.h>

// map util of JPS3d from sikang liu
// #include "jps_planner/jps_planner/jps_planner.h"

#define logit(x) (log((x) / (1 - (x))))

struct MappingParameters
{

    /* map properties */
    Eigen::Vector3d map_origin_, map_size_, local_map_origin_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in pos
    Eigen::Vector3i map_voxel_num_, local_map_voxel_num_;                       // map range in index
    Eigen::Vector3i map_min_idx_, map_max_idx_, local_min_idx_, local_max_idx_;
    Eigen::Vector3d local_update_range_;
    Eigen::Vector2d local_x_range_, local_y_range_, local_z_range_;
    Eigen::Vector3i local_index_range_;
    double resolution_, resolution_inv_;
    double obstacles_inflation_;
    geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr_;
    geometry_msgs::TransformStamped::Ptr TF_base_to_scan_ptr_;
    geometry_msgs::TransformStamped::Ptr odom_transform_ptr_;

    std::string frame_id_, base_id_;
    std::string pointcloud_sensor_frame_id_;
    std::string scan_sensor_frame_id_;

    /* raycasting */
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
        min_occupancy_log_;                  // logit of occupancy probability

    /* local map update and clear */
    double local_bound_inflate_;
    int local_map_margin_;

    /* visualization and computation time display */
    bool show_occ_time_;

    /* active mapping */
    double unknown_flag_;
};

struct MappingData
{
    // main map data, occupancy of each voxel and Euclidean distance
    int buffer_size_, local_buffer_size_;

    // global map
    // static occupancy map
    std::vector<char> occupancy_buffer_static_inflate_;
    std::vector<double> distance_buffer_static_all_;

    
    // local dynamic occupancy map
    // occupancy map(local) and distance map(local)
    std::vector<double>  local_occupancy_buffer_, local_distance_buffer_;
    // inflated occupancy map(local) and used for compute positive DT
    // used for compute negative distance
    std::vector<char> local_occupancy_buffer_inflate_, local_occupancy_buffer_neg_, local_occupancy_buffer_inflate_2D_slice_;
    std::vector<double> local_tmp_buffer_, local_tmp_buffer_1, local_distance_buffer_neg_, local_distance_buffer_all_;

    // map_util for JPS3d from sikang Liu
    // std::shared_ptr<JPS::VoxelMapUtil> map_util = std::make_shared<JPS::VoxelMapUtil>();
    // JPS::Tmap local_occupancy_buffer_inflate_for_JPS_3D;

    // laser position and pose data
    Eigen::Vector3d robot_pos_, last_robot_pos_;
    Eigen::Quaterniond robot_q_, last_robot_q_;

    // depth pointCloud2 data
    pcl::PointCloud<pcl::PointXYZ> depth_cloud, last_depth_cloud;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> fused_cloud_ptr_;
    int depth_cloud_cnt_;

    // laser scan projected point cloud
    std::vector<Eigen::Vector3d> proj_points_;
    int proj_points_cnt;

    // flags of map state
    bool occ_need_update_, local_updated_;
    bool has_first_depth_;
    bool has_odom_, has_cloud_;

    //[new] odom_depth_timeout_
    ros::Time last_occ_update_time_;
    bool flag_depth_odom_timeout_;
    bool flag_use_depth_fusion;

    // flag buffers for speeding up raycasting
    std::vector<short> count_hit_, count_hit_and_miss_;
    std::vector<char> flag_traverse_, flag_rayend_;
    char raycast_num_;
    std::queue<Eigen::Vector3i> cache_voxel_;

    // range of updating ESDF
    Eigen::Vector3i local_bound_min_, local_bound_max_;

    // computation time
    double fuse_time_, max_fuse_time_;
    int update_num_;

    // cloud curvature data
    std::vector<double> cloud_curvature_;
    std::vector<double> cloud_neighbor_picked_;
    std::vector<int> cloud_sort_ind_;
    std::vector<Eigen::Vector3d> interest_pts_xy_, interest_pts_xy_pruned_, interest_pts_xz_, interest_pts_xz_pruned_, interest_pts_xz_ground_check_ ,interest_pts_xyz_;
    std::vector<Eigen::Vector3d> test_pts_for_3D_, interest_pts_xz_unpruned_, pts_for_xz_;
    int num_of_interest_pts_xy_, num_of_interest_pts_xz_;
    int length_of_cloud_curvature_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
    GridMap() {}
    ~GridMap() {}
    typedef std::shared_ptr<GridMap> Ptr;

    enum
    {
        INVALID_IDX = -10000
    };

    void initMap(ros::NodeHandle &nh);

    geometry_msgs::PoseArray grad_pos_arr_;

    /* occupancy map management */
    // static map
    void get_static_buffer(std::vector<char> &static_buffer_inflate);

    // pos[meter] to index [pixel cell]
    inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
    inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
    inline Eigen::Vector3i posToIndex(const Eigen::Vector3d &pos);
    inline void localIndexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
    inline void localPosToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);


    // index [pixel cell] to buffer array id [num]
    inline int toAddress(const Eigen::Vector3i &id);
    inline int toAddress(int &x, int &y, int &z);
    inline int toLocalAddress(const Eigen::Vector3i &id);
    inline int toLocalAddress(int &x, int &y, int &z);
    inline int toLocalAddress(const Eigen::Vector2i &id);
    inline int toLocalAddress(int &x, int &y);

    // is in map
    inline bool isInMap(const Eigen::Vector3d &pos);
    inline bool isInMap(const Eigen::Vector3i &idx);
    inline bool isInLocalMap(const Eigen::Vector3d &point);
    inline bool isInLocalMap(const Eigen::Vector3i &idx);


    // occupancy manage
    inline int getLocalInflateOccupancy(Eigen::Vector3d pos);
    inline int getLocalInflateOccupancy(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    inline int getStaticInflateOccupancy(Eigen::Vector3d pos); // fuse local and static map

    inline int getFusedDynamicInflateOccupancy(Eigen::Vector3d pos); // fuse local and static map and dynamic obs
    inline int getFusedDynamicInflateOccupancyi(Eigen::Vector3i index);

    inline int getFusedDynamicInflateOccupancyInLocalMap(Eigen::Vector3d pos);

    // utils: bound index, known, unknown, free, occupied
    inline void boundIndex(Eigen::Vector3i &id);
    inline void boundLocalIndex(Eigen::Vector3i &id);
    inline bool isKnownOccupied(const Eigen::Vector3i &id);

    /* distance field management */
    // get distance
    inline double getDistance(const Eigen::Vector3d &pos);
    inline double getDistance(const Eigen::Vector3i &id);
    inline double getDistanceStatic(const Eigen::Vector3d &pos);
    inline double getDistanceDynamic(const Eigen::Vector3d &pos);
    inline double getDistanceDynamic(const Eigen::Vector3i &id);

    // get distance gradient
    void evaluateEDTBoth(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad);
    void evaluateEDTWithGrad(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad);
    void evaluateLocalEDT(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad);
    void debugEvalue(const std::vector<Eigen::Vector3d> &pos, const std::vector<double> &dist, const std::vector<Eigen::Vector3d> &grad);
    
    void getSurroundPts(const Eigen::Vector3d &pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d &diff);
    void getLocalSurroundPts(const Eigen::Vector3d &pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d &diff);
    
    void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
    void getLocalSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
    
    void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d &diff,
                             double &value, Eigen::Vector3d &grad);

    /* utils map */
    void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
    double getResolution();

    /* visualization publish */
    void publishDynamicMap();
    void publishRotatedMap();
    void publishDynamicInterestPointMap();
    void publishLeadingRayForDecompose();
    void publishLeadingRayAfterPruneForDecompose();
    void publishLeadingRayXYZ();

    // jps3d for sikang Liu
    // std::shared_ptr<JPS::VoxelMapUtil> getMapUtilForJPS3d();

private:
    ros::NodeHandle node_;
    MappingParameters mp_;
    MappingData md_;

    // scan to pointCloud2 projector
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    // sensor: message filter
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> SyncPolicyScanOdom;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyScanOdom>> SynchronizerScanOdom;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    SynchronizerScanOdom sync_scan_odom_;

    // sensor: subscriber
    ros::Subscriber async_scan_sub_, async_odom_sub_, async_pointcloud_sub_;
    ros::Subscriber goal_sub_;

    // map server service
    ros::ServiceClient static_map_client_;
    nav_msgs::OccupancyGrid static_map_;

    // local map point cloud
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> local_pointcloud_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> rotated_pointcloud_;

    // publiser
    ros::Publisher map_pub_, static_map_pub_, dynamic_map_pub_, local_map_pub_, local_map_interest_pts_pub_, rotated_map_pub_;
    ros::Publisher depth_pub_; // laser pointcloud2
    ros::Publisher update_range_pub_;
    ros::Publisher unknown_pub_;
    ros::Publisher debug_pub_, debug_grad_pub_;
    ros::Publisher visualize_leading_ray_for_decompose_pub_;
    ros::Publisher visualize_leading_ray_after_prune_for_decompose_pub_;
    ros::Publisher visualize_leading_ray_xyz_pub_;

    // timer
    ros::Timer occ_timer_;
    ros::Timer vis_timer_;

    // goal
    Eigen::Vector3d goal_pos_;

    /* Sensor Callbacks */
    void scanCallback(const sensor_msgs::PointCloud2ConstPtr &scan);
    void odomCallback(const nav_msgs::OdometryConstPtr &odom);
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
    void goalCallback(const geometry_msgs::PoseStampedPtr &msg);

    /* Time event callback: update occupancy by raycasting, and update ESDF*/
    void updateOccupancyCallback(const ros::TimerEvent & /*event*/);
    void visCallback(const ros::TimerEvent & /*event*/);

    // main update process
    /* occupancy map update */
    void projectPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud);
    void slicePointCloudInxyPlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud);
    void slicePointCloudInxzPlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud);

    void getInterestingPointsFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<Eigen::Vector3d> &pts, int numofInterestPoints);
    void computeCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<double> &curvature);
    void pickCornerPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<Eigen::Vector3d> &pts, int numofCornerPoints);
    void sortCloudBasedOnCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<double> &curvature, std::vector<int> &sort_ind);

    void pruneSimilarRaysInxyPlane(const std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &pruned_pts, double angle_threshold, std::vector<Eigen::Vector3d> &pts_for_xz);
    void pruneSimilarRaysInxzPlane(const std::vector<Eigen::Vector3d> &pts, std::vector<Eigen::Vector3d> &pruned_pts, double angle_threshold);
    bool isRayIntersectWithObstacle(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

    inline void inflatePoint(const Eigen::Vector3i &pt, int step, std::vector<Eigen::Vector3i> &pts);
    int setCacheOccupancy(Eigen::Vector3d pos, int occ);
    Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &robot_pos);
    // void fuseOccupancyBuffer();
    //  service static callback
    bool get_static_map();
};

inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
    for (int i = 0; i < 3; ++i)
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void GridMap::localPosToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - mp_.local_map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::localIndexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
    for (int i = 0; i < 3; ++i)
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.local_map_origin_(i);
}

inline Eigen::Vector3i GridMap::posToIndex(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i id;
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
    return id;
}

inline int GridMap::toAddress(const Eigen::Vector3i &id)
{
    return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int GridMap::toAddress(int &x, int &y, int &z)
{
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline int GridMap::toLocalAddress(const Eigen::Vector3i &id)
{
    return id(2) * mp_.local_map_voxel_num_(1) * mp_.local_map_voxel_num_(0) + id(1) * mp_.local_map_voxel_num_(0) + id(0);
}

inline int GridMap::toLocalAddress(int &x, int &y, int &z){
    return z * mp_.local_map_voxel_num_(1) * mp_.local_map_voxel_num_(0) + y * mp_.local_map_voxel_num_(0) + x;
}

inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 || pos(2) < mp_.map_min_boundary_(2) + 1e-4)
    {
        // cout << "less than min range!" << endl;
        return false;
    }

    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 || pos(2) > mp_.map_max_boundary_(2) - 1e-4)
    {
        return false;
    }

    return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i &idx)
{
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
    {
        return false;
    }
    if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 || idx(2) > mp_.map_voxel_num_(2) - 1)
    {
        return false;
    }
    return true;
}

inline bool GridMap::isInLocalMap(const Eigen::Vector3i &idx){
    if (idx(0) < 0 || idx(1) < 0 || idx(3) < 0)
    {
        return false;
    }
    if (idx(0) > mp_.local_index_range_(0) - 1 || idx(1) > mp_.local_index_range_(1) - 1 || idx(2) > mp_.local_index_range_(2) - 1)
    {
        return false;
    }
    return true;
}

inline bool GridMap::isInLocalMap(const Eigen::Vector3d &pos){
    if (pos(0) < mp_.local_x_range_(0) || pos(1) < mp_.local_y_range_(0) || pos(2) < mp_.local_z_range_(0))
    {
        return false;
    }
    if (pos(0) > mp_.local_x_range_(1) || pos(1) > mp_.local_y_range_(1) || pos(2) > mp_.local_z_range_(1))
    {
        return false;
    }
    return true;
}


inline int GridMap::getLocalInflateOccupancy(Eigen::Vector3d pos){
    if (!isInLocalMap(pos))
        return -1;

    Eigen::Vector3i id;
    localPosToIndex(pos, id);
    return int(md_.local_occupancy_buffer_inflate_[toLocalAddress(id)]);
}

inline int GridMap::getLocalInflateOccupancy(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud){
    const Eigen::Matrix4f odom_to_base_matrix = tf2::transformToEigen(*mp_.odom_transform_ptr_).matrix().cast<float>();
    pcl::transformPointCloud(*point_cloud, *point_cloud, odom_to_base_matrix.inverse());

    for(size_t i = 1; i < point_cloud->points.size(); i++){
        if(!isInLocalMap(Eigen::Vector3d(point_cloud->points[i].x, point_cloud->points[i].y, point_cloud->points[i].z))){
            // ROS_WARN("[Local map] Point %ld out of local map!", i);
            continue;
        }

        Eigen::Vector3i id;
        localPosToIndex(Eigen::Vector3d(point_cloud->points[i].x, point_cloud->points[i].y, point_cloud->points[i].z), id);
        if(md_.local_occupancy_buffer_[toLocalAddress(id)] > mp_.clamp_max_log_ - 0.1){
            // std::cout << point_cloud->points[i].x << " " << point_cloud->points[i].y << std::endl;
            // std::cout << md_.local_occupancy_buffer_[toLocalAddress(id)] << std::endl;
            ROS_ERROR("[Local map] Point %ld is occupied!", i);
            return i;
        }
    }
    return -1;
}

inline int GridMap::getStaticInflateOccupancy(Eigen::Vector3d pos)
{
    /* 
     *  Note: Get the static map occupancy with inflation
     */
    if (!isInMap(pos))
        return 0;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    if (md_.occupancy_buffer_static_inflate_[toAddress(id)] == 1)
    {
        return 1;
    }

    return 0;
}

inline int GridMap::getFusedDynamicInflateOccupancy(Eigen::Vector3d pos)
{
    /*
     *  Note: Note only check the local map occupancy, but also check the static map occupancy
     */
    if (!isInMap(pos))
        return 0;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    if (md_.occupancy_buffer_static_inflate_[toAddress(id)] == 1)
    {
        return 1;
    }

    Eigen::Matrix4f odom_to_base_matrix = tf2::transformToEigen(*(mp_.odom_transform_ptr_)).matrix().cast<float>();
    Eigen::Vector4f pos4(pos(0), pos(1), 0, 1);
    pos4 = odom_to_base_matrix.inverse() * pos4;
    pos(0) = pos4(0);
    pos(1) = pos4(1);

    if (!isInLocalMap(pos))
        return 0;

    localPosToIndex(pos, id);
    return int(md_.local_occupancy_buffer_inflate_[toLocalAddress(id)]);
}

inline int GridMap::getFusedDynamicInflateOccupancyi(Eigen::Vector3i index)
{
    Eigen::Vector3d pos;
    indexToPos(index, pos);
    return getFusedDynamicInflateOccupancy(pos);
}

inline int GridMap::getFusedDynamicInflateOccupancyInLocalMap(Eigen::Vector3d pos)
{
    // pos is in local map
    if (!isInLocalMap(pos))
        return 0;

    Eigen::Vector3i id;
    localPosToIndex(pos, id);
    return int(md_.local_occupancy_buffer_inflate_[toLocalAddress(id)]);
}

inline void GridMap::boundIndex(Eigen::Vector3i &id)
{
    Eigen::Vector3i id1;
    id1(0) = std::max(std::min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = std::max(std::min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id1(2) = std::max(std::min(id(2), mp_.map_voxel_num_(2) - 1), 0);
    id = id1;
}

inline void GridMap::boundLocalIndex(Eigen::Vector3i &id)
{
    Eigen::Vector3i id1;
    id1(0) = std::max(std::min(id(0), mp_.local_map_voxel_num_(0) - 1), 0);
    id1(1) = std::max(std::min(id(1), mp_.local_map_voxel_num_(1) - 1), 0);
    id1(2) = std::max(std::min(id(2), mp_.local_map_voxel_num_(2) - 1), 0);
    id = id1;
}


inline bool GridMap::isKnownOccupied(const Eigen::Vector3i &id)
{
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);

    return md_.occupancy_buffer_static_inflate_[adr] == 1;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, int step, std::vector<Eigen::Vector3i> &pts)
{
    int num = 0;


    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z){
            pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
        }
}

/* DISTANCE FIELD*/
inline double GridMap::getDistance(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    boundIndex(id);

    // return std::min(md_.distance_buffer_all_[toAddress(id)], md_.distance_buffer_static_all_[toAddress(id)]);
    return md_.distance_buffer_static_all_[toAddress(id)];
}

inline double GridMap::getDistance(const Eigen::Vector3i &id)
{
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    // return std::min(md_.distance_buffer_all_[toAddress(id)], md_.distance_buffer_static_all_[toAddress(id)]);
    return md_.distance_buffer_static_all_[toAddress(id)];
}

inline double GridMap::getDistanceStatic(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    boundIndex(id);

    return md_.distance_buffer_static_all_[toAddress(id)];
}

inline double GridMap::getDistanceDynamic(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i id;
    localPosToIndex(pos, id);

    return md_.local_distance_buffer_all_[toLocalAddress(id)];
}

inline double GridMap::getDistanceDynamic(const Eigen::Vector3i &id)
{
    Eigen::Vector3i id1 = id;
    boundLocalIndex(id1);
    return md_.local_distance_buffer_all_[toLocalAddress(id1)];
}
#endif