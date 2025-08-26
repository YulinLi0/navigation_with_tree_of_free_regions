#pragma once

#include <ros/ros.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/seed_decomp.h>
#include <decomp_util/line_segment.h>
#include <decomp_ros_msgs/EllipsoidArray.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
// tf
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// Odom & Pose
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// visual
#include <visualization_msgs/Marker.h>

#include "geo_utils.hpp"
#include "quickhull.hpp"
#include "sdlp.hpp"


class LocalDecomp
{   
public:
    LocalDecomp();
    ~LocalDecomp();

    void init(ros::NodeHandle &nh);
    void scanCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void odomTimerCallback(const ros::TimerEvent &e);
    void visTimerCallback(const ros::TimerEvent &e);
    void getPolyhedronArray(decomp_ros_msgs::PolyhedronArray &poly_array);
    void getOdometryInfo(Eigen::Matrix4d &T_odom);
    void pc2Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void interestPointsCallback(const visualization_msgs::MarkerPtr &msg);
    void pointsForPolyCallback(const visualization_msgs::MarkerPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &odom);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_scan_, sub_pc2_, sub_interest_points_, sub_odom_;
    ros::Publisher es_pub_, poly_pub_, points_for_poly_pub_;
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Timer odom_timer_, vis_timer_;
    geometry_msgs::TransformStamped::Ptr OdomTransformStamped_;
    sensor_msgs::LaserScan::_header_type scan_header_;
    sensor_msgs::PointCloud2::_header_type pc2_header_;

    std::vector<Eigen::Vector3d> interest_points_;
    geometry_msgs::TransformStamped::Ptr odom_transform_ptr_;
    geometry_msgs::TransformStamped::Ptr velodyne_to_base_ptr_, base_to_odom_ptr_;

    vec_E<Ellipsoid3D> es_;
    vec_E<Polyhedron3D> polys_;

    std::vector<Eigen::Vector3d> points_for_poly_;
};