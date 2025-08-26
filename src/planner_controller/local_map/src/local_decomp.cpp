#include <frtree_local_map/local_decomp.h>

LocalDecomp::LocalDecomp()
{}

LocalDecomp::~LocalDecomp()
{}

void LocalDecomp::init(ros::NodeHandle &nh){
    nh_ = nh;

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    odom_timer_ = nh_.createTimer(ros::Duration(0.05), &LocalDecomp::odomTimerCallback, this);
    vis_timer_ = nh_.createTimer(ros::Duration(0.1), &LocalDecomp::visTimerCallback, this);
    // sub_scan_ = nh_.subscribe("/scan", 1, &LocalDecomp::scanCallback, this);
    sub_interest_points_ = nh_.subscribe("/local_map/visualizeLeadingRayAfterPruneForDecompose", 1, &LocalDecomp::interestPointsCallback, this);
    sub_pc2_ = nh_.subscribe("/velodyne_points", 10, &LocalDecomp::pc2Callback, this);
    // sub_odom_ = nh_.subscribe("/odom", 10, &LocalDecomp::odomCallback, this);
    es_pub_ = nh_.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
    poly_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
    points_for_poly_pub_ = nh_.advertise<visualization_msgs::Marker>("points_for_polyhedron_array", 1, true);

    OdomTransformStamped_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    velodyne_to_base_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    // base_to_odom_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);

    tf2_ros::Buffer tf_buffer_base, tf_buffer_velodyne;
    tf2_ros::TransformListener tf_listener_base(tf_buffer_base), tf_listener_velodyne(tf_buffer_velodyne);
    geometry_msgs::TransformStamped transformStamped;
    // try
    // {
    //     *(base_to_odom_ptr_) = tf_buffer_base.lookupTransform("base", "odom", ros::Time::now(), ros::Duration(2.0));
    //     // ROS_INFO("odomTimerCallback: %f, %f, %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    // }
    // catch (tf2::TransformException &ex)
    // {
    //     ROS_WARN("%s", ex.what());
    // }
    try
    {
        *(velodyne_to_base_ptr_) = tf_buffer_velodyne.lookupTransform("unitree_scan", "base", ros::Time::now(), ros::Duration(2.0));
        // ROS_INFO("odomTimerCallback: %f, %f, %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
}


void LocalDecomp::pc2Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{   
    if (msg->data.size() == 0)
        return;
    // pc2_header_ = /;
    es_.clear();
    polys_.clear();
    points_for_poly_.clear();

    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);

    vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);

    vec_Vec3f obs3d;
    // only take the point that x is within the range [-3.0, 3.0], y is within the range [-3.0, 3.0], z is within the range [-0.8, 0.8], and the resolution is 0.05
   for (unsigned int i = 0; i < obs.size(); i++){
        if (obs[i][0] >= -3.0 && obs[i][0] <= 3.0 && obs[i][1] >= -3.0 && obs[i][1] <= 3.0 && obs[i][2] >= -0.8 && obs[i][2] <= 0.8)
            obs3d.push_back(obs[i]);
    }

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
    const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*(OdomTransformStamped_));
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
   
//     Vec3f pos1(0.3,0,-0.2);
//     Vec3f pos2(-0.32,0,-0.2);
//     Eigen::Vector4f pt1, pt2;
//     pt1 << pos1.cast<float>(), 1.0;
//     pt2 << pos2.cast<float>(), 1.0;
//     Eigen::Vector4f pt1_transformed = T_base_odom_matrix * T_velodyne_base_matrix.inverse() * pt1;
//     Eigen::Vector4f pt2_transformed = T_base_odom_matrix * T_velodyne_base_matrix.inverse() * pt2;

//     const Vec3f pos1_odom(pt1_transformed[0], pt1_transformed[1], pt1_transformed[2]);
//     const Vec3f pos2_odom(pt2_transformed[0], pt2_transformed[1], pt2_transformed[2]);
// // std::cout << "robot pos in decompose: " << ((pos1_odom + pos2_odom) / 2).transpose() << std::endl;
//     LineSegment3D decomp_util(pos1_odom,pos2_odom);
//     decomp_util.set_obs(obs3d_odom);
//     decomp_util.dilate(0.0);

//     // es_.push_back(decomp_util.get_ellipsoid());
//     polys_.push_back(decomp_util.get_polyhedron());

//     points_for_poly_.push_back((pos1_odom + pos2_odom) / 2);

    // compute the dist between two points from interest points
    if (interest_points_.size() > 1){

        for (unsigned int i = 0; i < interest_points_.size()/2; ++i){

            Eigen::Vector3d dir = interest_points_[2*i] - interest_points_[2*i+1];
            double dist = dir.norm();
            if (dist > 3)
                dist = 3;
            dir.normalize();

            Eigen::Vector3d pos1, pos2, pos3, pos4, pos5, pos6;
            pos1 = interest_points_[2*i+1] + 0.05 * dir * dist;
            pos2 = interest_points_[2*i+1] + 0.35*dir * dist;

            pos3 = interest_points_[2*i+1] + 0.3*dir * dist;
            pos4 = interest_points_[2*i+1] + 0.6*dir * dist;

            pos5 = interest_points_[2*i+1] + 0.5*dir * dist;
            pos6 = interest_points_[2*i] - 0.2*dir * dist;

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
            decomp_util1.set_obs(obs3d_odom);
            decomp_util1.set_local_bbox(Vec3f(1, 5, 1));
            decomp_util1.dilate(0.0);

            LineSegment3D decomp_util2(pos3_v,pos4_v);
            decomp_util2.set_obs(obs3d_odom);
            decomp_util2.set_local_bbox(Vec3f(1, 5, 1));
            decomp_util2.dilate(0.0);

            LineSegment3D decomp_util3(pos5_v,pos6_v);
            decomp_util3.set_obs(obs3d_odom);
            decomp_util3.set_local_bbox(Vec3f(1, 5, 1));
            decomp_util3.dilate(0.0);

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
}

void LocalDecomp::interestPointsCallback(const visualization_msgs::MarkerPtr &msg){
// std::cout << "msg->points.size(): " << msg->points.size() << std::endl;
    if (msg->points.size() == 0)
        return;
    interest_points_.clear();
    for (unsigned int i = 0; i < msg->points.size(); i++){
        Eigen::Vector3d pt;
        pt << msg->points[i].x, msg->points[i].y, msg->points[i].z;
        interest_points_.push_back(pt);
// std::cout << "interestPointsCallback: " << pt.transpose() << std::endl;
    }
    // all the points are in the base frame, we need to transform the points to the odom frame

    // const Eigen::Affine3d T_base_unitree = tf2::transformToEigen(*(base_to_odom_ptr_));
    const Eigen::Affine3d T_base_unitree = tf2::transformToEigen(*(OdomTransformStamped_));
    const Eigen::Matrix4f T_base_unitree_matrix = T_base_unitree.matrix().cast<float>();
    for (unsigned int i = 0; i < interest_points_.size(); i++){
        Eigen::Vector4f pt;
        pt << interest_points_[i].cast<float>(), 1.0;
        // Eigen::Vector4f pt_transformed = T_base_unitree_matrix.inverse() * pt;
        Eigen::Vector4f pt_transformed = T_base_unitree_matrix * pt;
        interest_points_[i] = pt_transformed.head<3>().cast<double>();
    }
}

void LocalDecomp::scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (msg->ranges.size() == 0)
        return;
    scan_header_ = msg->header;
    es_.clear();
    polys_.clear();

    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*msg, cloud);
    vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);
    // initialize seeds with zero
    vec_Vec3f obs3d;
    for(const auto& it: obs)
        obs3d.push_back(it.topRows<3>());
        
    const Vec3f pos1(0.3,0,0.3);
    const Vec3f pos2(-0.3,0,0.3);
    LineSegment3D decomp_util(pos1,pos2);
    decomp_util.set_obs(obs3d);
    decomp_util.dilate(0.0);
    
    es_.push_back(decomp_util.get_ellipsoid());
    polys_.push_back(decomp_util.get_polyhedron());
    // print out the ellipsoid and polyhedron
    // std::cout << "ellipsoid: " << es_[0].center.transpose() << " " << es_[0].radii.transpose() << std::endl;


}

void LocalDecomp::odomTimerCallback(const ros::TimerEvent &e)
{
    try
    {
        *OdomTransformStamped_ = tf_buffer_.lookupTransform("odom", "base", ros::Time(0));
        // ROS_INFO("odomTimerCallback: %f, %f, %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        Eigen::Matrix4d T_odom;
        getOdometryInfo(T_odom);

    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

void LocalDecomp::visTimerCallback(const ros::TimerEvent &e){
    if (!es_.empty()){
        // std::cout<<"es size: "<<es_.size()<<std::endl;
        decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(es_);
        es_msg.header.frame_id = "odom";
        es_pub_.publish(es_msg);
    }
    
    // if (!polys_.empty()){
        // std::cout<<"polys size: "<<polys_.size()<<std::endl;
        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_);
        poly_msg.header.frame_id = "odom";
        poly_pub_.publish(poly_msg);
    // }

    if (!points_for_poly_.empty()){
        visualization_msgs::Marker points_for_poly_msg;
        points_for_poly_msg.header.frame_id = "odom";
        points_for_poly_msg.header.stamp = ros::Time::now();
        points_for_poly_msg.ns = "points_for_poly";
        points_for_poly_msg.id = 0;
        points_for_poly_msg.type = visualization_msgs::Marker::POINTS;
        points_for_poly_msg.action = visualization_msgs::Marker::ADD;
        points_for_poly_msg.pose.orientation.w = 1.0;
        points_for_poly_msg.scale.x = 0.1;
        points_for_poly_msg.scale.y = 0.1;
        points_for_poly_msg.color.g = 1.0f;
        points_for_poly_msg.color.a = 1.0;
        for (unsigned int i = 0; i < points_for_poly_.size(); i++){
            geometry_msgs::Point p;
            p.x = points_for_poly_[i][0];
            p.y = points_for_poly_[i][1];
            p.z = points_for_poly_[i][2];
            points_for_poly_msg.points.push_back(p);
        }
        points_for_poly_pub_.publish(points_for_poly_msg);
    }
}

void LocalDecomp::getPolyhedronArray(decomp_ros_msgs::PolyhedronArray &poly_array){
    poly_array = DecompROS::polyhedron_array_to_ros(polys_);
}

void LocalDecomp::getOdometryInfo(Eigen::Matrix4d &T_odom){
    T_odom(0,3) = OdomTransformStamped_->transform.translation.x;
    T_odom(1,3) = OdomTransformStamped_->transform.translation.y;
    T_odom(2,3) = OdomTransformStamped_->transform.translation.z;
    T_odom(3,3) = 1.0;
    
    Eigen::Quaterniond q;
    q.x() = OdomTransformStamped_->transform.rotation.x;
    q.y() = OdomTransformStamped_->transform.rotation.y;
    q.z() = OdomTransformStamped_->transform.rotation.z;
    q.w() = OdomTransformStamped_->transform.rotation.w;
    T_odom.block<3,3>(0,0) = q.toRotationMatrix();
}

// void LocalDecomp::odomCallback(const nav_msgs::OdometryConstPtr &odom){
//     odom_transform_ptr_->transform.translation.x = odom->pose.pose.position.x;
//     odom_transform_ptr_->transform.translation.y = odom->pose.pose.position.y;
//     odom_transform_ptr_->transform.translation.z = odom->pose.pose.position.z;
//     odom_transform_ptr_->transform.rotation.x = odom->pose.pose.orientation.x;
//     odom_transform_ptr_->transform.rotation.y = odom->pose.pose.orientation.y;
//     odom_transform_ptr_->transform.rotation.z = odom->pose.pose.orientation.z;
//     odom_transform_ptr_->transform.rotation.w = odom->pose.pose.orientation.w;
//     }