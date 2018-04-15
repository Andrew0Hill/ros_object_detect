//
// Created by ros on 4/5/18.
//

#ifndef OBJECT_DETECT_POSE_H
#define OBJECT_DETECT_POSE_H
#include <opencv2/core/core.hpp>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <set>

class Pose {
public:
    int id;

    Pose(int nid, Eigen::Affine3d world_trans, Eigen::Translation3d xyz, Eigen::Quaterniond rotation, ros::Time stamp, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr){
        id = nid;
        base_to_world = world_trans;
        quat = rotation;
        coords = xyz;
        timestamp = stamp;
        cloud = cloud_ptr;
    }
    bool update_pose(Eigen::Affine3d world_trans, Eigen::Translation3d xyz){
        // Transform the point cloud by the correction between this transformation and the previous
        // (Update the point cloud's position)
        Eigen::Affine3d trans_diff = base_to_world.inverse() * world_trans;
        pcl::transformPointCloud(*cloud,*cloud,trans_diff);

        // Update the coordinates and transformation.
        base_to_world = world_trans;
        coords = xyz;
    }
    // Transform for representing a base to world transform. This will be optimized on loop closure.
    Eigen::Affine3d base_to_world;
    Eigen::Quaterniond quat;
    Eigen::Translation3d coords;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    ros::Time timestamp;
};


#endif //OBJECT_DETECT_POSE_H
