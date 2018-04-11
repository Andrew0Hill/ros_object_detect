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
#include <set>

class Pose {
public:
    int id;

    Pose(int id, Eigen::Affine3d world_trans, Eigen::Translation3d xyz, Eigen::Quaterniond rotation, ros::Time stamp){
        base_to_world = world_trans;
        quat = rotation;
        coords = xyz;
        timestamp = stamp;
    }
    bool update_pose(Eigen::Affine3d world_trans, Eigen::Translation3d xyz){
        base_to_world = world_trans;
        coords = xyz;
    }
    // Transform for representing a base to world transform. This will be optimized on loop closure.
    Eigen::Affine3d base_to_world;
    Eigen::Quaterniond quat;
    Eigen::Translation3d coords;
    ros::Time timestamp;
};


#endif //OBJECT_DETECT_POSE_H
