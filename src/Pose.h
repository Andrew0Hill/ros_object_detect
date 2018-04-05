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
    static int id;

    Pose(Eigen::Affine3d camera_trans, Eigen::Affine3d world_trans, std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud){
        camera_to_base = camera_trans;
        base_to_world = world_trans;
        this->cloud = cloud;
    }
    bool update_pose(Eigen::Affine3d world_trans){
        base_to_world = world_trans;
    }
    // Base to camera transform. This will probably stay constant.
    Eigen::Affine3d camera_to_base;
    // Transform for representing a base to world transform. This will be optimized on loop closure.
    Eigen::Affine3d base_to_world;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

};


#endif //OBJECT_DETECT_POSE_H
