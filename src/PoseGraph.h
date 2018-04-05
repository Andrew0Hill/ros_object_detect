//
// Created by ros on 4/5/18.
//

#ifndef OBJECT_DETECT_POSEGRAPH_H
#define OBJECT_DETECT_POSEGRAPH_H

#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "Pose.h"
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "PoseEdge.h"
class PoseGraph {
public:
    std::vector<std::shared_ptr<Pose>> poses;
    std::vector<std::shared_ptr<PoseEdge>> edges;
    PoseGraph(){
        // TF buffer
        buffer = std::make_shared<tf2_ros::Buffer>();
        // TF Listener
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
        camera_trans = Eigen::Affine3d(Eigen::Translation3d());
    }

    bool add_vertex_previous(nav_msgs::Odometry odom, std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud);
private:
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
    Eigen::Affine3d camera_trans;
};


#endif //OBJECT_DETECT_POSEGRAPH_H
