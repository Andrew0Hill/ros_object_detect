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
#include <vector>
#include <visualization_msgs/MarkerArray.h>
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
    }
    bool add_vertex(nav_msgs::Odometry odom);
    bool add_vertex_previous(nav_msgs::Odometry odom);
    void get_prev_transform(Eigen::Affine3d &transform, nav_msgs::Odometry odom);
    void get_graph_markers(std::vector<visualization_msgs::Marker>& markers);
private:
    int alloc_id = 0;
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
};


#endif //OBJECT_DETECT_POSEGRAPH_H
