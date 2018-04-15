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
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include "GraphOptimizer_G2O.h"
#include "PoseEdge.h"
class PoseGraph {
public:
    GraphOptimizer_G2O optimizer;
    std::vector<std::shared_ptr<Pose>> poses;
    std::vector<std::shared_ptr<PoseEdge>> edges;
    PoseGraph(){
        // TF buffer
        buffer = std::make_shared<tf2_ros::Buffer>();
        // TF Listener
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    }
    std::shared_ptr<Pose> add_vertex(nav_msgs::Odometry odom, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);
    std::shared_ptr<Pose> add_vertex_previous(nav_msgs::Odometry odom, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);
    bool optimize_graph();
    bool add_edge(std::shared_ptr<Pose> p1, std::shared_ptr<Pose> p2, Eigen::Vector3d relativeTransform, Eigen::Matrix3d covariance, bool loop_closure = false);
    void get_prev_transform(Eigen::Affine3d &transform, nav_msgs::Odometry odom);
    void get_graph_markers(std::vector<visualization_msgs::Marker>& markers);
    void write_graph_to_file();
private:
    int alloc_id = 0;
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};


#endif //OBJECT_DETECT_POSEGRAPH_H
