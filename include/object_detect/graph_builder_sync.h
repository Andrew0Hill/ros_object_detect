//
// Created by ros on 4/7/18.
//

#ifndef OBJECT_DETECT_GRAPH_BUILDER_SYNC_H
#define OBJECT_DETECT_GRAPH_BUILDER_SYNC_H
#include <ros/ros.h>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <tf/transform_datatypes.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <PoseGraph.h>
#include "GraphOptimizer_G2O.h"
#include <signal.h>
typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZRGB>, nav_msgs::Odometry> graph_sync_policy;
class SyncGraphBuilder{
public:
    SyncGraphBuilder();
    ~SyncGraphBuilder();
    void graph_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom);
private:
    // Node handle.
    ros::NodeHandle n;
    // Subscribers for point cloud and odometry.
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>> *cloud_sub;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub;
    // Synchronized subscriber
    message_filters::Synchronizer<graph_sync_policy> *synchronizer;
    // Publishers for stitched cloud and graph markers.
    ros::Publisher cloud_pub;
    ros::Publisher graph_pub;
    // Pointer to intermediate stitched cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr poses_cloud;
    // TF2 Buffer and Listener to lookup transforms.
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
    // ICP object for performing Iterative Closest Point alignment.
    pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp_gen;
    // VoxelGrid for downsampling Point Cloud.
    pcl::VoxelGrid<pcl::PointXYZRGB> filter_grid;
    // Graph Optimizer
    GraphOptimizer_G2O optimizer;
    // PoseGraph to hold poses.
    std::shared_ptr<PoseGraph> poseGraph;
};

#endif //OBJECT_DETECT_GRAPH_BUILDER_SYNC_H
