//
// Created by ros on 4/14/18.
//

#ifndef OBJECT_DETECT_OBJECT_DETECT_NODE_SYNC_H
#define OBJECT_DETECT_OBJECT_DETECT_NODE_SYNC_H


#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include "DetectionModel.h"
#include "Memory.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/impl/centroid.hpp>
#include "ORB_Featurizer.h"
#include "GraphOptimizer_G2O.h"
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include "PoseGraph.h"

#define MIN_FEATURE_NUM 50
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZRGB>, nav_msgs::Odometry> frame_sync_policy;
class ObjectDetector {
private:
    bool first_measurement;
    nav_msgs::Odometry prev_pose;
    // Pointer to detection model.
    std::shared_ptr<DetectionModel> dm;
    // Publishers
    ros::Publisher anomaly_pub, markers, graph_pub, cloud_pub;
    // Frame number.
    int frame_num;
    std::shared_ptr<ORB_Featurizer> featurizer;
    // Pointers to TF2 buffer and listeners.
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
    // Lock for memory to prevent race conditions.
    boost::shared_mutex memory_lock;
    // ICP object for performing Iterative Closest Point alignment.
    pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp_gen;
    // VoxelGrid for downsampling Point Cloud.
    pcl::VoxelGrid<pcl::PointXYZRGB> filter_grid;
    // Graph Optimizer
    //GraphOptimizer_G2O optimizer;
    // PoseGraph to hold poses.
    std::shared_ptr<PoseGraph> poseGraph;
    // Pointer to intermediate stitched cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr poses_cloud;
    // Create node handle, subscribers and synchronizer.
    ros::NodeHandle node;
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>>* cloud_sub;
    message_filters::Subscriber<sensor_msgs::Image>* image_sub;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub;
    message_filters::Synchronizer<frame_sync_policy>* synchronizer;
    // Function to check if odometry is the same as last pose.
    bool odom_has_changed(const nav_msgs::Odometry::ConstPtr& odom){
        return  odom->pose.pose.position.x != prev_pose.pose.pose.position.x
                || odom->pose.pose.position.y != prev_pose.pose.pose.position.y
                || odom->pose.pose.orientation.x != prev_pose.pose.pose.orientation.x
                || odom->pose.pose.orientation.y != prev_pose.pose.pose.orientation.y
                || odom->pose.pose.orientation.z != prev_pose.pose.pose.orientation.z
                || odom->pose.pose.orientation.w != prev_pose.pose.pose.orientation.w;
    }
public:
    ObjectDetector();
    void frame_callback(const sensor_msgs::Image::ConstPtr& rgb, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom);
    void publish_objects_vis();
    void sigint_handler(int);
    // Memory object
    Memory memory;
};


#endif //OBJECT_DETECT_OBJECT_DETECT_NODE_SYNC_H
