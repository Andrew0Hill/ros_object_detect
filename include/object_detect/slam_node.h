//
// Created by ros on 4/15/18.
//

#ifndef OBJECT_DETECT_SLAM_NODE_H
#define OBJECT_DETECT_SLAM_NODE_H
#define NN_THRESH 0.75
#define MIN_FEATURES 25
#define INLIERS_THRESH 20
#include <memory>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
#include <time.h>
#include "ORB_Featurizer.h"
#include "PoseGraph.h"
#include "FeaturizedImage.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZRGB>, nav_msgs::Odometry> frame_sync_policy;

class SLAMNode {
private:
    int frame_num;
    bool first_measurement;
    // Publishers
    ros::Publisher graph_pub, cloud_pub;
    // Image featurizer
    std::shared_ptr<ORB_Featurizer> featurizer;
    // TF2 transform buffer
    std::shared_ptr<tf2_ros::Buffer> buffer;
    // TF2 transform listener
    std::shared_ptr<tf2_ros::TransformListener> listener;
    // PCL ICP implementation for stitching clouds and Loop Closure
    pcl::IterativeClosestPoint<pcl::PointNormal,pcl::PointNormal> icp_gen;
    // PCL Normal Estimation to improve ICP.
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_estimator;
    // PCL KD-Tree pointer for Normal Estimation.
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree;
    // PCL VoxelGrid implementation for filtering point clouds.
    pcl::VoxelGrid<pcl::PointXYZRGB> filter_grid;
    // Pointer to a Pose Graph
    std::shared_ptr<PoseGraph> poseGraph;
    // Pointer to the pose cloud (The entire stitched cloud).
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr poses_cloud;
    // ROS Node Handle
    ros::NodeHandle node;
    // Vector of Featurized Images
    //boost::circular_buffer<std::shared_ptr<FeaturizedImage>> images;
    std::vector<std::shared_ptr<FeaturizedImage>> images;
    // ROS Message Filter subscribers.
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>>* cloud_sub;
    message_filters::Subscriber<sensor_msgs::Image>* image_sub;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub;
    message_filters::Synchronizer<frame_sync_policy>* synchronizer;
    // FLANN-based matcher.
    cv::FlannBasedMatcher matcher;

public:
    SLAMNode();
    void frame_callback(const sensor_msgs::Image::ConstPtr& rgb, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom);
    // Output file streams for performance analysis
    std::ofstream analytics_file;
};


#endif //OBJECT_DETECT_SLAM_NODE_H
