//
// Created by ros on 3/31/18.
//

#ifndef OBJECT_DETECT_GRAPH_BUILDER_H
#define OBJECT_DETECT_GRAPH_BUILDER_H
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
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <tf2_ros/transform_listener.h>
void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_ptr);

void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

void graph_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom);
void build_graph();
#endif //OBJECT_DETECT_GRAPH_BUILDER_H
