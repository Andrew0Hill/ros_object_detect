//
// Created by ros on 3/1/18.
//

#ifndef OBJECT_DETECT_NODE_H
#define OBJECT_DETECT_NODE_H

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <tf2_ros/transform_listener.h>
#include "DetectionModel.h"
#include "ORB_Featurizer.h"
#include "Memory.h"
#include <uchar.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <signal.h>
//#include <pcl-1.7/pcl/registration/icp.h>
#define MIN_FEATURE_NUM 50
/*
+--------+----+----+----+----+------+------+------+------+
|        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
+--------+----+----+----+----+------+------+------+------+
| CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
| CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
| CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
| CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
| CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
| CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
| CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
+--------+----+----+----+----+------+------+------+------+
 */
void frame_callback(const sensor_msgs::Image::ConstPtr& rgb, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom);
void publish_objects_vis();
void on_sigint(int code);
#endif //OBJECT_DETECT_NODE_H
