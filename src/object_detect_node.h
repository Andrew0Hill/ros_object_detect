//
// Created by ros on 3/1/18.
//

#ifndef OBJECT_DETECT_NODE_H
#define OBJECT_DETECT_NODE_H

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include "DetectionModel.h"
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
void frame_callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth);

#endif //OBJECT_DETECT_NODE_H
