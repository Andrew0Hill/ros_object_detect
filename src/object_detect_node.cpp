//
// Created by ros on 3/1/18.
//

#include "object_detect_node.h"
#include "tensorflow/core/public/session.h"
int frame_num = 0;
int main(int argc, char** argv) {
    // Initialize the ROS node.
    ros::init(argc, argv, "hybrid_matcher");
    ros::NodeHandle node;
    // Subscriber for color image.
    message_filters::Subscriber<sensor_msgs::Image> rgb_s(node,"/camera/rgb/image_raw",1);
    // Subscriber for depth image
    //message_filters::Subscriber<sensor_msgs::Image> depth_s(node,"/camera/depth/points/image_raw",1);

    //message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_s(rgb_s,depth_s,10);

    //sync_s.registerCallback(boost::bind(&frame_callback, _1,_2));
    //ros::Subscriber<sensor_msgs::Image> subscriber = ros::Subscriber()
    ros::spin();
}

void frame_callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth) {
    cv_bridge::CvImagePtr  im_ptr = cv_bridge::toCvCopy(rgb);
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth);
    cv::Mat new_img;
    depth_ptr->image.convertTo(depth_ptr->image,CV_8UC3, 0.0039);
    cv::cvtColor(depth_ptr->image, new_img, cv::COLOR_GRAY2BGR);
    cv::addWeighted(im_ptr->image, 0.15, new_img, 0.85, 0.0, new_img);
    //cv::imshow("RGB", new_img);
    cv::imwrite("img_"+std::to_string(frame_num)+".jpg", new_img);
    cv::waitKey(3);
    frame_num++;
}