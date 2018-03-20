//
// Created by ros on 3/1/18.
//

#include "object_detect_node.h"
int frame_num = 0;
DetectionModel dm;
ros::Publisher p;
void frame_callback(const sensor_msgs::ImageConstPtr& rgb) {
    cv_bridge::CvImagePtr  im_ptr = cv_bridge::toCvCopy(rgb);
    std::vector<std::shared_ptr<DetectedObject>> objects;
    objects = dm.detectImage(im_ptr->image);
    for (int i = 0; i < objects.size(); ++i){
        std::cout << objects[i]->to_string() << std::endl;
    }
    std_msgs::String s;
    s.data = "test";
    p.publish(s);
    frame_num++;
}

int main(int argc, char** argv) {
    // Initialize the ROS node.
    ros::init(argc, argv, "hybrid_matcher");
    ros::NodeHandle node;
    image_transport::ImageTransport itnode(node);
    dm = DetectionModel();

    image_transport::Subscriber s = itnode.subscribe("/camera/rgb/image_raw",1,frame_callback);
    p = node.advertise<std_msgs::String>("test_topic/",1);
    ros::spin();
}

