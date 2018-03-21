//
// Created by ros on 3/1/18.
//

#include "object_detect_node.h"
int frame_num = 0;
std::shared_ptr<DetectionModel> dm;
ros::Publisher p;
auto featurizer = std::make_shared<ORB_Featurizer>();
void frame_callback(const sensor_msgs::ImageConstPtr& rgb) {
    // Get image pointer
    cv_bridge::CvImagePtr  im_ptr = cv_bridge::toCvCopy(rgb);
    std::vector<std::shared_ptr<DetectedObject>> objects;
    // Object detection on image.
    objects = dm->detectImage(im_ptr->image);

    cv::namedWindow("test_window");
    // Print out lit of objects detected in frame
    for (int i = 0; i < objects.size(); ++i){
        // Featurize RGB image.
        std::vector<cv::KeyPoint> kps;
        cv::Mat descriptors;
        featurizer->featurizeImage(objects[i]->image,kps,descriptors);
        // Print # of Keypoints
        std::cout << kps.size() << std::endl;

        // If enough features?

        // Get depth image ROI and centroid

        // Match object into memory.

        // Print out object name and info.
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
    dm = std::make_shared<DetectionModel>();

    image_transport::Subscriber s = itnode.subscribe("/camera/rgb/image_raw",1,frame_callback);
    p = node.advertise<std_msgs::String>("test_topic/",1);
    ros::spin();
    cv::destroyAllWindows();
}

