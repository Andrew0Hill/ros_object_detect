//
// Created by ros on 3/1/18.
//

#include "object_detect_node.h"


int frame_num = 0;
std::shared_ptr<DetectionModel> dm;
ros::Publisher p;
auto featurizer = std::make_shared<ORB_Featurizer>();
Memory memory;
void frame_callback(const sensor_msgs::ImageConstPtr& rgb) {
    // Get image pointer
    cv_bridge::CvImagePtr  im_ptr = cv_bridge::toCvCopy(rgb);
    std::vector<std::shared_ptr<DetectedObject>> objects;
    // Object detection on image.
    objects = dm->detectImage(im_ptr->image);

    //cv::namedWindow("test_window");
    // Print out list of objects detected in frame
    for (int i = 0; i < objects.size(); ++i){
        // Featurize RGB image.
        featurizer->featurizeImage(objects[i]->image,objects[i]->keypoints,objects[i]->descriptors);
        // Print # of Keypoints
        //std::cout << kps.size() << std::endl;

        // If enough features?
        if (objects[i]->keypoints.size() < MIN_FEATURE_NUM){
            ROS_INFO_STREAM("Object does not contain enough features! Need: " << MIN_FEATURE_NUM << " Found: " << objects[i]->keypoints.size());
            continue;
        }
        std::cout << "Object has: " << objects[i]->descriptors.cols << " columns and: " << objects[i]->descriptors.rows << " rows." << std::endl;
        //std::cout << objects[i] -> descriptors;
        // Get depth image ROI and centroid
        // Match object into memory.
        ROS_INFO_STREAM("Matching object into Memory...");
        memory.match(objects[i]);
        // Print out object name and info.
        ROS_INFO_STREAM(objects[i]->to_string());
        //std::cout << objects[i]->to_string() << std::endl;
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
}

