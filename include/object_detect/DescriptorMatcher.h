//
// Created by ros on 3/26/18.
//

#ifndef OBJECT_DETECT_DESCRIPTORLIST_H
#define OBJECT_DETECT_DESCRIPTORLIST_H
#define INLIERS_THRESH 7
#define NN_THRESH 0.75
//#define DRAW_MATCHED_IMAGES
// Class that wraps a set of descriptors for a Type.

#include <map>
#include "Descriptor.h"
#include <rosconsole/macros_generated.h>
#include <opencv2/calib3d/calib3d.hpp>
class DescriptorMatcher {
public:
    // Returns the best matching ObjectInstance, or a pointer to a new ObjectInstance
    bool match(std::shared_ptr<DetectedObject>);
    DescriptorMatcher(){
        // Set up params for matching based on ORB descriptors (binary descriptors)
        matcher = cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20,15,2));

    }
private:
    // Maps from an index in the descriptor list to a high_level descriptor.
    std::unordered_map<int,std::shared_ptr<Descriptor>> descriptor_map;
    // FLANN-based matcher for matching descriptors.
    cv::FlannBasedMatcher matcher;
    // List of raw descriptors.
    cv::Mat descriptor_list;
    // Vector of high-level descriptors.
    //std::vector<Descriptor> descriptor_vec;
    // Matches a cv::Mat of descriptors to a high-level set of Descriptor objects.

    // Get a pointer to a high-level Descriptor given a index in the descriptor list.
    std::shared_ptr<Descriptor> get_descriptor(int index);

    void get_descriptors(std::vector<int> &indices, std::vector<std::shared_ptr<Descriptor>> &out_vec);

    std::shared_ptr<DetectedObject> get_best_img_match(std::vector<std::shared_ptr<Descriptor>> &descriptors, std::vector<std::shared_ptr<Descriptor>> &filtered_descriptors, std::vector<int> &filtered_indices);

    void get_img_keypoints(std::shared_ptr<DetectedObject> obj, std::vector<std::shared_ptr<Descriptor>> descriptors, std::vector<cv::KeyPoint> &keypoints);

    int get_inliers(std::vector<cv::KeyPoint> &train_kps, std::vector<cv::KeyPoint> &query_kps);

    void add_reference(std::vector<std::shared_ptr<Descriptor>> update_descs, std::shared_ptr<DetectedObject> new_obj, std::vector<cv::KeyPoint> new_kps);

    void create_descriptors(std::shared_ptr<DetectedObject> new_obj, std::vector<int> &rejected_indices, cv::Mat &descriptors, std::vector<cv::KeyPoint> &rejected_keypoints);

    void create_descriptors(std::shared_ptr<DetectedObject> new_obj, cv::Mat &descriptors, std::vector<cv::KeyPoint> &keypoints);
};


#endif //OBJECT_DETECT_DESCRIPTORLIST_H
