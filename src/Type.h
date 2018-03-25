//
// Created by ros on 3/19/18.
//

#ifndef OBJECT_DETECT_TYPE_H
#define OBJECT_DETECT_TYPE_H


#include <memory>
#include "DetectedObject.h"
#include "ObjectInstance.h"
#include <rosconsole/macros_generated.h>
#include <ros/assert.h>
#include <opencv2/calib3d.hpp>
#include <unordered_map>
#define NN_THRESH 0.75
#define INLIERS_THRESH 5
class Type {
public:
    // ID number to uniquely identify this type.
    // This should be the ID number that represents the Type's
    // class in the ClassMap
    int id;


    // Boolean value to determine if this type should be used for loop closures.
    bool use_loop_closure;

    // Static variable to allocate IDs to DetectedObjects in this
    // Type.
    int alloc_id;

    std::unordered_map<int,int> desc_map;
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20,15,2));
    // TODO: Vector of cv::Mat for each bounded image.
    // Use cv::Mat constructor to just store reference to the cv::Mat
    // inside the DetectedObject

    // TODO: Vector of ObjectInstance for this type.
    std::vector<std::shared_ptr<ObjectInstance>> obj_insts;

    std::vector<std::shared_ptr<DetectedObject>> objs;
    // TODO: Vector of Descriptors for this type.
    cv::Mat descriptors;
    // TODO: Vector of cv::KeyPoint for this type.
    // Do we actually need this?
    std::vector<cv::KeyPoint> keypoints;


    // Constructor requires that an ID is specified.
    Type(int oid){
        id = oid;
        alloc_id = 0;
        ROS_INFO_STREAM("New type added for class: " << ClassMap::get_class(id));
    }

    // Function to allocate IDs to new ObjectInstances in this type.
    int allocate_id(){return alloc_id++;}

    // Function to match a given DetectedObject against all of the
    // DetectedObject instances in this type.
    void match(std::shared_ptr<DetectedObject> object);

    // Function to add a new ObjectInstance to this type.
    void add_new_obj(std::shared_ptr<DetectedObject> object);


};


#endif //OBJECT_DETECT_TYPE_H
