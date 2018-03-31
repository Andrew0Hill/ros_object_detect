//
// Created by ros on 3/22/18.
//

#ifndef OBJECT_DETECT_DESCRIPTOR_H
#define OBJECT_DETECT_DESCRIPTOR_H


#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <set>
#include "ObjectInstance.h"
#include "DetectedObject.h"
#include <rosconsole/macros_generated.h>
/*
 * Descriptor
 *
 * Descriptor is a high-level wrapper for pairs of
 * cv::Mat (A descriptor) and cv::KeyPoint (A keypoint in an image).
 */

class Descriptor {
public:
    // cv::Mat for a descriptor.
    // Each Descriptor instance refers to a single descriptor vector.
    cv::Mat desc;
    // Vector of cv::KeyPoint for this Descriptor. A Descriptor may have any number of
    // KeyPoints, as a Descriptor can appear in multiple images.
    std::set<std::shared_ptr<cv::KeyPoint>> keypoints;

    std::set<std::shared_ptr<DetectedObject>> image_set;
    // Map to map between DetectedObjects and indices in that
    // DetectedObject's descriptor and keypoint sets.
    std::unordered_map<std::shared_ptr<DetectedObject>,std::shared_ptr<cv::KeyPoint>> obj_ind;
    void setDescriptor(cv::Mat ndesc){
        desc = ndesc;
    }
    void addParent(std::shared_ptr<DetectedObject> parent, std::shared_ptr<cv::KeyPoint> keypoint){
        image_set.insert(parent);
        keypoints.insert(keypoint);
        obj_ind[parent] = keypoint;

    }
    void removeParent(std::shared_ptr<DetectedObject> parent){
        // Erase DetectedObject from the parent image.
        image_set.erase(parent);
        // If the DetectedObject had an associated KeyPoint (it should if it's here),
        // Find it using the map and remove it.
        if (obj_ind.find(parent) != obj_ind.end()){
            keypoints.erase(obj_ind.at(parent));
        }
    }
    // Returns an iterator to all of the parents of this Descriptor
    void getParent(std::set<std::shared_ptr<DetectedObject>>::iterator &it, std::set<std::shared_ptr<DetectedObject>>::iterator &end){
        it = image_set.begin();
        end = image_set.end();
    }
    cv::Mat getDescriptor(){
        return desc;
    }
    std::shared_ptr<cv::KeyPoint> getKeyPoint(std::shared_ptr<DetectedObject> det_obj){
        //std::cout << "IN getKeyPoint." << std::endl;
        return obj_ind.at(det_obj);
    }
};


#endif //OBJECT_DETECT_DESCRIPTOR_H
