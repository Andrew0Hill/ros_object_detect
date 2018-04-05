//
// Created by ros on 3/18/18.
//

#ifndef OBJECT_DETECT_DETECTEDOBJECT_H
#define OBJECT_DETECT_DETECTEDOBJECT_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include "ClassMap.h"
#include <Eigen/Core>
#include "ObjectInstance.h"
#include "Descriptor.h"

class ObjectInstance;

class Descriptor;

// TODO: Should this replace FeatureImage??
// Just add members for feature descriptors, image ROI, etc.
class DetectedObject {
public:
    // Object class and bounding box coordinates
    int oclass,ymin,ymax,xmin,xmax,id;
    // Confidence value (0-1) for the detected object.
    float score;
    // Mask for the detected object
    cv::Mat mask;
    // Image for the detected object
    cv::Mat image;
    // Vector of keypoints for the detected object's image
    std::vector<cv::KeyPoint> keypoints;
    // Matrix of descriptors for the image;
    cv::Mat descriptors;
    // Pointer to parent ObjectInstance
    std::shared_ptr<ObjectInstance> parent;
    // Set of pointers to descriptors for this DetectedObject.
    std::set<std::shared_ptr<Descriptor>> desc_list;
    // Position in the world frame (/odom right now) of the object.
    Eigen::Matrix<double,4,1> world_pos;


    DetectedObject(int c_num){
        oclass = c_num;
    }
    void set_parent(std::shared_ptr<ObjectInstance> parent){
        this->parent = parent;
    }

    std::string to_string(){
        std::stringstream stream;
        stream << print_detected_class() << print_bounding_box();
        return stream.str();
    }
    std::string print_bounding_box(){
        std::stringstream stream;
        stream << "Bounding Box: (" << xmin << "," << ymin << ") (" << xmax << "," << ymax << ")";
        return stream.str();
    }
    std::string print_detected_class(){
        std::stringstream stream;
        stream << "Detected Class: " << get_class();
        return stream.str();
    }
    std::string get_class() {return ClassMap::get_class(oclass);}
    // Returns true if the class number exists in the class map, and false otherwise.
    // TODO: Move this outside of this class and into ClassMap (static member function)

    // Gets the class string associated with a class number.
    // TODO: Also move this outside of the class.

};


#endif //OBJECT_DETECT_DETECTEDOBJECT_H
