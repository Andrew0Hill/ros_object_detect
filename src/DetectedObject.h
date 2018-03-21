//
// Created by ros on 3/18/18.
//

#ifndef OBJECT_DETECT_DETECTEDOBJECT_H
#define OBJECT_DETECT_DETECTEDOBJECT_H

#include <unordered_map>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include "ClassMap.h"


// TODO: Should this replace FeatureImage??
// Just add members for feature descriptors, image ROI, etc.
class DetectedObject {
public:
    // Object class and bounding box coordinates
    int oclass,ymin,ymax,xmin,xmax;
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

    DetectedObject(int c_num){
        oclass = c_num;
    }
    std::string to_string(){
        std::stringstream stream;
        stream << print_detected_class() << print_bounding_box();
        return stream.str();
    }
    std::string print_bounding_box(){
        std::stringstream stream;
        stream << "Bounding Box: (" << xmin << "," << ymin << ") (" << xmax << "," << ymax << ")" << std::endl;
        return stream.str();
    }
    std::string print_detected_class(){
        std::stringstream stream;
        stream << "Detected Class: " << get_class() << std::endl;
        return stream.str();
    }
    std::string get_class() {return ClassMap::class_map.at(oclass);}
    // Returns true if the class number exists in the class map, and false otherwise.
    // TODO: Move this outside of this class and into ClassMap (static member function)
    static bool class_exists(int c_num){ return (ClassMap::class_map.find(c_num) != ClassMap::class_map.end()); }
    // Gets the class string associated with a class number.
    // TODO: Also move this outside of the class.
    static std::string get_class(int c_num){ return ClassMap::class_map.at(c_num); }
};


#endif //OBJECT_DETECT_DETECTEDOBJECT_H
