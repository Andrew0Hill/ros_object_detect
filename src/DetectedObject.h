//
// Created by ros on 3/18/18.
//

#ifndef OBJECT_DETECT_DETECTEDOBJECT_H
#define OBJECT_DETECT_DETECTEDOBJECT_H

#include <unordered_map>
#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>
#include "ClassMap.h"

class DetectedObject {
public:
    int oclass,ymin,ymax,xmin,xmax;
    float score;
    cv::Mat mask;
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
    static bool class_exists(int c_num){ return (ClassMap::class_map.find(c_num) != ClassMap::class_map.end()); }
    // Gets the class string associated with a class number.
    static std::string get_class(int c_num){ return ClassMap::class_map.at(c_num); }
};


#endif //OBJECT_DETECT_DETECTEDOBJECT_H
