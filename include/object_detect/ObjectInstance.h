//
// Created by ros on 3/20/18.
//

#ifndef OBJECT_DETECT_OBJECTINSTANCE_H
#define OBJECT_DETECT_OBJECTINSTANCE_H


#include <opencv-3.3.1/opencv2/core/mat.hpp>
#include <memory>
#include <opencv2/xfeatures2d.hpp>
#include "DetectedObject.h"
class DetectedObject;
class ObjectInstance {
public:
    // ID to uniquely represent this object within its Type.
    int oid;
    // Vector of DetectedObject instances for this ObjectInstance.
    std::vector<std::shared_ptr<DetectedObject>> images;
    ObjectInstance(int id, std::shared_ptr<DetectedObject> obj_ptr){
        oid = id;
        images.push_back(obj_ptr);
    }
    void add_image(std::shared_ptr<DetectedObject> obj_ptr){
        images.push_back(obj_ptr);
    }
    int get_id(){return oid;}
};


#endif //OBJECT_DETECT_OBJECTINSTANCE_H
