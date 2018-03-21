//
// Created by ros on 3/20/18.
//

#ifndef OBJECT_DETECT_OBJECTINSTANCE_H
#define OBJECT_DETECT_OBJECTINSTANCE_H


#include <opencv-3.3.1/opencv2/core/mat.hpp>
#include <memory>
#include <opencv2/xfeatures2d.hpp>
class ObjectInstance {
    // ID to uniquely represent this object within its Type.
    int oid;
    std::vector<std::shared_ptr<cv::Mat>> images;
};


#endif //OBJECT_DETECT_OBJECTINSTANCE_H
