//
// Created by ros on 4/15/18.
//

#ifndef OBJECT_DETECT_FEATURIZEDIMAGE_H
#define OBJECT_DETECT_FEATURIZEDIMAGE_H


#include <opencv2/core/mat.hpp>
#include "Pose.h"

class FeaturizedImage {
public:
    cv::Mat image;
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    std::shared_ptr<Pose> pose;
};


#endif //OBJECT_DETECT_FEATURIZEDIMAGE_H
