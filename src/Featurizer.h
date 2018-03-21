//
// Created by ros on 3/21/18.
//

#ifndef OBJECT_DETECT_FEATUREMATCHER_H
#define OBJECT_DETECT_FEATUREMATCHER_H


#include <opencv2/core.hpp>

class Featurizer {
public:
    virtual void featurizeImage(cv::Mat& image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) = 0;

};


#endif //OBJECT_DETECT_FEATUREMATCHER_H
