//
// Created by ros on 3/21/18.
//

#ifndef OBJECT_DETECT_SURF_FEATURIZER_H
#define OBJECT_DETECT_SURF_FEATURIZER_H


#include "Featurizer.h"
#include <opencv2/xfeatures2d.hpp>
class SURF_Featurizer : Featurizer {
public:
    void featurizeImage(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) override;
};


#endif //OBJECT_DETECT_SURF_FEATURIZER_H
