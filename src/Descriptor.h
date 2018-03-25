//
// Created by ros on 3/22/18.
//

#ifndef OBJECT_DETECT_DESCRIPTOR_H
#define OBJECT_DETECT_DESCRIPTOR_H


#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

/*
 * Descriptor
 *
 * Descriptor is a high-level wrapper for pairs of
 * cv::Mat (A descriptor) and cv::KeyPoint (A keypoint in an image).
 */

class Descriptor {
    cv::Mat desc;
};


#endif //OBJECT_DETECT_DESCRIPTOR_H
