//
// Created by ros on 3/18/18.
//

#ifndef OBJECT_DETECT_DETECTIONMODEL_H
#define OBJECT_DETECT_DETECTIONMODEL_H

#define MODEL_PATH  "frozen_inference_graph.pb"
#define IMAGE_WIDTH 1024
#define IMAGE_HEIGHT 636
#define IMAGE_CHANNELS 3
// TensorFlow includes
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/framework/types.h"
#include "DetectedObject.h"

// OpenCV include
#include <opencv2/core.hpp>

#include <utility>
#include <cstdint>
/*
 * DetectionModel -
 *
 * Encapsulates the functionality of a TensorFlow object detection model.
 * This code can use models from the TensorFlow Model Zoo at:
 * https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md
 *
 */

// Use abbreviated namespace name for readability.
namespace tf = tensorflow;
class DetectionModel {
public:
    DetectionModel(std::string model_path=MODEL_PATH,
                   int im_width=IMAGE_WIDTH,
                   int im_height=IMAGE_HEIGHT,
                   int im_channels=IMAGE_CHANNELS);
    std::vector<DetectedObject> detectImage(cv::Mat image);


private:
    // Holds the current TensorFlow session.
    tf::Session* session;

    // Tensor to hold the input image.
    tf::Tensor image_tensor;

    // cv::Mat mapped to the image_tensor's memory
    cv::Mat image_mat;

    // Holds the graph definition for the TensorFlow session.
    tf::GraphDef graphDef;

    std::vector<tf::Tensor> output_tensors;

    std::vector<DetectedObject> get_valid_objects(std::vector<tf::Tensor> output_tensors);
    // Reads a frozen graph from file.
    void readModelFromFile(std::string filename);

    // Initialize the TF session member with a graph definition.
    void buildSession(tf::GraphDef graph);
};


#endif //OBJECT_DETECT_DETECTIONMODEL_H
