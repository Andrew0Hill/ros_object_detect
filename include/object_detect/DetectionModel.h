/*
 * DetectionModel -
 *
 * Encapsulates the functionality of a TensorFlow object detection model.
 * This code can use models from the TensorFlow Model Zoo at:
 * https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md
 *
 * Contains a class (DetectionModel) which can be instantiated to create a new TF session, load a frozen model graph,
 * and perform image object detection based on that model graph.
 */

#ifndef OBJECT_DETECT_DETECTIONMODEL_H
#define OBJECT_DETECT_DETECTIONMODEL_H

#define MODEL_PATH  "frozen_inference_graph.pb"
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define IMAGE_CHANNELS 3
// TensorFlow includes
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/framework/types.h"
#include "DetectedObject.h"

// OpenCV include
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility>
#include <cstdint>


// Use abbreviated namespace name for readability.
namespace tf = tensorflow;
class DetectionModel {
public:
    DetectionModel(std::string model_path=MODEL_PATH,
                   int im_width=IMAGE_WIDTH,
                   int im_height=IMAGE_HEIGHT,
                   int im_channels=IMAGE_CHANNELS);
    ~DetectionModel(){
        std::cout << "Closing TensorFlow Session." << std::endl;
        session->Close();
    }
    std::vector<std::shared_ptr<DetectedObject>> detectImage(cv::Mat &image);
    void shutdownSession();

private:
    // Holds the current TensorFlow session.
    tf::Session* session;

    // Tensor to hold the input image.
    tf::Tensor image_tensor;

    // cv::Mat mapped to the image_tensor's memory
    cv::Mat image_mat;

    // Holds the graph definition for the TensorFlow session.
    tf::GraphDef graphDef;

    // Vector to hold the results of a Session->run call.
    std::vector<tf::Tensor> output_tensors;

    int width,height,dims;

    // Helper function to filter the results of a Session->run by creating a vector of
    // DetectedObjects only for objects which pass a confidence threshold.
    std::vector<std::shared_ptr<DetectedObject>> get_valid_objects(std::vector<tf::Tensor> &output_tensors, float thresh = 0.5);

    void set_abs_coords(std::shared_ptr<DetectedObject> obj, float* boxes, int row);

    void set_mask(std::shared_ptr<DetectedObject> obj, float* mask, int row);

    void set_image_roi(std::shared_ptr<DetectedObject> obj, float* boxes);
    // Reads a frozen graph from file.
    void readModelFromFile(std::string filename);

    // Initialize the TF session member with a graph definition.
    void buildSession(tf::GraphDef graph);
};


#endif //OBJECT_DETECT_DETECTIONMODEL_H
