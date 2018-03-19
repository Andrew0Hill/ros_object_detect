//
// Created by ros on 3/18/18.
//

#include "DetectionModel.h"

void DetectionModel::readModelFromFile(std::string filename) {
    // Read the file from filename into the graphDef and set the status variable
    tf::Status s = tf::ReadBinaryProto(tf::Env::Default(), filename, &graphDef);
    if(!s.ok())
        std::cout << "ERROR: Failed to read frozen graph. " << s.error_message() << std::endl;
}

void DetectionModel::buildSession(tensorflow::GraphDef graph) {
    // Build the TF session.
    tf::SessionOptions opt;
    session = tf::NewSession(opt);
    tf::Status s = session->Create(graph);
    if (!s.ok())
        std::cout << "ERROR: Failed to create TF session." << s.error_message() << std::endl;
}
/*
 * detectImage
 *
 * Detects the objects in a given image (cv::Mat) using the TF model.
 *
 * To prevent a slowdown from copying the cv::Mat into a Tensor,
 * we maintain an internal cv::Mat that is mapped on top of the input
 * image Tensor.
 */
std::vector<DetectedObject> DetectionModel::detectImage(cv::Mat image) {
    //std::cout << image.size();
    image.convertTo(image_mat,CV_8UC3);
    //std::cout << image_mat.size();
    //std::cout << (int) image_tensor.flat<tf::uint8>().data()[0] << std::endl;
    tf::Status s = session->Run({std::pair<std::string,tf::Tensor>("image_tensor:0",image_tensor)}, // Input image tensor
                 {"detection_boxes:0","detection_scores:0","detection_classes:0","detection_masks:0","num_detections:0"}, // Set output tensors
                 {},
                 &output_tensors); // Vector to hold output tensors.

    if (!s.ok()){
        std::cout << "ERROR: Error during detection. " << s << std::endl;
    }

    std::cout << output_tensors[0].flat<float>()(400);

}

DetectionModel::DetectionModel(std::string model_path, int im_width, int im_height, int im_channels) {
    std::cout << "Reading model from file" << std::endl;
    readModelFromFile(model_path);
    std::cout << "Building Session" << std::endl;
    buildSession(graphDef);
    // Set up image_tensor
    std::cout << "Making new image tensor object" << std::endl;
    image_tensor = tf::Tensor(tf::DT_UINT8,tf::TensorShape({1,im_height,im_width,im_channels}));
    // Set up memory-mapped cv::Mat
    std::cout << "Making new cv::Mat object" << std::endl;
    image_mat = cv::Mat(im_height,im_width,CV_8UC3,image_tensor.flat<tf::uint8>().data());
}

std::vector<DetectedObject> DetectionModel::get_valid_objects(std::vector<tf::Tensor> output_tensors) {
    // Detection with masks
    if (output_tensors.size() == 5){

    }
    // Detection without masks
    else if (output_tensors.size() == 4){

    }
    // Invalid size.
    else{

    }

    return std::vector<DetectedObject>();
}




