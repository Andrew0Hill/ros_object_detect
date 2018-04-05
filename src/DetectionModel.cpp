//
// Created by ros on 3/18/18.
//

#include <opencv-3.3.1/opencv2/highgui.hpp>
#include <opencv-3.3.1/opencv2/features2d.hpp>
#include "../include/object_detect/DetectionModel.h"

void DetectionModel::readModelFromFile(std::string filename) {
    // Read the file from filename into the graphDef and set the status variable
    tensorflow::Status s = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), filename, &graphDef);
    if(!s.ok()) {
        std::cout << "ERROR: Failed to read frozen graph. " << s.error_message() << std::endl;
        exit(-1);
    }
}

void DetectionModel::buildSession(tensorflow::GraphDef graph) {
    // Build the TF session.
    tensorflow::SessionOptions opt;
    session = tensorflow::NewSession(opt);
    tensorflow::Status s = session->Create(graph);
    if (!s.ok()) {
        std::cout << "ERROR: Failed to create TF session." << s.error_message() << std::endl;
        exit(-1);
    }
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
std::vector<std::shared_ptr<DetectedObject>> DetectionModel::detectImage(cv::Mat &image) {
    // Move the input image into the memory-mapped image_mat which points
    // to the same memory as our input tensor.
    image.convertTo(image_mat,CV_8UC3);

    // Run the session. Pass in the image_tensor, and get the results from the operation.
    tensorflow::Status s = session->Run({std::pair<std::string,tensorflow::Tensor>("image_tensor:0",image_tensor)}, // Input image tensor
                 {"detection_boxes:0","detection_scores:0","detection_classes:0","detection_masks:0","num_detections:0"}, // Set output tensors
                 {},
                 &output_tensors); // Vector to hold output tensors.


    if (!s.ok()){
        std::cout << "ERROR: Error during detection. " << s << std::endl;
        exit(-1);
    }
    return get_valid_objects(output_tensors);
}

DetectionModel::DetectionModel(std::string model_path, int im_width, int im_height, int im_channels) {
    // Save the dimensions of the image we will be detecting for later reference.
    width = im_width;
    height = im_height;
    dims = im_channels;
    std::cout << "Reading Model from file" << std::endl;
    // Read the frozen model from file.
    readModelFromFile(model_path);
    std::cout << "Building session" << std::endl;
    // Build the session using the graphDef we read the file into.
    buildSession(graphDef);
    // Set up image_tensor
    image_tensor = tensorflow::Tensor(tensorflow::DT_UINT8,tensorflow::TensorShape({1,im_height,im_width,im_channels}));
    // Set up memory-mapped cv::Mat
    image_mat = cv::Mat(im_height,im_width,CV_8UC3,image_tensor.flat<tensorflow::uint8>().data());
}

std::vector<std::shared_ptr<DetectedObject>> DetectionModel::get_valid_objects(std::vector<tensorflow::Tensor> &output_tensors, float thresh) {
    // Get pointers to each of the Tensor objects.
    float* boxes = output_tensors[0].flat<float>().data();
    float* scores = output_tensors[1].flat<float>().data();
    float* classes = output_tensors[2].flat<float>().data();
    float* masks = output_tensors[3].flat<float>().data();
    float num_detections = output_tensors[4].flat<float>().data()[0];

    std::vector<std::shared_ptr<DetectedObject>> detected_objects = std::vector<std::shared_ptr<DetectedObject>>();
    for(int i = 0; i < num_detections; ++i){
        // The scores tensor is sorted highest->lowest.
        // If we come across a score that is lower than the threshold,
        // we can skip checking all other scores after this score at well.
        if (scores[i] < thresh)
            break;
        // Check that the detected object class exists in our class map.
        if (ClassMap::class_exists(static_cast<int>(classes[i]))){
            // If so, make a new DetectedObject of this class type.
            std::shared_ptr<DetectedObject> obj(new DetectedObject(static_cast<int>(classes[i])));
            // Calculate the absolute coordinates of the object's position in the frame.
            set_abs_coords(obj,boxes,i*4);
            // Calculate the mask coordinates of this object position in the frame.
            set_mask(obj,masks,i*15*15);
            // Set the image ROI from the image vector
            set_image_roi(obj,boxes);
            // Add detected object to the objects vector.
            detected_objects.push_back(obj);
        }
    }
    // Return the vector of detected objects.
    return detected_objects;
}
/*
 * set_abs_coords
 *
 * Helper function to calculate the aboslute image coordinates of a Object's bounding box, given
 * four relative coordinate floats [0,1].
 * Modifies the DetectedObject directly to avoid copy-in & copy-out.
 */
void DetectionModel::set_abs_coords(std::shared_ptr<DetectedObject> obj, float* boxes, int row) {
    obj->ymin = static_cast<int> (height * boxes[row]);
    obj->xmin = static_cast<int> (width * boxes[row+1]);
    obj->ymax = static_cast<int> (height * boxes[row+2]);
    obj->xmax = static_cast<int> (width * boxes[row+3]);
}
/*
 * set_mask
 *
 * Helper function to calculate the mask of an image given its bounding box.
 * The TF network returns masks as a 15x15 tensor, which we must stretch into the size
 * of the image's bounding box.
 *
 * Modifies the DetectedObject's mask variable directly for speed.
 */
void DetectionModel::set_mask(std::shared_ptr<DetectedObject> obj, float *mask, int row) {
    // Create a cv::Mat that refers to the 15x15 mask tensor
    cv::Mat small_mat = cv::Mat(15,15,CV_32FC1,mask+row);
    // Calculate the width and height of the bounding box
    int x_size = obj->xmax - obj->xmin;
    int y_size = obj->ymax - obj->ymin;
    // Resize the 15x15 matrix based on the size of the bounding box
    cv::resize(small_mat,obj->mask,cv::Size(x_size,y_size),0,0,cv::INTER_CUBIC);
    // Threshold to turn float values into 0 or 255.
    cv::threshold(obj->mask,obj->mask,0.5,255,cv::THRESH_BINARY);
    // Convert mask to a 1-channel, 8-bit type (A mask image).
    obj->mask.convertTo(obj->mask,CV_8UC1);
}
/*
 * set_image_roi
 *
 * Helper function to set the image ROI matrix of the detection object.
 * Because image_mat is a pointer and gets reset after every call to detectImage, we need to
 * copy the current value of the ROI from image_mat into a new matrix.
 */
void DetectionModel::set_image_roi(std::shared_ptr<DetectedObject> obj, float *boxes) {
    // Calculate the width and height of the bounding box
    int x_size = obj->xmax - obj->xmin;
    int y_size = obj->ymax - obj->ymin;
    // Create a rectangle ROI for our image.
    cv::Rect roi(obj->xmin,obj->ymin,x_size,y_size);
    // Create a new ROI cv::Mat that points to the ROI inside image_mat
    cv::Mat roi_image = cv::Mat(image_mat,roi);
    // Copy the ROI into the objects cv::Mat
    roi_image.copyTo(obj->image);
}

void DetectionModel::shutdownSession() {
    session->Close();
    std::cout << "Closing TensorFlow Session." << std::endl;
}




