//
// Created by ros on 3/18/18.
//

#include "DetectionModel.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
int main(){
    // Create a new DetectionModel instance
    std::cout << "Creating Detection Model" << std::endl;
    DetectionModel dm = DetectionModel();
    std::cout << "Reading Image" << std::endl;
    cv::Mat test_image = cv::imread("test_image.jpg",cv::IMREAD_COLOR);
    if (!test_image.data) {
        std::cout << "Error loading image." << std::endl;
        exit(1);
    }
    std::cout << "Detecting Image" << std::endl;
    dm.detectImage(test_image);
    return 0;
}
