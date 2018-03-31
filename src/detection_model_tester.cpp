//
// Created by ros on 3/18/18.
//

#include "../include/object_detect/DetectionModel.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../include/object_detect/detection_model_tester.h"

int main(){
    // Create a new DetectionModel instance
    auto featurizer = std::make_shared<ORB_Featurizer>();
    std::cout << "Reading Image" << std::endl;
    cv::Mat test_image = cv::imread("test_image.jpg",cv::IMREAD_COLOR);
    if (!test_image.data) {
        std::cout << "Error loading image." << std::endl;
        exit(1);
    }

    std::cout << "Creating Detection Model" << std::endl;
    DetectionModel dm = DetectionModel(MODEL_PATH,test_image.cols,test_image.rows);
    std::cout << "Read image of width: " << test_image.cols << " height: " << test_image.rows;
    std::cout << "Detecting Image" << std::endl;
    std::vector<std::shared_ptr<DetectedObject>> objects;
    objects = dm.detectImage(test_image);
    cv::namedWindow("image window");
    for (int i = 0; i < objects.size(); ++i){
        //cv::Mat image_and_mask;
        //objects[i]->image.copyTo(image_and_mask,objects[i]->mask);
        //cv::imshow("image window",objects[i]->image);
        //cv::waitKey();
        std::vector<cv::KeyPoint> kps;
        cv::Mat descriptors;
        featurizer->featurizeImage(objects[i]->image,kps,descriptors);

        cv::Mat draw_kps;
        std::cout << kps.size() << std::endl;
        cv::drawKeypoints(objects[i]->image,kps,objects[i]->image);
        cv::imshow("test_window", draw_kps);
        cv::waitKey(1);
        //cv::imshow("image window",image_and_mask);
        //cv::waitKey();
    }
    cv::destroyAllWindows();

    return 0;
}
