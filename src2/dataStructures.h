#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

struct Log_Database { // represents the log information of every possible combinations of detector/descriptor types
    
    std::string detectorType, descriptorType, matcherType, selectorType;

    std::array<double, 10> det_timetaken, desc_timetaken, mat_timetaken;
    std::array<int, 10> total_kpts, filtered_kpts, matched_pairs_n;
    //constructor
    Log_Database(std::string detectorT, std::string descriptorT, std::string matcherT, std::string selectorT)
        : detectorType{detectorT}, descriptorType{descriptorT}, matcherType{matcherT}, selectorType{selectorT} {}

};

#endif /* dataStructures_h */
