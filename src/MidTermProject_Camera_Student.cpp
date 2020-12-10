/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

void initialize_log_vector(std::vector<Log_Database>& total_log_data) {
    const std::vector<std::string> detectorTypes{ "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT" };
    const std::vector<std::string> descriptorTypes{ "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" };
    const std::vector<std::string> matcherTypes{ "MAT_BF" };
    const std::vector<std::string> selectorTypes{ "SEL_KNN" };

    for(auto detType : detectorTypes)
        for(auto descType: descriptorTypes)
            for(auto matType: matcherTypes)
                for (auto selType : selectorTypes) {
                    if ((descType.compare("AKAZE") == 0 && detType.compare("AKAZE") != 0)||
                        (descType.compare("ORB") == 0 && detType.compare("SIFT") == 0))
                        
                    {
                        continue;
                    }
                    Log_Database temp(detType, descType, matType, selType);
                    total_log_data.push_back(temp);
                }
}

void writeToFile(const std::vector<Log_Database>& total_log_data)
{
    std::string fileName{ "../log/Diwakar_Manickavelu_MidtermProject.csv" };
    std::cout << "Writing to O/P file" << fileName << std::endl;

    std::ofstream file{ fileName };
    file << "Image Index: " << ",";
    file << "Detector Type: " << ",";
    file << "Descriptor Type: " << ",";
    file << "Matcher Type: " << ",";
    file << "Selector Type: " << ",";
    file << "Detection timetaken: " << ",";
    file << "Total No. of detected points: " << ",";
    file << "Num. of Filtered points: " << ",";
    file << "Descriptor timetaken: " << ",";
    file << "Matcher timetaken: " << ",";
    file << "No. of Matched pairs: " << ",";
    file << std::endl;

    for (auto &log_data : total_log_data)
    {
        for (size_t i = 0; i < log_data.det_timetaken.size(); i++)
        {
            file << i << ",";
            file << log_data.detectorType << ",";
            file << log_data.descriptorType << ",";
            file << log_data.matcherType << ",";
            file << log_data.selectorType << ",";
            file << log_data.det_timetaken[i] << ",";
            file << log_data.total_kpts[i] << ",";
            file << log_data.filtered_kpts[i] << ",";
            file << log_data.desc_timetaken[i] << ",";
            file << log_data.mat_timetaken[i] << ",";
            file << log_data.matched_pairs_n[i] << ",";
            file << std::endl;
        }
        file << std::endl;
    } file.close();
}
/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    std::vector<Log_Database> total_log_data;
    initialize_log_vector(total_log_data);

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    for (auto &log_data : total_log_data)
    {
        dataBuffer.clear();

        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
        {
            /* LOAD IMAGE INTO BUFFER */

            // assemble filenames for current index
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // load image from file and convert to grayscale
            cv::Mat img, imgGray;
            img = cv::imread(imgFullFilename);
            cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

            //// STUDENT ASSIGNMENT
            //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = imgGray;
            dataBuffer.push_back(frame);
            while (dataBuffer.size() > dataBufferSize)
                dataBuffer.erase(dataBuffer.begin());
            cout<< "Image index: " << (imgIndex + 1)<< std::endl;
            //// EOF STUDENT ASSIGNMENT
            cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

            /* DETECT IMAGE KEYPOINTS */

            // extract 2D keypoints from current image
            vector<cv::KeyPoint> keypoints; // create empty feature list for current image
            string detectorType = log_data.detectorType;

            //// STUDENT ASSIGNMENT
            //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
            //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
            std::pair<int, double> detInfo;

            if (detectorType.compare("SHITOMASI") == 0)
            {
                detInfo = detKeypointsShiTomasi(keypoints, imgGray, false);
            }
            else if (detectorType.compare("HARRIS") == 0)
            {
                detInfo = detKeypointsHarris(keypoints, imgGray, false);
            }
            else
            {
                detInfo = detKeypointsModern(keypoints, imgGray, detectorType, false);
            }
            log_data.total_kpts[imgIndex] = detInfo.first;
            log_data.det_timetaken[imgIndex] = detInfo.second;
            //// EOF STUDENT ASSIGNMENT

            //// STUDENT ASSIGNMENT
            //// TASK MP.3 -> only keep keypoints on the preceding vehicle

            // only keep keypoints on the preceding vehicle
            bool bFocusOnVehicle = true;
            cv::Rect vehicleRect(535, 180, 180, 150);
            if (bFocusOnVehicle)
            {
                keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
                    [&](const cv::KeyPoint& x) {return !vehicleRect.contains(x.pt); }), keypoints.end());
            }
            log_data.filtered_kpts[imgIndex] = keypoints.size();

            //// EOF STUDENT ASSIGNMENT

            // optional : limit number of keypoints (helpful for debugging and learning)
            bool bLimitKpts = false;
            if (bLimitKpts)
            {
                int maxKeypoints = 50;

                if (detectorType.compare("SHITOMASI") == 0)
                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                cout << " NOTE: Keypoints have been limited!" << endl;
            }

            // push keypoints and descriptor for current frame to end of data buffer
            (dataBuffer.end() - 1)->keypoints = keypoints;
            cout << "#2 : DETECT KEYPOINTS done" << endl;

            /* EXTRACT KEYPOINT DESCRIPTORS */

            //// STUDENT ASSIGNMENT
            //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
            //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

            cv::Mat descriptors;
            string descriptorType = log_data.descriptorType; // BRIEF, ORB, FREAK, AKAZE, SIFT
            double tt = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
            log_data.desc_timetaken[imgIndex] = tt;
            //// EOF STUDENT ASSIGNMENT

            // push descriptors for current frame to end of data buffer
            (dataBuffer.end() - 1)->descriptors = descriptors;

            cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

            if (dataBuffer.size() > 1) // wait until at least two images have been processed
            {

                /* MATCH KEYPOINT DESCRIPTORS */

                vector<cv::DMatch> matches;
                string matcherType = log_data.matcherType;        // MAT_BF, MAT_FLANN
                string descriptorKind;
                if (log_data.descriptorType == "SIFT")
                    descriptorKind = "DES_HOG";
                else
                    descriptorKind = "DES_BINARY";
                //string descriptorKind{ ((log_data.descriptorType.compare("SIFT") == 0) ? "DES_HOG" : "DES_BINARY") }; // DES_BINARY, DES_HOG
                string selectorType = log_data.selectorType;       // SEL_NN, SEL_KNN

                //// STUDENT ASSIGNMENT
                //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                std::pair<int, double> matInfo = matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                    (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                    matches, descriptorKind, matcherType, selectorType);
                log_data.matched_pairs_n[imgIndex] = matInfo.first;
                log_data.mat_timetaken[imgIndex] = matInfo.second;
                //// EOF STUDENT ASSIGNMENT

                // store matches in current data frame
                (dataBuffer.end() - 1)->kptMatches = matches;

                cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                // visualize matches between current and previous image
                bVis = false;
                if (bVis)
                {
                    cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                    cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                        matches, matchImg,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                    string windowName = "Matching keypoints between two camera images";
                    cv::namedWindow(windowName, 7);
                    cv::imshow(windowName, matchImg);
                    cout << "Press key to continue to next image" << endl;
                    cv::waitKey(0); // wait for key to be pressed
                }
                bVis = false;
            }

        } // eof loop over all images
    }
    writeToFile(total_log_data);
    return 0;
}
