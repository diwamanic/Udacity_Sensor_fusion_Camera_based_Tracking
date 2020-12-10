## Project Rubric points are addressed here

#### MP.1 Data Buffer Optimization
- When the Databuffer size exceeds the ring buffer size, the databuffer's first element is removed continuously until it is equal to the ring buffer size

#### MP.2 Keypoint Detection
- Struct Log_Database is additionally created in the dataStructures.h for logging the information into .csv file for performance evaluation of different 
combinations
- Initialized the combinations of detector/descriptors in a vector of struct Log_Database
- Looping through each combination, keypoint detection gets the corresponding member detectorType each time for each different combination

#### MP.3 Keypoint Removal
- Next, the obtained keypoints are filtered through the rectangle definition and vector::erase function and std::remove_if.

#### MP.4 Keypoint Descriptors
- Looping through each combination, keypoint descriptor gets the corresponding member descriptorType each time for each different combination
- The corresponding descriptor's definition is implemented in the matching2D_Student.cpp

#### MP.5 Descriptor Matching
- Looping through each combination, descriptor matching gets the corresponding members matcherType and selectorType each time for each different combination
- The corresponding matchers's definition is implemented in the matching2D_Student.cpp accordingly.

#### MP.6 Descriptor Distance Ratio
- With the threshold for descriptor distance ratio, the outlier correspondences are considerably eliminated.

#### MP.7 Performance Evaluation
- Along with all the information for logging, the number or keypoints, number of filtered keypoints are also entered in the logfile.

#### MP.8 Performance Evaluation 2
- Along with all the information for logging, the number of keypoint matches are also entered in the logfile.

#### MP.9 Performance Evaluation 3
- The following three best choices, I find suitable for the purpose of detecting keypoints on vehicles.
    * ) Detector - ORB; Descriptor - BRISK
    * ) Detector - ORB; Descriptor - ORB
    * ) Detector - HARRIS; Descriptor - ORB

These choices are chosen after analysing the .csv file, considering the factors as total time taken calculation and number of matched pairs.