/*
 * Author: Nihal Sandadi
 * This program looks at and calculates the features for Harris Corners, ORB, and SIFT
 */

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <sstream>

/*
 * gray : Input grayscale image for corner detection
 * corners : Output vector of detected corner positions
 * threshold : Sensitivity threshold for corner detection
 * Detects Harris corners and applies non-maximum suppression
 */
void detectHarrisCorners(const cv::Mat& gray, std::vector<cv::Point2f>& corners, int threshold) {
    cv::Mat dst, dst_norm;
    dst = cv::Mat::zeros(gray.size(), CV_32FC1);

    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;

    cv::cornerHarris(gray, dst, blockSize, apertureSize, k);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    double maxVal;
    cv::minMaxLoc(dst_norm, NULL, &maxVal, NULL, NULL);
    double thresholdValue = maxVal * threshold / 200.0;

    for (int i = 1; i < dst_norm.rows - 1; i++) {
        for (int j = 1; j < dst_norm.cols - 1; j++) {
            if ((int)dst_norm.at<float>(i, j) > thresholdValue) {
                bool isLocalMax = true;
                float centerVal = dst_norm.at<float>(i, j);

                for (int di = -1; di <= 1 && isLocalMax; di++) {
                    for (int dj = -1; dj <= 1 && isLocalMax; dj++) {
                        if (di == 0 && dj == 0) continue;
                        if (dst_norm.at<float>(i + di, j + dj) >= centerVal) {
                            isLocalMax = false;
                        }
                    }
                }

                if (isLocalMax) {
                    corners.push_back(cv::Point2f(j, i));
                }
            }
        }
    }
}

/*
 * gray : Input grayscale image for feature detection
 * keypoints : Output vector of detected ORB keypoints
 * maxFeatures : Maximum number of features to detect
 * Detects ORB features in the input image
 */
void detectORBFeatures(const cv::Mat& gray, std::vector<cv::KeyPoint>& keypoints, int maxFeatures) {
    cv::Ptr<cv::ORB> orbDetector = cv::ORB::create();
    orbDetector->setMaxFeatures(maxFeatures);
    orbDetector->detect(gray, keypoints);
}

/*
 * gray : Input grayscale image for feature detection
 * keypoints : Output vector of detected SIFT keypoints
 * threshold : Response threshold for filtering keypoints
 * Detects SIFT features and filters by response threshold
 */
void detectSIFTFeatures(const cv::Mat& gray, std::vector<cv::KeyPoint>& keypoints, double threshold) {
    cv::Ptr<cv::SIFT> siftDetector = cv::SIFT::create();
    siftDetector->detect(gray, keypoints);

    std::vector<cv::KeyPoint> filteredKeypoints;
    for (const auto& kp : keypoints) {
        if (kp.response > threshold) {
            filteredKeypoints.push_back(kp);
        }
    }
    keypoints = filteredKeypoints;
}

/*
 * image : Image where features will be visualized
 * corners : Detected Harris corners to draw
 * keypoints : Detected keypoints to draw
 * featureType : Type of feature detector used
 * featureCount : Number of features detected
 * threshold : Current threshold parameter value
 * Draws detected features and UI information on the image
 */
void drawFeaturesAndUI(cv::Mat& image, const std::vector<cv::Point2f>& corners,
    const std::vector<cv::KeyPoint>& keypoints, int featureType,
    int featureCount, int threshold) {
    cv::Mat result = image.clone();

    // Draw features based on type
    switch (featureType) {
    case 0: // Harris corners
        for (const auto& corner : corners) {
            cv::circle(result, corner, 4, cv::Scalar(0, 0, 255), -1);
        }
        break;
    case 1: // ORB features
        cv::drawKeypoints(result, keypoints, result, cv::Scalar(0, 255, 0),
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        break;
    case 2: // SIFT features
        cv::drawKeypoints(result, keypoints, result, cv::Scalar(255, 0, 0),
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        break;
    }

    // Display feature information
    std::string featureInfo;
    switch (featureType) {
    case 0: featureInfo = "Harris Corners: " + std::to_string(featureCount); break;
    case 1: featureInfo = "ORB Features: " + std::to_string(featureCount); break;
    case 2: featureInfo = "SIFT Features: " + std::to_string(featureCount); break;
    default: featureInfo = "No Feature Detection";
    }

    cv::putText(result, featureInfo, cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

    // Display current parameter
    std::string paramInfo = "Threshold: " + std::to_string(threshold);
    cv::putText(result, paramInfo, cv::Point(10, 60),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

    // Display instructions
    cv::putText(result, "Press 1:Harris 2:ORB 3:SIFT c:Clear s:Save q:Quit",
        cv::Point(10, result.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5,
        cv::Scalar(255, 255, 255), 1);

    image = result;
}

/*
 * controlPanel : Control panel image to update
 * featureInfo : Current feature detection information
 * paramInfo : Current parameter information
 * Updates the control panel window with current settings
 */
void updateControlPanel(cv::Mat& controlPanel, const std::string& featureInfo, const std::string& paramInfo) {
    controlPanel = cv::Mat::zeros(200, 400, CV_8UC3);
    cv::putText(controlPanel, "Feature Detection Controls", cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    cv::putText(controlPanel, "Current: " + featureInfo, cv::Point(10, 60),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(controlPanel, "Parameter: " + paramInfo, cv::Point(10, 80),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(controlPanel, "Adjust parameters using trackbars above", cv::Point(10, 110),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
}

/*
 * output : Integer exit status
 * Main function demonstrating robust feature detection algorithms
 */
int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "Error: Cannot open webcam!" << std::endl;
        return -1;
    }

    int featureType = 0;
    int harrisThreshold = 100;
    int orbMaxFeatures = 500;
    int siftThresholdInt = 3;

    cv::namedWindow("Feature Detection", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Control Panel", cv::WINDOW_AUTOSIZE);

    cv::Mat controlPanel = cv::Mat::zeros(200, 400, CV_8UC3);

    cv::createTrackbar("Harris Threshold", "Control Panel", &harrisThreshold, 200);
    cv::createTrackbar("ORB Max Features", "Control Panel", &orbMaxFeatures, 1000);
    cv::createTrackbar("SIFT Threshold x100", "Control Panel", &siftThresholdInt, 100);

    std::cout << "Robust Feature Detection Demo Started" << std::endl;
    std::cout << "Controls: 1=Harris, 2=ORB, 3=SIFT, c=Clear, s=Save, q=Quit" << std::endl;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat displayFrame = frame.clone();
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::KeyPoint> keypoints;
        std::vector<cv::Point2f> corners;
        int featureCount = 0;
        double siftThreshold = siftThresholdInt / 100.0;

        // Perform feature detection
        switch (featureType) {
        case 0:
            detectHarrisCorners(gray, corners, harrisThreshold);
            featureCount = corners.size();
            break;
        case 1:
            detectORBFeatures(gray, keypoints, orbMaxFeatures);
            featureCount = keypoints.size();
            break;
        case 2:
            detectSIFTFeatures(gray, keypoints, siftThreshold);
            featureCount = keypoints.size();
            break;
        }

        // Get current parameter for display
        int currentThreshold = 0;
        switch (featureType) {
        case 0: currentThreshold = harrisThreshold; break;
        case 1: currentThreshold = orbMaxFeatures; break;
        case 2: currentThreshold = siftThresholdInt; break;
        }

        drawFeaturesAndUI(displayFrame, corners, keypoints, featureType, featureCount, currentThreshold);

        // Update control panel
        std::string featureInfo;
        switch (featureType) {
        case 0: featureInfo = "Harris Corners: " + std::to_string(featureCount); break;
        case 1: featureInfo = "ORB Features: " + std::to_string(featureCount); break;
        case 2: featureInfo = "SIFT Features: " + std::to_string(featureCount); break;
        default: featureInfo = "No Feature Detection";
        }
        std::string paramInfo = "Parameter: " + std::to_string(currentThreshold);
        updateControlPanel(controlPanel, featureInfo, paramInfo);

        cv::imshow("Feature Detection", displayFrame);
        cv::imshow("Control Panel", controlPanel);

        char key = cv::waitKey(1);
        if (key == 'q') break;
        else if (key == '1') featureType = 0;
        else if (key == '2') featureType = 1;
        else if (key == '3') featureType = 2;
        else if (key == 'c') featureType = -1;
        else if (key == 's') {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::tm tm;
            localtime_s(&tm, &time_t);

            std::stringstream filename;
            filename << "feature_detection_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".png";
            cv::imwrite(filename.str(), displayFrame);
            std::cout << "Saved: " << filename.str() << std::endl;
        }
    }

    return 0;
}