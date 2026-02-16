/*
 * Author: Nihal Sandadi
 * This program looks and finds the checkerboard on the current camera frame
 */

#include "CameraCalibrationAR.h"

CheckerboardDetector::CheckerboardDetector(cv::Size boardSize) : boardSize(boardSize) {
    generateWorldPoints();
}

/*
 * output : None
 * Generates 3D world points for checkerboard corners
 */
void CheckerboardDetector::generateWorldPoints() {
    worldPoints.clear();
    for (int y = 0; y < boardSize.height; y++) {
        for (int x = 0; x < boardSize.width; x++) {
            worldPoints.push_back(cv::Vec3f(x, -y, 0));
        }
    }
}

/*
 * image : Input image to search for checkerboard
 * corners : Output vector of detected corner positions
 * output : Boolean indicating success
 * Detects checkerboard corners in the input image
 */
bool CheckerboardDetector::detect(const cv::Mat& image, std::vector<cv::Point2f>& corners) {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = image.clone();
    }

    bool found = cv::findChessboardCorners(gray, boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }

    return found;
}

/*
 * image : Image where corners will be drawn
 * corners : Detected corner positions to visualize
 * Draws detected checkerboard corners on the image
 */
void CheckerboardDetector::drawDetectedCorners(cv::Mat& image, const std::vector<cv::Point2f>& corners) {
    if (!corners.empty()) {
        cv::drawChessboardCorners(image, boardSize, cv::Mat(corners), true);
    }
}