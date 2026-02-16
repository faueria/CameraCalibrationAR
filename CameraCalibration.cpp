/*
 * Author: Nihal Sandadi
 * This program calibrates the camera with certain saved calibration images
 */

#include "CameraCalibrationAR.h"
#include <iostream>

CameraCalibrator::CameraCalibrator() {
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
}

/*
 * cornersList : List of detected 2D corner points from calibration images
 * pointsList : List of corresponding 3D world points for calibration
 * imageSize : Size of the calibration images in pixels
 * output : Boolean indicating success
 * Performs camera calibration using the provided points and images
 */
bool CameraCalibrator::calibrateCamera(const std::vector<std::vector<cv::Point2f>>& cornersList,
    const std::vector<std::vector<cv::Vec3f>>& pointsList,
    cv::Size imageSize) {
    if (cornersList.size() < 5) return false;

    cameraMatrix.at<double>(0, 2) = imageSize.width / 2.0;
    cameraMatrix.at<double>(1, 2) = imageSize.height / 2.0;

    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3;
    reprojectionError = cv::calibrateCamera(pointsList, cornersList, imageSize,
        cameraMatrix, distCoeffs, rvecs, tvecs, flags);

    isCalibrated = true;
    return true;
}

/*
 * filename : Path where calibration parameters will be saved
 * Saves camera calibration parameters to an XML file
 */
void CameraCalibrator::saveCalibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs << "reprojection_error" << reprojectionError;
    }
}

/*
 * filename : Path to load calibration parameters from
 * output : Boolean indicating success
 * Loads camera calibration parameters from an XML file
 */
bool CameraCalibrator::loadCalibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs["reprojection_error"] >> reprojectionError;

    isCalibrated = true;
    return true;
}