/*
 * Author: Nihal Sandadi
 * This is a header file a combined header file which declared all the 
 * functions and classes for CameraCalibrator, VirtualObjects, and CheckerboardDetector
 */

#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class CameraCalibrator {
public:
    CameraCalibrator();
    bool calibrateCamera(const std::vector<std::vector<cv::Point2f>>& cornersList,
        const std::vector<std::vector<cv::Vec3f>>& pointsList,
        cv::Size imageSize);
    void saveCalibration(const std::string& filename);
    bool loadCalibration(const std::string& filename);

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    double reprojectionError;
    bool isCalibrated = false;

private:
    std::vector<cv::Mat> rvecs, tvecs;
};

class VirtualObjects {
public:
    VirtualObjects();
    void renderAll(cv::Mat& image, const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs, const cv::Mat& rvec,
        const cv::Mat& tvec);
    void updateAnimation();

private:
    std::vector<cv::Point3f> cubePoints;
    std::vector<std::pair<int, int>> cubeConnections;
    std::vector<cv::Point3f> pyramidPoints;
    std::vector<std::pair<int, int>> pyramidConnections;
    std::vector<cv::Point3f> tetraPoints;
    std::vector<std::pair<int, int>> tetraConnections;
    double animationTime = 0.0;

    void createCube();
    void createPyramid();
    void createTetrahedron();
    void renderObject(cv::Mat& image, const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs, const cv::Mat& rvec,
        const cv::Mat& tvec,
        const std::vector<cv::Point3f>& points,
        const std::vector<std::pair<int, int>>& connections,
        const cv::Scalar& color);
};

class CheckerboardDetector {
public:
    CheckerboardDetector(cv::Size boardSize = cv::Size(9, 6));
    bool detect(const cv::Mat& image, std::vector<cv::Point2f>& corners);
    void drawDetectedCorners(cv::Mat& image, const std::vector<cv::Point2f>& corners);

    cv::Size boardSize;
    std::vector<cv::Vec3f> worldPoints;

private:
    void generateWorldPoints();
};