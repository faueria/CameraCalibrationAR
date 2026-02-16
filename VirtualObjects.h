/*
 * Author: Nihal Sandadi
 * This is the header file for the virtual objects class which holds data on the AR objects I use.
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class VirtualObjects {
public:
    VirtualObjects();
    void renderAll(cv::Mat& image, const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs, const cv::Mat& rvec,
        const cv::Mat& tvec);
    void updateAnimation();

private:
    // Cube
    std::vector<cv::Point3f> cubePoints;
    std::vector<std::pair<int, int>> cubeConnections;

    // Pyramid
    std::vector<cv::Point3f> pyramidPoints;
    std::vector<std::pair<int, int>> pyramidConnections;

    // Tetrahedron
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