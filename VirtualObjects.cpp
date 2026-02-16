/*
 * Author: Nihal Sandadi
 * This class have the point data for all the different virtual objects I have created for this project
 */

#include "CameraCalibrationAR.h"
#include <cmath>

VirtualObjects::VirtualObjects() {
    createCube();
    createPyramid();
    createTetrahedron();
}

/*
 * Creates 3D points and connections for a cube object
 */
void VirtualObjects::createCube() {
    cubePoints.clear();
    cubeConnections.clear();

    float s = 0.5f;
    cubePoints = {
        cv::Point3f(-s, -s, 0), cv::Point3f(s, -s, 0),
        cv::Point3f(s,  s, 0), cv::Point3f(-s,  s, 0),
        cv::Point3f(-s, -s, 1), cv::Point3f(s, -s, 1),
        cv::Point3f(s,  s, 1), cv::Point3f(-s,  s, 1)
    };

    cubeConnections = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };
}

/*
 * Creates 3D points and connections for a pyramid object
 */
void VirtualObjects::createPyramid() {
    pyramidPoints.clear();
    pyramidConnections.clear();

    float baseSize = 0.6f;
    float height = 1.2f;

    pyramidPoints = {
        cv::Point3f(-baseSize, -baseSize, 0),
        cv::Point3f(baseSize, -baseSize, 0),
        cv::Point3f(baseSize,  baseSize, 0),
        cv::Point3f(-baseSize,  baseSize, 0),
        cv::Point3f(0, 0, height)
    };

    pyramidConnections = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {0, 4}, {1, 4}, {2, 4}, {3, 4}
    };
}

/*
 * Creates 3D points and connections for a tetrahedron object
 */
void VirtualObjects::createTetrahedron() {
    tetraPoints.clear();
    tetraConnections.clear();

    float size = 0.7f;

    tetraPoints = {
        cv::Point3f(0, 0, size),
        cv::Point3f(-size, -size, 0),
        cv::Point3f(size, -size, 0),
        cv::Point3f(0, size, 0)
    };

    tetraConnections = {
        {1, 2}, {2, 3}, {3, 1},
        {0, 1}, {0, 2}, {0, 3}
    };
}

/*
 * Updates animation time for object movements
 */
void VirtualObjects::updateAnimation() {
    animationTime += 0.05;
}

/*
 * image : Frame where object will be rendered
 * cameraMatrix : Camera intrinsic parameters
 * distCoeffs : Camera distortion coefficients
 * rvec : Rotation vector for object placement
 * tvec : Translation vector for object placement
 * points : 3D points defining the object geometry
 * connections : Lines between points to form object edges
 * color : Color for rendering the object edges
 * Renders a single 3D object onto the image frame
 */
void VirtualObjects::renderObject(cv::Mat& image, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, const cv::Mat& rvec,
    const cv::Mat& tvec,
    const std::vector<cv::Point3f>& points,
    const std::vector<std::pair<int, int>>& connections,
    const cv::Scalar& color) {
    if (points.empty()) return;

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    for (const auto& connection : connections) {
        int startIdx = connection.first;
        int endIdx = connection.second;

        if (startIdx < imagePoints.size() && endIdx < imagePoints.size()) {
            cv::line(image, imagePoints[startIdx], imagePoints[endIdx], color, 2);
        }
    }

    for (const auto& point : imagePoints) {
        cv::circle(image, point, 2, cv::Scalar(255, 255, 255), -1);
    }
}

/*
 * image : Frame where objects will be rendered
 * cameraMatrix : Camera intrinsic parameters
 * distCoeffs : Camera distortion coefficients
 * rvec : Rotation vector for object placement
 * tvec : Translation vector for object placement
 * Renders all virtual objects at different positions with animation
 */
void VirtualObjects::renderAll(cv::Mat& image, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, const cv::Mat& rvec,
    const cv::Mat& tvec) {
    cv::Mat cubeRvec = rvec.clone();
    cv::Mat cubeTvec = tvec.clone();
    cubeTvec.at<double>(0) += 2.0;

    cv::Mat pyramidRvec = rvec.clone();
    cv::Mat pyramidTvec = tvec.clone();
    pyramidTvec.at<double>(1) -= 2.0;

    cv::Mat tetraRvec = rvec.clone();
    cv::Mat tetraTvec = tvec.clone();
    tetraTvec.at<double>(0) += 2.0;
    tetraTvec.at<double>(1) -= 2.0;

    double wobble = sin(animationTime) * 0.1;
    pyramidRvec.at<double>(2) += wobble;
    tetraRvec.at<double>(0) += wobble;

    renderObject(image, cameraMatrix, distCoeffs, cubeRvec, cubeTvec,
        cubePoints, cubeConnections, cv::Scalar(255, 0, 0));

    renderObject(image, cameraMatrix, distCoeffs, pyramidRvec, pyramidTvec,
        pyramidPoints, pyramidConnections, cv::Scalar(0, 255, 0));

    renderObject(image, cameraMatrix, distCoeffs, tetraRvec, tetraTvec,
        tetraPoints, tetraConnections, cv::Scalar(0, 0, 255));
}