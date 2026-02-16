/*
 * Author: Nihal Sandadi
 * This program looks at and calculates the features for Harris Corners, ORB, and SIFT
 */

#include "CameraCalibrationAR.h"
#include <iostream>
#include <vector>
#include <sstream>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "Error: Cannot open webcam!" << std::endl;
        return -1;
    }

    CameraCalibrator calibrator;
    VirtualObjects virtualObjects;
    CheckerboardDetector detector(cv::Size(9, 6));

    std::vector<std::vector<cv::Point2f>> cornerList;
    std::vector<std::vector<cv::Vec3f>> pointList;
    std::vector<cv::Point2f> lastCorners;
    cv::Mat lastGoodFrame;
    bool lastFrameHasCorners = false;

    int calibrationCount = 0;
    bool showPose = false;
    bool showObjects = true;
    cv::Size imageSize;

    std::cout << "AR Multiple Objects Demo Started" << std::endl;
    std::cout << "Controls: s=save, b=calibrate, w=write, l=load, v=toggle objects, p=toggle pose, q=quit" << std::endl;

    cv::Mat rvec, tvec;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        if (imageSize.width == 0) {
            imageSize = frame.size();
        }

        cv::Mat displayFrame = frame.clone();
        std::vector<cv::Point2f> corners;
        bool found = detector.detect(frame, corners);

        if (found) {
            detector.drawDetectedCorners(displayFrame, corners);
            lastCorners = corners;
            lastGoodFrame = frame.clone();
            lastFrameHasCorners = true;

            if (calibrator.isCalibrated) {
                bool pnpSuccess = cv::solvePnP(detector.worldPoints, corners,
                    calibrator.cameraMatrix, calibrator.distCoeffs,
                    rvec, tvec);

                if (pnpSuccess) {
                    virtualObjects.updateAnimation();

                    if (showObjects) {
                        virtualObjects.renderAll(displayFrame, calibrator.cameraMatrix,
                            calibrator.distCoeffs, rvec, tvec);
                    }

                    if (showPose) {
                        std::stringstream poseText;
                        poseText << "Pose - R: [" << std::fixed << std::setprecision(2)
                            << rvec.at<double>(0) << ", " << rvec.at<double>(1) << ", " << rvec.at<double>(2)
                            << "] T: [" << tvec.at<double>(0) << ", " << tvec.at<double>(1) << ", " << tvec.at<double>(2) << "]";
                        cv::putText(displayFrame, poseText.str(), cv::Point(10, 150),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
                    }
                }
            }
        }
        else {
            lastFrameHasCorners = false;
        }

        std::stringstream info;
        info << "Calib Images: " << calibrationCount << "/5";
        cv::putText(displayFrame, info.str(), cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        if (found) {
            cv::putText(displayFrame, "Checkerboard Found - Press 's' to save",
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }

        if (calibrator.isCalibrated) {
            std::string calStatus = "Calibrated! Error: " + std::to_string(calibrator.reprojectionError).substr(0, 5) + " px";
            cv::putText(displayFrame, calStatus, cv::Point(10, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        }

        if (showObjects && calibrator.isCalibrated && found) {
            cv::putText(displayFrame, "Objects: Cube, Pyramid, Tetrahedron", cv::Point(10, 120),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        }

        cv::imshow("AR Multiple Objects Demo", displayFrame);

        char key = cv::waitKey(1);
        if (key == 'q') break;
        else if (key == 's' && lastFrameHasCorners) {
            cornerList.push_back(lastCorners);
            pointList.push_back(detector.worldPoints);
            calibrationCount++;
            std::cout << "Saved calibration image #" << calibrationCount << std::endl;
        }
        else if (key == 'b' && calibrationCount >= 5) {
            std::cout << "Calibrating camera..." << std::endl;
            if (calibrator.calibrateCamera(cornerList, pointList, imageSize)) {
                std::cout << "Calibration complete! Error: " << calibrator.reprojectionError << " pixels" << std::endl;
            }
        }
        else if (key == 'w' && calibrator.isCalibrated) {
            calibrator.saveCalibration("camera_calibration.xml");
            std::cout << "Calibration saved to file" << std::endl;
        }
        else if (key == 'l') {
            if (calibrator.loadCalibration("camera_calibration.xml")) {
                std::cout << "Calibration loaded from file" << std::endl;
            }
        }
        else if (key == 'p') showPose = !showPose;
        else if (key == 'v') showObjects = !showObjects;
    }

    return 0;
}