# AR Camera Calibration and Virtual Objects
I am using 2 travel days
## Overview
This project showcases how to show simple but diverse objects in an AR fashion using open cv’s AR libraries. This is based on a checkered board which acts like a background for open cv functions to recognize and project the AR objects onto that checkered board. Using this and calibration images, we can show different objects at the same time. Later on we also explore features like Harris corners, ORB, and SIFT and talk about how they can be used to improve on this opencv process.

---

## Features
- **Camera Calibration**: Calculate camera intrinsic parameters using checkerboard pattern  
- **Real-time Pose Estimation**: Track checkerboard position and orientation  
- **Multiple Virtual Objects**: Display cube, pyramid, and tetrahedron simultaneously  
- **Interactive Controls**: Save calibration images, toggle objects, and adjust settings  

---

## Requirements
- OpenCV 4.x  
- C++17 compiler  
- Webcam  
- Checkerboard pattern (9x6 internal corners)  

---

## How to Run

### Compilation
build with visual studio and run the executable for this project