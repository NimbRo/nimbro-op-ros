// Soccer Vision Gui
// Author: Julio Pastrana <pastrana@ais.uni-bonn.de>

// camera_parameters.cpp
// Defines the constants required to distort and undistort points in the camera.
#include "camera_parameters.h"

//
// CamParam struct
//

// Notes:
// The constants were obtained using the OpenCV camera calibration (version 2.4.6) using a checkerboard pattern
// fx and fy are the camera focal lengths
// cx and cy specify the camera optical center
// k1 k2 p1 p2 k3 k4 k5 k6 are the distortion parameters
//
// Refer to:
// http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

// Camera resolution
const float CamParam::rx = 800;
const float CamParam::ry = 600;

// Camera parameters
const float CamParam::fx =  277.248051116677;
const float CamParam::fy =  277.012844558782;
const float CamParam::cx =  409.527988669458;
const float CamParam::cy =  290.346002896436;

// Radial distortion parameters
const float CamParam::k1 = -0.256287819037122;
const float CamParam::k2 =  0.0384571071799440;
const float CamParam::k3 =  0.00565974661973100;
const float CamParam::k4 =  0.0453204862850670;
const float CamParam::k5 = -0.0678832605487869;
const float CamParam::k6 =  0.0254666472507643;

// Tangential distortion parameters
const float CamParam::p1 =  7.18632832098473e-04;
const float CamParam::p2 = -4.62466522195010e-04;

// Linear distortion extension parameters (calculated based on the other OpenCV-generated constants using calcparams.m)
const float CamParam::api =  8.0;
const float CamParam::apo =  14.0;
const float CamParam::ani = -8.0;
const float CamParam::ano = -12.0;
const float CamParam::bpi =  8.0;
const float CamParam::bpo =  12.0;
const float CamParam::bni = -8.0;
const float CamParam::bno = -14.0;
const float CamParam::mx =  63.1076285607051;
const float CamParam::bx =  332.989362945858;
const float CamParam::my =  62.7702450488603;
const float CamParam::by =  406.812571523206;
// EOF