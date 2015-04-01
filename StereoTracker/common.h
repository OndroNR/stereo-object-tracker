#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#if defined(WIN32) || defined(_WIN32) 
#define PATH_SEPARATOR "\\" 
#else 
#define PATH_SEPARATOR "/" 
#endif 

using namespace cv;

struct StereoPair
{
	cv::Mat frames[2];
	double timestamp;
};

struct StereoCalibration
{
	Mat cameraMatrix[2];
	Mat distCoeffs[2];
	Mat R; // Rotation matrix between the 1st and the 2nd cameras coordinate systems
	Mat T; // Translation vector between the cameras coordinate systems
	Mat E; // Essential matrix
	Mat F; // Fundamental matrix
	Size chessboard_size;
	double chessboard_square_size;

	double rms; // reprojection error

	StereoCalibration()
	{
		cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
		cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	}
};

struct RectificationParams
{
    Mat R1, R2;		// 3 x 3 rectification transforms (rotation matrices) for the first and the second cameras
	Mat P1, P2;		// 3 x 4 projection matrices in the new (rectified) coordinate systems
	Mat Q;			// 4 x 4 disparity-to-depth mapping matrix
    Rect validRoi[2];
};

cv::Mat sideBySideMat(cv::Mat a, cv::Mat b);
