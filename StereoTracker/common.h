#pragma once

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#define _USE_MATH_DEFINES
#include "math.h"

#if defined(WIN32) || defined(_WIN32) 
#define PATH_SEPARATOR "\\" 
#else 
#define PATH_SEPARATOR "/" 
#endif 

using namespace cv;
using namespace std;

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
	float chessboard_square_size;

	double rms; // RMS error
	double avg_reprojection_error; // Average reprojection error

	StereoCalibration()
	{
		cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
		cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	}

    void write(FileStorage& fs) const // Write serialization for this class
    {
		fs << "{"
			<< "CM0" << cameraMatrix[0]
			<< "CM1" << cameraMatrix[1]
			<< "DC0" << distCoeffs[0]
			<< "DC1" << distCoeffs[1]
			<< "R" << R
			<< "T" << T
			<< "E" << E
			<< "F" << F
			<< "chessboard_size" << chessboard_size
			<< "chessboard_square_size" << chessboard_square_size
			<< "rms" << rms
			<< "avg_reprojection_error" << avg_reprojection_error
			<< "}";
    }

	void read(const FileNode& node) // Read serialization for this class
    {
		node["CM0"] >> cameraMatrix[0];
		node["CM1"] >> cameraMatrix[1];
		node["DC0"] >> distCoeffs[0];
		node["DC1"] >> distCoeffs[1];
		node["R"] >> R;
		node["T"] >> T;
		node["E"] >> E;
		node["F"] >> F;
		node["chessboard_size"] >> chessboard_size;
		node["chessboard_square_size"] >> chessboard_square_size;
		node["rms"] >> rms;
		node["avg_reprojection_error"] >> avg_reprojection_error;
    }

	friend ostream& operator<<(ostream& out, const StereoCalibration& d);
};

struct RectificationParams
{
    Mat R1, R2;		// 3 x 3 rectification transforms (rotation matrices) for the first and the second cameras
	Mat P1, P2;		// 3 x 4 projection matrices in the new (rectified) coordinate systems
	Mat Q;			// 4 x 4 disparity-to-depth mapping matrix
    Rect validRoi[2];
};

cv::Mat sideBySideMat(cv::Mat a, cv::Mat b);
void setLabel(cv::Mat& im, const std::string label, const cv::Point & or);
void write(FileStorage& fs, const std::string&, const StereoCalibration& x);
void read(const FileNode& node, StereoCalibration& x, const StereoCalibration& default_value = StereoCalibration());

std::string trim_right_copy(
	const std::string& s,
	const std::string& delimiters = " \f\n\r\t\v" );
std::string trim_left_copy(
	const std::string& s,
	const std::string& delimiters = " \f\n\r\t\v" );
std::string trim_copy(
	const std::string& s,
	const std::string& delimiters = " \f\n\r\t\v" );

void _drawKeypoint( Mat& img, const KeyPoint& p, const Scalar& color, int flags );

double angleBetween(Point3f pt1, Point3f pt2);

inline double degreesToRadians(double dgr)
{
	return dgr * (M_PI/180);
}

inline double radiansToDegrees(double rad)
{
	return rad * (180/M_PI);
}

float vecAverage(vector<float> v);
float vecMedian(vector<float> v);
