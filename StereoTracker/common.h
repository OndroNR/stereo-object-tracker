#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#if defined(WIN32) || defined(_WIN32) 
#define PATH_SEPARATOR "\\" 
#else 
#define PATH_SEPARATOR "/" 
#endif 

struct StereoPair
{
	cv::Mat frames[2];
	double timestamp;
};

cv::Mat sideBySideMat(cv::Mat a, cv::Mat b);
