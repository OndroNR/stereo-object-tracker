#pragma once
#include "StereoRecordInput.h"
#include "common.h"
#include "opencv2/calib3d/calib3d.hpp"
#include <vector>

using namespace cv;

class StereoCalibrate
{
private:
	StereoVideoInput* svi;
	bool is_calibrated;
	Size board_size;
	double board_square_size;
public:
	StereoCalibrate(StereoVideoInput* svi, Size board_size, double board_square_size);
	~StereoCalibrate(void);
	void calibrate(bool showImages);
	bool isCalibrated();
};

