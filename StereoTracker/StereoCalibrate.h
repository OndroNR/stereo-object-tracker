#pragma once
#include "StereoRecordInput.h"

class StereoCalibrate
{
private:
	StereoVideoInput* svi;
public:
	StereoCalibrate(StereoVideoInput* svi);
	~StereoCalibrate(void);
	void calibrate(bool showImages);
	bool isCalibrated();
};

