#include "StereoCalibrate.h"


StereoCalibrate::StereoCalibrate(StereoVideoInput* svi)
{
	if (!svi)
		throw std::runtime_error("StereoVideoInput is null");

	this->svi = svi;
}


StereoCalibrate::~StereoCalibrate(void)
{
}

void StereoCalibrate::calibrate(bool showImages)
{
}

bool StereoCalibrate::isCalibrated()
{
	return false; // TODO:
}
