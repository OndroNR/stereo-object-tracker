#pragma once

#include "common.h"

using namespace cv;

class StereoPreprocessing
{
public:
	StereoPreprocessing(StereoCalibration* scp, Size imageSize, float alpha);
	~StereoPreprocessing(void);
	bool ProcessPair(struct StereoPair& inputPair, struct StereoPair& outputPair);
public:
	StereoCalibration* scp;
	float alpha;
	Size imageSize;
	RectificationParams* rp;
	Mat rmap[2][2];
};

