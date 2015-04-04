#pragma once

#include "common.h"
#include "KeyPointEx.h"

using namespace cv;

class MotionTracking
{
public:
	MotionTracking(void);
	~MotionTracking(void);
	bool ProcessPair(struct StereoPair& frames, struct StereoPair& fg_mask);
	vector<KeyPointEx*> kpx[2]; // global keypoints array
	int frame_number;
	Ptr<FeatureDetector> fd;
	StereoPair oldFrames;
};
