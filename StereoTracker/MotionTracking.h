#pragma once

#include "common.h"
#include "KeyPointEx.h"

using namespace cv;

#define KPX_REASON_LOST_TRACKING 2
#define KPX_REASON_UNUSED 3
#define KPX_REASON_DUPLICATE 4
#define KPX_REASON_PAIR_DELETE 5


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
