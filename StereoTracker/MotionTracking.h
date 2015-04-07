#pragma once

#include "common.h"
#include "KeyPointEx.h"
#include "ConfigStore.h"

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
	int unused_keypoint_frame_limit;
	int keypoint_detect_rate;
	int duplicate_removal_rate;
	float similiar_as_limit;
};
