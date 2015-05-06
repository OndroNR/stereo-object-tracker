#pragma once

#include "common.h"
#include "opencv2/nonfree/features2d.hpp"
#include "KeyPointEx.h"
#include "KeyPointPair.h"
#include "ConfigStore.h"

class StereoReconstruction
{
public:
	StereoReconstruction(void);
	~StereoReconstruction(void);
	Ptr<DescriptorExtractor> extractor;
	Ptr<DescriptorMatcher> matcher;
	vector<KeyPointPair*> pairs;
	int frame_number;
	bool Process(vector<KeyPointEx*> kpx[2], StereoPair& frames);
	bool line_filter_enabled;
	int line_filter_limit;
	bool same_movement_filter_enabled;
	bool regular_check_pair_validity;
	int regular_check_frame_rate;
	int unused_pair_frame_limit;
protected:
	void Cleanup(StereoPair& frames);
	void Match(vector<KeyPointEx*>* kpx, StereoPair& frames);
};

