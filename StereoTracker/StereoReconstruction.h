#pragma once

#include "common.h"
#include "opencv2/nonfree/features2d.hpp"
#include "KeyPointEx.h"
#include "KeyPointPair.h"

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
};

