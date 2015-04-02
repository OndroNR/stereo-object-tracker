#pragma once

#include "common.h"

using namespace cv;

class BackgroundProcessing
{
public:
	BackgroundProcessing(void);
	~BackgroundProcessing(void);
	bool ProcessPair(struct StereoPair& frames, struct StereoPair& foregroundMask);
	BackgroundSubtractorMOG2 mog[2];
	BackgroundSubtractorMOG2 initMOG();
};

