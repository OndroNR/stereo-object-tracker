#pragma once

#include "common.h"

class StereoVideoInput
{
public:
	StereoVideoInput(void);
	~StereoVideoInput(void);
	virtual bool GetNextPair(struct StereoPair &stereoPair) = 0;
	virtual void Reset() = 0;
	virtual void OpenSettings(bool right_cam) = 0;
	bool swap_cams;
};

