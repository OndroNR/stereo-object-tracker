#pragma once

#include "common.h"

class StereoVideoInput
{
public:
	StereoVideoInput(void);
	~StereoVideoInput(void);
	virtual bool GetNextPair(struct StereoPair &stereoPair) = 0;
};

