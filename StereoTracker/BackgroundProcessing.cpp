#include "BackgroundProcessing.h"


BackgroundProcessing::BackgroundProcessing(void)
{
	mog[0] = initMOG();
	mog[1] = initMOG();
}


BackgroundProcessing::~BackgroundProcessing(void)
{
}


bool BackgroundProcessing::ProcessPair(struct StereoPair& frames, struct StereoPair& foregroundMask)
{
	mog[0](frames.frames[0], foregroundMask.frames[0]);
	mog[1](frames.frames[1], foregroundMask.frames[1]);

	return true;
}

BackgroundSubtractorMOG2 BackgroundProcessing::initMOG()
{
	return BackgroundSubtractorMOG2();
}
