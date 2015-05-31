#include "BackgroundProcessing.h"


BackgroundProcessing::BackgroundProcessing(void)
{
	mog[0] = initMOG();
	mog[1] = initMOG();
	morphologyProcessing = true;

	int dilation_size = 1;
	morphElement = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
}


BackgroundProcessing::~BackgroundProcessing(void)
{
}


bool BackgroundProcessing::ProcessPair(struct StereoPair& frames, struct StereoPair& foregroundMask)
{
	#pragma omp parallel for
	for (int k = 0; k < 2; k++)
	{
		mog[k](frames.frames[k], foregroundMask.frames[k]);
		if (morphologyProcessing)
		{
			morphologyEx(foregroundMask.frames[k], foregroundMask.frames[k], MORPH_OPEN, morphElement, Point(-1,-1), 1);
		}
	}

	return true;
}

BackgroundSubtractorMOG2 BackgroundProcessing::initMOG()
{
	return BackgroundSubtractorMOG2();
}
