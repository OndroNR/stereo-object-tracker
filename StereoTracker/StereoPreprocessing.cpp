#include "StereoPreprocessing.h"

using namespace cv;

StereoPreprocessing::StereoPreprocessing()
{
}

StereoPreprocessing::StereoPreprocessing(StereoCalibration* scp, Size imageSize, float alpha)
{
	this->scp = scp;
	this->imageSize = imageSize;
	this->alpha = alpha;

	rp = new RectificationParams();

	stereoRectify(scp->cameraMatrix[0], scp->distCoeffs[0],
					scp->cameraMatrix[1], scp->distCoeffs[1],
					this->imageSize, scp->R, scp->T, rp->R1, rp->R2, rp->P1, rp->P2, rp->Q,
					CALIB_ZERO_DISPARITY, this->alpha, this->imageSize, &rp->validRoi[0], &rp->validRoi[1]);

	cout << "Q = " << endl << rp->Q << endl << endl;
	
	initUndistortRectifyMap(scp->cameraMatrix[0], scp->distCoeffs[0], rp->R1, rp->P1, this->imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(scp->cameraMatrix[1], scp->distCoeffs[1], rp->R2, rp->P2, this->imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
}


StereoPreprocessing::~StereoPreprocessing(void)
{
	delete rp;
}


bool StereoPreprocessing::ProcessPair(struct StereoPair& inputPair, struct StereoPair& outputPair)
{
	cv::remap(inputPair.frames[0], outputPair.frames[0], rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
	cv::remap(inputPair.frames[1], outputPair.frames[1], rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
	outputPair.timestamp = inputPair.timestamp;

	return true;
}
