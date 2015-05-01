#pragma once

#include "common.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ConfigStore.h"
#include "StereoVideoInput.h"
#include "StereoCalibrate.h"
#include "StereoPreprocessing.h"
#include "BackgroundProcessing.h"
#include "MotionTracking.h"
#include "StereoReconstruction.h"
#include "WorldCalibration.h"
#include "Clustering.h"
#include "OutputPostprocessing.h"
#include "Fps.h"
#include "Tracker.h"

using namespace cv;
using namespace std;

class Tracker
{
public:
	Tracker(StereoVideoInput* svi, StereoCalibration* scp, Size frameSize, WorldCalibration* wc);
	~Tracker(void);

	bool ProcessFrame();
	bool Finish();

	StereoVideoInput* svi;
	StereoCalibration* scp;
	Size frameSize;
	WorldCalibration* wc;

	Fps fps;
	int counter;

	StereoPair remap;
	StereoPreprocessing* stereoPrep;

	BackgroundProcessing bgProc;
	StereoPair fgMaskMOG2;

	StereoPair remap_kp;

	MotionTracking mt;
	
	StereoReconstruction sr;

	Clustering clustering;

	OutputPostprocessing outputPostprocessing;
	
	bool has_fg;
	int frame_num;

private:
	void DrawKeypoints();
	void DrawClusterPoints();
	void DrawClusters();

};

