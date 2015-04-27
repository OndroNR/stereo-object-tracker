#pragma once
#include "common.h"
#include "StereoVideoInput.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <chrono>

class StereoCameraInput :
	public StereoVideoInput
{
private:
	VideoCapture left_cam, right_cam;
public:
	StereoCameraInput(int left_cam_id, int right_cam_id);
	~StereoCameraInput(void);
	virtual bool GetNextPair(struct StereoPair &stereoPair) override;
	virtual void Reset() override;
	virtual void OpenSettings(bool _right_cam) override;
	chrono::high_resolution_clock::time_point start_time_point;
};
