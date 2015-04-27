#include "StereoCameraInput.h"

using namespace std;
using namespace cv;

StereoCameraInput::StereoCameraInput(int left_cam_id, int right_cam_id)
{
	left_cam = VideoCapture(left_cam_id);
	if (!left_cam.isOpened())
	{
		cerr << "Could not open left camera, index: " << left_cam_id << endl;
	}
	right_cam = VideoCapture(right_cam_id);
	if (!right_cam.isOpened())
	{
		cerr << "Could not open right camera, index: " << right_cam_id << endl;
		left_cam.release();
	}
	start_time_point = chrono::high_resolution_clock::now();
	swap_cams = false;
}

StereoCameraInput::~StereoCameraInput(void)
{
	if (left_cam.isOpened())
		left_cam.release();
	if (right_cam.isOpened())
		right_cam.release();
}

bool StereoCameraInput::GetNextPair(struct StereoPair& stereoPair)
{
	if (!swap_cams)
	{
		left_cam >> stereoPair.frames[0];
		right_cam >> stereoPair.frames[1];
	}
	else
	{
		left_cam >> stereoPair.frames[1];
		right_cam >> stereoPair.frames[0];		
	}

	auto now = chrono::high_resolution_clock::now();
	stereoPair.timestamp = (double) (chrono::duration_cast<chrono::milliseconds>(now - start_time_point).count()) / 1000.0;

	if (stereoPair.frames[0].empty())
		return false;
	if (stereoPair.frames[1].empty())
		return false;

	return true;
}

void StereoCameraInput::Reset()
{
}

void StereoCameraInput::OpenSettings(bool _right_cam)
{
	if (!_right_cam)
	{
		left_cam.set(CV_CAP_PROP_SETTINGS, 0);
	}
	else
	{
		right_cam.set(CV_CAP_PROP_SETTINGS, 0);
	}
}
