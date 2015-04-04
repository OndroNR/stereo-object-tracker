#pragma once

#include <math.h>
#include "common.h"

using namespace cv;

class KeyPointEx :
	public cv::KeyPoint
{
public:
	bool scheduledDelete;
	int unusedFor;
	Point2f lastMove;
	KeyPointEx() : KeyPoint()
	{
		lastMove = Point2f(0,0);
		scheduledDelete = false;
		unusedFor = 0;
	}
	KeyPointEx(KeyPoint& kp)
	{
		this->pt = kp.pt;
		this->size = kp.size;
		this->angle = kp.angle;
		this->response = kp.response;
		this->octave = kp.octave;
		this->class_id = kp.class_id;

		lastMove = Point2f(0,0);
		scheduledDelete = false;
		unusedFor = 0;
	}
	bool sameAs(KeyPoint& kp)
	{
		return this->pt.x == kp.pt.x && this->pt.y == kp.pt.y;
	}
	bool sameAs(KeyPointEx& kp)
	{
		return this->pt.x == kp.pt.x && this->pt.y == kp.pt.y;
	}
	bool similiarAs(KeyPoint& kp)
	{
		return (std::abs(this->pt.x - kp.pt.x) + std::abs(this->pt.y - kp.pt.y)) < 10.0;
	}
	bool similiarAs(KeyPointEx& kp)
	{
		return (std::abs(this->pt.x - kp.pt.x) + std::abs(this->pt.y - kp.pt.y)) < 10.0;
	}
    static void convert(const vector<KeyPointEx*>& keypoints,
                        CV_OUT vector<Point2f>& points2f,
                        const vector<int>& keypointIndexes=vector<int>());
};
