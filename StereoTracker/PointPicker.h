#pragma once

#include "common.h"

class PointPicker
{
public:
	PointPicker(void);
	~PointPicker(void);
	void mouseHandler(int event, int x, int y, int flags, void *param);
	Point pickPoint(Mat img, string name, Point defaultPoint);
	bool picked;
	Point tmpPoint;
};
