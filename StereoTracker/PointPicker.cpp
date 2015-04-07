#include "PointPicker.h"


PointPicker::PointPicker(void)
{
	picked = false;
	tmpPoint = Point(0,0);
}


PointPicker::~PointPicker(void)
{
}


void PointPicker::mouseHandler(int event, int x, int y, int flags, void *param)
{
	switch(event) {
		case CV_EVENT_LBUTTONDOWN:
			tmpPoint = Point(x,y);
			break;
		case CV_EVENT_RBUTTONDOWN:
			picked = true;
			break;
		break;
	}
}

void pointPickerMouseHandlerWrapper( int event, int x, int y, int flags, void* param )
{
	PointPicker* _this = (PointPicker *)(param);
    _this->mouseHandler(event, x, y, flags, 0);
}

Point PointPicker::pickPoint(Mat img, string name, Point defaultPoint)
{
	tmpPoint = defaultPoint;

	cv::imshow(name, img);
	cv::setMouseCallback(name, pointPickerMouseHandlerWrapper, this);

	while (!picked)
	{
		int key = cv::waitKey(30);
		if (key == 13) // enter
		{
			picked = true;
		}
	}

	destroyWindow(name);

	picked = false;
	return tmpPoint;
}
