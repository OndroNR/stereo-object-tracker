#pragma once

#include "common.h"
#include "KeyPointPair.h"
#include "PointPicker.h"

class WorldCalibration
{
public:
	WorldCalibration(void);
	~WorldCalibration(void);
	void setPoints(StereoPair frames);
	Mat calcTransformationMatrix();
	vector<Point3f> transform(vector<Point3f> pts);
	
	vector<Point> imagePoints[2]; // left and right points in images
	vector<Point3f> points[2]; // [0] - image world points, [1] - real world points
	Mat transformMatrix; // transformation matrix
};
