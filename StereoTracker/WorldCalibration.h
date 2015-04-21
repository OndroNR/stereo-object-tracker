#pragma once

#include "common.h"
#include "KeyPointPair.h"
#include "PointPicker.h"

class WorldCalibration
{
public:
	WorldCalibration(void);
	~WorldCalibration(void);
	void setOrigin(StereoPair frames);
	void setPoints(StereoPair frames);
	Mat calcTransformationMatrix();
	vector<Point3f> transform(vector<Point3f> pts);
	Point3f transform(Point3f pt);
	vector<Point3f> transformOrigin(vector<Point3f> pts);	
	Point3f transformOrigin(Point3f pts);	

	vector<Point> imagePoints[2]; // left and right points in images
	vector<Point3f> points[2]; // [0] - image world points, [1] - real world points
	Mat transformMatrix; // transformation matrix

	Point imageOrigin[2];
	Point3f cameraOrigin, worldOrigin;
};
