#pragma once

#include "common.h"
#include "KeyPointEx.h"
#include "Cluster.h"

using namespace cv;

class Cluster;

class KeyPointPair
{
public:
	KeyPointEx* kpx[2];
	bool scheduledDelete;
	Point3f pt;
	Point3f movement_vec;
	double last_time; // so we can calculate movement vector
	int uncheckedFor; // so we can check if pairs are still valid

	bool hasCluster;
	Cluster* cluster;
	int unusedFor;

	static Mat Q; // TODO: may not be best approach to pass and store Q matrix

	KeyPointPair(KeyPointEx* a, KeyPointEx* b, double time);
	float disparity();
	void scheduleDelete();
	void update(double time);
	static Point3f calcWorldPt(Point3f camPt);
};
