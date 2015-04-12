#pragma once

#include "common.h"
#include "KeyPointPair.h"

using namespace std;

class KeyPointPair;

class Cluster
{
public:
	int id;
	vector<KeyPointPair*> pairs;
	int unusedFor; // frames not used for
	int deadFor; // frames without enough KPP
	bool scheduledDelete;
	Point3f pt;
	double timestamp;
	vector<Point3f> history_pt; // first item is current point
	vector<double> history_timestamp; // first item is current timestamp, same as this->timestamp
	Scalar color;

	Cluster(int id, KeyPointPair* kpp, double timestamp);
	~Cluster(void);
	void computePt(double timestamp);
	Point3f movementVector();

	void mergeCluster(Cluster* cluster);

	pair<Point3f, Point3f> boundingBox();
	float boundingSphere(); // max distance as radius
	
	// TODO: float representativeSphere(); // some quantil or what, more noise and outlier immune
	void scheduleDelete();
	float distanceTo(KeyPointPair* kpp);
	float distanceTo(Cluster* cluster);
};
