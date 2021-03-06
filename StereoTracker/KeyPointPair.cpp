#include "KeyPointPair.h"
#include "MotionTracking.h"

Mat KeyPointPair::Q;

KeyPointPair::KeyPointPair(KeyPointEx* a, KeyPointEx* b, double time)
{
	kpx[0] = a;
	kpx[1] = b;
	a->hasPair = true;
	b->hasPair = true;
	b->color = a->color;
	scheduledDelete = false;
	pt = calcWorldPt(Point3f(kpx[0]->pt.x, kpx[0]->pt.y, disparity()));
	movement_vec = Point3f(0,0,0);
	last_time = time;
	uncheckedFor = 0;

	hasCluster = false;
	cluster = NULL;
	unusedFor = 0;
}

float KeyPointPair::disparity()
{
	return abs(kpx[0]->pt.x - kpx[1]->pt.x);
}

void KeyPointPair::scheduleDelete()
{
	kpx[0]->scheduledDelete = KPX_REASON_PAIR_DELETE;
	kpx[1]->scheduledDelete = KPX_REASON_PAIR_DELETE;
	scheduledDelete = true;
}

void KeyPointPair::update(double time)
{
	Point3f new_pt = calcWorldPt(Point3f(kpx[0]->pt.x, kpx[0]->pt.y, disparity()));
	float delta_time = (float)(time - last_time);
	movement_vec = (new_pt - pt);
	movement_vec.x /= delta_time;
	movement_vec.y /= delta_time;
	movement_vec.z /= delta_time;
	pt = new_pt;
	last_time = time;
}
	
Point3f KeyPointPair::calcWorldPt(Point3f camPt)
{
	vector<Point3f> camPtVec, worldPtVec;
	camPtVec.push_back(camPt);
	perspectiveTransform(camPtVec, worldPtVec, KeyPointPair::Q);
	return worldPtVec[0];
}
