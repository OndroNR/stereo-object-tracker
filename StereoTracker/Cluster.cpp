#include "Cluster.h"


Cluster::Cluster(int id, KeyPointPair* kpp, double timestamp)
{
	this->id = id;
	unusedFor = 0;
	deadFor = 0;
	lowPointCountFor = 0;
	scheduledDelete = false;
	pt = kpp->pt;
	this->timestamp = timestamp;
	history_pt.insert(history_pt.begin(), pt);
	history_timestamp.insert(history_timestamp.begin(), timestamp);

	RNG& rng = theRNG();
	color = Scalar(rng(256), rng(256), rng(256));

	pairs.push_back(kpp);
	kpp->hasCluster = true;
	kpp->cluster = this;
}

Cluster::Cluster(const Cluster& obj)
{
	this->id = obj.id;
	this->pairs = obj.pairs; // do not attempt to access KPP on cloned object, they might be already freed from memory
	this->unusedFor = obj.unusedFor;
	this->deadFor = obj.deadFor;
	this->lowPointCountFor = obj.lowPointCountFor;
	this->scheduledDelete = obj.scheduledDelete;
	this->pt = obj.pt;
	this->timestamp = obj.timestamp;
	this->history_pt = obj.history_pt;
	this->history_timestamp = obj.history_timestamp;
	this->color = obj.color;
}

Cluster::~Cluster(void)
{
}

void Cluster::computePt(double timestamp)
{
	size_t history_size = 5;

	if (pairs.size() > 0)
	{
		vector<float> vec_x, vec_y, vec_z;
		for (vector<KeyPointPair*>::iterator it = pairs.begin()++; it < pairs.end(); ++it)
		{
			vec_x.push_back( (*it)->pt.x );
			vec_y.push_back( (*it)->pt.y);
			vec_z.push_back( (*it)->pt.z );
		}

		Point3f avg;
		avg.x = vecAverage(vec_x);
		avg.y = vecAverage(vec_y);
		avg.z = vecAverage(vec_z);
		//avg.x = vecMedian(vec_x);
		//avg.y = vecMedian(vec_y);
		//avg.z = vecMedian(vec_z);
		
		history_pt.insert(history_pt.begin(), avg);
		history_timestamp.insert(history_timestamp.begin(), timestamp);

		// keep limited history
		if (history_pt.size() > history_size)
		{
			history_pt.pop_back();
		}
		if (history_timestamp.size() > history_size)
		{
			history_timestamp.pop_back();
		}

		// get current position by smoothing history with gaussian
		Mat history_pt_mat(history_size, 3, CV_32F, Scalar(0));
		Mat history_pt_mat_smoothed(history_size, 3, CV_32F, Scalar(0));
		for (size_t i = 0; i < history_pt.size(); i++)
		{
			history_pt_mat.at<float>(i, 0) = history_pt[i].x;
			history_pt_mat.at<float>(i, 1) = history_pt[i].y;
			history_pt_mat.at<float>(i, 2) = history_pt[i].z;
		}
		GaussianBlur(history_pt_mat, history_pt_mat_smoothed, Size(1, history_size), 1.0);

		this->pt = Point3f(history_pt_mat.at<float>(0, 0), history_pt_mat.at<float>(0, 1), history_pt_mat.at<float>(0, 2));
		this->timestamp = timestamp;
	}
	else
	{
		// dead reckoning
		float delta_time = (float)(timestamp - this->timestamp);

		Point3f new_position = history_pt[0] + (movementVector() * delta_time);

		this->pt = new_position;
		this->timestamp = timestamp;

		history_pt.insert(history_pt.begin(), new_position);
		history_timestamp.insert(history_timestamp.begin(), timestamp);

		// keep limited history
		if (history_pt.size() > history_size)
		{
			history_pt.pop_back();
		}
		if (history_timestamp.size() > history_size)
		{
			history_timestamp.pop_back();
		}
	}
}

Point3f Cluster::movementVector()
{
	if (history_pt.size() == 1)
	{
		return Point3f(0,0,0);
	}

	// workaround
	if (history_timestamp[0] == history_timestamp[1])
	{
		return Point3f(0,0,0);
	}

	float delta_time = (float)(history_timestamp[1] - history_timestamp[0]);
	Point3f movement_vec = (history_pt[1] - history_pt[0]);
	movement_vec.x /= delta_time;
	movement_vec.y /= delta_time;
	movement_vec.z /= delta_time;
	return movement_vec;
}

void Cluster::mergeCluster(Cluster* cluster)
{
	int ours = this->pairs.size();
	int theirs = cluster->pairs.size();
	int sum = ours + theirs;

	if (ours == 0)
		return; // nothing to merge (can't merge dead cluster)

	for (vector<KeyPointPair*>::iterator it = cluster->pairs.begin(); it < cluster->pairs.end(); ++it)
	{
		this->pairs.push_back(*it);
		(*it)->cluster = this;
	}

	this->pt = (this->pt * ours + cluster->pt * theirs) * (1/sum);
	// TODO: merhe history
}

pair<Point3f, Point3f> Cluster::boundingBox()
{
	Point3f min, max;
	min = max = this->pairs[0]->pt;
	for (vector<KeyPointPair*>::iterator it = pairs.begin()++; it < pairs.end(); it++)
	{
		Point3f pt = (*it)->pt;

		if (pt.x <= min.x)
			min.x = pt.x;
		if (pt.y <= min.y)
			min.y = pt.y;
		if (pt.z <= min.z)
			min.z = pt.z;

		if (pt.x >= max.x)
			max.x = pt.x;
		if (pt.y >= max.y)
			max.y = pt.y;
		if (pt.z >= max.z)
			max.z = pt.z;
	}

	return pair<Point3f, Point3f>(min, max);
}

float Cluster::boundingSphere()
{
	float max_distance = 0;

	// find max distance
	for (vector<KeyPointPair*>::iterator it = pairs.begin()++; it < pairs.end(); it++)
	{
		Point3f pt = (*it)->pt;
		float distance = this->distanceTo(*it);

		if (distance > max_distance)
		{
			max_distance = distance;
		}
	}

	return max_distance;
}

float Cluster::averagePointDistance()
{
	if (pairs.size() == 0)
		return 0;

	float avg = 0;
	for (vector<KeyPointPair*>::iterator it = pairs.begin()++; it < pairs.end(); it++)
	{
		Point3f pt = (*it)->pt;
		avg += this->distanceTo(*it);
	}
	return avg / pairs.size();
}

float Cluster::medianPointDistance()
{
	if (pairs.size() == 0)
		return 0;

	vector<float> distances;
	for (vector<KeyPointPair*>::iterator it = pairs.begin()++; it < pairs.end(); it++)
	{
		Point3f pt = (*it)->pt;
		distances.push_back(this->distanceTo(*it));
	}

	return vecMedian(distances);
}

void Cluster::scheduleDelete()
{
	scheduledDelete = true;
	for (vector<KeyPointPair*>::iterator it = pairs.begin(); it < pairs.end(); it++)
	{
		(*it)->scheduleDelete();
	}
}

float Cluster::distanceTo(KeyPointPair* kpp)
{
	return (float) norm(kpp->pt - this->pt);
}

float Cluster::distanceTo(Cluster* cluster)
{
	return (float) norm(cluster->pt - this->pt);
}
