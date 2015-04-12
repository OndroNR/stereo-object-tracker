#pragma once

#include "common.h"
#include "Cluster.h"
#include "ConfigStore.h"

class Clustering
{
public:
	Clustering(void);
	~Clustering(void);
	vector<Cluster*> clusters;
	int next_cluster_id;

	int export_min_pairs;

	float max_cluster_distance;
	float max_pair_distance;

	float cluster_merge_distance_limit;
	float cluster_merge_angle_limit;
	float cluster_merge_length_limit;
	float cluster_pair_angle_limit;
	float cluster_pair_length_limit;

	bool Process(vector<KeyPointPair*> pairs, double timestamp);
	vector<Cluster*> Export(void);

	bool isMovementAngleSimilar(Cluster* cluster, KeyPointPair* kpp, float threshold);
	bool isMovementLengthSimilar(Cluster* cluster, KeyPointPair* kpp, float threshold);
	bool isMovementAngleSimilar(Cluster* cluster1, Cluster* cluster2, float threshold);
	bool isMovementLengthSimilar(Cluster* cluster1, Cluster* cluster2, float threshold);
	bool isMovementAngleSimilar(Point3f a, Point3f b, float threshold);
	bool isMovementLengthSimilar(Point3f a, Point3f b, float threshold);
};
