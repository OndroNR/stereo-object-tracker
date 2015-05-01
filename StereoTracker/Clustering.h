#pragma once

#include "common.h"
#include "Cluster.h"
#include "ConfigStore.h"

class Clustering
{
public:
	Clustering(void);
	~Clustering(void);
	void Cleanup(int& active_clusters_count);
	void MakeClusters(vector<KeyPointPair*>& pairs, double timestamp, int active_clusters_count);
	void MergeClusters();
	void ClusterProcessing(double timestamp);
	vector<Cluster*> clusters;
	int next_cluster_id;

	vector<pair<int, int>> justMergedWith; // cluster B was just merged into cluster A
	vector<int> justDeletedForNoPoints;
	vector<int> justDeletedForLowPointCount;

	int export_min_pairs;

	float max_cluster_distance;
	float max_pair_distance;

	float cluster_merge_distance_limit;
	float cluster_merge_angle_limit;
	float cluster_merge_length_limit;
	float cluster_pair_angle_limit;
	float cluster_pair_length_limit;
	float cluster_pair_max_distance_from_average_multiplier;
	float cluster_pair_max_distance_from_median_multiplier;
	int cluster_lowpoint_threshold;
	int cluster_lowpoint_frame_limit;
	int cluster_dead_frame_limit;

	bool Process(vector<KeyPointPair*> pairs, double timestamp);
	vector<Cluster*> Export(void);

	bool isMovementAngleSimilar(Cluster* cluster, KeyPointPair* kpp, float threshold);
	bool isMovementLengthSimilar(Cluster* cluster, KeyPointPair* kpp, float threshold);
	bool isMovementAngleSimilar(Cluster* cluster1, Cluster* cluster2, float threshold);
	bool isMovementLengthSimilar(Cluster* cluster1, Cluster* cluster2, float threshold);
	bool isMovementAngleSimilar(Point3f a, Point3f b, float threshold);
	bool isMovementLengthSimilar(Point3f a, Point3f b, float threshold);
};
