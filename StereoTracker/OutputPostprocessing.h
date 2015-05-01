#pragma once

#include "common.h"
#include "Clustering.h"
#include "Cluster.h"
#include "WorldCalibration.h"

using namespace std;

class OutputEntry
{
public:
	OutputEntry(double timestamp, vector<Cluster*> clusters/*, vector<pair<int, int>> justMerged*/);
	double timestamp;
	map<int, Cluster*> clusters;
	//vector<pair<int, int>> justMerged;
};

class OutputPostprocessing
{
public:
	OutputPostprocessing();
	OutputPostprocessing(Clustering* clustering, WorldCalibration* wc);
	~OutputPostprocessing(void);

	Clustering* clustering;
	WorldCalibration* wc;
	vector<OutputEntry*> entries;
	map<int, int> mergedMapping; // A was merged into B

	vector< pair< double, map<int, Point3f> > > outputBuffer;

	void ProcessFrame(double timestamp);
	void ComputeOutput();
	void WriteOutput(string dir);

	int GetCanonicalCluster(int id);
	bool IsCanonicalCluster(int id);
};
