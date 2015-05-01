#include "OutputPostprocessing.h"

using namespace std;

OutputEntry::OutputEntry(double timestamp, vector<Cluster*> clusters/*, vector<pair<int, int>> justMerged*/)
{
	this->timestamp = timestamp;
	//this->justMerged = justMerged;
	for (auto cluster = clusters.begin(); cluster < clusters.end(); ++cluster)
	{
		this->clusters[ (*cluster)->id ] = new Cluster(**cluster);
	}
}

OutputPostprocessing::OutputPostprocessing()
{
}

OutputPostprocessing::OutputPostprocessing(Clustering* clustering, WorldCalibration* wc)
{
	this->clustering = clustering;
	this->wc = wc;
}

OutputPostprocessing::~OutputPostprocessing(void)
{
	for (auto entry = entries.begin(); entry < entries.end();)
	{
		for (auto cluster : (*entry)->clusters)
		{
			delete cluster.second;
		}
		delete *entry;
		entry = entries.erase(entry);
	}
}

void OutputPostprocessing::ProcessFrame(double timestamp)
{
	entries.push_back( new OutputEntry(timestamp, clustering->Export()/*, clustering->justMergedWith*/) );

	for (auto merge = clustering->justMergedWith.begin(); merge < clustering->justMergedWith.end(); ++merge)
	{
		mergedMapping[merge->second] = merge->first;
	}
}

void OutputPostprocessing::ComputeOutput()
{
	outputBuffer.clear();

	for (auto entry_it = entries.begin(); entry_it < entries.end(); ++entry_it)
	{
		OutputEntry* entry = *entry_it;

		pair<double, map<int, Point3f> > outputBufferItem;
		outputBufferItem.first = entry->timestamp;


		vector<int> canonicalClusterIds;
		for (auto const &cluster_it : entry->clusters)
		{
			canonicalClusterIds.push_back(GetCanonicalCluster(cluster_it.second->id));
		}
		// unique cluster ids
		sort(canonicalClusterIds.begin(), canonicalClusterIds.end());
		canonicalClusterIds.erase( unique( canonicalClusterIds.begin(), canonicalClusterIds.end() ), canonicalClusterIds.end() );

		for (auto canonicalClusterId : canonicalClusterIds)
		{
			// canonical cluster can be composed of multiple clusters (which are merged later, bud belongs together ever)
			// compute weighted average

			Point3f clusterPoint(0,0,0);
			int total_points = 0;

			for (auto cluster_it : entry->clusters)
			{
				Cluster* cluster = cluster_it.second;
				if (GetCanonicalCluster(cluster->id) == canonicalClusterId)
				{
					clusterPoint += (cluster->pt * (float) cluster->pairs.size());
					total_points += cluster->pairs.size();
				}
			}

			if (total_points > 0)
			{
				clusterPoint *= 1.0/total_points;
				outputBufferItem.second[canonicalClusterId] = wc->transformOrigin(clusterPoint);
			}
		}

		outputBuffer.push_back(outputBufferItem);
	}
}

void OutputPostprocessing::WriteOutput(string dir)
{
	map<int, ofstream*> ofstreams;

	for (auto outputBufferItem : outputBuffer)
	{
		double timestamp = outputBufferItem.first;

		for (auto clusterPt : outputBufferItem.second)
		{
			int cluster_id = clusterPt.first;
			Point3f pt = clusterPt.second;

			if (ofstreams.count(cluster_id) == 0)
			{
				ofstreams[cluster_id] = new ofstream( (dir + to_string(cluster_id)).c_str() );
				*ofstreams[cluster_id] << "x;y;z;timestamp" << endl;
			}

			*ofstreams[cluster_id] << to_string(pt.x) << ";" << to_string(pt.y) << ";" << to_string(pt.z) << ";"  << to_string(timestamp) << endl;
		}
	}

	for (auto stream : ofstreams)
	{
		stream.second->close();
		delete stream.second;
	}
}

int OutputPostprocessing::GetCanonicalCluster(int id)
{
	if (!IsCanonicalCluster(id))
	{
		return GetCanonicalCluster(mergedMapping[id]);
	}
	else
	{
		return id;
	}
}

bool OutputPostprocessing::IsCanonicalCluster(int id)
{
	return mergedMapping.count(id) == 0;
}
