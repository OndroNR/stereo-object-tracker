#include "Clustering.h"


Clustering::Clustering(void)
{
	next_cluster_id = 0;
	// load config
	export_min_pairs = ConfigStore::get().getInt("clustering.export_min_pairs");
	max_cluster_distance = ConfigStore::get().getFloat("clustering.max_cluster_distance");
	max_pair_distance = ConfigStore::get().getFloat("clustering.max_pair_distance");
	cluster_merge_distance_limit = degreesToRadians(ConfigStore::get().getFloat("clustering.cluster_merge_distance_limit"));
	cluster_merge_angle_limit = (float) degreesToRadians(ConfigStore::get().getFloat("clustering.cluster_merge_angle_limit"));
	cluster_merge_length_limit = ConfigStore::get().getFloat("clustering.cluster_merge_length_limit");
	cluster_pair_angle_limit = (float) degreesToRadians(ConfigStore::get().getFloat("clustering.cluster_pair_angle_limit"));
	cluster_pair_length_limit = ConfigStore::get().getFloat("clustering.cluster_pair_length_limit");
}


Clustering::~Clustering(void)
{
	for (vector<Cluster*>::iterator cluster = clusters.begin(); cluster < clusters.end();)
	{
		delete *cluster;
		cluster = clusters.erase(cluster);
	}
}


bool Clustering::Process(vector<KeyPointPair*> pairs, double timestamp)
{
	int active_clusters_count = 0;
	// cleanup
	for (vector<Cluster*>::iterator cluster = clusters.begin(); cluster < clusters.end();)
	{
		if ((*cluster)->scheduledDelete)
		{
			delete *cluster;
			cluster = clusters.erase(cluster);
		}
		else
		{
			// remove invalidated pairs from cluster
			if ((*cluster)->pairs.size() > 0)
			{
				for (vector<KeyPointPair*>::iterator pair = (*cluster)->pairs.begin(); pair < (*cluster)->pairs.end();)
				{
					if ((*pair)->scheduledDelete)
					{
						pair = (*cluster)->pairs.erase(pair);
					}
					else
					{
						++pair;
					}
				}
			}

			++cluster;
			active_clusters_count++;
		}
	}

	// Ak ziadne klastre, tak vytvor klaster z prveho paru
	if (active_clusters_count == 0)
	{
		if (pairs.size() > 0)
		{
			Cluster* initial_cluster = new Cluster(next_cluster_id++, pairs[0], timestamp);
			clusters.push_back(initial_cluster);
		}
	}

	// (Vsetky zvysne nepouzite pary).each do |kpp|
	if (pairs.size() > 0)
	{
		for (vector<KeyPointPair*>::iterator kpp = pairs.begin(); kpp < pairs.end(); ++kpp)
		{
			if ((*kpp)->hasCluster == false)
			{
				vector<Cluster *> candidates;

				// najst vhodnych cluster kandidatov podla klastrov
				if (clusters.size() > 0)
				{
					for (vector<Cluster*>::iterator cluster = clusters.begin(); cluster < clusters.end(); ++cluster)
					{
						float cluster_distance = (*cluster)->distanceTo(*kpp);
						// TODO: different threshold for dead reckoning?
						if (cluster_distance < max_cluster_distance)
						{
							candidates.push_back(*cluster);
						}
					}
				}
				
				// najst vhodnych cluster kandidatov podla KPP
				if (pairs.size() > 0)
				{
					for (vector<KeyPointPair*>::iterator pair = pairs.begin(); pair < pairs.end(); ++pair)
					{
						if ((*pair)->hasCluster)
						{
							float pair_distance = (float) norm( (*kpp)->pt - (*pair)->pt );
							if (pair_distance < max_pair_distance)
							{
								candidates.push_back( (*pair)->cluster );
							}
						}
					}
				}

				Cluster* real_candidate = NULL;

				if (candidates.size() > 0)
				{
					// zoradit podla vzdialenosti
					sort(candidates.begin(), candidates.end(),
						[kpp](Cluster* a, Cluster* b) -> bool
						{ 
							return a->distanceTo(*kpp) < b->distanceTo(*kpp); 
						}
					);

					// unikatne
					candidates.erase( unique( candidates.begin(), candidates.end() ), candidates.end() );

					// postupne testovat vyhovujuci pohyb
					for (vector<Cluster*>::iterator cluster = candidates.begin(); cluster < candidates.end(); ++cluster)
					{
						// vyhovuje pohyb?
						if (isMovementAngleSimilar(*cluster, *kpp, cluster_pair_angle_limit) && isMovementLengthSimilar(*cluster, *kpp, cluster_pair_length_limit))
						{
							real_candidate = *cluster;
							(*cluster)->pairs.push_back(*kpp);
							(*kpp)->hasCluster = true;
							(*kpp)->cluster = *cluster;
						}
						else
						{
							cout << "Movement bad: angle = " << isMovementAngleSimilar(*cluster, *kpp, cluster_pair_angle_limit) << ", length" << isMovementLengthSimilar(*cluster, *kpp, cluster_pair_length_limit) << endl;
						}
					}
				}

				// ked nemame vyhovujuci klaster, vytvorime novy klaster
				if (real_candidate == NULL)
				{
					real_candidate = new Cluster(next_cluster_id++, *kpp, timestamp);
					clusters.push_back(real_candidate);
				}
			}
		}
	}

	// Merge close clusters if similar movement
	if (clusters.size() > 1)
	{
		bool something_left;
		while (true)
		{
			something_left = false;
			for (vector<Cluster*>::iterator cluster1 = clusters.begin(); cluster1 < clusters.end(); ++cluster1)
			{
				if ((*cluster1)->scheduledDelete)
					continue;

				for (vector<Cluster*>::iterator cluster2 = cluster1 + 1; cluster2 < clusters.end(); ++cluster2)
				{
					if ((*cluster2)->scheduledDelete)
						continue;

					if (abs(norm( (*cluster1)->pt - (*cluster2)->pt )) < cluster_merge_distance_limit)
					{
						if (isMovementAngleSimilar(*cluster1, *cluster2, cluster_pair_angle_limit) && isMovementLengthSimilar(*cluster1, *cluster2, cluster_pair_length_limit))
						{
							something_left = true;
							(*cluster1)->mergeCluster(*cluster2);
							(*cluster2)->scheduledDelete = true;
						}
					}
				}
			}
			
			if (something_left == false)
				break;
		}
	}


	if (pairs.size() > 0)
	{
		for (vector<KeyPointPair*>::iterator pair = pairs.begin(); pair < pairs.end(); ++pair)
		{
			(*pair)->unusedFor++; // will be reset on used pairs later
		}
	}

	// Each cluster processing
	if (clusters.size() > 0)
	{
		for (vector<Cluster*>::iterator cluster = clusters.begin(); cluster < clusters.end(); ++cluster)
		{
			if ((*cluster)->scheduledDelete == false)
			{
				// 1. update polohy + 2. dead reckoning
				(*cluster)->computePt(timestamp);

				// TODO: 3. overenie ci pary suhlasia s pohybom, tak nie, vyclenit prec
				// cluster_pair_angle_limit
				// cluster_pair_length_limit

				//	4. kpp[].unusedFor = 0; deadFor management
				if ((*cluster)->pairs.size() > 0)
				{
					(*cluster)->deadFor = 0;
					for (vector<KeyPointPair*>::iterator pair = (*cluster)->pairs.begin(); pair < (*cluster)->pairs.end(); ++pair)
					{
						(*pair)->unusedFor = 0;
					}
				}
				else
				{
					(*cluster)->deadFor++;
					if ( (*cluster)->deadFor > 20)
					{
						(*cluster)->scheduleDelete();
					}
				}
			}
		}
	}

	return true;
}


vector<Cluster*> Clustering::Export(void)
{
	vector<Cluster*> buffer;

	for (vector<Cluster*>::iterator cluster = clusters.begin(); cluster < clusters.end(); cluster++)
	{
		if ((*cluster)->scheduledDelete)
			continue;


		if ((*cluster)->pairs.size() >= (size_t) export_min_pairs)
		{
			buffer.push_back(*cluster);
		}
		else
		{
			(*cluster)->unusedFor++;
		}
	}

	return buffer;
}

bool Clustering::isMovementAngleSimilar(Cluster* cluster, KeyPointPair* kpp, float threshold)
{
	return isMovementAngleSimilar(cluster->movementVector(), kpp->movement_vec, threshold);
}

bool Clustering::isMovementLengthSimilar(Cluster* cluster, KeyPointPair* kpp, float threshold)
{
	return isMovementLengthSimilar(cluster->movementVector(), kpp->movement_vec, threshold);
}

bool Clustering::isMovementAngleSimilar(Cluster* cluster1, Cluster* cluster2, float threshold)
{
	return isMovementAngleSimilar(cluster1->movementVector(), cluster2->movementVector(), threshold);
}

bool Clustering::isMovementLengthSimilar(Cluster* cluster1, Cluster* cluster2, float threshold)
{
	return isMovementLengthSimilar(cluster1->movementVector(), cluster2->movementVector(), threshold);
}

bool Clustering::isMovementAngleSimilar(Point3f a, Point3f b, float threshold)
{
	float angle_diff = (float) abs(angleBetween(a, b));
	return angle_diff <= threshold;
}

bool Clustering::isMovementLengthSimilar(Point3f a, Point3f b, float threshold)
{
	float length_diff = (float) abs(norm(a) - norm(b));
	return length_diff <= threshold;
}