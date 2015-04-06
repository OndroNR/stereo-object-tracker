#include "StereoReconstruction.h"


StereoReconstruction::StereoReconstruction(void)
{
	extractor = new SiftDescriptorExtractor();
	matcher = new BFMatcher();
}


StereoReconstruction::~StereoReconstruction(void)
{
	for (vector<KeyPointPair*>::iterator it = pairs.begin(); it < pairs.end();)
	{
		delete *it;
		it = pairs.erase(it);
	}
}

bool StereoReconstruction::Process(vector<KeyPointEx*> kpx[2], StereoPair& frames)
{
	if (pairs.size() != 0)
	{
		// iterate all pairs for cleanup and updates
		for (vector<KeyPointPair*>::iterator it = pairs.begin(); it < pairs.end();)
		{
			// delete invalidated pairs
			if ((*it)->scheduledDelete)
			{
				//cout << "Deleting pair, Ptr = " << static_cast<void*>(*it) << endl;
				delete *it;
				it = pairs.erase(it);
			}
			else
			{
				// invalidate pairs with invalidated keypoint(s)
				if ( (*it)->kpx[0]->scheduledDelete || (*it)->kpx[1]->scheduledDelete)
				{
					//if ((*it)->kpx[0]->scheduledDelete)
					//	cout << "KP[0], Ptr " << static_cast<void*>((*it)->kpx[0]) << " is scheduled for delete" << endl;
					//if ((*it)->kpx[1]->scheduledDelete)
					//	cout << "KP[1], Ptr " << static_cast<void*>((*it)->kpx[1]) << " is scheduled for delete" << endl;
					//cout << "Scheduling pair, Ptr = " << static_cast<void*>(*it) << ", KPX[0] = " << static_cast<void*>((*it)->kpx[0]) << ", KPX[1] = " << static_cast<void*>((*it)->kpx[1]) << endl;
					(*it)->scheduleDelete();
				}
				else
				{
					(*it)->update(frames.timestamp); // update world coordinates, movement vector
					(*it)->uncheckedFor++;
				}

				++it;
			}
		}

		// TODO: kontrola parov, ci este sedia ak neboli skontrolovane po N framoch. Ak nesedie, scheduleDelete na KPP aj KPX[2]
		//for (vector<KeyPointPair*>::iterator it = pairs.begin(); it < pairs.end(); ++it)
		//{
		//	if ( (*it)->uncheckedFor > 10 )
		//	{
		//		// check
		//		// scheduleDelete if failed
		//	}
		//}
	}

	if (kpx[0].size() != 0 && kpx[1].size() != 0)
	{
		vector<KeyPoint> kp_direct[2];
		vector<KeyPointEx*> kp_direct_kpx[2];
		vector<bool> kp_direct_matched[2];
		Mat descriptors[2];

		for (int k = 0; k < 2; k++)
		{
			// extract descriptor for keypoints, which won't be deleted and are not paired
			for(vector<KeyPointEx*>::iterator it = kpx[k].begin(); it < kpx[k].end() - 1; ++it)
			{
				if ( !(*it)->scheduledDelete && !(*it)->hasPair )
				{
					kp_direct[k].push_back( static_cast<KeyPoint>(*(*it)) );
					kp_direct_kpx[k].push_back( *it );
					kp_direct_matched[k].push_back(false);
				}
			}

			extractor->compute(frames.frames[k], kp_direct[k], descriptors[k]);
		}

		if (kp_direct[0].size() != 0 && kp_direct[1].size() != 0)
		{
			vector<DMatch> matches;
			matcher->match(descriptors[0], descriptors[1], matches);

			// best matches first
			sort(matches.begin(), matches.end());
		
			for(vector<DMatch>::iterator match = matches.begin(); match < matches.end() - 1; ++match)
			{
				if (true) // line filter
				{
					float line_diff = abs(kp_direct_kpx[0][match->queryIdx]->pt.y - kp_direct_kpx[1][match->trainIdx]->pt.y);
					if (line_diff > 2)
						continue;
				}

				if (true) // same movement
				{
					// TODO: - check na suladny pohyb - nesmie byt prilis rozdielny
					// gain * deltaAngle + gain deltaVecSize

				}

				//if (kp_direct_kpx[0][match->queryIdx]->hasPair)
				//{
				//	cout << "KP[0], Ptr " << static_cast<void*>(kp_direct_kpx[0][match->queryIdx]) << " is already used!!!" << endl;
				//}

				//if (kp_direct_kpx[1][match->trainIdx]->hasPair)
				//{
				//	cout << "KP[1], Ptr " << static_cast<void*>(kp_direct_kpx[1][match->trainIdx]) << " is already used!!!" << endl;
				//}

				// Do not allow keypoint duplicate use (problems, later crashing)
				if (kp_direct_kpx[0][match->queryIdx]->hasPair || kp_direct_kpx[1][match->trainIdx]->hasPair)
				{
					continue;
				}

				kp_direct_matched[0][match->queryIdx] = true;
				kp_direct_matched[1][match->trainIdx] = true;

				pairs.push_back(new KeyPointPair(kp_direct_kpx[0][match->queryIdx], kp_direct_kpx[1][match->trainIdx], frames.timestamp));
			}
		}
		
		// unsuedFor++ for unmatched and bad matched keypoints
		for (int k = 0; k < 2; k++)
		{
			for (size_t i = 0; i < kp_direct_matched[k].size(); i++)
			{
				if (!kp_direct_matched[k][i])
				{
					kp_direct_kpx[k][i]->unusedFor++;
				}
			}
		}
	}

	frame_number++;
	return true;
}
