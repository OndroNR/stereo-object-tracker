#include "MotionTracking.h"

using namespace cv;
using namespace std;

MotionTracking::MotionTracking(void)
{
	frame_number = 0;
	//fd = new GoodFeaturesToTrackDetector(100);
	fd = new FastFeatureDetector();

	unused_keypoint_frame_limit = ConfigStore::get().getInt("mt.unused_keypoint_frame_limit");
	keypoint_detect_rate = ConfigStore::get().getInt("mt.keypoint_detect_rate");
	duplicate_removal_rate = ConfigStore::get().getInt("mt.duplicate_removal_rate");
	similiar_as_limit = ConfigStore::get().getFloat("mt.similiar_as_limit");
}


MotionTracking::~MotionTracking(void)
{
	for (int k = 0; k < 2; k++)
	{
		for (vector<KeyPointEx*>::iterator it = kpx[k].begin(); it < kpx[k].end();)
		{
			delete *it;
			it = kpx[k].erase(it);
		}
	}
	//delete fd; // crashing
}


bool MotionTracking::ProcessPair(struct StereoPair& frames, struct StereoPair& fg_mask)
{
	// for left & right frame
	#pragma omp parallel for
	for (int k = 0; k < 2; k++)
	{
		// remove keypoints scheduled for deletion
		for (vector<KeyPointEx*>::iterator it = kpx[k].begin(); it < kpx[k].end();)
		{
			if ((*it)->scheduledDelete)
			{
				//cout << "Deleting keypoint[" << k << "] Ptr = " << static_cast<void*>(*it) << ", reason: " << (*it)->scheduledDelete << endl;
				delete *it;
				it = kpx[k].erase(it);
			}
			else
			{
				++it;
			}
		}

		// reinitialisation - find new keypoints
		if (frame_number % keypoint_detect_rate == 0) // sometime
		{
			vector<KeyPoint> new_kp;
			vector<KeyPointEx*> new_kpx;
			new_kpx.clear();
			fd->detect(frames.frames[k], new_kp, fg_mask.frames[k]);

			// add new unique keypoints; O(N^K)!
			//kpx[k].resize(kpx[k].size() + new_kp.size());
			for(vector<KeyPoint>::iterator it = new_kp.begin(); it != new_kp.end(); ++it)
			{
				if (kpx[k].size() == 0)
				{
					new_kpx.push_back(new KeyPointEx(*it));
				}
				else
				{
					bool unique = true;
					for(vector<KeyPointEx*>::iterator it2 = kpx[k].begin(); it2 != kpx[k].end(); ++it2)
					{
						//if (it->pt.x != (*it2)->pt.x || it->pt.y != (*it2)->pt.y) // round?
						//if ((*it2)->sameAs(*it))
						if ((*it2)->similiarAs(*it, similiar_as_limit))
						{
							unique = false;
							break;
						}
					}
					if (unique)
						new_kpx.push_back(new KeyPointEx(*it));
				}
			}

			kpx[k].insert(kpx[k].end(), new_kpx.begin(), new_kpx.end());
		}

		// calc optical flow, if we have some keypoints and previous frame
		if (frame_number != 0)
		{
			vector<Point2f> prev_points, next_points;
			vector<uchar> status;
			vector<float> err;
			KeyPointEx::convert(kpx[k], prev_points);
			
			if (prev_points.size() != 0)
				calcOpticalFlowPyrLK(oldFrames.frames[k], frames.frames[k], prev_points, next_points, status, err);

			for( size_t i = 0; i < next_points.size(); i++ ) 
			{
				if (status[i] == 0) // lost keypoint
				{
					kpx[k][i]->scheduledDelete = KPX_REASON_LOST_TRACKING;
				}
				else
				{
					kpx[k][i]->lastMove = next_points[i] - kpx[k][i]->pt; // calculate movement vector
					kpx[k][i]->pt = next_points[i]; // set new found position
				}
			}
		}

		// schedule unused keypoints for deletion
		for (vector<KeyPointEx*>::iterator it = kpx[k].begin(); it < kpx[k].end(); ++it)
		{
			if ((*it)->unusedFor >= unused_keypoint_frame_limit)
			{
				(*it)->scheduledDelete = KPX_REASON_UNUSED;
			}
		}

		// find duplicates and schedule removal
		if (frame_number % duplicate_removal_rate == 0) // not always
		{
			if (kpx[k].size() != 0)
			{
				for(vector<KeyPointEx*>::iterator it = kpx[k].begin(); it < kpx[k].end() - 1; ++it)
				{
					if ((*it)->scheduledDelete)
						continue;

					for(vector<KeyPointEx*>::iterator it2 = it + 1; it2 < kpx[k].end(); ++it2)
					{
						if ((*it2)->scheduledDelete)
							continue;

						//if ((*it)->sameAs(**it2))
						if ((*it)->similiarAs(**it2, similiar_as_limit))
						{
							(*it2)->scheduledDelete = KPX_REASON_DUPLICATE;
						}
					}
				}
			}
		}
	}

	frames.frames[0].copyTo(oldFrames.frames[0]);
	frames.frames[1].copyTo(oldFrames.frames[1]);

	frame_number++;
	return true;
}
