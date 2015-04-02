#include "MotionTracking.h"

using namespace cv;
using namespace std;

void KeyPointEx::convert(const std::vector<KeyPointEx*>& keypoints, std::vector<Point2f>& points2f,
						const vector<int>& keypointIndexes)
{
	if( keypointIndexes.empty() )
	{
		points2f.resize( keypoints.size() );
		for( size_t i = 0; i < keypoints.size(); i++ )
			points2f[i] = keypoints[i]->pt;
	}
	else
	{
		points2f.resize( keypointIndexes.size() );
		for( size_t i = 0; i < keypointIndexes.size(); i++ )
		{
			int idx = keypointIndexes[i];
			if( idx >= 0 )
				points2f[i] = keypoints[idx]->pt;
			else
			{
				CV_Error( CV_StsBadArg, "keypointIndexes has element < 0. TODO: process this case" );
				//points2f[i] = Point2f(-1, -1);
			}
		}
	}
}


MotionTracking::MotionTracking(void)
{
	frame_number = 0;
	fd = new GoodFeaturesToTrackDetector();
}


MotionTracking::~MotionTracking(void)
{
}


bool MotionTracking::ProcessPair(struct StereoPair& frames, struct StereoPair& fg_mask)
{
	// for left & right frame
	for (int k = 0; k < 2; k++)
	{
		// remove keypoints scheduled for deletion
		for (vector<KeyPointEx*>::iterator it = kpx[k].begin(); it < kpx[k].end();)
		{
			if ((*it)->scheduledDelete)
			{
				delete *it;
				it = kpx[k].erase(it);
			}
			else
			{
				++it;
			}
		}

		// reinitialisation - find new keypoints
		if (frame_number % 10 == 0) // sometime
		{
			vector<KeyPoint> new_kp;
			fd->detect(frames.frames[k], new_kp, fg_mask.frames[k]);

			// add new unique keypoints; O(N^K)!
			for(vector<KeyPoint>::iterator it = new_kp.begin(); it != new_kp.end(); ++it)
			{
				for(vector<KeyPointEx*>::iterator it2 = kpx[k].begin(); it2 != kpx[k].end(); ++it2)
				{
					//if (it->pt.x != (*it2)->pt.x || it->pt.y != (*it2)->pt.y) // round?
					if (!(*it2)->sameAs(*it))
					{
						kpx[k].push_back(new KeyPointEx(*it));
					}
				}
			}
		}

		// calc optical flow, if we have some keypoints and previous frame
		if (frame_number != 0)
		{
			vector<Point2f> prev_points, next_points;
			vector<uchar> status;
			vector<float> err;
			KeyPointEx::convert(kpx[k], prev_points);
			
			calcOpticalFlowPyrLK(oldFrames.frames[k], frames.frames[k], prev_points, next_points, status, err);

			for( size_t i = 0; i < next_points.size(); i++ ) 
			{
				if (status[i] == 0) // lost keypoint
				{
					kpx[k][i]->scheduledDelete = true;
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
			if ((*it)->unusedFor >= 10)
			{
				(*it)->scheduledDelete = true;
			}
		}

		// find duplicates and schedule removal
		if (frame_number % 3 == 0) // not always
		{
			for(vector<KeyPointEx*>::iterator it = kpx[k].begin(); it != kpx[k].end(); ++it)
			{
				if ((*it)->scheduledDelete)
					continue;

				for(vector<KeyPointEx*>::iterator it2 = ++it; it2 != kpx[k].end(); ++it2)
				{
					if ((*it2)->scheduledDelete)
						continue;

					if ((*it)->sameAs(**it2))
					{
						(*it2)->scheduledDelete = true;
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
