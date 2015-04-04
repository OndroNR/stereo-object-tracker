#include "KeyPointEx.h"

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
