#include "common.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "StereoVideoInput.h"
#include "StereoRecordInput.h"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
	StereoVideoInput* svi = new StereoRecordInput("f:\\galbavy\\data\\dp\\stereo\\150326-104514 kalibracia\\", "", "list.txt");

	for (;;)
	{

		struct StereoPair sp;

		svi->GetNextPair(sp);

		imshow("Input pair", sideBySideMat(sp.frames[0], sp.frames[1]));


		int key = waitKey(15);
		if(key == 32)
		{
		}
		else if(key >= 0)
		{
			break;
		}
	}
}
