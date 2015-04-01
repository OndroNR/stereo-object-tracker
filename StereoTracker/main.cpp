#include "common.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "StereoVideoInput.h"
#include "StereoRecordInput.h"
#include "StereoCalibrate.h"
#include "Fps.h"

using namespace cv;
using namespace std;

void showMenu()
{
	cout << "\n";
	cout << "Menu\n";
	cout << "1 - open stereo record\n";
	cout << "2 - run calibration\n";
	cout << "3 - save calibration\n";
	cout << "4 - play video\n";
	cout << "5 - reset stream\n";
	cout << "q - quit\n";
	cout << "Select command: ";
}


int main( int argc, char** argv )
{
	bool quit = false;
	char menu_cmd = '%'; // nothing

	StereoVideoInput* svi = NULL;
	StereoCalibrate* sc = NULL;
	Fps fps;
	int counter = 0;

	while (!quit)
	{
		showMenu();
		cin >> menu_cmd;

		switch(menu_cmd)
		{
		case '1':
			if (svi != NULL)
			{
				delete svi;
			}
			//svi = new StereoRecordInput("f:\\galbavy\\data\\dp\\stereo\\150326-104514 kalibracia\\", "", "list.txt");
			svi = new StereoRecordInput("f:\\galbavy\\data\\dp\\stereo\\150326-121755 kalibracia autofocus\\", "", "list.txt");
			break;
		case '2':
			if (svi == NULL)
			{
				cout << "Stereo stream not loaded";
				break;
			}

			if (sc != NULL)
				delete sc;
			sc = new StereoCalibrate(svi, Size(10, 7), 2.4); // 10x7 squares, 2.4cm square size
			sc->calibrate(true);

			break;
		case '3':
			break;
		case '4':
			if (svi == NULL)
			{
				cout << "Stereo stream not loaded";
				break;
			}

			fps = Fps();
			counter = 0;

			for (;;)
			{
				struct StereoPair sp;
				svi->GetNextPair(sp);
				imshow("Input pair", sideBySideMat(sp.frames[0], sp.frames[1]));

				fps.update();
				counter++;
				if (counter % 10 == 0)
					std::cout << "Processing fps: " << fps.get() << endl;

				int key = waitKey(5);
				if(key >= 0)
				{
					break;
				}
			}

			cv::destroyWindow("Input pair");
			break;
		case '5':
			if (svi != NULL)
				svi->Reset();

			break;
		case 'q':
		case 'Q':
			quit = true;
			break;
		default:
			cout << "Bad command\n";
			break;
		}
	}

	 


}
