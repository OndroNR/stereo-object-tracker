#include "common.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "StereoVideoInput.h"
#include "StereoRecordInput.h"
#include "StereoCalibrate.h"
#include "StereoPreprocessing.h"
#include "BackgroundProcessing.h"
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
	cout << "4 - load calibration\n";
	cout << "5 - play video\n";
	cout << "6 - reset stream\n";
	cout << "7 - play remapped + bg subtraction\n";
	cout << "q - quit\n";
	cout << "Select command: ";
}


int main( int argc, char** argv )
{
	bool quit = false;
	char menu_cmd = '%'; // nothing

	StereoVideoInput* svi = NULL;
	StereoCalibrate* sc = NULL;
	StereoCalibration* scp = NULL;
	FileStorage fs;
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
			sc = new StereoCalibrate(svi, Size(10, 7), 2.4f); // 10x7 squares, 2.4cm square size
			sc->calibrate(true);

			if (sc->isCalibrated())
				scp = sc->getCalibrationParams();

			cout << *scp << endl;

			break;
		case '3':
			if (sc == NULL)
			{
				cout << "Not calibrated";
				break;
			}
			else if (scp == NULL)
			{
				cout << "Calibration missing (failed?)";
				break;
			}

			scp = sc->getCalibrationParams();
			fs = FileStorage("stereo_calibration.xml", FileStorage::WRITE);
			fs << "stereo_calibration" << *scp;
			fs.release();
			cout << "Calibration saved" << endl;

			break;
		case '4':
			fs = FileStorage();
			fs.open("stereo_calibration.xml", FileStorage::READ);

			if (!fs.isOpened())
			{
				cerr << "Failed to open calibration file" << endl;
				break;
			}

			scp = new StereoCalibration();
			fs["stereo_calibration"] >> *scp;
			cout << "Calibration loaded" << endl;
			cout << *scp;

			break;
		case '5':
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
		case '6':
			if (svi != NULL)
				svi->Reset();

			break;
		case '7':
			{
				if (svi == NULL)
				{
					cout << "Stereo stream not loaded";
					break;
				}

				if (scp == NULL)
				{
					cout << "Calibration missing";
					break;
				}

				fps = Fps();
				counter = 0;

				StereoPair remap;

				StereoPreprocessing stereoPrep(scp, Size(640,480), 1);

				BackgroundProcessing bgProc;
				StereoPair fgMaskMOG2;


				for (;;)
				{
					struct StereoPair sp;
					svi->GetNextPair(sp);
					imshow("Input pair", sideBySideMat(sp.frames[0], sp.frames[1]));

					stereoPrep.ProcessPair(sp, remap);

					imshow("Remapped pair", sideBySideMat(remap.frames[0], remap.frames[1]));

					bgProc.ProcessPair(remap, fgMaskMOG2);

					imshow("Foreground mask left", fgMaskMOG2.frames[0]);
					imshow("Foreground mask right", fgMaskMOG2.frames[1]);

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
				cv::destroyWindow("Remapped pair");
				//cv::destroyWindow("Foreground mask");
				break;
			}
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
