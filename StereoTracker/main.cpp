//#ifdef _WIN32
//#ifdef _DEBUG
//#include <vld.h>
//#endif
//#endif

#include "common.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ConfigStore.h"
#include "StereoVideoInput.h"
#include "StereoRecordInput.h"
#include "StereoCalibrate.h"
#include "StereoPreprocessing.h"
#include "BackgroundProcessing.h"
#include "MotionTracking.h"
#include "StereoReconstruction.h"
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
	cout << "0 - reload config\n";
	cout << "q - quit\n";
	cout << "Select command: ";
}

const int draw_shift_bits = 4;
const int draw_multiplier = 1 << draw_shift_bits;
static inline void _drawKeypoint( Mat& img, const KeyPoint& p, const Scalar& color, int flags )
{
    CV_Assert( !img.empty() );
    Point center( cvRound(p.pt.x * draw_multiplier), cvRound(p.pt.y * draw_multiplier) );

    if( flags & DrawMatchesFlags::DRAW_RICH_KEYPOINTS )
    {
        int radius = cvRound(p.size/2 * draw_multiplier); // KeyPoint::size is a diameter

        // draw the circles around keypoints with the keypoints size
        circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );

        // draw orientation of the keypoint, if it is applicable
        if( p.angle != -1 )
        {
            float srcAngleRad = p.angle*(float)CV_PI/180.f;
            Point orient( cvRound(cos(srcAngleRad)*radius ),
                          cvRound(sin(srcAngleRad)*radius )
                        );
            line( img, center, center+orient, color, 1, CV_AA, draw_shift_bits );
        }
#if 0
        else
        {
            // draw center with R=1
            int radius = 1 * draw_multiplier;
            circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
        }
#endif
    }
    else
    {
        // draw center with R=3
        int radius = 3 * draw_multiplier;
        circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
    }
}

bool loadConfig()
{
	std::ifstream configInput("config.txt");
	if (!configInput)
	{
		cerr << "Failed to open config file!" << endl;
		return false;
	}
	ConfigStore::get().parseFile(configInput);
	configInput.close();	
	return true;
}

int main( int argc, char** argv )
{
	if (!loadConfig())
	{
		return 1;
	}

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
			svi = new StereoRecordInput(ConfigStore::get().getString("sri.path"), ConfigStore::get().getString("sri.frams_subpath"), ConfigStore::get().getString("sri.list_filename"));
			
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
			fs = FileStorage(ConfigStore::get().getString("stereo_calibration_path"), FileStorage::WRITE);
			fs << "stereo_calibration" << *scp;
			fs.release();
			cout << "Calibration saved" << endl;

			break;
		case '4':
			fs = FileStorage();
			fs.open(ConfigStore::get().getString("stereo_calibration_path"), FileStorage::READ);

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

				StereoPair remap_kp;

				MotionTracking mt;
				
				KeyPointPair::Q = stereoPrep.rp->Q;

				StereoReconstruction sr;
				

				bool has_fg = false;

				int frame_num = 0;


				for (;;)
				{
					cout << endl << "Frame number: " << frame_num++ << endl;
					struct StereoPair sp;
					svi->GetNextPair(sp);
					imshow("Input pair", sideBySideMat(sp.frames[0], sp.frames[1]));

					stereoPrep.ProcessPair(sp, remap);

					imshow("Remapped pair", sideBySideMat(remap.frames[0], remap.frames[1]));

					bgProc.ProcessPair(remap, fgMaskMOG2);

					imshow("Foreground mask", sideBySideMat(fgMaskMOG2.frames[0], fgMaskMOG2.frames[1]));

					if (!has_fg)
					{
						has_fg = true;
					}
					else
					{
						mt.ProcessPair(remap, fgMaskMOG2);

						sr.Process(mt.kpx, remap);

						//drawKeypoints(remap.frames[0], mt.kpx[0], remap_kp.frames[0]);
						//drawKeypoints(remap.frames[0], dynamic_cast<vector<KeyPoint>>(mt.kpx[0]), remap_kp.frames[0]);

						for (int k = 0; k < 2; k++)
						{
							remap.frames[k].copyTo( remap_kp.frames[k] );
							if (mt.kpx[k].size() == 0)
								continue;

							vector<KeyPointEx*>::const_iterator it = mt.kpx[k].begin(),
															 end = mt.kpx[k].end();
							RNG& rng=theRNG();
							for( ; it != end; ++it )
							{
								Scalar color = Scalar(rng(256), rng(256), rng(256));
								_drawKeypoint( remap_kp.frames[k], **it, color, DrawMatchesFlags::DEFAULT );
							}
						}
					
						//imshow("Keypoints", remap_kp.frames[0]);
						imshow("Keypoints", sideBySideMat(remap_kp.frames[0], remap_kp.frames[1]));

						
					}

					fps.update();
					counter++;
					if (counter % 10 == 0)
					{
						
						std::cout << "Processing fps: " << fps.get() << endl;
					}
					cout << "Current keypoint count: " << mt.kpx[0].size() << "; " << mt.kpx[1].size() << endl;
					cout << "Current pairs count: " << sr.pairs.size() << endl;

					int key = waitKey(5);
					if(key >= 0)
					{
						break;
					}
				}

				cv::destroyWindow("Input pair");
				cv::destroyWindow("Remapped pair");
				cv::destroyWindow("Foreground mask");
				cv::destroyWindow("Keypoints");
				break;
			}
		case '0':
			if (loadConfig())
			{
				cout << "Config reloaded" << endl;
			}
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

	 
	if (svi)
		delete svi;
	if (scp)
		delete scp;
}
