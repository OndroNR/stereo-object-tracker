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
#include "StereoRecordOutput.h"
#include "StereoCameraInput.h"
#include "StereoCalibrate.h"
#include "StereoPreprocessing.h"
#include "BackgroundProcessing.h"
#include "MotionTracking.h"
#include "StereoReconstruction.h"
#include "WorldCalibration.h"
#include "Clustering.h"
#include "OutputPostprocessing.h"
#include "StereoRecordOutput.h"
#include "Fps.h"
#include "Tracker.h"

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
	cout << "7 - stereo track!\n";
	cout << "8 - calibrate world\n";
	cout << "9 - set world origin\n";
	cout << "p - save world calibration\n";
	cout << "o - load world calibration\n";
	cout << "v - open stereo camera\n";
	cout << "b - swap stereo camera\n";
	cout << "n - open left camera settings\n";
	cout << "m - open right camera settings\n";
	cout << "r - record stereo\n";
	cout << "0 - reload config\n";
	cout << "q - quit\n";
	cout << "Select command: ";
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

void open_stereo_record(StereoVideoInput* &svi)
{
	if (svi != nullptr)
	{
		delete svi;
	}
	svi = new StereoRecordInput(ConfigStore::get().getString("sri.path"), ConfigStore::get().getString("sri.frames_subpath"), ConfigStore::get().getString("sri.list_filename"));
}

void run_calibration(StereoVideoInput* &svi, StereoCalibrate* &sc, StereoCalibration* &scp)
{
	if (svi == nullptr)
	{
		cout << "Stereo stream not loaded";
		return;
	}

	if (sc != nullptr)
		delete sc;
	sc = new StereoCalibrate(svi,
		Size(ConfigStore::get().getInt("calibration_pattern.cols"), ConfigStore::get().getInt("calibration_pattern.rows")),
		ConfigStore::get().getFloat("calibration_pattern.square_size")); // 10x7 squares, 2.4cm square size
	sc->calibrate(false);

	if (sc->isCalibrated())
		scp = sc->getCalibrationParams();

	cout << *scp << endl;
}

void save_calibration(StereoCalibrate* &sc, StereoCalibration* &scp)
{
	if (sc == nullptr)
	{
		cout << "Not calibrated";
		return;
	}
	else if (scp == nullptr)
	{
		cout << "Calibration missing (failed?)";
		return;
	}

	scp = sc->getCalibrationParams();
	FileStorage fs = FileStorage(ConfigStore::get().getString("stereo_calibration_path"), FileStorage::WRITE);
	fs << "stereo_calibration" << *scp;
	fs.release();
	cout << "Calibration saved" << endl;
}

void load_calibration(StereoCalibration* &scp)
{
	FileStorage fs = FileStorage();
	fs.open(ConfigStore::get().getString("stereo_calibration_path"), FileStorage::READ);

	if (!fs.isOpened())
	{
		cerr << "Failed to open calibration file" << endl;
		return;
	}

	scp = new StereoCalibration();
	fs["stereo_calibration"] >> *scp;
	cout << "Calibration loaded" << endl;
	cout << *scp;	
}

void play_video(StereoVideoInput* &svi)
{
	if (svi == nullptr)
	{
		cout << "Stereo stream not loaded";
		return;
	}

	Fps fps = Fps();
	int counter = 0;

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
}

void reset_stream(StereoVideoInput* &svi)
{
	if (svi == nullptr)
	{
		cout << "Stereo stream not loaded";
		return;
	}

	if (svi != nullptr)
		svi->Reset();
}

void magic(StereoVideoInput* &svi, StereoCalibration* &scp, Size &frameSize, WorldCalibration* &wc)
{
	if (svi == nullptr)
	{
		cout << "Stereo stream not loaded";
		return;
	}

	if (scp == nullptr)
	{
		cout << "Calibration missing";
		return;
	}

	Tracker tracker(svi, scp, frameSize, wc);

	for (;;)
	{
		if (!tracker.ProcessFrame())
		{
			break;
		}
	}

	tracker.Finish();
}

void calibrate_world(StereoVideoInput* &svi, StereoCalibration* &scp, Size &frameSize, WorldCalibration* &wc)
{
	if (svi == nullptr)
	{
		cout << "Stereo stream not loaded";
		return;
	}

	if (scp == nullptr)
	{
		cout << "Calibration missing";
		return;
	}

	StereoPair remap;
	StereoPreprocessing stereoPrep(scp, frameSize, 1);
	KeyPointPair::Q = stereoPrep.rp->Q;

	struct StereoPair sp;
	svi->GetNextPair(sp);
	stereoPrep.ProcessPair(sp, remap);
	imshow("Remapped pair", sideBySideMat(remap.frames[0], remap.frames[1]));

	wc->setPoints(remap);
	wc->calcTransformationMatrix();
	cout << "Transformation matrix = " << endl;
	cout << wc->transformMatrix << endl << endl;

	destroyWindow("Remapped pair");

	cout << "Done" << endl;
}

void set_world_origin(StereoVideoInput* &svi, StereoCalibration* &scp, Size &frameSize, WorldCalibration* &wc)
{
	if (svi == nullptr)
	{
		cout << "Stereo stream not loaded";
		return;
	}

	if (scp == nullptr)
	{
		cout << "Calibration missing";
		return;
	}

	StereoPair remap;
	StereoPreprocessing stereoPrep(scp, frameSize, 1);
	KeyPointPair::Q = stereoPrep.rp->Q;

	struct StereoPair sp;
	svi->GetNextPair(sp);
	stereoPrep.ProcessPair(sp, remap);
	imshow("Remapped pair", sideBySideMat(remap.frames[0], remap.frames[1]));

	wc->setOrigin(remap);
	cout << "World origin = " << endl;
	cout << wc->worldOrigin << endl << endl;

	destroyWindow("Remapped pair");

	cout << "Done" << endl;
}

void load_world_calibration(WorldCalibration* &wc)
{
	FileStorage fs = FileStorage();
	fs.open(ConfigStore::get().getString("world_calibration_path"), FileStorage::READ);

	if (!fs.isOpened())
	{
		cerr << "Failed to open world calibration file" << endl;
		return;
	}

	fs["imagePoints0"] >> wc->imagePoints[0];
	fs["imagePoints1"] >> wc->imagePoints[1];
	fs["points0"] >> wc->points[0];
	fs["points1"] >> wc->points[1];
	fs["transformMatrix"] >> wc->transformMatrix;

	fs["imageOriginLeft"] >> wc->imageOrigin[0];
	fs["imageOriginRight"] >> wc->imageOrigin[1];
	fs["cameraOrigin"] >> wc->cameraOrigin;
	fs["worldOrigin"] >> wc->worldOrigin;

	fs.release();
	cout << "World calibration loaded" << endl;
}

void save_world_calibration(WorldCalibration* &wc)
{
	FileStorage fs = FileStorage(ConfigStore::get().getString("world_calibration_path"), FileStorage::WRITE);
	fs << "imagePoints0" << wc->imagePoints[0];
	fs << "imagePoints1" << wc->imagePoints[1];
	fs << "points0" << wc->points[0];
	fs << "points1" << wc->points[1];
	fs << "transformMatrix" << wc->transformMatrix;

	fs << "imageOriginLeft" << wc->imageOrigin[0];
	fs << "imageOriginRight" << wc->imageOrigin[1];
	fs << "cameraOrigin" << wc->cameraOrigin;
	fs << "worldOrigin" << wc->worldOrigin;

	fs.release();
	cout << "World calibration saved" << endl;
}

void open_stereo_camera(StereoVideoInput* &svi)
{
	if (svi == nullptr)
	{
		delete svi;
	}
			
	svi = new StereoCameraInput(ConfigStore::get().getInt("sci.left_cam"), ConfigStore::get().getInt("sci.right_cam"));
}

void swap_stereo_camera(StereoVideoInput* &svi)
{
	if (svi == nullptr)
	{
		cout << "Stereo stream not loaded";
		return;
	}

	svi->swap_cams = !svi->swap_cams;
}

void stereo_stream_open_settings(StereoVideoInput* &svi, bool right_camera)
{
	if (svi == nullptr)
	{
		cout << "Stereo stream not loaded";
		return;
	}

	svi->OpenSettings(right_camera);
}

void record_stereo(StereoVideoInput* &svi)
{
	if (svi == nullptr)
	{
		cout << "Stereo stream not loaded";
		return;
	}

	Fps fps = Fps();
	int counter = 0;

	StereoRecordOutput* sro = new StereoRecordOutput(ConfigStore::get().getString("sro.path"), ConfigStore::get().getString("sro.frames_subpath"), ConfigStore::get().getString("sro.list_filename"));

	while (true)
	{
		struct StereoPair sp;
		svi->GetNextPair(sp);
		sro->WritePair(sp);

		fps.update();
		counter++;
		if (counter % 5 == 0)
		{
			imshow("Stereo video", sideBySideMat(sp.frames[0], sp.frames[1]));
			std::cout << "Processing fps: " << fps.get() << endl;
		}

		int key = waitKey(5);
		if(key >= 0)
		{
			break;
		}
	}
	destroyWindow("Stereo video");
	delete sro;
}

int main( int argc, char** argv )
{
	if (!loadConfig())
	{
		return 1;
	}

	bool quit = false;
	char menu_cmd = '%'; // nothing

	StereoVideoInput* svi = nullptr;
	StereoCalibrate* sc = nullptr;
	StereoCalibration* scp = nullptr;
	WorldCalibration* wc = new WorldCalibration();
	FileStorage fs;
	Fps fps;
	int counter = 0;
	Size frameSize(0,0);

	frameSize.width = ConfigStore::get().getInt("frame_width");
	frameSize.height = ConfigStore::get().getInt("frame_height");


	while (!quit)
	{
		showMenu();
		cin >> menu_cmd;

		switch(menu_cmd)
		{
		case '1':
			open_stereo_record(svi);
			break;

		case '2':
			run_calibration(svi, sc, scp);
			break;

		case '3':
			save_calibration(sc, scp);
			break;

		case '4':
			load_calibration(scp);
			break;

		case '5':
			play_video(svi);
			break;

		case '6':
			reset_stream(svi);
			break;

		case '7':
			magic(svi, scp, frameSize, wc);
			break;

		case '8':
			calibrate_world(svi, scp, frameSize, wc);
			break;

		case '9':
			set_world_origin(svi, scp, frameSize, wc);
			break;

		case 'o':
		case 'O':
			load_world_calibration(wc);
			break;
	
		case 'p':
		case 'P':
			save_world_calibration(wc);
			break;

		case 'v':
		case 'V':
			open_stereo_camera(svi);
			break;

		case 'b':
		case 'B':
			swap_stereo_camera(svi);
			break;

		case 'n':
		case 'N':
			stereo_stream_open_settings(svi, false);
			break;

		case 'm':
		case 'M':
			stereo_stream_open_settings(svi, true);
			break;

		case 'r':
		case 'R':
			record_stereo(svi);
			break;

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
	if (sc)
		delete sc;
	if (wc)
		delete wc;
}
