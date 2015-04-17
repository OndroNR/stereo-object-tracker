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
#include "WorldCalibration.h"
#include "Clustering.h"
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
	cout << "8 - calibrate world\n";
	cout << "p - save world calibration\n";
	cout << "o - load world calibration\n";
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
			sc = new StereoCalibrate(svi,
				Size(ConfigStore::get().getInt("calibration_pattern.cols"), ConfigStore::get().getInt("calibration_pattern.rows")),
				ConfigStore::get().getFloat("calibration_pattern.square_size")); // 10x7 squares, 2.4cm square size
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
			if (svi == NULL)
			{
				cout << "Stereo stream not loaded";
				break;
			}

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

				StereoPreprocessing stereoPrep(scp, frameSize, ConfigStore::get().getFloat("stereo_rectify_alpha"));

				BackgroundProcessing bgProc;
				StereoPair fgMaskMOG2;

				StereoPair remap_kp;

				MotionTracking mt;
				
				KeyPointPair::Q = stereoPrep.rp->Q;

				StereoReconstruction sr;

				Clustering clustering;
				

				bool has_fg = false;

				int frame_num = 0;


				for (;;)
				{
					struct StereoPair sp;
					svi->GetNextPair(sp);
					cout << endl << "Frame number: " << frame_num++ << ", timestamp: " << sp.timestamp << endl;

					//imshow("Input pair", sideBySideMat(sp.frames[0], sp.frames[1]));
					

					stereoPrep.ProcessPair(sp, remap);

					//imshow("Remapped pair", sideBySideMat(remap.frames[0], remap.frames[1]));

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

						clustering.Process(sr.pairs, remap.timestamp);

						//drawKeypoints(remap.frames[0], mt.kpx[0], remap_kp.frames[0]);
						//drawKeypoints(remap.frames[0], dynamic_cast<vector<KeyPoint>>(mt.kpx[0]), remap_kp.frames[0]);

						for (int k = 0; k < 2; k++)
						{
							remap.frames[k].copyTo( remap_kp.frames[k] );
							if (mt.kpx[k].size() == 0)
								continue;

							vector<KeyPointEx*>::const_iterator it = mt.kpx[k].begin(),
															 end = mt.kpx[k].end();
							for( ; it != end; ++it )
							{
								_drawKeypoint( remap_kp.frames[k], **it, (*it)->color, DrawMatchesFlags::DEFAULT );
							}
						}

						Mat keypoints = sideBySideMat(remap_kp.frames[0], remap_kp.frames[1]);
						for (size_t i = 0; i < sr.pairs.size(); i++)
						{
							line(keypoints, sr.pairs[i]->kpx[0]->pt, Point((int)sr.pairs[i]->kpx[1]->pt.x+frameSize.width, (int)sr.pairs[i]->kpx[1]->pt.y), sr.pairs[i]->kpx[0]->color);

							Point pt = sr.pairs[i]->kpx[1]->pt;
							pt.x += frameSize.width;
							setLabel(keypoints, to_string((int)sr.pairs[i]->pt.z), pt);
							//putText(keypoints, to_string((int)sr.pairs[i]->pt.z).c_str(), pt, FONT_HERSHEY_PLAIN, 0.7, Scalar(255,255,255));
						}

						vector<Point3f> camPoints;
						vector<Point3f> worldCalibratedPoints;
						if (sr.pairs.size() > 0)
						{
							for (size_t i = 0; i < sr.pairs.size(); i++)
							{
								camPoints.push_back(sr.pairs[i]->pt);
							}
							worldCalibratedPoints = wc->transform(camPoints);
						}

						// write ply
						ofstream ply_file(("ply" + to_string(counter) + ".ply").c_str());
						ply_file << "ply\nformat ascii 1.0\n";
						ply_file << "element vertex " << to_string(sr.pairs.size()*3) << "\n";
						ply_file << "property float x\nproperty float y\nproperty float z\n";
						ply_file << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
						ply_file << "end_header\n";

						for (size_t i = 0; i < sr.pairs.size(); i++)
						{
							KeyPointPair kpp = *sr.pairs[i];
							Point3f worldPoint = worldCalibratedPoints[i];

							ply_file << to_string(kpp.kpx[0]->pt.x) << " " << to_string(kpp.kpx[0]->pt.y) << " " << to_string(kpp.pt.z) << " 255 0 0\n";
							ply_file << to_string(kpp.pt.x) << " " << to_string(kpp.pt.y) << " " << to_string(kpp.pt.z) << " 0 255 0\n";
							ply_file << to_string(worldPoint.x) << " " << to_string(worldPoint.y) << " " << to_string(worldPoint.z) << " 0 0 255\n";
						}
						ply_file.close();

						//imshow("Keypoints", remap_kp.frames[0]);
						imshow("Keypoints", keypoints);


						Mat clusterPoints;
						remap.frames[0].copyTo( clusterPoints );
						for (vector<Cluster*>::iterator cluster = clustering.clusters.begin(); cluster < clustering.clusters.end(); ++cluster)
						{
							if ((*cluster)->pairs.size() > 0)
							{
								for (vector<KeyPointPair*>::iterator pair = (*cluster)->pairs.begin(); pair < (*cluster)->pairs.end(); ++pair)
								{
									_drawKeypoint( clusterPoints, *(*pair)->kpx[0], (*cluster)->color, DrawMatchesFlags::DEFAULT );
								}
							}
						}
		
						imshow("Cluster keypoints", clusterPoints);
					}

					fps.update();
					counter++;
					if (counter % 5 == 0)
					{
						
						std::cout << "Processing fps: " << fps.get() << endl;
					}
					cout << "Current keypoint count: " << mt.kpx[0].size() << "; " << mt.kpx[1].size() << endl;
					cout << "Current pairs count: " << sr.pairs.size() << endl;
					cout << "Current cluster count: " << clustering.clusters.size() << endl;

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
				cv::destroyWindow("Cluster keypoints");
				break;
			}
		case '8':
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
				break;
			}
		case 'o':
		case 'O':
			fs = FileStorage();
			fs.open(ConfigStore::get().getString("world_calibration_path"), FileStorage::READ);

			if (!fs.isOpened())
			{
				cerr << "Failed to open world calibration file" << endl;
				break;
			}

			fs["imagePoints0"] >> wc->imagePoints[0];
			fs["imagePoints1"] >> wc->imagePoints[1];
			fs["points0"] >> wc->points[0];
			fs["points1"] >> wc->points[1];
			fs["transformMatrix"] >> wc->transformMatrix;
			fs.release();
			cout << "World calibration loaded" << endl;

			break;
	
		case 'p':
		case 'P':
			fs = FileStorage(ConfigStore::get().getString("world_calibration_path"), FileStorage::WRITE);
			fs << "imagePoints0" << wc->imagePoints[0];
			fs << "imagePoints1" << wc->imagePoints[1];
			fs << "points0" << wc->points[0];
			fs << "points1" << wc->points[1];
			fs << "transformMatrix" << wc->transformMatrix;
			fs.release();
			cout << "World calibration saved" << endl;

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
}
