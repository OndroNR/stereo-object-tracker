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
			svi = new StereoRecordInput(ConfigStore::get().getString("sri.path"), ConfigStore::get().getString("sri.frames_subpath"), ConfigStore::get().getString("sri.list_filename"));
			
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
				
				OutputPostprocessing outputPostprocessing(&clustering, wc);

				bool has_fg = false;

				int frame_num = 0;


				for (;;)
				{
					struct StereoPair sp;
					if (!svi->GetNextPair(sp))
					{
						std::cout << "End of stream (or error getting frames)" << endl;
						break;
					}

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

						outputPostprocessing.ProcessFrame(remap.timestamp);

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

						//vector<Point3f> camPoints;
						//vector<Point3f> worldCalibratedPoints;
						//if (sr.pairs.size() > 0)
						//{
						//	for (size_t i = 0; i < sr.pairs.size(); i++)
						//	{
						//		camPoints.push_back(sr.pairs[i]->pt);
						//	}
						//	worldCalibratedPoints = wc->transformOrigin(camPoints);
						//}

						//// write ply
						//ofstream ply_file(("ply" + to_string(counter) + ".ply").c_str());
						//ply_file << "ply\nformat ascii 1.0\n";
						//ply_file << "element vertex " << to_string(sr.pairs.size()*3) << "\n";
						//ply_file << "property float x\nproperty float y\nproperty float z\n";
						//ply_file << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
						//ply_file << "end_header\n";

						//for (size_t i = 0; i < sr.pairs.size(); i++)
						//{
						//	KeyPointPair kpp = *sr.pairs[i];
						//	Point3f worldPoint = worldCalibratedPoints[i];

						//	ply_file << to_string(kpp.kpx[0]->pt.x) << " " << to_string(kpp.kpx[0]->pt.y) << " " << to_string(kpp.pt.z) << " 255 0 0\n";
						//	ply_file << to_string(kpp.pt.x) << " " << to_string(kpp.pt.y) << " " << to_string(kpp.pt.z) << " 0 255 0\n";
						//	ply_file << to_string(worldPoint.x) << " " << to_string(worldPoint.y) << " " << to_string(worldPoint.z) << " 0 0 255\n";
						//}
						//ply_file.close();

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

						// draw clusters on map
						Mat cluster_img(480, 640, CV_8UC3);
						float cluster_img_scale = 640.0 / 3.0; // 3m = 640px
						cluster_img.setTo(Scalar(0,0,0));
						for (int x = 0; x <= 5 ; x++)
						{
							line(cluster_img, Point(cluster_img_scale*x*0.6, 0), Point(cluster_img_scale*x*0.6, 479), Scalar(255,255,255));
						}
						for (int y = 0; y <= 3 ; y++)
						{
							line(cluster_img, Point(0, 480-cluster_img_scale*y*0.6), Point(639, 480-cluster_img_scale*y*0.6), Scalar(255,255,255));
						}
						line(cluster_img, Point(cluster_img_scale*4.3*0.6, 0), Point(cluster_img_scale*4.3*0.6, 479), Scalar(255,255,255));
						line(cluster_img, Point(0, 480-cluster_img_scale*3.2*0.6), Point(639, 480-cluster_img_scale*3.2*0.6), Scalar(255,255,255));

						vector<Cluster*> out_clusters = clustering.Export();
						for (Cluster* cluster : out_clusters)
						{
							// Can't transform bounding box to world coordinates this way
							// Bounding box is positioned and rotated in camera space. It is skewed after transformation.
							//pair<Point3f, Point3f> bbox = cluster->boundingBox();
							//bbox.first = wc->transformOrigin(bbox.first);
							//bbox.second = wc->transformOrigin(bbox.second);

							Point3f cluster_pt = wc->transformOrigin(cluster->pt);
							circle(cluster_img, Point(cluster_img_scale * (cluster_pt.x), 480-cluster_img_scale*cluster_pt.z), 3, cluster->color);

							Point3f bbox_seed = wc->transformOrigin(cluster->pairs[0]->pt);
							pair<Point3f, Point3f> bbox = pair<Point3f, Point3f>(bbox_seed, bbox_seed);

							for (auto kpp : cluster->pairs)
							{
								Point3f kpp_pt = wc->transformOrigin(kpp->pt);
								circle(cluster_img, Point(cluster_img_scale * (kpp_pt.x), 480-cluster_img_scale*kpp_pt.z), 1, cluster->color);

								bbox.first.x = MIN(bbox.first.x, kpp_pt.x);
								bbox.first.y = MIN(bbox.first.y, kpp_pt.y);
								bbox.first.z = MIN(bbox.first.z, kpp_pt.z);
								bbox.second.x = MAX(bbox.second.x, kpp_pt.x);
								bbox.second.y = MAX(bbox.second.y, kpp_pt.y);
								bbox.second.z = MAX(bbox.second.z, kpp_pt.z);
							}

							// ignore Y (height)
							Point corner1 = Point(cluster_img_scale * (bbox.first.x), 480-cluster_img_scale * bbox.first.z);
							Point corner2 = Point(cluster_img_scale * (bbox.second.x), 480-cluster_img_scale * bbox.second.z);
							rectangle(cluster_img, corner1, corner2, cluster->color);
						}

						rectangle(cluster_img, Point(0,0), Point(200, 20), Scalar(0,0,0), CV_FILLED);
						putText(cluster_img, to_string(remap.timestamp), Point(0,15), CV_FONT_HERSHEY_PLAIN, 1.0, Scalar(255,255,255));

						imshow("Clusters", cluster_img);


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
					bool end = false;
					switch (key)
					{
						case 'q':
						case 'Q':
							end = true;
							cout << "Processing was ended by user." << endl;
							break;
						case 'c':
							clustering.cluster_merge_distance_limit = 0;
							cout << "Cluster merging disabled." << endl;
							break;
						case 'v':
							clustering.cluster_merge_distance_limit = ConfigStore::get().getFloat("clustering.cluster_merge_distance_limit");
							cout << "Cluster merging enabled." << endl;
							break;
					}
					if(end)
					{
						break;
					}
				}

				cv::destroyWindow("Input pair");
				cv::destroyWindow("Remapped pair");
				cv::destroyWindow("Foreground mask");
				cv::destroyWindow("Keypoints");
				cv::destroyWindow("Cluster keypoints");
				cv::destroyWindow("Clusters");

				cout << "Writing output..." << endl;
				outputPostprocessing.ComputeOutput();
				outputPostprocessing.WriteOutput(ConfigStore::get().getString("output.path"));
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
		case '9':
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

				wc->setOrigin(remap);
				cout << "World origin = " << endl;
				cout << wc->worldOrigin << endl << endl;

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

			fs["imageOriginLeft"] >> wc->imageOrigin[0];
			fs["imageOriginRight"] >> wc->imageOrigin[1];
			fs["cameraOrigin"] >> wc->cameraOrigin;
			fs["worldOrigin"] >> wc->worldOrigin;

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

			fs << "imageOriginLeft" << wc->imageOrigin[0];
			fs << "imageOriginRight" << wc->imageOrigin[1];
			fs << "cameraOrigin" << wc->cameraOrigin;
			fs << "worldOrigin" << wc->worldOrigin;

			fs.release();
			cout << "World calibration saved" << endl;

			break;

		case 'v':
		case 'V':
			if (svi == NULL)
			{
				delete svi;
			}
			
			svi = new StereoCameraInput(ConfigStore::get().getInt("sci.left_cam"), ConfigStore::get().getInt("sci.right_cam"));

			break;

		case 'b':
		case 'B':
			if (svi == NULL)
			{
				cout << "Stereo stream not loaded";
				break;
			}

			svi->swap_cams = !svi->swap_cams;
			break;

		case 'n':
		case 'N':
			if (svi == NULL)
			{
				cout << "Stereo stream not loaded";
				break;
			}

			svi->OpenSettings(false);
			break;

		case 'm':
		case 'M':
			if (svi == NULL)
			{
				cout << "Stereo stream not loaded";
				break;
			}

			svi->OpenSettings(true);
			break;

		case 'r':
		case 'R':
			{
				if (svi == NULL)
				{
					cout << "Stereo stream not loaded";
					break;
				}

				fps = Fps();
				counter = 0;

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
