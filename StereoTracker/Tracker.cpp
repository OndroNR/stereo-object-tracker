#include "Tracker.h"

using namespace cv;
using namespace std;

Tracker::Tracker(StereoVideoInput* svi, StereoCalibration* scp, Size frameSize, WorldCalibration* wc)
{
	this->svi = svi;
	this->scp = scp;
	this->frameSize = frameSize;
	this->wc = wc;
	fps = Fps();
	counter = 0;

	stereoPrep = new StereoPreprocessing(scp, frameSize, ConfigStore::get().getFloat("stereo_rectify_alpha"));
	KeyPointPair::Q = stereoPrep->rp->Q;
	outputPostprocessing = OutputPostprocessing(&clustering, wc);

	has_fg = false;
	frame_num = 0;
}


Tracker::~Tracker(void)
{
}

bool Tracker::ProcessFrame()
{
	struct StereoPair sp;
	if (!svi->GetNextPair(sp))
	{
		std::cout << "End of stream (or error getting frames)" << endl;
		return false;
	}

	cout << endl << "Frame number: " << frame_num++ << ", timestamp: " << sp.timestamp << endl;

	//imshow("Input pair", sideBySideMat(sp.frames[0], sp.frames[1]));
					
	Interval iTotal;

	Interval iSP;
	stereoPrep->ProcessPair(sp, remap);
	auto iSPv = iSP.value();
	//imshow("Remapped pair", sideBySideMat(remap.frames[0], remap.frames[1]));

	Interval iBG;
	bgProc.ProcessPair(remap, fgMaskMOG2);
	auto iBGv = iBG.value();

	imshow("Foreground mask", sideBySideMat(fgMaskMOG2.frames[0], fgMaskMOG2.frames[1]));

	if (!has_fg)
	{
		has_fg = true;
	}
	else
	{
		

		Interval iMT;
		mt.ProcessPair(remap, fgMaskMOG2);
		auto iMTv = iMT.value();

		Interval iSR;
		sr.Process(mt.kpx, remap);
		auto iSRv = iSR.value();

		Interval iC;
		clustering.Process(sr.pairs, remap.timestamp);
		auto iCv = iC.value();

		cout << "T:" << iTotal.value() << ";" << iSPv << ";" << iBGv << ";" << iMTv << ";" << iSRv << ";" << iCv << endl;

		outputPostprocessing.ProcessFrame(remap.timestamp);

		DrawKeypoints();

		DrawClusterPoints();

		DrawClusters();
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
		return false;
	}

	return true;
}

bool Tracker::Finish()
{
	cv::destroyWindow("Input pair");
	cv::destroyWindow("Remapped pair");
	cv::destroyWindow("Foreground mask");
	cv::destroyWindow("Keypoints");
	cv::destroyWindow("Cluster keypoints");
	cv::destroyWindow("Clusters");

	cout << "Writing output..." << endl;
	outputPostprocessing.ComputeOutput();
	outputPostprocessing.WriteOutput(ConfigStore::get().getString("output.path"));

	return true;
}

void Tracker::DrawKeypoints()
{
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
}

void Tracker::DrawClusterPoints()
{
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

void Tracker::DrawClusters()
{
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
