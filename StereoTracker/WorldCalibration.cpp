#include "WorldCalibration.h"

using namespace std;
using namespace cv;

WorldCalibration::WorldCalibration(void)
{
	for (int i = 0; i < 4; i++)
	{
		points[0].push_back(Point3f(0,0,0));
		imagePoints[0].push_back(Point(0,0));
		imagePoints[1].push_back(Point(0,0));
	}
	
	// default real world points
	points[1].push_back(Point3f(0,0,0));
	points[1].push_back(Point3f(1,0,0));
	points[1].push_back(Point3f(0,1,0));
	points[1].push_back(Point3f(0,0,1));
}


WorldCalibration::~WorldCalibration(void)
{
}

void WorldCalibration::setOrigin(StereoPair frames)
{
	PointPicker picker;

	cout << "=== Origin point (0,0,0) ===" << endl;
		
	cout << "Pick point in left image!" << endl;
	imageOrigin[0] = picker.pickPoint(frames.frames[0], "Left point", imageOrigin[0]);
	cout << "Pick point in right image!" << endl;
	imageOrigin[1] = picker.pickPoint(frames.frames[1], "Right point", imageOrigin[1]);

	Point3f imagePoint = Point3f((float)imageOrigin[0].x, (float)imageOrigin[0].y, (float)abs(imageOrigin[0].x - imageOrigin[1].x));
	cameraOrigin = KeyPointPair::calcWorldPt(imagePoint);

	worldOrigin = transform(cameraOrigin);
}

void WorldCalibration::setPoints(StereoPair frames)
{
	PointPicker picker;

	// need four points, each left and right
	for (int i = 0; i < 4; i++)
	{
		cout << "=== Calibration point " << to_string(i) << "===" << endl;
		Point3f world = points[1][i];
		cout << "World coordinates: " << to_string(world.x) << ", " << to_string(world.y) << ", " << to_string(world.z) << endl;
		
		cout << "Pick point in left image!" << endl;
		Point leftPt = picker.pickPoint(frames.frames[0], "Left point", imagePoints[0][i]);
		cout << "Pick point in right image!" << endl;
		Point rightPt = picker.pickPoint(frames.frames[1], "Right point", imagePoints[1][i]);

		Point3f imagePoint = Point3f((float)leftPt.x, (float)leftPt.y, (float)abs(leftPt.x - rightPt.x));
		Point3f imageWorldPoint = KeyPointPair::calcWorldPt(imagePoint);

		imagePoints[0][i] = leftPt;
		imagePoints[1][i] = rightPt;
		points[0][i] = imageWorldPoint;
	}

}

Mat WorldCalibration::calcTransformationMatrix()
{
	Mat outputMat(3,4,CV_32F);
	vector<uchar> outliers;
	estimateAffine3D(points[0], points[1], outputMat, outliers);
	transformMatrix = outputMat;
	return transformMatrix;
}

vector<Point3f> WorldCalibration::transform(vector<Point3f> pts)
{
	vector<Point3f> dst;
	perspectiveTransform(pts, dst, transformMatrix);
	return dst;
}

Point3f WorldCalibration::transform(Point3f pt)
{
	vector<Point3f> pts;
	pts.push_back(pt);
	vector<Point3f> dst;
	perspectiveTransform(pts, dst, transformMatrix);
	return dst[0];
}