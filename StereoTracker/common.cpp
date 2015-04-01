#include "common.h"

using namespace cv;
using namespace std;

Mat sideBySideMat(Mat a, Mat b)
{
	Size a_s = a.size();
	Size b_s = b.size();
	Mat out(max(a_s.height, b_s.height), a_s.width + b_s.width, CV_8UC3);

    out.adjustROI(0, 0, 0, -b_s.width);
    a.copyTo(out);

    out.adjustROI(0, 0, -a_s.width, b_s.width);
    b.copyTo(out);

    out.adjustROI(0, 0, a_s.width, 0);

	return out;
}

void write(FileStorage& fs, const std::string&, const StereoCalibration& x)
{
	x.write(fs);
}

void read(const FileNode& node, StereoCalibration& x, const StereoCalibration& default_value)
{
	if(node.empty())
		x = default_value;
	else
		x.read(node);
}

ostream& operator<<(ostream& out, const StereoCalibration& d)
{
	cout << "<Stereo calibration parameters>" << endl;
	cout << "cameraMatrix[0] = " << endl << d.cameraMatrix[0] << endl << endl;
	cout << "cameraMatrix[1] = " << endl << d.cameraMatrix[1] << endl << endl;
	cout << "distCoeffs[0] = " << endl << d.distCoeffs[0] << endl << endl;
	cout << "distCoeffs[1] = " << endl << d.distCoeffs[1] << endl << endl;
	cout << "R = " << endl << d.R << endl << endl;
	cout << "T = " << endl << d.T << endl << endl;
	cout << "E = " << endl << d.E << endl << endl;
	cout << "F = " << endl << d.F << endl << endl;
	cout << "chessboard_size = " << d.chessboard_size << endl << endl;
	cout << "chessboard_square_size = " << d.chessboard_square_size << endl << endl;
	cout << "rms error = " << d.rms << endl << endl;
	cout << "avg_reprojection_error = " << d.avg_reprojection_error << endl << endl;
    return out;
}
