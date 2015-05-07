#include "common.h"

using namespace cv;
using namespace std;

Mat sideBySideMat(Mat a, Mat b)
{
	Size a_s = a.size();
	Size b_s = b.size();
	Mat out(max(a_s.height, b_s.height), a_s.width + b_s.width, a.type());

    out.adjustROI(0, 0, 0, -b_s.width);
    a.copyTo(out);

    out.adjustROI(0, 0, -a_s.width, b_s.width);
    b.copyTo(out);

    out.adjustROI(0, 0, a_s.width, 0);

	return out;
}

// http://answers.opencv.org/question/27695/puttext-with-black-background/
void setLabel(cv::Mat& im, const std::string label, const cv::Point & or)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::rectangle(im, or + cv::Point(0, baseline), or + cv::Point(text.width, -text.height), CV_RGB(0,0,0), CV_FILLED);
	cv::putText(im, label, or, fontface, scale, CV_RGB(255,255,255), thickness, 8);
}

// StereoCalibration class read/write functions for OpenCV FileStorage
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

// Print StereoCalibration to console
ostream& operator<<(ostream& out, const StereoCalibration& d)
{
	out << "<Stereo calibration parameters>" << endl;
	out << "cameraMatrix[0] = " << endl << d.cameraMatrix[0] << endl << endl;
	out << "cameraMatrix[1] = " << endl << d.cameraMatrix[1] << endl << endl;
	out << "distCoeffs[0] = " << endl << d.distCoeffs[0] << endl << endl;
	out << "distCoeffs[1] = " << endl << d.distCoeffs[1] << endl << endl;
	out << "R = " << endl << d.R << endl << endl;
	out << "T = " << endl << d.T << endl << endl;
	out << "E = " << endl << d.E << endl << endl;
	out << "F = " << endl << d.F << endl << endl;
	out << "chessboard_size = " << d.chessboard_size << endl << endl;
	out << "chessboard_square_size = " << d.chessboard_square_size << endl << endl;
	out << "rms error = " << d.rms << endl << endl;
	out << "avg_reprojection_error = " << d.avg_reprojection_error << endl << endl;
    return out;
}


// string trim http://www.cplusplus.com/faq/sequences/strings/trim/
std::string trim_right_copy(
	const std::string& s,
	const std::string& delimiters )
{
	if (s.empty())
		return s;
	return s.substr( 0, s.find_last_not_of( delimiters ) + 1 );
}

std::string trim_left_copy(
	const std::string& s,
	const std::string& delimiters )
{
	if (s.empty())
		return s;
	return s.substr( s.find_first_not_of( delimiters ) );
}

std::string trim_copy(
	const std::string& s,
	const std::string& delimiters )
{
	if (s.empty())
		return s;
	return trim_left_copy( trim_right_copy( s, delimiters ), delimiters );
}

// from OpenCV source
const int draw_shift_bits = 4;
const int draw_multiplier = 1 << draw_shift_bits;
void _drawKeypoint( Mat& img, const KeyPoint& p, const Scalar& color, int flags )
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

double angleBetween(Point3f pt1, Point3f pt2)
{
	return atan2(norm(pt1.cross(pt2)), pt1.dot(pt2));
}

double angleBetween(Point pt1, Point pt2)
{
	return atan2(norm(pt1.cross(pt2)), pt1.dot(pt2));
}

float vecAverage(vector<float> v)
{
	float acc = 0;
	for (auto it = v.begin(); it < v.end(); ++it)
	{
		acc += *it;
	}
	return acc / v.size();
}

float vecMedian(vector<float> v)
{
	size_t size = v.size();
	sort(v.begin(), v.end());

	if (size  % 2 == 0)
	{
		return (v[size / 2 - 1] + v[size / 2]) / 2;
	}
	else 
	{
		return v[size / 2];
	}
}
