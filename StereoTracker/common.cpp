#include "common.h"

using namespace cv;

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
