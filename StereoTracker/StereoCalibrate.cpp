#include "StereoCalibrate.h"
#include "common.h"

using namespace std;
using namespace cv;

StereoCalibrate::StereoCalibrate(StereoVideoInput* svi, Size board_size, float board_square_size)
{
	if (!svi)
		throw std::runtime_error("StereoVideoInput is null");

	this->svi = svi;
	this->board_size = board_size;
	this->board_square_size = board_square_size;
	this->sc = new StereoCalibration();
}


StereoCalibrate::~StereoCalibrate(void)
{
}

// base on OpenCV C++ sample stereo_calib
void StereoCalibrate::calibrate(bool showImages)
{
	vector<vector<Point2f>> imagePoints[2];
    vector<vector<Point3f>> objectPoints;
    Size image_size;
	int chessboard_count = 0;
	int frame_no = 0;

	imagePoints[0].resize(2000);
	imagePoints[1].resize(2000);

	cout << "Stereo calibration begin" << endl;

	struct StereoPair sp;
	while(svi->GetNextPair(sp))
	{
		bool found = false;

		frame_no++;
		cout << "Frame number: " << frame_no << endl;

		for (int i = 0; i < 2; i++)
		{
			found = false;
			Mat frame = sp.frames[i];
			image_size = frame.size();

			vector<Point2f>& corners = imagePoints[i][chessboard_count];

			found = findChessboardCorners(frame, board_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

			if (showImages)
				drawChessboardCorners(frame, board_size, corners, found);

			if (found)
			{
				Mat grayscale_frame;
				cvtColor(frame, grayscale_frame, CV_BGR2GRAY);
				cornerSubPix(grayscale_frame, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
			}
			else
			{
				break;
			}
		}

		if (showImages)
			imshow("Stereo calibration", sideBySideMat(sp.frames[0], sp.frames[1]));

		if (found)
		{
			chessboard_count++;
			cout << "Chessboards: " << chessboard_count << endl;
		}

		int key = waitKey(5);
		if(key >= 0)
		{
			break;
		}
	}

	destroyWindow("Stereo calibration");

	if (chessboard_count < 2){
		cerr << "Not enough chessboards found" << endl;
		return;
    }

	imagePoints[0].resize(chessboard_count);
    imagePoints[1].resize(chessboard_count);
    objectPoints.resize(chessboard_count);

	for(int i = 0; i < chessboard_count; i++){
        for(int j = 0; j < board_size.height; j++)
            for(int k = 0; k < board_size.width; k++)
                objectPoints[i].push_back(Point3f(j * board_square_size, k * board_square_size, 0));
	}

	cout << "Stereo calibration calculation begin" << endl;

    sc->rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    sc->cameraMatrix[0], sc->distCoeffs[0],
                    sc->cameraMatrix[1], sc->distCoeffs[1],
                    image_size, sc->R, sc->T, sc->E, sc->F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);

	cout << "Stereo calibration done" << endl;
	cout << "RMS error: " << sc->rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for( int i = 0; i < chessboard_count; i++ )
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for( int k = 0; k < 2; k++ )
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], sc->cameraMatrix[k], sc->distCoeffs[k], Mat(), sc->cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k+1, sc->F, lines[k]);
		}
		for( int j = 0; j < npt; j++ )
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
				imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	sc->avg_reprojection_error = err/npoints;
	cout << "Average reprojection error = " <<  sc->avg_reprojection_error << endl;
	cout << endl;

	sc->chessboard_size = board_size;
	sc->chessboard_square_size = board_square_size;

	is_calibrated = true;
}

bool StereoCalibrate::isCalibrated()
{
	return is_calibrated;
}

StereoCalibration* StereoCalibrate::getCalibrationParams()
{
	return sc;
}
