#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>

using namespace cv;
using namespace std;


static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,	const vector<vector<Point2f> >& imagePoints,
	                                    const vector<Mat>& rvecs, const vector<Mat>& tvecs,	const Mat& cameraMatrix, const Mat& distCoeffs,
	                                    vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.resize(0);
	
	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
		}
	}
		
}

static bool runCalibration(vector<vector<Point2f> > imagePoints, Size imageSize, Size boardSize, float squareSize, float grid_width, bool release_object,
	                       int flags, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, 
	                       vector<Point3f>& newObjPoints, double& totalAvgErr)
{
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcChessboardCorners(boardSize, squareSize, objectPoints[0]);
	objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
	newObjPoints = objectPoints[0];

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms;
	int iFixedPoint = -1;
	if (release_object)
	{
		iFixedPoint = boardSize.width - 1;
	}
	rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,	flags | CALIB_FIX_K3 | CALIB_USE_LU);
	//printf("RMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	if (release_object) {
		cout << "New board corners: " << endl;
		cout << newObjPoints[0] << endl;
		cout << newObjPoints[boardSize.width - 1] << endl;
		cout << newObjPoints[boardSize.width * (boardSize.height - 1)] << endl;
		cout << newObjPoints.back() << endl;
	}

	objectPoints.clear();
	objectPoints.resize(imagePoints.size(), newObjPoints);
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}


static void saveCameraParams(const string& filename,Size imageSize, Size boardSize,	float squareSize,  int flags,
	                         const Mat& cameraMatrix, const Mat& distCoeffs,const vector<Mat>& rvecs, const vector<Mat>& tvecs,	const vector<float>& reprojErrs,
	                         const vector<vector<Point2f> >& imagePoints,const vector<Point3f>& newObjPoints,double totalAvgErr)
{
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm t2;
	localtime_s(&t2,&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", &t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;


	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}

	if (!newObjPoints.empty())
	{
		fs << "grid_points" << newObjPoints;
	}
}

static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	size_t dir_pos = filename.rfind('/');
	if (dir_pos == string::npos)
		dir_pos = filename.rfind('\\');
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
	{
		string fname = (string)*it;
		if (dir_pos != string::npos)
		{
			string fpath = samples::findFile(filename.substr(0, dir_pos + 1) + fname, false);
			if (fpath.empty())
			{
				fpath = samples::findFile(fname);
			}
			fname = fpath;
		}
		else
		{
			fname = samples::findFile(fname);
		}
		l.push_back(fname);
	}
	return true;
}


static bool runAndSave(const string& outputFilename, const vector<vector<Point2f> >& imagePoints,Size imageSize, Size boardSize, 
	                   float squareSize, float grid_width, bool release_object,	int flags, Mat& cameraMatrix, Mat& distCoeffs,vector<Mat> &view)
{
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;
	vector<Point3f> newObjPoints;

	//标定
	bool ok = runCalibration(imagePoints, imageSize, boardSize, squareSize, grid_width, release_object, flags, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, newObjPoints, totalAvgErr);

	//printf("%s. avg reprojection error = %.7f\n", ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr);

	if (ok)
	{
		for (int i = 0; i < view.size(); i++)
		{	
			//画坐标轴
			drawFrameAxes(view[i], cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 100, 2);
			imshow("Image View", view[i]);
			waitKey(40);
		}


		//保存参数
		saveCameraParams(outputFilename, imageSize, boardSize, squareSize, flags, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints, newObjPoints, totalAvgErr);
	}
	return ok;
}


int main(int argc, char** argv)
{
	Size boardSize, imageSize;
	Mat cameraMatrix, distCoeffs;
	vector<vector<Point2f> > imagePoints;
	vector<string> imageList;

	//参数初始化
	boardSize.width = 11;
	boardSize.height = 8;
	float squareSize = 30;
	int  nframes = 20;
	int flags = 0;
	string outputFilename = "out_camera_data.yml";
	string inputFilename = "imagelist.yaml";
	int winSize = 5;///////////
	float grid_width = squareSize * (boardSize.width - 1);
	bool release_object = false;

	bool undistortImage = false;
	bool showUndistorted = true;

	readStringList(inputFilename, imageList);	
	
	if (squareSize <= 0)
		return fprintf(stderr, "Invalid board square width\n"), -1;
	if (nframes <= 3)
		return printf("Invalid number of images\n"), -1;
	if (boardSize.width <= 0)
		return fprintf(stderr, "Invalid board width\n"), -1;
	if (boardSize.height <= 0)
		return fprintf(stderr, "Invalid board height\n"), -1;


	namedWindow("Image View", 0);

	vector<Mat> view1;

	for (int i = 0;i<nframes; i++)
	{
		Mat view, viewGray;

		/*if (capture.isOpened())
		{
			Mat view0;
			capture >> view0;
			view0.copyTo(view);
		}
		else*/
		if (i < (int)imageList.size())
		{
			view = imread(imageList[i], 1);
		}
		view1.push_back( view);

		/*if (view.empty())
		{
			if (imagePoints.size() > 0)
				runAndSave(outputFilename, imagePoints, imageSize,
					boardSize, pattern, squareSize, grid_width, release_object, aspectRatio,
					flags, cameraMatrix, distCoeffs,
					writeExtrinsics, writePoints, writeGrid);
			break;
		}*/

		imageSize = view.size();

		vector<Point2f> pointbuf;
		cvtColor(view, viewGray, COLOR_BGR2GRAY);

		//bool found = findChessboardCorners(viewGray, boardSize, pointbuf,CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_FAST_CHECK );
		bool found = findChessboardCornersSB(viewGray, boardSize, pointbuf, CALIB_CB_ACCURACY + CALIB_CB_EXHAUSTIVE);

		
		// improve the found corners' coordinate accuracy
		if (found)
		{
			cornerSubPix(viewGray, pointbuf, Size(winSize, winSize), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
			drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
			imagePoints.push_back(pointbuf);
		}

		imshow("Image View", view);
		waitKey(1);
		
	}
	if (imagePoints.size() >= 3)
	{
		runAndSave(outputFilename, imagePoints, imageSize, boardSize, squareSize, grid_width, release_object, flags, cameraMatrix, distCoeffs,view1);

	}

	//if (!capture.isOpened() && showUndistorted)
	//{
	//	Mat view, rview, map1, map2;
	//	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
	//		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
	//		imageSize, CV_16SC2, map1, map2);

	//	for (i = 0; i < (int)imageList.size(); i++)
	//	{
	//		view = imread(imageList[i], 1);
	//		if (view.empty())
	//			continue;
	//		//undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
	//		remap(view, rview, map1, map2, INTER_LINEAR);
	//		imshow("Image View", rview);
	//		char c = (char)waitKey();
	//		if (c == 27 || c == 'q' || c == 'Q')
	//			break;
	//	}
	//}

	return 0;
}
