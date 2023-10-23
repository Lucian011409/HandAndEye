#include "EndoscopeAndNDI.h"
#include <iomanip>
#include <thread>


//Endoscope calibration start-------------------------------------------------------
/*================================================================
*
* 函 数 名：extrackChessBoardCorners
*
* 参　　数：
*            
*           
*
* 功能描述:
*			提取棋盘格样式的标定图像中的角点坐标
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
int EndoscopeAndNDI::extrackChessBoardCorners()
{
	if (this->squareSize <= 0)
	{
		cout << "Invalid board square width\n";
		return -1;
	}
		
	if (this->nframes <= 3)
	{
		cout << "Invalid number of images\n";
		return -1;
	}
		
	if (this->boardSize.width <= 0)
	{
		cout << "Invalid board width\n";
		return -1;
	}
		
	if (this->boardSize.height <= 0)
	{
		cout << "Invalid board height\n";
		return -1;
	}
		
	namedWindow("Image View", 0);
	resizeWindow("Image View", imageSize.width, imageSize.height);//修改2021-7-7
	imagePoints.clear();

	countConerImage = 0;
	noCornerImageID.resize(0);

	//提取图像中的角点
	for (int i = 0; i < nframes; i++)
	{
		Mat  view = EndoscopeImage[i];	
		
		if (view.empty())
		{
			cout << "iamge " << i + 1 << " is empty\n";
			break;
		}

		Mat  viewGray;
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
			countConerImage++;
		}
		else
		{
			noCornerImageID.push_back(i + 1);
		}

		imshow("Image View", view);
		waitKey(1);

	}//for

	cv::destroyWindow("Image View");

	return 1;
}

/*================================================================
*
* 函 数 名： runCalibration
*
* 参　　数：
*
*
* 功能描述:
*			标定内窥镜，计算内窥镜的内参、外参、畸变系数
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
bool EndoscopeAndNDI::runCalibration()
{
	//世界坐标，z为0
	vector<vector<Point3f> > objectPoints(1);
	calcChessboardCorners(objectPoints[0]);
	objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
	newObjPoints = objectPoints[0];

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms;
	int iFixedPoint = -1;
	if (release_object)
	{
		iFixedPoint = boardSize.width - 1;
	}
	rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint, cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints, 0| CALIB_FIX_K3 | CALIB_USE_LU);
	printf("RMS error reported by calibrateCamera: %g\n", rms);
	
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
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, reprojErrs);

	return ok;
}

/*================================================================
*
* 函 数 名： saveCameraParams
*
* 参　　数：                     
*
*
* 功能描述:
*			将内窥镜的标定参数保存到 cameraParament.yal 文件中
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::saveCameraParams()
{
	string filename = "./Data\\cameraParament\\cameraParament.yal";
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm t2;
	localtime_s(&t2, &tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", &t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nframes" << (int)max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

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

/*================================================================
*
* 函 数 名： saveNDIParams
*
* 参　　数：
*
*
* 功能描述:
*			将NDI空间变换信息保存到文件，并记录剔除信息序号
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::saveNDIParams()
{
	const char * tfilename = "./Data\\NDI\\transformationData.txt";
	const char * cfilename = "./Data\\NDI\\removeDataID.txt";

	FILE * tf;
	FILE * cf;

	tf = fopen(tfilename, "w");
	cf = fopen(cfilename, "w");

	//保存未能成功提取图像角点的图像ID
	for (int i = 0; i < noCornerImageID.size(); i++)
	{
		fprintf(cf, "%d\n", noCornerImageID[i]);
	}

	fclose(cf);

	//保存NDI空间变换信息
	int tsize = transformationData.size();

	fprintf(tf, "qw\tqx\tqy\tqz\tx\ty\tz\n");

	for (int i = 0; i < tsize; i++)
	{
		fprintf(tf, "%d:\n", i + 1);

		fprintf(tf, "%1f\t%1f\t%1f\t%1f\t%1f\t%1f\t%1f\n", transformationData[i].rotation.q0, transformationData[i].rotation.qx,
			transformationData[i].rotation.qy, transformationData[i].rotation.qz, transformationData[i].translation.x,
			transformationData[i].translation.y, transformationData[i].translation.z);
		
	}

	fclose(tf);
}

/*================================================================
*
* 函 数 名： computeReprojectionErrors
*
* 参　　数：
*
*            const vector<vector<Point3f>>& objectPoints
*
*            const vector<vector<Point2f>>& imagePoints
*
*           vector<float>& perViewErrors
*
*           
*
* 功能描述:
*			通过反投影计算内窥镜标定参数的误差
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
double EndoscopeAndNDI::computeReprojectionErrors(const vector<vector<Point3f>>& objectPoints, const vector<vector<Point2f>>& imagePoints, vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);

		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

/*================================================================
*
* 函 数 名： calcChessboardCorners
*
* 参　　数：
*
*            vector<Point3f>& corners
*
*
*
* 功能描述:
*			
*            计算棋盘格角点的世界坐标
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::calcChessboardCorners(vector<Point3f>& corners)
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

/*================================================================
*
* 函 数 名： runAndSave
*
* 参　　数：
*
*
* 功能描述:
*			1、提取角点，并绘制显示
*           2、标定内窥镜，计算内窥镜成像参数，
*           3、保存内窥镜参数
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
bool EndoscopeAndNDI::runAndSave()
{
	//检验捕获数据数量是否有效
	if (checkCaptureData() == -1)
	{
		cout << "捕获数据无效!\n";
		return false;
	}
	   
	//提取角点
	if (extrackChessBoardCorners() == -1)
	{
		cout << "提取角点失败!\n";
		return false;
	}

	//标定
	if (imagePoints.size() < 3)
	{
		return false;
	}

	bool ok = runCalibration();
	printf("%s. avg reprojection error = %.7f\n", ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr);
	//printf("\0");

	if (ok)
	{
		namedWindow("Image View", 0);
		resizeWindow("Image View", imageSize.width, imageSize.height);

		//有图像未提取角点
		if (noCornerImageID.size() > 0)
		{
			int flagTemp = 0;                          //noCornerImageID中的序号
			int itemp = noCornerImageID[flagTemp];     //未能提取角点的图像的ID，从1开始

			int flagConerImage = 0;

			//画坐标轴
			for (int i = 0; i < nframes; i++)
			{
				if ((i + 1) == itemp)
				{
					flagTemp++;
					//防止越界
					if (flagTemp < noCornerImageID.size())
					{
						itemp = noCornerImageID[flagTemp];
					}
					continue;
				}
				else
				{
					Mat view = EndoscopeImage[i];
					//This function draws the axes of the world / object coordinate system w.r.t.to the camera frame.
					//OX is drawn in red, OY in green and OZ in blue.
					drawFrameAxes(view, cameraMatrix, distCoeffs, rvecs[flagConerImage], tvecs[flagConerImage], 100, 2);
					imshow("Image View", view);
					flagConerImage++;
					waitKey(10);
					
				}
			}//for
		}//if
		else
		{//图像全部采集角点
			//画坐标轴
			for (int i = 0; i < nframes; i++)
			{
					Mat view = EndoscopeImage[i];
					drawFrameAxes(view, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 100, 2);
					imshow("Image View", view);					
					waitKey(10);
			}//for
		}//else

		cv::destroyWindow("Image View");

		//保存相机参数
		saveCameraParams();

		//保存NDI参数
		saveNDIParams();
	}
	return ok;
	
}

//Endoscope calibration end---------------------------------------------------------


//Endoscope and NDI data extract start-------------------------------------------------------
/*================================================================
*
* 函 数 名： OnNDITracker
*
* 参　　数：
*
*            
*
* 功能描述:
*			打开NDI，追踪刚体，记录旋转和平移数据（刚体局部坐标系到NDI世界坐标系）
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::OnNDITracker()
{
	string returnMessage;
	
	if (m_ndiTracker.DirectStartTracking(returnMessage))
	{//打开导航

		//记录数据
		m_ndiTracker.GetSysTransformData(transformationData,stopCapture, flagCaptureSampleEToN, flagCaptureSampleNToE);

		//停止导航
		if (m_ndiTracker.StopTracking())
		{
			cout << "\n导航停止追踪！\n";
		}
		else
		{
			cout << "\n导航停止追踪失败！\n";
		}

	}
	else
	{
		cout << returnMessage << endl;
	}
}

/*================================================================
*
* 函 数 名： OnNDITrackerMarker
*
* 参　　数：
*
*
*
* 功能描述:
*			打开NDI，追踪刚体，记录旋转和平移数据,标记球位置坐标（刚体局部坐标系到NDI世界坐标系）
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::OnNDITrackerMarker()
{
	string returnMessage;
	transformationData.resize(0);

	if (m_ndiTracker.DirectStartTracking(returnMessage))
	{//打开导航

		//记录数据
		//m_nditracker.getsystransformdata(transformationdata,markerdata, stopcapture, flagcapturesampleeton, flagcapturesamplentoe);

		//停止导航
		if (m_ndiTracker.StopTracking())
		{
			cout << "\n导航停止追踪！\n";
		}
		else
		{
			cout << "\n导航停止追踪失败！\n";
		}

	}
	else
	{
		cout << returnMessage << endl;
	}
}


/*================================================================
*
* 函 数 名： OnEndoscopeTrackerKey
*
* 参　　数：
*
*
*
* 功能描述:
*			打开内窥镜，捕获图像,通过按键捕获图像
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
int EndoscopeAndNDI::OnEndoscopeTrackerKey()
{
	//initialize and allocate memory to load the video stream from camera
	VideoCapture camera0(0);
	camera0.set(CAP_PROP_FRAME_WIDTH, imageSize.width);
	camera0.set(CAP_PROP_FRAME_HEIGHT, imageSize.height);
	//imageSize.width = camera0.get(CAP_PROP_FRAME_WIDTH);
	//imageSize.height = camera0.get(CAP_PROP_FRAME_HEIGHT);

	if (!camera0.isOpened())
		return -1;

	nframes = 0;
	string file_folder = ".\\Data\\Image\\";
	string type = ".bmp";

	namedWindow("Endoscope", 0);
	resizeWindow("Endoscope", imageSize.width, imageSize.height);

	flagCaptureSampleEToN = false;
    stopCapture = false;

	while (!stopCapture)
	{
		//grab and retrieve each frames of the video sequentially
		Mat3b frame0;
		camera0 >> frame0;

		//为读取图像
		if (frame0.empty())
		{
			int c = waitKey(2);
			if (c >= 0)
			{
				stopCapture = true;
			}
			else
			{
				continue;
			}
		}

		imshow("Endoscope", frame0);

		//wait for 30 milliseconds
		int c = waitKey(30);

		if (c >= 0 && c != 27&& flagCaptureSampleEToN ==false) //按除esc之外的任意键截取图像
		{
			//删除目录中的图像
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//捕获图像
			EndoscopeImage.push_back(frame0);

			//内窥镜向NDI发出捕获消息
			flagCaptureSampleEToN = true;

			//样本数加一
			nframes++;

			

			//检验NDI是否回应内窥镜消息
			this_thread::sleep_for(chrono::milliseconds(20));
			if (flagCaptureSampleNToE == false)
			{//未回应
				flagCaptureSampleEToN = false;
				EndoscopeImage.pop_back();
				nframes--;
			}
			else
			{//回应
				flagCaptureSampleNToE = false;

				//存储图像
				stringstream ss;
				ss << setw(5) << setfill('0') << nframes;
				string imgPathL = file_folder + ss.str() + type;
				imwrite(imgPathL, frame0);

				cout << "成功获取数据 " << nframes << endl;

				
			}

		}
		else  if (27 == c)//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
		{
			stopCapture = true;
		}
	}

	cv::destroyWindow("Endoscope");
	return 0;
}

/*================================================================
*
* 函 数 名： OnEndoscopeTrackerTime
*
* 参　　数：
*
*
*
* 功能描述:
*			打开内窥镜，捕获图像，通过计时捕获图像
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
int EndoscopeAndNDI::OnEndoscopeTrackerTime()
{
	//initialize and allocate memory to load the video stream from camera
	VideoCapture camera0(0);
	camera0.set(CAP_PROP_FRAME_WIDTH, imageSize.width);
	camera0.set(CAP_PROP_FRAME_HEIGHT, imageSize.height);

	if (!camera0.isOpened())
		return -1;

	nframes = 0;
	string file_folder = ".\\Data\\Image\\";
	string type = ".bmp";

	namedWindow("Endoscope", 0);
	resizeWindow("Endoscope", 960, 540);

	flagCaptureSampleEToN = false;
	stopCapture = false;

	while (!stopCapture)
	{
		//grab and retrieve each frames of the video sequentially
		Mat3b frame0;
		camera0 >> frame0;

		//为读取图像
		if (frame0.empty())
		{
			int c = waitKey(2);
			if (c >= 0)
			{
				stopCapture = true;
			}
			else
			{
				continue;
			}
		}

		imshow("Endoscope", frame0);

		//wait for 30 milliseconds
		int c = waitKey(30);

		if (c >= 0 && c != 27 && flagCaptureSampleEToN == false) //按除esc之外的任意键截取图像
		{
			//删除目录中的图像
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//捕获图像
			EndoscopeImage.push_back(frame0);

			//内窥镜向NDI发出捕获消息
			flagCaptureSampleEToN = true;

			//样本数加一
			nframes++;

			//存储图像
			stringstream ss;
			ss << setw(5) << setfill('0') << nframes;
			string imgPathL = file_folder + ss.str() + type;
			imwrite(imgPathL, frame0);

			//检验NDI是否回应内窥镜消息
			this_thread::sleep_for(chrono::milliseconds(10));
			if (flagCaptureSampleNToE == false)
			{//未回应
				flagCaptureSampleEToN = false;
				EndoscopeImage.pop_back();
				nframes--;
			}
			else
			{//回应
				flagCaptureSampleNToE = false;
				cout << "成功获取数据 " << nframes << endl;
			}

		}
		else  if (27 == c)//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
		{
			stopCapture = true;
		}
	}

	cv::destroyWindow("Endoscope");
	return 0;
}

/*================================================================
*
* 函 数 名： OnEndoscopeTrackerMarker
*
* 参　　数：
*
*
*
* 功能描述:
*			打开内窥镜，捕获标记球图像，用于反投影验证
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
int EndoscopeAndNDI::OnEndoscopeTrackerMarker()
{
	//initialize and allocate memory to load the video stream from camera
	VideoCapture camera0(0);
	camera0.set(CAP_PROP_FRAME_WIDTH, imageSize.width);
	camera0.set(CAP_PROP_FRAME_HEIGHT, imageSize.height);

	if (!camera0.isOpened())
		return -1;

	nframes = 0;
	string file_folder = ".\\Data\\backProjection\\";
	string type = ".bmp";

	namedWindow("Endoscope", 0);
	//resizeWindow("Endoscope", 960, 540);

	flagCaptureSampleEToN = false;
	stopCapture = false;
	EndoscopeImage.resize(0);

	while (!stopCapture)
	{
		//grab and retrieve each frames of the video sequentially
		Mat3b frame0;
		camera0 >> frame0;

		//为读取图像
		if (frame0.empty())
		{
			int c = waitKey(2);
			if (c >= 0)
			{
				stopCapture = true;
			}
			else
			{
				continue;
			}
		}

		imshow("Endoscope", frame0);

		//wait for 30 milliseconds
		int c = waitKey(30);

		if (c >= 0 && c != 27 && flagCaptureSampleEToN == false) //按除esc之外的任意键截取图像
		{
			//删除目录中的图像
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//捕获图像
			EndoscopeImage.push_back(frame0);

			//内窥镜向NDI发出捕获消息
			flagCaptureSampleEToN = true;

			//样本数加一
			nframes++;

			//存储图像
			stringstream ss;
			ss << setw(5) << setfill('0') << nframes;
			string imgPathL = file_folder + ss.str() + type;
			imwrite(imgPathL, frame0);

			//检验NDI是否回应内窥镜消息
			this_thread::sleep_for(chrono::milliseconds(10));
			if (flagCaptureSampleNToE == false)
			{//未回应
				flagCaptureSampleEToN = false;
				EndoscopeImage.pop_back();
				nframes--;
			}
			else
			{//回应
				flagCaptureSampleNToE = false; 
				cout << "成功获取数据 " << nframes << endl;
			}

		}
		else  if (27 == c)//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
		{
			stopCapture = true;
		}
	}

	cv::destroyWindow("Endoscope");
	return 0;
}

/*================================================================
*
* 函 数 名： CaptureData
*
* 参　　数：
*
*
*
* 功能描述:
*			线程1 捕获内窥镜图像
*
*           线程2 捕获对应NDI旋转和平移数据
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::CaptureData()
{
	//开启线程
	 thread t_endoscope(&EndoscopeAndNDI::OnEndoscopeTrackerKey, this);
	 thread t_NDI(&EndoscopeAndNDI::OnNDITracker,this);
	 t_endoscope.join();
	 t_NDI.join();
	 
}

/*================================================================
*
* 函 数 名： CaptureData
*
* 参　　数：
*
*
*
* 功能描述:
*			线程1 捕获内窥镜图像
*
*           线程2 捕获对应NDI旋转和平移数据,标记球位置坐标
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::CaptureAllData()
{
	//开启线程
	thread t_endoscope(&EndoscopeAndNDI::OnEndoscopeTrackerMarker, this);
	thread t_NDI(&EndoscopeAndNDI::OnNDITrackerMarker, this);
	t_endoscope.join();
	t_NDI.join();
}

/*================================================================
*
* 函 数 名： OnNDIAndEndoscopeTracker
*
* 参　　数：
*
*
*
* 功能描述:
*			单线程
*           捕获内窥镜图像
*           捕获对应NDI旋转和平移数据
*           时间计时捕获图像
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-26
*
================================================================*/
int EndoscopeAndNDI::OnNDIAndEndoscopeTracker()
{
	//打开NDI——————————————————————————————
	string returnMessage;
	transformationData.resize(0);
	boardRigidData.resize(0);

	if (!m_ndiTracker.DirectStartTracking(returnMessage))
	{//打开导航
		cout << returnMessage << endl;
		return -1;
	}
	////end NDI—————————————————————————————————


	//初始化内窥镜initialize and allocate memory to load the video stream from camera
	VideoCapture camera0(0);
	camera0.set(CAP_PROP_FRAME_WIDTH, imageSize.width);//修改2021-7-7
	camera0.set(CAP_PROP_FRAME_HEIGHT, imageSize.height);	
	

	if (!camera0.isOpened())
		return -1;

	nframes = 0;
	EndoscopeImage.resize(0);
	string file_folder = ".\\Data\\Image\\";
	string type = ".bmp";

	namedWindow("Endoscope", 0);
	resizeWindow("Endoscope", imageSize.width, imageSize.height);
	//end initialize—————————————————————————————————————

	//--验证图像大小（添加2021-7-7）--------------------------------------------------------
	
	if (true)
	{
		Mat3b frametemp;
		camera0 >> frametemp;
		if (frametemp.cols != imageSize.width || frametemp.rows != imageSize.height)
		{
			imageSize.width = frametemp.cols;
			imageSize.height = frametemp.rows;

		}
	}
	//--end--------------------------------------------------------------------------
	
	waitKey(3000);
	stopCapture = false;
	std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();

	while (!stopCapture)
	{
		Mat3b frame0;
		QuatTransformation qtransformationData;
		QuatTransformation qboardRigidData;

		//grab and retrieve each frames of the video sequentially		
		camera0 >> frame0;
		//记录数据
		int flagNDI = m_ndiTracker.GetSysTransformData(qtransformationData, qboardRigidData, 0, 1);

		imshow("Endoscope", frame0);

		//wait for 30 milliseconds
		waitKey(10);

		if (flagNDI != 1)
		{//未采集数据
			continue;
		}
		//为读取图像
		if (frame0.empty())
		{
			continue;
		}
		
		std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
		std::chrono::duration<double,std::milli> timeLag = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(endTime - startTime);
		double dtimeLag = timeLag.count();

		//捕获数据--------------------------------------------------------------------------------------
		if(dtimeLag > 700.0)
		{
			//删除目录中的图像
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//捕获图像
			EndoscopeImage.push_back(frame0);
			//捕获NDI数据
			transformationData.push_back(qtransformationData);
			boardRigidData.push_back(qboardRigidData);
	
			//样本数加一
			nframes++;

			//存储图像
			stringstream ss;
			ss << setw(5) << setfill('0') << nframes;
			string imgPathL = file_folder + ss.str() + type;
			imwrite(imgPathL, frame0);

			if (nframes > 25)
				stopCapture = true;

			startTime = endTime;
		}
		/* else if (dtimeLag >= 410.0)
		{
			startTime = endTime;
		}*/
		//-------------------------------------------------------------------------------------------------

		
	}

	//停止导航
	if (m_ndiTracker.StopTracking())
	{
		cout << "\n导航停止追踪！\n";
	}
	else
	{
		cout << "\n导航停止追踪失败！\n";
	}

	//关闭窗口
	cv::destroyWindow("Endoscope");
	camera0.release();//添加
	return 0;
}

int EndoscopeAndNDI::OnNDIAndEndoscopeTrackerKey()
{
	//打开NDI——————————————————————————————
	string returnMessage;
	transformationData.resize(0);
	boardRigidData.resize(0);

	if (!m_ndiTracker.DirectStartTracking(returnMessage))
	{//打开导航
		cout << returnMessage << endl;
		return -1;
	}
	//end NDI—————————————————————————————————


	//初始化内窥镜initialize and allocate memory to load the video stream from camera
	VideoCapture camera0(0);
	camera0.set(CAP_PROP_FRAME_WIDTH, imageSize.width);
	camera0.set(CAP_PROP_FRAME_HEIGHT, imageSize.height);
	/*imageSize.width = camera0.get(CAP_PROP_FRAME_WIDTH);
	imageSize.height = camera0.get(CAP_PROP_FRAME_HEIGHT);*/

	if (!camera0.isOpened())
		return -1;

	nframes = 0;
	EndoscopeImage.resize(0);
	string file_folder = ".\\Data\\Image\\";
	string type = ".bmp";

	namedWindow("Endoscope", 0);
	//resizeWindow("Endoscope", 960, 540);
	resizeWindow("Endoscope", imageSize.width, imageSize.height);
	//end initialize————————————————————————————————

	stopCapture = false;

	while (!stopCapture)
	{
		Mat3b frame0;
		QuatTransformation qtransformationData;
		QuatTransformation qboardRigidData;

		//grab and retrieve each frames of the video sequentially		
		camera0 >> frame0;
		//记录数据
		int flagNDI = m_ndiTracker.GetSysTransformData(qtransformationData, qboardRigidData, 0, 1);

		imshow("Endoscope", frame0);

		//wait for 30 milliseconds
		waitKey(1);

		if (flagNDI != 1)
		{//未采集数据
			continue;
		}
		//为读取图像
		if (frame0.empty())
		{
			continue;
		}

		int c = waitKey(30);

		//捕获数据--------------------------------------------------------------------------------------
		if (c >= 0 && c != 27 && flagCaptureSampleEToN == false) //按除esc之外的任意键截取图像
		{
			//删除目录中的图像
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//捕获图像
			EndoscopeImage.push_back(frame0);
			Mat a = EndoscopeImage[0];
			//捕获NDI数据
			transformationData.push_back(qtransformationData);
			boardRigidData.push_back(qboardRigidData);

			//样本数加一
			nframes++;

			//存储图像
			stringstream ss;
			ss << setw(5) << setfill('0') << nframes;
			string imgPathL = file_folder + ss.str() + type;
			
			imwrite(imgPathL, frame0);
			

			cout << "成功获取数据 " << nframes << endl;
		}
		else  if (27 == c)//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
		{
			stopCapture = true;
		}
		//-------------------------------------------------------------------------------------------------


	}

	//停止导航
	if (m_ndiTracker.StopTracking())
	{
		cout << "\n导航停止追踪！\n";
	}
	else
	{
		cout << "\n导航停止追踪失败！\n";
	}

	//关闭窗口
	cv::destroyWindow("Endoscope");
	return 0;
}


/*================================================================
*
* 函 数 名： checkCaptureData
*
* 参　　数：
*
*
*
* 功能描述:
*			
*            检验NDI与内窥镜捕获样本数据的数量是否相等
*           
*
* 返 回 值：
*
*           -1 为不相等，1为相等
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
*
================================================================*/
int EndoscopeAndNDI::checkCaptureData()
{
	int c_NDI = transformationData.size();
	int c_Endoscope = EndoscopeImage.size();


	if ((c_NDI == c_Endoscope) && (c_Endoscope == nframes) && (nframes >= 3))
	{
		return 1;
	}
	return -1;
}

//Endoscope and NDI data extract end---------------------------------------------------------


EndoscopeAndNDI::EndoscopeAndNDI()
{
	flagCaptureSampleEToN = false;
	flagCaptureSampleNToE = false;
	stopCapture = false;

	countConerImage = 0;

	//calibration--------------------------------------------------------
	
	cameraMatrix = Mat::eye(3, 3, CV_64FC1);//修改2021-7-12
	distCoeffs = Mat::zeros(8, 1, CV_64FC1);//修改2021-7-12

	winSize = 5;
	release_object = false;
}

EndoscopeAndNDI::EndoscopeAndNDI(Size imageSize ,Size boardSize,float squareSize)
{
	//extrat------------------------------------------------------------
	this->imageSize = imageSize;
	this->boardSize = boardSize;
	this->squareSize = squareSize;

	flagCaptureSampleEToN = false;
	flagCaptureSampleNToE = false;
	stopCapture = false;

	countConerImage = 0;

	//calibration--------------------------------------------------------
	
	cameraMatrix = Mat::eye(3, 3, CV_64FC1);
	distCoeffs = Mat::zeros(8, 1, CV_64FC1);

	winSize = 5;
	grid_width = squareSize * (boardSize.width - 1);
	release_object = false;	
}

void EndoscopeAndNDI::setImageSize(Size imageSize)
{
	this->imageSize = imageSize;
}

void EndoscopeAndNDI::setBoardSize(Size boardSize)
{
	this->boardSize = boardSize;
}

void EndoscopeAndNDI::setSquareSize(float squareSize)
{
	this->squareSize = squareSize;
}

void EndoscopeAndNDI::setGridWidth()
{
	grid_width = squareSize * (boardSize.width - 1);
}


