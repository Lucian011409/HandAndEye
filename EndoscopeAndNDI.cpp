#include "EndoscopeAndNDI.h"
#include <iomanip>
#include <thread>


//Endoscope calibration start-------------------------------------------------------
/*================================================================
*
* �� �� ����extrackChessBoardCorners
*
* �Ρ�������
*            
*           
*
* ��������:
*			��ȡ���̸���ʽ�ı궨ͼ���еĽǵ�����
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
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
	resizeWindow("Image View", imageSize.width, imageSize.height);//�޸�2021-7-7
	imagePoints.clear();

	countConerImage = 0;
	noCornerImageID.resize(0);

	//��ȡͼ���еĽǵ�
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
* �� �� ���� runCalibration
*
* �Ρ�������
*
*
* ��������:
*			�궨�ڿ����������ڿ������ڲΡ���Ρ�����ϵ��
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
*
================================================================*/
bool EndoscopeAndNDI::runCalibration()
{
	//�������꣬zΪ0
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
* �� �� ���� saveCameraParams
*
* �Ρ�������                     
*
*
* ��������:
*			���ڿ����ı궨�������浽 cameraParament.yal �ļ���
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
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
* �� �� ���� saveNDIParams
*
* �Ρ�������
*
*
* ��������:
*			��NDI�ռ�任��Ϣ���浽�ļ�������¼�޳���Ϣ���
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
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

	//����δ�ܳɹ���ȡͼ��ǵ��ͼ��ID
	for (int i = 0; i < noCornerImageID.size(); i++)
	{
		fprintf(cf, "%d\n", noCornerImageID[i]);
	}

	fclose(cf);

	//����NDI�ռ�任��Ϣ
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
* �� �� ���� computeReprojectionErrors
*
* �Ρ�������
*
*            const vector<vector<Point3f>>& objectPoints
*
*            const vector<vector<Point2f>>& imagePoints
*
*           vector<float>& perViewErrors
*
*           
*
* ��������:
*			ͨ����ͶӰ�����ڿ����궨���������
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
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
* �� �� ���� calcChessboardCorners
*
* �Ρ�������
*
*            vector<Point3f>& corners
*
*
*
* ��������:
*			
*            �������̸�ǵ����������
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
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
* �� �� ���� runAndSave
*
* �Ρ�������
*
*
* ��������:
*			1����ȡ�ǵ㣬��������ʾ
*           2���궨�ڿ����������ڿ������������
*           3�������ڿ�������
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
*
================================================================*/
bool EndoscopeAndNDI::runAndSave()
{
	//���鲶�����������Ƿ���Ч
	if (checkCaptureData() == -1)
	{
		cout << "����������Ч!\n";
		return false;
	}
	   
	//��ȡ�ǵ�
	if (extrackChessBoardCorners() == -1)
	{
		cout << "��ȡ�ǵ�ʧ��!\n";
		return false;
	}

	//�궨
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

		//��ͼ��δ��ȡ�ǵ�
		if (noCornerImageID.size() > 0)
		{
			int flagTemp = 0;                          //noCornerImageID�е����
			int itemp = noCornerImageID[flagTemp];     //δ����ȡ�ǵ��ͼ���ID����1��ʼ

			int flagConerImage = 0;

			//��������
			for (int i = 0; i < nframes; i++)
			{
				if ((i + 1) == itemp)
				{
					flagTemp++;
					//��ֹԽ��
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
		{//ͼ��ȫ���ɼ��ǵ�
			//��������
			for (int i = 0; i < nframes; i++)
			{
					Mat view = EndoscopeImage[i];
					drawFrameAxes(view, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 100, 2);
					imshow("Image View", view);					
					waitKey(10);
			}//for
		}//else

		cv::destroyWindow("Image View");

		//�����������
		saveCameraParams();

		//����NDI����
		saveNDIParams();
	}
	return ok;
	
}

//Endoscope calibration end---------------------------------------------------------


//Endoscope and NDI data extract start-------------------------------------------------------
/*================================================================
*
* �� �� ���� OnNDITracker
*
* �Ρ�������
*
*            
*
* ��������:
*			��NDI��׷�ٸ��壬��¼��ת��ƽ�����ݣ�����ֲ�����ϵ��NDI��������ϵ��
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::OnNDITracker()
{
	string returnMessage;
	
	if (m_ndiTracker.DirectStartTracking(returnMessage))
	{//�򿪵���

		//��¼����
		m_ndiTracker.GetSysTransformData(transformationData,stopCapture, flagCaptureSampleEToN, flagCaptureSampleNToE);

		//ֹͣ����
		if (m_ndiTracker.StopTracking())
		{
			cout << "\n����ֹͣ׷�٣�\n";
		}
		else
		{
			cout << "\n����ֹͣ׷��ʧ�ܣ�\n";
		}

	}
	else
	{
		cout << returnMessage << endl;
	}
}

/*================================================================
*
* �� �� ���� OnNDITrackerMarker
*
* �Ρ�������
*
*
*
* ��������:
*			��NDI��׷�ٸ��壬��¼��ת��ƽ������,�����λ�����꣨����ֲ�����ϵ��NDI��������ϵ��
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::OnNDITrackerMarker()
{
	string returnMessage;
	transformationData.resize(0);

	if (m_ndiTracker.DirectStartTracking(returnMessage))
	{//�򿪵���

		//��¼����
		//m_nditracker.getsystransformdata(transformationdata,markerdata, stopcapture, flagcapturesampleeton, flagcapturesamplentoe);

		//ֹͣ����
		if (m_ndiTracker.StopTracking())
		{
			cout << "\n����ֹͣ׷�٣�\n";
		}
		else
		{
			cout << "\n����ֹͣ׷��ʧ�ܣ�\n";
		}

	}
	else
	{
		cout << returnMessage << endl;
	}
}


/*================================================================
*
* �� �� ���� OnEndoscopeTrackerKey
*
* �Ρ�������
*
*
*
* ��������:
*			���ڿ���������ͼ��,ͨ����������ͼ��
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
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

		//Ϊ��ȡͼ��
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

		if (c >= 0 && c != 27&& flagCaptureSampleEToN ==false) //����esc֮����������ȡͼ��
		{
			//ɾ��Ŀ¼�е�ͼ��
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//����ͼ��
			EndoscopeImage.push_back(frame0);

			//�ڿ�����NDI����������Ϣ
			flagCaptureSampleEToN = true;

			//��������һ
			nframes++;

			

			//����NDI�Ƿ��Ӧ�ڿ�����Ϣ
			this_thread::sleep_for(chrono::milliseconds(20));
			if (flagCaptureSampleNToE == false)
			{//δ��Ӧ
				flagCaptureSampleEToN = false;
				EndoscopeImage.pop_back();
				nframes--;
			}
			else
			{//��Ӧ
				flagCaptureSampleNToE = false;

				//�洢ͼ��
				stringstream ss;
				ss << setw(5) << setfill('0') << nframes;
				string imgPathL = file_folder + ss.str() + type;
				imwrite(imgPathL, frame0);

				cout << "�ɹ���ȡ���� " << nframes << endl;

				
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
* �� �� ���� OnEndoscopeTrackerTime
*
* �Ρ�������
*
*
*
* ��������:
*			���ڿ���������ͼ��ͨ����ʱ����ͼ��
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
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

		//Ϊ��ȡͼ��
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

		if (c >= 0 && c != 27 && flagCaptureSampleEToN == false) //����esc֮����������ȡͼ��
		{
			//ɾ��Ŀ¼�е�ͼ��
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//����ͼ��
			EndoscopeImage.push_back(frame0);

			//�ڿ�����NDI����������Ϣ
			flagCaptureSampleEToN = true;

			//��������һ
			nframes++;

			//�洢ͼ��
			stringstream ss;
			ss << setw(5) << setfill('0') << nframes;
			string imgPathL = file_folder + ss.str() + type;
			imwrite(imgPathL, frame0);

			//����NDI�Ƿ��Ӧ�ڿ�����Ϣ
			this_thread::sleep_for(chrono::milliseconds(10));
			if (flagCaptureSampleNToE == false)
			{//δ��Ӧ
				flagCaptureSampleEToN = false;
				EndoscopeImage.pop_back();
				nframes--;
			}
			else
			{//��Ӧ
				flagCaptureSampleNToE = false;
				cout << "�ɹ���ȡ���� " << nframes << endl;
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
* �� �� ���� OnEndoscopeTrackerMarker
*
* �Ρ�������
*
*
*
* ��������:
*			���ڿ�������������ͼ�����ڷ�ͶӰ��֤
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
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

		//Ϊ��ȡͼ��
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

		if (c >= 0 && c != 27 && flagCaptureSampleEToN == false) //����esc֮����������ȡͼ��
		{
			//ɾ��Ŀ¼�е�ͼ��
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//����ͼ��
			EndoscopeImage.push_back(frame0);

			//�ڿ�����NDI����������Ϣ
			flagCaptureSampleEToN = true;

			//��������һ
			nframes++;

			//�洢ͼ��
			stringstream ss;
			ss << setw(5) << setfill('0') << nframes;
			string imgPathL = file_folder + ss.str() + type;
			imwrite(imgPathL, frame0);

			//����NDI�Ƿ��Ӧ�ڿ�����Ϣ
			this_thread::sleep_for(chrono::milliseconds(10));
			if (flagCaptureSampleNToE == false)
			{//δ��Ӧ
				flagCaptureSampleEToN = false;
				EndoscopeImage.pop_back();
				nframes--;
			}
			else
			{//��Ӧ
				flagCaptureSampleNToE = false; 
				cout << "�ɹ���ȡ���� " << nframes << endl;
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
* �� �� ���� CaptureData
*
* �Ρ�������
*
*
*
* ��������:
*			�߳�1 �����ڿ���ͼ��
*
*           �߳�2 �����ӦNDI��ת��ƽ������
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::CaptureData()
{
	//�����߳�
	 thread t_endoscope(&EndoscopeAndNDI::OnEndoscopeTrackerKey, this);
	 thread t_NDI(&EndoscopeAndNDI::OnNDITracker,this);
	 t_endoscope.join();
	 t_NDI.join();
	 
}

/*================================================================
*
* �� �� ���� CaptureData
*
* �Ρ�������
*
*
*
* ��������:
*			�߳�1 �����ڿ���ͼ��
*
*           �߳�2 �����ӦNDI��ת��ƽ������,�����λ������
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
*
================================================================*/
void EndoscopeAndNDI::CaptureAllData()
{
	//�����߳�
	thread t_endoscope(&EndoscopeAndNDI::OnEndoscopeTrackerMarker, this);
	thread t_NDI(&EndoscopeAndNDI::OnNDITrackerMarker, this);
	t_endoscope.join();
	t_NDI.join();
}

/*================================================================
*
* �� �� ���� OnNDIAndEndoscopeTracker
*
* �Ρ�������
*
*
*
* ��������:
*			���߳�
*           �����ڿ���ͼ��
*           �����ӦNDI��ת��ƽ������
*           ʱ���ʱ����ͼ��
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-26
*
================================================================*/
int EndoscopeAndNDI::OnNDIAndEndoscopeTracker()
{
	//��NDI������������������������������������������������������������
	string returnMessage;
	transformationData.resize(0);
	boardRigidData.resize(0);

	if (!m_ndiTracker.DirectStartTracking(returnMessage))
	{//�򿪵���
		cout << returnMessage << endl;
		return -1;
	}
	////end NDI������������������������������������������������������������������


	//��ʼ���ڿ���initialize and allocate memory to load the video stream from camera
	VideoCapture camera0(0);
	camera0.set(CAP_PROP_FRAME_WIDTH, imageSize.width);//�޸�2021-7-7
	camera0.set(CAP_PROP_FRAME_HEIGHT, imageSize.height);	
	

	if (!camera0.isOpened())
		return -1;

	nframes = 0;
	EndoscopeImage.resize(0);
	string file_folder = ".\\Data\\Image\\";
	string type = ".bmp";

	namedWindow("Endoscope", 0);
	resizeWindow("Endoscope", imageSize.width, imageSize.height);
	//end initialize��������������������������������������������������������������������������

	//--��֤ͼ���С�����2021-7-7��--------------------------------------------------------
	
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
		//��¼����
		int flagNDI = m_ndiTracker.GetSysTransformData(qtransformationData, qboardRigidData, 0, 1);

		imshow("Endoscope", frame0);

		//wait for 30 milliseconds
		waitKey(10);

		if (flagNDI != 1)
		{//δ�ɼ�����
			continue;
		}
		//Ϊ��ȡͼ��
		if (frame0.empty())
		{
			continue;
		}
		
		std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
		std::chrono::duration<double,std::milli> timeLag = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(endTime - startTime);
		double dtimeLag = timeLag.count();

		//��������--------------------------------------------------------------------------------------
		if(dtimeLag > 700.0)
		{
			//ɾ��Ŀ¼�е�ͼ��
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//����ͼ��
			EndoscopeImage.push_back(frame0);
			//����NDI����
			transformationData.push_back(qtransformationData);
			boardRigidData.push_back(qboardRigidData);
	
			//��������һ
			nframes++;

			//�洢ͼ��
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

	//ֹͣ����
	if (m_ndiTracker.StopTracking())
	{
		cout << "\n����ֹͣ׷�٣�\n";
	}
	else
	{
		cout << "\n����ֹͣ׷��ʧ�ܣ�\n";
	}

	//�رմ���
	cv::destroyWindow("Endoscope");
	camera0.release();//���
	return 0;
}

int EndoscopeAndNDI::OnNDIAndEndoscopeTrackerKey()
{
	//��NDI������������������������������������������������������������
	string returnMessage;
	transformationData.resize(0);
	boardRigidData.resize(0);

	if (!m_ndiTracker.DirectStartTracking(returnMessage))
	{//�򿪵���
		cout << returnMessage << endl;
		return -1;
	}
	//end NDI������������������������������������������������������������������


	//��ʼ���ڿ���initialize and allocate memory to load the video stream from camera
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
	//end initialize����������������������������������������������������������������

	stopCapture = false;

	while (!stopCapture)
	{
		Mat3b frame0;
		QuatTransformation qtransformationData;
		QuatTransformation qboardRigidData;

		//grab and retrieve each frames of the video sequentially		
		camera0 >> frame0;
		//��¼����
		int flagNDI = m_ndiTracker.GetSysTransformData(qtransformationData, qboardRigidData, 0, 1);

		imshow("Endoscope", frame0);

		//wait for 30 milliseconds
		waitKey(1);

		if (flagNDI != 1)
		{//δ�ɼ�����
			continue;
		}
		//Ϊ��ȡͼ��
		if (frame0.empty())
		{
			continue;
		}

		int c = waitKey(30);

		//��������--------------------------------------------------------------------------------------
		if (c >= 0 && c != 27 && flagCaptureSampleEToN == false) //����esc֮����������ȡͼ��
		{
			//ɾ��Ŀ¼�е�ͼ��
			if (0 == nframes)
			{
				string deleteImgPathL = "del " + file_folder + "*" + type;
				system(deleteImgPathL.c_str());
			}

			//����ͼ��
			EndoscopeImage.push_back(frame0);
			Mat a = EndoscopeImage[0];
			//����NDI����
			transformationData.push_back(qtransformationData);
			boardRigidData.push_back(qboardRigidData);

			//��������һ
			nframes++;

			//�洢ͼ��
			stringstream ss;
			ss << setw(5) << setfill('0') << nframes;
			string imgPathL = file_folder + ss.str() + type;
			
			imwrite(imgPathL, frame0);
			

			cout << "�ɹ���ȡ���� " << nframes << endl;
		}
		else  if (27 == c)//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
		{
			stopCapture = true;
		}
		//-------------------------------------------------------------------------------------------------


	}

	//ֹͣ����
	if (m_ndiTracker.StopTracking())
	{
		cout << "\n����ֹͣ׷�٣�\n";
	}
	else
	{
		cout << "\n����ֹͣ׷��ʧ�ܣ�\n";
	}

	//�رմ���
	cv::destroyWindow("Endoscope");
	return 0;
}


/*================================================================
*
* �� �� ���� checkCaptureData
*
* �Ρ�������
*
*
*
* ��������:
*			
*            ����NDI���ڿ��������������ݵ������Ƿ����
*           
*
* �� �� ֵ��
*
*           -1 Ϊ����ȣ�1Ϊ���
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
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
	
	cameraMatrix = Mat::eye(3, 3, CV_64FC1);//�޸�2021-7-12
	distCoeffs = Mat::zeros(8, 1, CV_64FC1);//�޸�2021-7-12

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


