#pragma once

/*================================================================
*
* �� �� ����
*           EndoscopeAndNDI.h
*
* �ļ�����:
*			�����ڿ�����NDI֮�����ز�����
*           1�������ڿ�������ͼ��
*           2��NDIͬ����¼��������
*           3�����ڿ�����������궨
*          
*
* �� �� �ţ�
*           1.0
*
* �������ߣ�
*           ���� 2021-4-26
*
================================================================*/

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "NDI_File\NDI.h"

#include <chrono>

using namespace cv;
using namespace std;

class EndoscopeAndNDI
{

//Endoscope calibration  start-------------------------------------------------------
public:
	Size                                boardSize;            //���̸��к��з���ǵ�����    
	Mat                                 cameraMatrix;         //�ڲξ���
	Mat                                 distCoeffs;           //����ϵ��k1k2p1p2k3	
	vector<float>                       reprojErrs;	          //��ͶӰ���
	float                               squareSize ;          //���̸��������η���Ĵ�С
	int                                 nframes ;             //�궨����������
	int                                 winSize ;             //���ڴ�С�������Ż���ȡ�ǵ�����
	float                               grid_width;           //
	vector<vector<Point2f>>             imagePoints;          //���̸�ǵ�ͼ������
	double                              totalAvgErr;          //�ܵ�ƽ�����
	vector<Point3f>                     newObjPoints;         
	bool                                release_object;   

public:
	int                                 countConerImage;      //�ɹ���ȡ�ǵ��ͼ������
	vector<int>                         noCornerImageID;      //δ����ȡ�ǵ��ͼ�����ţ���1��ʼ

	vector<Mat>                         rvecs;                //�����ת����
	vector<Mat>                         tvecs;                //���ƽ������



public:
	 int    extrackChessBoardCorners();

	 bool   runCalibration();
	
	 void   saveCameraParams();
	
	 double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints, const vector<vector<Point2f> >& imagePoints, vector<float>& perViewErrors);
	
	 void   calcChessboardCorners(vector<Point3f>& corners);
	
	 bool   runAndSave();

//Endoscope calibration end---------------------------------------------------------

	 void   saveNDIParams();

//Endoscope and NDI data extract start-------------------------------------------------------
public:
	int                                     sampleNumber;           //�ɼ�����������
	bool                                    flagCaptureSampleEToN;    //�ڿ�����NDI���Ͳ�����Ϣ  false
	bool                                    flagCaptureSampleNToE;    //NDI��Ӧ�ڿ�����Ϣ        false
	bool                                    stopCapture;              //NDI���ڿ���׷�ٱ�־      false

	vector<Mat>                             EndoscopeImage;           //�ڿ�������ͼ��
	
	NDI                                     m_ndiTracker;
	Size                                    imageSize;                //���ò���ͼ��Ĵ�С      1920*1080

	vector<QuatTransformation>              transformationData;       //NDI��¼�ռ�任��Ϣ

	vector<QuatTransformation>              boardRigidData;           //NDI���������

public:
	void    OnNDITracker();

	void    OnNDITrackerMarker();

	int     OnEndoscopeTrackerKey();

	int     OnEndoscopeTrackerTime();

	int     OnEndoscopeTrackerMarker();

    void    CaptureData();

	void    CaptureAllData();

	int     OnNDIAndEndoscopeTracker();//���߳�  ʱ��

	int     OnNDIAndEndoscopeTrackerKey();//���߳�  ����

	int     checkCaptureData();
//Endoscope and NDI data extract end---------------------------------------------------------
	

public:

	//���캯��
	EndoscopeAndNDI();
	EndoscopeAndNDI(Size imageSize, Size boardSize, float squareSize);
	//EndoscopeAndNDI();

	void setImageSize(Size imageSize);
	void setBoardSize(Size boardSize);
	void setSquareSize(float  squareSize);
	void setGridWidth();
};