#pragma once

/*================================================================
*
* 文 件 名：
*           EndoscopeAndNDI.h
*
* 文件描述:
*			进行内窥镜与NDI之间的相关操作：
*           1、利用内窥镜捕获图像
*           2、NDI同步记录刚体数据
*           3、对内窥镜进行相机标定
*          
*
* 版 本 号：
*           1.0
*
* 作　　者：
*           许毅 2021-4-26
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
	Size                                boardSize;            //棋盘格行和列方向角点数量    
	Mat                                 cameraMatrix;         //内参矩阵
	Mat                                 distCoeffs;           //畸变系数k1k2p1p2k3	
	vector<float>                       reprojErrs;	          //重投影误差
	float                               squareSize ;          //棋盘格中正方形方格的大小
	int                                 nframes ;             //标定样本的数量
	int                                 winSize ;             //窗口大小，用于优化提取角点坐标
	float                               grid_width;           //
	vector<vector<Point2f>>             imagePoints;          //棋盘格角点图像坐标
	double                              totalAvgErr;          //总的平均误差
	vector<Point3f>                     newObjPoints;         
	bool                                release_object;   

public:
	int                                 countConerImage;      //成功提取角点的图像数量
	vector<int>                         noCornerImageID;      //未能提取角点的图像的序号，从1开始

	vector<Mat>                         rvecs;                //外参旋转向量
	vector<Mat>                         tvecs;                //外参平移向量



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
	int                                     sampleNumber;           //采集样本的数量
	bool                                    flagCaptureSampleEToN;    //内窥镜向NDI发送捕获消息  false
	bool                                    flagCaptureSampleNToE;    //NDI回应内窥镜消息        false
	bool                                    stopCapture;              //NDI与内窥镜追踪标志      false

	vector<Mat>                             EndoscopeImage;           //内窥镜捕获图像
	
	NDI                                     m_ndiTracker;
	Size                                    imageSize;                //设置捕获图像的大小      1920*1080

	vector<QuatTransformation>              transformationData;       //NDI记录空间变换信息

	vector<QuatTransformation>              boardRigidData;           //NDI标记球坐标

public:
	void    OnNDITracker();

	void    OnNDITrackerMarker();

	int     OnEndoscopeTrackerKey();

	int     OnEndoscopeTrackerTime();

	int     OnEndoscopeTrackerMarker();

    void    CaptureData();

	void    CaptureAllData();

	int     OnNDIAndEndoscopeTracker();//单线程  时间

	int     OnNDIAndEndoscopeTrackerKey();//单线程  按键

	int     checkCaptureData();
//Endoscope and NDI data extract end---------------------------------------------------------
	

public:

	//构造函数
	EndoscopeAndNDI();
	EndoscopeAndNDI(Size imageSize, Size boardSize, float squareSize);
	//EndoscopeAndNDI();

	void setImageSize(Size imageSize);
	void setBoardSize(Size boardSize);
	void setSquareSize(float  squareSize);
	void setGridWidth();
};