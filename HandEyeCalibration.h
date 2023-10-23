#pragma once

/*================================================================
*
* 文 件 名：
*           HandEyeCalibration.h
*
* 文件描述:
*			内窥镜与NDI刚体之间的手眼标定，主要操作：
*           1、获得对应手、眼数据
*           2、处理手、眼数据，创建A和B
*           3、基于Robust算法进行手眼标定
*           
*
* 版 本 号：
*           1.0
*
* 作　　者： 
*           许毅 2021-4-26
*
================================================================*/

#include "quaternion.h"
#include <vector>
#include <opencv2/core.hpp>
#include <mist/numeric.h>
#include "EndoscopeAndNDI.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//#include <Eigen\dense>



using namespace std;
using namespace cv;

/**************************************
*
*description:旋转（四元数）和平移（向量）
*
***************************************/
template <class T>
struct Transformation
{
	int              i;
	int              j;
	Quaternion<T>    q;
	mist::vector3<T> t;
};

class HandEye
{
public:

	vector<Transformation<double>>        As;                     //手 NDI
											                      
	vector<Transformation<double>>        Bs;                     //眼  Endoscope
												                  
	vector<mist::matrix<double>>          handData;               //NDI
			                              		                  
	vector<Mat>                           eyeData;                // Endoscope

	mist::matrix<double>                  X;

	int                                   countSample;            //手、眼样本数据的数量

	EndoscopeAndNDI                       en;

	vector<vector<double>>                markerCenter;

	double                                ER;                     //旋转误差  

	double                                Et;                     //平移误差


public:

//--acquire data-------------------------------------------------------
	bool                   getHandData(vector<QuatTransformation>  &transformationData, vector<int> &noCornerImageID);
					      
	bool                   getHandData();
		                   
	bool                   getEyeData(vector<Mat> &rvecs, vector<Mat> &tvecs);
					      
	bool                   getEyeData();
		                   
	void                   getMovementsPair();
				     
	void                   getMovementsPairHtoE();
//--------------------------------------------------------------------

//--method of calibration---------------------------------------------
	mist::matrix<double>   GetAntiSymmetrixOfVector(mist::matrix<double> vec);
				     
	mist::matrix<double>   SolveRobust(const mist::matrix<double> & matrixL);
				     
	mist::matrix<double>   GetXRobust();
				     
	mist::matrix<double>   GetXRobust1();

	matrix<double>         GetXofAX_XB(vector3<double>* t_for_debug, QuaternionD * q_for_debug);

	matrix<double>         SolveTq_0(const matrix<double>& matrixT, vector3<double>* t_for_debug, QuaternionD * q_for_debug);
//--------------------------------------------------------------------

//--operation method--------------------------------------------------
	bool                   runHandEyeCalibration(vector<QuatTransformation>  &transformationData, vector<int> &noCornerImageID, vector<Mat> &rvecs, vector<Mat> &tvecs);
					      
	bool                   runHandEyeCalibration();

	bool                   runHandEyeCalibrationHtoE();	
					      
	bool                   runEndoscopeAndNDI();
//-------------------------------------------------------------------

//--error------------------------------------------------------------

	void                   testResult();
					      
	bool                   errorMatrix();

	void                   calculateError();

	bool                   calculateMeanValue(vector<double> a, double &mv);

	bool                   verifyCalibrationOfTriangular();//2022-3-2gengxin
//-------------------------------------------------------------------
					      
	void                   MistMatrix2CvMat(Mat &cvmat, mist::matrix<double> &mat);
					      
	void                   CvMat2MistMatrix(Mat cvmat, mist::matrix<double> &mat);
					      
	void                   OutputMatrixToFile(const char * filename, mist::matrix<double> mat);

	
public:

	HandEye(int countSample);
	//HandEye();
	HandEye(Size imageSize = Size (1280,720));//添加2021-6-29
};
