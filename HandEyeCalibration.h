#pragma once

/*================================================================
*
* �� �� ����
*           HandEyeCalibration.h
*
* �ļ�����:
*			�ڿ�����NDI����֮������۱궨����Ҫ������
*           1����ö�Ӧ�֡�������
*           2�������֡������ݣ�����A��B
*           3������Robust�㷨�������۱궨
*           
*
* �� �� �ţ�
*           1.0
*
* �������ߣ� 
*           ���� 2021-4-26
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
*description:��ת����Ԫ������ƽ�ƣ�������
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

	vector<Transformation<double>>        As;                     //�� NDI
											                      
	vector<Transformation<double>>        Bs;                     //��  Endoscope
												                  
	vector<mist::matrix<double>>          handData;               //NDI
			                              		                  
	vector<Mat>                           eyeData;                // Endoscope

	mist::matrix<double>                  X;

	int                                   countSample;            //�֡����������ݵ�����

	EndoscopeAndNDI                       en;

	vector<vector<double>>                markerCenter;

	double                                ER;                     //��ת���  

	double                                Et;                     //ƽ�����


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
	HandEye(Size imageSize = Size (1280,720));//���2021-6-29
};
