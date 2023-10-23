#include "HandEyeCalibration.h"



/*=======================================================================
*
* 函 数 名： getHandData
*
* 参　　数：
*
*
* 功能描述:
*			获得手的数据，手数据为NDI的空间变换数据，从EndoscopeAndNDI类
*           中transformationData获得，提取过程中剔除内窥镜图像中未获得角
*           点的图像对应的NDI空间变换数据，未能提取角点的图像的序号保存在
*           noCornerImageID中
*           手的数据保存在 handData 中
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
bool HandEye::getHandData(vector<QuatTransformation>  &transformationData, vector<int> &noCornerImageID)
{
	handData.resize(0);

	//有未提取角点的图像
	if (noCornerImageID.size() != 0)               //noCornerImageID是未能提取角点的图像的序号，从1开始
	{

		int flagTemp = 0;                          //noCornerImageID中的序号
		int itemp = noCornerImageID[flagTemp];     //未能提取角点的图像的ID，从1开始

		//获得手的数据
		for (int i = 0; i < (countSample + noCornerImageID.size()); i++)  //countSample手、眼样本数据的数量
		{
			//剔除未提取角点的图像对应的手的数据
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
				Quaternion<double> q(transformationData[i].rotation.q0, transformationData[i].rotation.qx, transformationData[i].rotation.qy, transformationData[i].rotation.qz);
				mist::matrix<double> m_qtomatrix = q.ConvertToMatrix();
				m_qtomatrix(0, 3) = transformationData[i].translation.x;
				m_qtomatrix(1, 3) = transformationData[i].translation.y;
				m_qtomatrix(2, 3) = transformationData[i].translation.z;
				//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAConvertToMatrix()将将四元数转换为 4x4 矩阵AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
				handData.push_back(m_qtomatrix);
			}
		}
	}
	else
	{//所有图像角点提取成功

		//获得手的数据
		for (int i = 0; i < countSample; i++)
		{			
				Quaternion<double> q(transformationData[i].rotation.q0, transformationData[i].rotation.qx, transformationData[i].rotation.qy, transformationData[i].rotation.qz);
				mist::matrix<double> m_qtomatrix = q.ConvertToMatrix();
				m_qtomatrix(0, 3) = transformationData[i].translation.x;
				m_qtomatrix(1, 3) = transformationData[i].translation.y;
				m_qtomatrix(2, 3) = transformationData[i].translation.z;

				handData.push_back(m_qtomatrix);			
		}
	}

	return true;
}

bool HandEye::getHandData()
{
	handData.resize(0);

	//有未提取角点的图像
	if (en.noCornerImageID.size() != 0)
	{

		int flagTemp = 0;                          //noCornerImageID中的序号
		int itemp = en.noCornerImageID[flagTemp];     //未能提取角点的图像的ID，从1开始

		//获得手的数据
		for (int i = 0; i < (countSample + en.noCornerImageID.size()); i++)
		{
			//剔除未提取角点的图像对应的手的数据
			if ((i + 1) == itemp)
			{
				flagTemp++;
				//防止越界
				if (flagTemp < en.noCornerImageID.size())
				{
					itemp = en.noCornerImageID[flagTemp];
				}
				continue;
			}
			else
			{
				Quaternion<double> q(en.transformationData[i].rotation.q0, en.transformationData[i].rotation.qx, en.transformationData[i].rotation.qy, en.transformationData[i].rotation.qz);
				mist::matrix<double> m_qtomatrix = q.ConvertToMatrix();
				m_qtomatrix(0, 3) = en.transformationData[i].translation.x;
				m_qtomatrix(1, 3) = en.transformationData[i].translation.y;
				m_qtomatrix(2, 3) = en.transformationData[i].translation.z;

				handData.push_back(m_qtomatrix);
			}
		}
	}
	else
	{//所有图像角点提取成功

		//获得手的数据
		for (int i = 0; i < countSample; i++)
		{
			Quaternion<double> q(en.transformationData[i].rotation.q0, en.transformationData[i].rotation.qx, en.transformationData[i].rotation.qy, en.transformationData[i].rotation.qz);
			mist::matrix<double> m_qtomatrix = q.ConvertToMatrix();
			m_qtomatrix(0, 3) = en.transformationData[i].translation.x;
			m_qtomatrix(1, 3) = en.transformationData[i].translation.y;
			m_qtomatrix(2, 3) = en.transformationData[i].translation.z;

			handData.push_back(m_qtomatrix);
		}
	}

	return true;
}

/*=======================================================================
*
* 函 数 名： getEyeData
*
* 参　　数：
*
*
* 功能描述:
*			获得眼的数据，眼数据为标定内窥镜获得的外参数，从EndoscopeAndNDI类
*           中rvecs 和 tvecs获得，
*           眼的数据保存在 eyeData 中 
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
bool HandEye::getEyeData(vector<Mat> &rvecs, vector<Mat> &tvecs)
{
	if ((rvecs.size() == tvecs.size())&&(rvecs.size() == countSample))
	{
		if (rvecs[0].type() != tvecs[0].type())
		{
			return false;
		}

		
		
		//Mat t = m(Range(3, 4), Range(0, 3));
		
		for (int k = 0; k < countSample; k++)
		{
			Mat m = Mat::eye(4, 4, rvecs[0].type());

			//旋转矩阵和平移向量转化为 4*4齐次矩阵
			Mat t = m(Rect(3, 0, 1, 3));
			//Mat t = m(Range(0, 3), Range(3, 4));
			Mat r = m(Range(0, 3), Range(0, 3));

			//旋转向量转化为旋转矩阵
			Rodrigues(rvecs[k], r);

			CV_Assert(tvecs[k].rows == 3 && tvecs[k].cols == 1);
			//t = tvecs[k].clone();
			tvecs[k].copyTo(t);

			eyeData.push_back(m);
		}//for

		return true;
	}//if

	return false;
}

bool HandEye::getEyeData()
{
	if ((en.rvecs.size() == en.tvecs.size()) && (en.rvecs.size() == countSample))
	{
		if (en.rvecs[0].type() != en.tvecs[0].type())
		{
			return false;
		}

		for (int k = 0; k < countSample; k++)
		{
			Mat m = Mat::eye(4, 4, en.rvecs[0].type());

			//旋转矩阵和平移向量转化为 4*4齐次矩阵
			Mat t = m(Rect(3, 0, 1, 3));
			//Mat t = m(Range(0, 3), Range(3, 4));
			Mat r = m(Range(0, 3), Range(0, 3));

			//旋转向量转化为旋转矩阵
			Rodrigues(en.rvecs[k], r);

			CV_Assert(en.tvecs[k].rows == 3 && en.tvecs[k].cols == 1);
			//t = tvecs[k].clone();
			en.tvecs[k].copyTo(t);

			eyeData.push_back(m);
		}//for

		//将眼数据输入到文件
		FILE *f;
		f = fopen("./Data\\cameraParament\\eyedata.txt", "w");

		for (int k = 0; k < eyeData.size(); k++)
		{
			Mat temp;
			eyeData[k].copyTo(temp);

			for (int i = 0; i < temp.rows; i++)
			{
				for (int j = 0; j < temp.cols; j++)
				{
					fprintf(f, "%lf ", temp.at<double>(i, j));
				}
				//fprintf(f, "\n");
			}
			fprintf(f, "\n");
		}

		fclose(f);

		return true;
	}//if


	return false;
}


/*=======================================================================
*
* 函 数 名： CvMat2MistMatrix
*
* 参　　数：
*
*
* 功能描述:
*			
*            opencv中Mat转换为Mist中matrix
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/

void HandEye::CvMat2MistMatrix(Mat cvmat, mist::matrix<double> &mat)
{
	for (int i = 0; i < mat.rows(); i++)
	{
		double *temp = cvmat.ptr<double>(i);
		for (int j = 0; j < mat.cols(); j++)
		{
			mat(i, j) = temp[j];
		}
	}
}

/*=======================================================================
*
* 函 数 名： MistMatrix2CvMat
*
* 参　　数：
*
*
* 功能描述:
*			
*            Mist中matrix中转化为opencv中Mat
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
void HandEye::MistMatrix2CvMat(Mat &cvmat, mist::matrix<double> &mat)
{
	for (int i = 0; i < mat.rows(); i++)
	{ 
		double *cvtemp = cvmat.ptr<double>(i);
		for (int j = 0; j < mat.cols(); j++)
		{
			cvtemp[j] = mat(i, j);
		}
	}
}


/*=======================================================================
*
* 函 数 名： getMovementsPair
*
* 参　　数：
*
*
* 功能描述:
*			
*            手的数据A= Aj-1Ai
*            眼的数据B= BjBi-1
*            样本数量总共生成 N*（N-1）/2
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/


void HandEye::getMovementsPair()
{
	int index = 0;
	Transformation<double> aTransformation;

	//int motionCount = countSample * (countSample - 1) / 2;// N eye or hand motion 可以形成的运动总数
	int motionCount = (countSample - 1) * (countSample - 2) / 2;//为了检验时最后一个用于检验

	As.resize(motionCount);
	Bs.resize(motionCount);

	for (int j = 1; j < countSample - 1; j++)
	{
		for (int i = 0; i < j; i++)
		{
			//NDI (hand)
			Mat mAi(4, 4,CV_64FC1);
			Mat mAj(4, 4, CV_64FC1);
			Mat mAjInv(4, 4, CV_64FC1);

			MistMatrix2CvMat(mAi, handData[i]);
			MistMatrix2CvMat(mAj, handData[j]);

			invert(mAj, mAjInv);

			Mat mA = mAjInv * mAi;

			mist::matrix<double> hR(3, 3);
			vector<double> ht;
			for (int ii = 0; ii < 3; ii++)
			{
				double *tempPtr = mA.ptr<double>(ii);
				for (int jj = 0; jj < 3; jj++)
				{
					hR(ii, jj) = tempPtr[jj];
				}
				ht.push_back(tempPtr[3]);
			}

			Quaternion<double> q;
			q.ConvertFromMatrix(hR);

			aTransformation.q.w = q.w;
			aTransformation.q.x = q.x;
			aTransformation.q.y = q.y;
			aTransformation.q.z = q.z;

			aTransformation.t.x = ht[0];
			aTransformation.t.y = ht[1];
			aTransformation.t.z = ht[2];

			aTransformation.i = i;
			aTransformation.j = j;

			As[index] = aTransformation;


			//Endoscope (eye)
			Mat mBiInv(4, 4, CV_64FC1);
			Mat mBi(4, 4, CV_64FC1);
			Mat mBj(4, 4, CV_64FC1);

			mBi = eyeData[i].clone();
			mBj = eyeData[j].clone();

			invert(eyeData[i], mBiInv);
			Mat mB = eyeData[j] * mBiInv;

			mist::matrix<double> eR(3, 3);
			vector<double> et;
			for (int ii = 0; ii < 3; ii++)
			{
				double *tempPtr = mB.ptr<double>(ii);
				for (int jj = 0; jj < 3; jj++)
				{
					eR(ii, jj) = tempPtr[jj];
				}
				et.push_back(tempPtr[3]);
			}

			q.ConvertFromMatrix(eR);

			aTransformation.q.w = q.w;
			aTransformation.q.x = q.x;
			aTransformation.q.y = q.y;
			aTransformation.q.z = q.z;

			aTransformation.t.x = et[0];
			aTransformation.t.y = et[1];
			aTransformation.t.z = et[2];

			aTransformation.i = i;
			aTransformation.j = j;

			Bs[index] = aTransformation;

			index++;		
		}//for i
	}//for j
}

/*=======================================================================
*
* 函 数 名： getMovementsPairHtoE
*
* 参　　数：
*
*
* 功能描述:
*            hand to eye 型
*            手的数据A= Aj-1Ai
*            眼的数据B= Bj-1Bi
*            样本数量总共生成 N*（N-1）/2
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/


void HandEye::getMovementsPairHtoE()
{
	int index = 0;
	Transformation<double> aTransformation;
	int motionCount = countSample * (countSample - 1) / 2;// N eye or hand motion 可以形成的运动总数
	As.resize(motionCount);
	Bs.resize(motionCount);

	for (int j = 1; j < countSample; j++)
	{
		for (int i = 0; i < j; i++)
		{
			//NDI (hand)
			Mat mAi(4, 4, CV_64FC1);
			Mat mAj(4, 4, CV_64FC1);
			Mat mAjInv(4, 4, CV_64FC1);

			MistMatrix2CvMat(mAi, handData[i]);
			MistMatrix2CvMat(mAj, handData[j]);

			invert(mAj, mAjInv);

			Mat mA = mAjInv * mAi;

			mist::matrix<double> hR(3, 3);
			vector<double> ht;
			for (int ii = 0; ii < 3; ii++)
			{
				double *tempPtr = mA.ptr<double>(ii);
				for (int jj = 0; jj < 3; jj++)
				{
					hR(ii, jj) = tempPtr[jj];
				}
				ht.push_back(tempPtr[3]);
			}

			Quaternion<double> q;
			q.ConvertFromMatrix(hR);

			aTransformation.q.w = q.w;
			aTransformation.q.x = q.x;
			aTransformation.q.y = q.y;
			aTransformation.q.z = q.z;

			aTransformation.t.x = ht[0];
			aTransformation.t.y = ht[1];
			aTransformation.t.z = ht[2];

			aTransformation.i = i;
			aTransformation.j = j;

			As[index] = aTransformation;


			//Endoscope (eye)
			Mat mBjInv(4, 4, CV_64FC1);
			Mat mBi(4, 4, CV_64FC1);
			Mat mBj(4, 4, CV_64FC1);

			mBi = eyeData[i].clone();
			mBj = eyeData[j].clone();

			invert(eyeData[j], mBjInv);
			Mat mB = mBjInv*eyeData[i] ;

			mist::matrix<double> eR(3, 3);
			vector<double> et;
			for (int ii = 0; ii < 3; ii++)
			{
				double *tempPtr = mB.ptr<double>(ii);
				for (int jj = 0; jj < 3; jj++)
				{
					eR(ii, jj) = tempPtr[jj];
				}
				et.push_back(tempPtr[3]);
			}

			q.ConvertFromMatrix(eR);

			aTransformation.q.w = q.w;
			aTransformation.q.x = q.x;
			aTransformation.q.y = q.y;
			aTransformation.q.z = q.z;

			aTransformation.t.x = et[0];
			aTransformation.t.y = et[1];
			aTransformation.t.z = et[2];

			aTransformation.i = i;
			aTransformation.j = j;

			Bs[index] = aTransformation;

			index++;
		}//for i
	}//for j
}

mist::matrix<double> HandEye::GetAntiSymmetrixOfVector(mist::matrix<double> vec)
{
	mist::matrix<double> mat(3, 3);

	mat(0, 0) = 0;
	mat(0, 1) = -vec(2, 0);
	mat(0, 2) = vec(1, 0);
	mat(1, 0) = vec(2, 0);
	mat(1, 1) = 0;
	mat(1, 2) = -vec(0, 0);
	mat(2, 0) = -vec(1, 0);
	mat(2, 1) = vec(0, 0);
	mat(2, 2) = 0;
	
	return mat;
}


/************************************************
*
*description:（1）鲁棒性算法中
*               to solve the equation Tq=0
*
*
*
*************************************************/
mist::matrix<double> HandEye::SolveRobust(const mist::matrix<double>& matrixL)
{
#pragma region	to solve the equation Tq=0

	//firstly, decompose the matrix T using singular value decomposition
	int m = matrixL.rows(), n = 4;
	mist::matrix<double> U(m, m), D(m, n), V(n, n);
	mist::svd(matrixL, U, D, V);

	V = V.t();

	mist::matrix<double> uu3(4, 1);
	uu3(0, 0) = 0;
	uu3(1, 0) = 0;
	uu3(2, 0) = 0;
	uu3(3, 0) = 1;

	mist::matrix<double> PP(4, 1);
	PP = V * uu3;

	return PP;
}

mist::matrix<double> HandEye::GetXRobust()
{
	mist::matrix<double> matrixL;
	int SampleCount = As.size();
	matrixL.resize(4 * SampleCount, 4);

#pragma region fill the Matrix L
	for (int i = 0; i < SampleCount; i++)
	{

		QuaternionD qA, qB;
		double a0, b0;
		vector3<double> vA, vB;
		mist::matrix<double> vectorA(3, 1), vectorB(3, 1);

		qA = As[i].q;
		vA = qA;

		qB = Bs[i].q;
		vB = qB;

		vectorA[0] = vA.x;
		vectorA[1] = vA.y;
		vectorA[2] = vA.z;

		vectorB[0] = vB.x;
		vectorB[1] = vB.y;
		vectorB[2] = vB.z;

		a0 = qA.w;
		b0 = qB.w;


		//fill the Matrix L with formula(20) in the paper.
		double a0_sub_b0 = a0 - b0;
		mist::matrix<double> a_sub_b = vectorA - vectorB;
		mist::matrix<double> neg_T_a_sub_b = -a_sub_b.t();
		mist::matrix<double> a_add_b = vectorA + vectorB;
		mist::matrix<double> antiSymmetrix = GetAntiSymmetrixOfVector(a_add_b);//get antisymmetric or skew-symmetric mist::matrix 
		mist::matrix<double> I3(3, 3);
		for (int i = 0; i < 3; i++)
			I3(i, i) = 1;
		mist::matrix<double> forth_Item = antiSymmetrix + (a0_sub_b0)*I3;

		matrixL(i * 4, 0) = a0_sub_b0;
		for (int ii = 0; ii < 3; ii++)
		{
			matrixL(i * 4, ii + 1) = neg_T_a_sub_b(0, ii);
		}
		for (int ii = 0; ii < 3; ii++)
		{
			int row = i * 4 + 1 + ii;

			matrixL(row, 0) = a_sub_b(ii, 0);

			for (int jj = 0; jj < 3; jj++)
			{
				matrixL(row, jj + 1) = forth_Item(ii, jj);
			}

		}
	}
#pragma endregion


	return matrixL;
}


/************************************************
*
*name:GetXRobust1
*
*description:（1）利用robust算法计算  X
*             (2)与GetXRobust（）不同之处，GetXRobust1（）中调用了GetXRobust（）
*
*参考论文：//Abed malti and Joao P.Barreto method.
*  From the paper "Robust Hand-Eye Calibration for Computer Aided Medical Endoscopy"
*
***************************************************************************************/
//make the matrix Ldash
mist::matrix<double> HandEye::GetXRobust1()
{
	mist::matrix<double> matrixLDash;
	int SampleCount = As.size();
	matrixLDash.resize(4 * SampleCount, 4);

#pragma region fill the Matrix LDash
	for (int i = 0; i < SampleCount; i++)
	{

		QuaternionD qA, qAp, qB, qBp;
		double a0, b0, a0Dash, b0Dash;
		vector3<double> vA, vAp, vB, vBp;
		mist::matrix<double> vectorA(3, 1), vectorADash(3, 1), vectorB(3, 1), vectorBDash(3, 1);

		qA = As[i].q;
		qAp = As[i].t;
		vA = qA;
		vAp = 0.5 * qAp * qA;

		qB = Bs[i].q;
		qBp = Bs[i].t;
		vB = qB;
		vBp = 0.5 * qBp * qB;

		vectorA[0] = vA.x;
		vectorA[1] = vA.y;
		vectorA[2] = vA.z;

		vectorADash[0] = vAp.x;
		vectorADash[1] = vAp.y;
		vectorADash[2] = vAp.z;

		vectorB[0] = vB.x;
		vectorB[1] = vB.y;
		vectorB[2] = vB.z;

		vectorBDash[0] = vBp.x;
		vectorBDash[1] = vBp.y;
		vectorBDash[2] = vBp.z;

		a0 = qA.w;
		b0 = qB.w;

		a0Dash = qAp.w;
		b0Dash = qBp.w;


		//fill the Matrix L with formula(20) in the paper.
		double a0Dash_sub_b0Dash = a0Dash - b0Dash;
		mist::matrix<double> aDash_sub_bDash = vectorADash - vectorBDash;
		mist::matrix<double> neg_T_aDash_sub_bDash = -aDash_sub_bDash.t();
		mist::matrix<double> aDash_add_bDash = vectorADash + vectorBDash;
		mist::matrix<double> antiSymmetrix = GetAntiSymmetrixOfVector(aDash_add_bDash);//get antisymmetric or skew-symmetric mist::matrix 
		mist::matrix<double> I3(3, 3);
		for (int i = 0; i < 3; i++)
			I3(i, i) = 1;
		mist::matrix<double> forth_Item = antiSymmetrix + (a0Dash_sub_b0Dash)*I3;

		matrixLDash(i * 4, 0) = a0Dash_sub_b0Dash;
		for (int ii = 0; ii < 3; ii++)
		{
			matrixLDash(i * 4, ii + 1) = neg_T_aDash_sub_bDash(0, ii);
		}
		for (int ii = 0; ii < 3; ii++)
		{
			int row = i * 4 + 1 + ii;

			matrixLDash(row, 0) = aDash_sub_bDash(ii, 0);

			for (int jj = 0; jj < 3; jj++)
			{
				matrixLDash(row, jj + 1) = forth_Item(ii, jj);
			}

		}

	}
#pragma endregion

	////////
	mist::matrix<double> matrixL = GetXRobust();
	mist::matrix<double> qq = SolveRobust(matrixL);
	mist::matrix<double> BB = -matrixLDash * qq;
	int m = matrixL.rows(), n = 4;
	mist::matrix<double> UU(m, n), DD(n, n), VV(n, n);
	mist::svd(matrixL, UU, DD, VV);

	VV = VV.t();

	//printMat(mist::inverse(DD));
	mist::matrix<double> qqDash = VV * mist::inverse(DD)*UU.t()*BB;

	Quaternion<double> QQ, QQDash;
	QQ.w = qq(0, 0);
	QQ.x = qq(1, 0);
	QQ.y = qq(2, 0);
	QQ.z = qq(3, 0);

	QQDash.w = qqDash(0, 0);
	QQDash.x = qqDash(1, 0);
	QQDash.y = qqDash(2, 0);
	QQDash.z = qqDash(3, 0);

	vector3<double> tt = 2 * QQDash * QQ.conjugate();
	mist::matrix<double> transformMatrix = QQ.ConvertToMatrix();

	transformMatrix(0, 3) = tt.x;
	transformMatrix(1, 3) = tt.y;
	transformMatrix(2, 3) = tt.z;

	return transformMatrix;
}

/************************************************
*name: matrix<T> GetXofAX_XB
*
*description:利用经典手眼标定算法计算  X
*
*参数：const std::vector<Transformation<T>> & As, 	const std::vector<Transformation<T>>  & Bs,
						vector3<T> * t_for_debug = 0,	QuaternionD * q_for_debug = 0
*
*************************************************/
matrix<double> HandEye::GetXofAX_XB(vector3<double> * t_for_debug = 0, QuaternionD * q_for_debug = 0)
{
	matrix<double> matrixT;
	int SampleCount = As.size();
	matrixT.resize(6 * SampleCount, 8);

#ifdef DEBUG
	matrix<double> matrixT1;
	matrixT1.resize(6 * SampleCount, 8);
#endif


#pragma region fill the Matrix T
	for (int i = 0; i < SampleCount; i++)
	{

		QuaternionD qA, qAp, qB, qBp;
		vector3<double> vA, vAp, vB, vBp;

		matrix<double> vectorA(3, 1), vectorADash(3, 1), vectorB(3, 1), vectorBDash(3, 1);

		qA = As[i].q;
		qAp = As[i].t;
		vA = qA;
		vAp = 0.5 * qAp * qA;

		qB = Bs[i].q;
		qBp = Bs[i].t;
		vB = qB;
		vBp = 0.5 * qBp * qB;



		vectorA[0] = vA.x;
		vectorA[1] = vA.y;
		vectorA[2] = vA.z;

		vectorADash[0] = vAp.x;
		vectorADash[1] = vAp.y;
		vectorADash[2] = vAp.z;


		vectorB[0] = vB.x;
		vectorB[1] = vB.y;
		vectorB[2] = vB.z;

		vectorBDash[0] = vBp.x;
		vectorBDash[1] = vBp.y;
		vectorBDash[2] = vBp.z;

		//fill the Matrix T with formula(31) in the paper.
		matrix<double> a_sub_b = vectorA - vectorB;
		matrix<double> aDash_sub_bDash = vectorADash - vectorBDash;
		matrix<double> a_add_b = vectorA + vectorB;
		matrix<double> aDash_add_bDash = vectorADash + vectorBDash;
		matrix<double> antiSymmetrix = GetAntiSymmetrixOfVector(a_add_b);//get antisymmetric or skew-symmetric matrix 
		matrix<double> antiSymmetrixADash = GetAntiSymmetrixOfVector(aDash_add_bDash);

		for (int ii = 0; ii < 3; ii++)
		{
			int row = i * 6 + ii;
			matrixT(row, 0) = a_sub_b(ii, 0);

			for (int jj = 0; jj < 3; jj++)
			{
				matrixT(row, jj + 1) = antiSymmetrix(ii, jj);
			}
			for (int jj = 0; jj < 4; jj++)
			{
				matrixT(row, jj + 4) = 0;
			}
		}
		for (int ii = 0; ii < 3; ii++)
		{
			int row = i * 6 + 3 + ii;

			matrixT(row, 0) = aDash_sub_bDash(ii, 0);

			for (int jj = 0; jj < 3; jj++)
			{
				matrixT(row, jj + 1) = antiSymmetrixADash(ii, jj);
			}

			matrixT(row, 4) = a_sub_b(ii, 0);

			for (int jj = 0; jj < 3; jj++)
			{
				matrixT(row, jj + 5) = antiSymmetrix(ii, jj);
			}
		}

	}
#pragma endregion


	return SolveTq_0(matrixT, t_for_debug, q_for_debug);
}

/************************************************
*
*description:（1）to solve the equation Tq=0
*            （2）solve the equation(35) and
*                 equation(34)
*
*************************************************/


matrix<double> HandEye::SolveTq_0(const matrix<double> & matrixT, vector3<double> * t_for_debug = 0, QuaternionD * q_for_debug = 0)
{
#pragma region	to solve the equation Tq=0

	//firstly, decompose the matrix T using singular value decomposition
	int m = matrixT.rows(), n = 8;
	matrix<double> U(m, m), D(m, n), V(n, n);
	mist::svd(matrixT, U, D, V);
	
	V = V.t();
	   
	matrix<double> u1(4, 1), v1(4, 1), u2(4, 1), v2(4, 1);

	//load V7,V8 to four vectors.
	for (int i = 0; i < 4; i++)
	{
		u1(i, 0) = V(i, 6);
		u2(i, 0) = V(i, 7);

		v1(i, 0) = V(i + 4, 6);
		v2(i, 0) = V(i + 4, 7);
	}

#pragma region solve the equation(35) and equation(34)

	double s;
	double lambda1, lambda2;
	double
		coefficientA = (u1.t()*v1)(0, 0),
		coefficientB = (u1.t()*v2 + u2.t()*v1)(0, 0),
		coefficientC = (u2.t()*v2)(0, 0);


	double discriminant = coefficientB * coefficientB - 4 * coefficientA * coefficientC;


	//if discriminant is less than 0, it denote that there is no any real resolution about it.
	if (discriminant < 0)
		return matrix<double>(0, 0);
	else //discriminant is greater than 0, it denote that there are two solutions about it.
	{
		double s1, s2;
		s1 = (-coefficientB + sqrt(discriminant)) / (2 * coefficientA);
		s2 = (-coefficientB - sqrt(discriminant)) / (2 * coefficientA);

		//double s1_discriminant = (s1 * s1 * u1.t() * u1 + 2 * s1 * u1.t() * u2 + u2.t() * u2)(0,0);
		//double s2_discriminant = (s2 * s2 * u1.t() * u1 + 2 * s2 * u1.t() * u2 + u2.t() * u2)(0,0);
		double s1_discriminant = (s1 * s1 * u1.t() * u1 + s1 * (u1.t() * u2 + u2.t()*u1) + u2.t() * u2)(0, 0);
		double s2_discriminant = (s2 * s2 * u1.t() * u1 + s2 * (u1.t() * u2 + u2.t()*u1) + u2.t() * u2)(0, 0);

		if (s1_discriminant > s2_discriminant)
		{
			s = s1;


			lambda2 = sqrt(1 / s1_discriminant);

		}
		else
		{
			s = s2;

			lambda2 = sqrt(1 / s2_discriminant);
		}


		lambda1 = s * lambda2;
	}
#pragma endregion solve the equation(35)

	// to calculate q in here with using lambda1,lambda2,V6 and V7
	Quaternion<double> q, qDash;
	q.w = lambda1 * V(0, 6) + lambda2 * V(0, 7);
	q.x = lambda1 * V(1, 6) + lambda2 * V(1, 7);
	q.y = lambda1 * V(2, 6) + lambda2 * V(2, 7);
	q.z = lambda1 * V(3, 6) + lambda2 * V(3, 7);

	qDash.w = lambda1 * V(4, 6) + lambda2 * V(4, 7);
	qDash.x = lambda1 * V(5, 6) + lambda2 * V(5, 7);
	qDash.y = lambda1 * V(6, 6) + lambda2 * V(6, 7);
	qDash.z = lambda1 * V(7, 6) + lambda2 * V(7, 7);

#pragma endregion	

	vector3<double> t = 2 * qDash * q.conjugate();

	if (t_for_debug) * t_for_debug = t;
	if (q_for_debug) * q_for_debug = q;


	matrix<double> transformMatrix = q.ConvertToMatrix();

	transformMatrix(0, 3) = t.x;
	transformMatrix(1, 3) = t.y;
	transformMatrix(2, 3) = t.z;

	return transformMatrix;
}

/*=======================================================================
*
* 函 数 名： runHandEyeCalibration
*
* 参　　数：
*
*
* 功能描述:
*			1、获得手和眼的数据
*           2、计算A和B
*           3、计算X
*           4、测试结果AX-XB
*           5、
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
bool HandEye::runHandEyeCalibration(vector<QuatTransformation>  &transformationData, vector<int> &noCornerImageID, vector<Mat> &rvecs, vector<Mat> &tvecs)
{
	//获得手与眼的数据
	if (!getEyeData(rvecs, tvecs))
	{
		cout << "获得眼的数据失败!\n";
		return false;
	}
	getHandData(transformationData, noCornerImageID);

	//判断数量样本数量是否相等
	if ((handData.size() != eyeData.size()) && (handData.size() != countSample))
	{
		cout << "获得手和眼的样本数量不相等!\n";
		return false;
	}

	//手与眼的数据生成A与B AX=XB
	getMovementsPair();

	//计算X
	X = GetXRobust1();

	testResult();

	OutputMatrixToFile("./Data\\resultX\\X.txt", X);

	//误差矩阵法计算手眼标定误差
	errorMatrix();

	calculateError();

	return true;
}

bool HandEye::runHandEyeCalibration()
{
	countSample = en.countConerImage;

	//获得手与眼的数据
	if (!getEyeData())
	{
		cout << "获得眼的数据失败!\n";
		return false;
	}
	getHandData();

	//判断数量样本数量是否相等
	if ((handData.size() != eyeData.size()) || (handData.size() != countSample))//修改
	{
		cout << "获得手和眼的样本数量不相等!\n";
		return false;
	}

	//手与眼的数据生成A与B AX=XB
	getMovementsPair();

	//计算X
	X = GetXRobust1();
	//X = GetXofAX_XB();

	testResult();

	OutputMatrixToFile("./Data\\resultX\\X.txt", X);

	//误差矩阵法计算手眼标定误差
	errorMatrix();

	calculateError();

	return true;
}

/*=======================================================================
*
* 函 数 名： runHandEyeCalibrationHtoE
*
* 参　　数：
*
*
* 功能描述:
*			1、获得手和眼的数据
*           2、计算A和B
*           3、计算X
*           4、测试结果AX-XB
*           5、hand to eye 类型
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
bool HandEye::runHandEyeCalibrationHtoE()
{
	countSample = en.countConerImage;

	//获得手与眼的数据
	if (!getEyeData())
	{
		cout << "获得眼的数据失败!\n";
		return false;
	}
	getHandData();

	//判断数量样本数量是否相等
	if ((handData.size() != eyeData.size()) && (handData.size() != countSample))
	{
		cout << "获得手和眼的样本数量不相等!\n";
		return false;
	}

	//手与眼的数据生成A与B AX=XB
	getMovementsPairHtoE();

	//计算X
	X = GetXRobust1();

	testResult();

	OutputMatrixToFile("./Data\\resultX\\X.txt", X);

	//误差矩阵法计算手眼标定误差
	//errorMatrix();

	return true;
}

/*=======================================================================
*
* 函 数 名： testResult
*
* 参　　数：
*
*
* 功能描述:
*
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
void HandEye::testResult()
{
	//test if AX=XB or not
	mist::matrix<double> tempAs;
	mist::matrix<double>tempBs;

	FILE * fp;
	fp = fopen("./Data\\calibrationError\\testdata.txt", "w");
	fprintf(fp, "AsSize: %d\t BsSize: %d\n", As.size(), Bs.size());

	//用于计算AX-XB=M平移误差的均值
	double errorTofM = 0.0;

	for (int i = 0; i < As.size(); i++)
	{
		tempAs = As[i].q.ConvertToMatrix();
		tempBs = Bs[i].q.ConvertToMatrix();

		tempAs(0, 3) = As[i].t.x;
		tempAs(1, 3) = As[i].t.y;
		tempAs(2, 3) = As[i].t.z;
		tempBs(0, 3) = Bs[i].t.x;
		tempBs(1, 3) = Bs[i].t.y;
		tempBs(2, 3) = Bs[i].t.z;

		mist::matrix<double> temp1;
		mist::matrix<double> temp2;
		mist::matrix<double> error_result;
		temp1 = tempAs * X;


		temp2 = X * tempBs;

		error_result = temp1 - temp2;

		//*计算齐次矩阵平移误差
		double tempSumT = fabs(error_result(0, 3)) + fabs(error_result(1, 3)) + fabs(error_result(2, 3)) ;
		errorTofM += tempSumT /3 ;		
		//*/

		fprintf(fp, "\n误差\n");
		for (int i = 0; i < error_result.rows(); i++)
		{
			for (int j = 0; j < tempAs.cols(); j++)
			{
				fprintf(fp, "%lf ", tempAs(i, j));
			}
			fprintf(fp, "\t\t");

			for (int j = 0; j < tempBs.cols(); j++)
			{
				fprintf(fp, "%lf ", tempBs(i, j));
			}
			fprintf(fp, "\t\t");

			for (int j = 0; j < error_result.cols(); j++)
			{
				fprintf(fp, "%lf ", error_result(i, j));
			}
			fprintf(fp, "\n");
		}
		fprintf(fp, "\n");
	}


	//计算平移误差均值
	Et = errorTofM / As.size();
	
	fclose(fp);
	//return error_result;
}

bool HandEye::runEndoscopeAndNDI()
{
	//en.CaptureData();//多线程 按键
	//en.OnNDIAndEndoscopeTrackerKey();//单线程 按键
	en.OnNDIAndEndoscopeTracker();//单线程 时间
	if (en.runAndSave())
	{
		return true;
	}
	return false;
}


/*=======================================================================
*
* 函 数 名： errorMatrix
*
* 参　　数：
*
*
* 功能描述:
*
*          根据误差矩阵法计算旋转和平移误差
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
bool HandEye::errorMatrix()
{
	vector<double> vER(As.size());
	vector<double> vEt(As.size());

	//分离出X的R和t
	Mat Rx(3, 3, CV_64FC1);
	Mat tx(3, 1, CV_64FC1);

	for (int i = 0; i < 3; i++)
	{
		double *rPtr = Rx.ptr<double>(i);
		for (int j = 0; j < 3; j++)
		{
			rPtr[j] = X(i, j);
		}
	}
	double *tPtr = tx.ptr<double>(0);
	tPtr[0] = X(0, 3);
	tPtr[1] = X(1, 3);
	tPtr[2] = X(2, 3);


	for (int k = 0; k < As.size(); k++)
	{
		//分离出A和B的R和t--------------------------------------------------------------------
		mist::matrix<double> Ai = As[k].q.ConvertToMatrix();
		mist::matrix<double> Bi = Bs[k].q.ConvertToMatrix();

		Mat Ra(3, 3, CV_64FC1);
		Mat ta(3, 1, CV_64FC1);
		Mat Rb(3, 3, CV_64FC1);
		Mat tb(3, 1, CV_64FC1);

		for (int i = 0; i < 3; i++)
		{
			double *rAPtr = Ra.ptr<double>(i);
			double *rBPtr = Rb.ptr<double>(i);
			for (int j = 0; j < 3; j++)
			{
				rAPtr[j] = Ai(i, j);
				rBPtr[j] = Bi(i, j);
			}
		}
		double *tAPtr = ta.ptr<double>(0);
		double *tBPtr = tb.ptr<double>(0);
		tAPtr[0] = As[k].t.x;
		tAPtr[1] = As[k].t.y;
		tAPtr[2] = As[k].t.z;
		tBPtr[0] = Bs[k].t.x;
		tBPtr[1] = Bs[k].t.y;
		tBPtr[2] = Bs[k].t.z;
		//end-----------------------------------------------------------------------------------

		//旋转误差-------------------------------------------------------------------------------
		Mat Rxb = Rx * Rb;
		Mat Rax = Ra * Rx;
		Mat Rax_inv(3, 3, CV_64FC1);
		invert(Rax, Rax_inv);

		Mat rM = Rxb * Rax_inv;
		Mat rV(3, 1, CV_64FC1);
		Rodrigues(rM, rV);

		double dER = norm(rV, NORM_L2);
		vER[k] = dER;
		//end-------------------------------------------------------------------------------------

		//平移误差--------------------------------------------------------------------------------
		Mat I = Mat::eye(3, 3, CV_64FC1);
		Mat temp_a = Ra - I;
		Mat Ratx = temp_a * tx;

		Mat Rxtb = Rx * tb;
		Mat temp_b = ta - Rxtb;
		Mat temp_ab = Ratx + temp_b;

		double molecule = norm(temp_ab, NORM_L2);
		double denominator = norm(temp_b, NORM_L2);
		double dEt = molecule / denominator;
		vEt[k] = dEt;

		//end-------------------------------------------------------------------------------------
	}

	if (calculateMeanValue(vER, ER) && calculateMeanValue(vEt, Et))
	{
		FILE *ferr;
		ferr = fopen("./Data\\calibrationError\\error.txt", "w");

		fprintf(ferr, "旋转：%1f\n平移：%1f", ER, Et);

		fclose(ferr);

		//gengxin2022-3-2------------------------------------

		if (verifyCalibrationOfTriangular())
		{
			cout << "error reported by calibrate: " << Et << endl;
		}
		else
		{
			cout << "error reported by calibrate: " << Et+1 << endl;
		}
		//end--------------------------------------
		return true;
	}

	return false;
}


/*=======================================================================
*
* 函 数 名： calculateMeanValue
*
* 参　　数：
*
*
* 功能描述:
*
*            计算vector中数据的均值mv
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
bool HandEye::calculateMeanValue(vector<double> a, double & mv)
{
	if (a.size() > 0)
	{
		int vsize = a.size();
		double temp = 0.0;

		for (int i = 0; i < vsize; i++)
		{
			temp += a[i];
		}

		mv = temp / vsize;

		return true;
	}

	return false;
}

/*=======================================================================
*
* 函 数 名： OutputMatrixToFile
*
* 参　　数：
*
*
* 功能描述:
*
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
void HandEye::OutputMatrixToFile(const char *filename, mist::matrix<double> mat)
{

	FILE * fp;
	fp = fopen(filename,"w");

	for (int i = 0; i < mat.rows(); i++)
	{
		for (int j = 0; j < mat.cols(); j++)
		{
			fprintf(fp, "%lf ", mat(i, j));
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
}

/*=======================================================================
*
* 函 数 名： calculateError
*
* 参　　数：
*
*
* 功能描述:
*           通过在标定板上安装刚体，计算内窥镜标定误差
*           X1表示标定板到刚体的变换
*           
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-8
*
=========================================================================*/
void HandEye::calculateError()
{
	if (en.transformationData.size() != en.boardRigidData.size())
	{
		return;
	}
	
	Mat X1 = (Mat_<double>(4, 4) << -0.014022, - 0.825371, 0.564417, 0.053688,
		0.956838 ,0.152798, 0.247214, -40.259365,
		-0.290285, 0.543521, 0.787604 ,-52.663955,
		0.000000, 0.000000, 0.000000, 1.000000);

	Mat X1inv(4, 4, CV_64FC1);
	invert(X1, X1inv);

	//填充boardRigid数据----------------------------------------------------------------------------------------------------------------------------------------------------------
	vector<Mat>    boardRigid;

	//有未提取角点的图像
	if (en.noCornerImageID.size() != 0)
	{

		int flagTemp = 0;                          //noCornerImageID中的序号
		int itemp = en.noCornerImageID[flagTemp];     //未能提取角点的图像的ID，从1开始

		//获得标定板刚体的数据
		for (int i = 0; i < (countSample + en.noCornerImageID.size()); i++)
		{
			//剔除未提取角点的图像对应的手的数据
			if ((i + 1) == itemp)
			{
				flagTemp++;
				//防止越界
				if (flagTemp < en.noCornerImageID.size())
				{
					itemp = en.noCornerImageID[flagTemp];
				}
				continue;
			}
			else
			{
				Quaternion<double> q(en.boardRigidData[i].rotation.q0, en.boardRigidData[i].rotation.qx, en.boardRigidData[i].rotation.qy, en.boardRigidData[i].rotation.qz);
				mist::matrix<double> m_qtomatrix = q.ConvertToMatrix();
				m_qtomatrix(0, 3) = en.transformationData[i].translation.x;
				m_qtomatrix(1, 3) = en.transformationData[i].translation.y;
				m_qtomatrix(2, 3) = en.transformationData[i].translation.z;

				Mat cvMatrix(4, 4, CV_64FC1);
				MistMatrix2CvMat(cvMatrix, m_qtomatrix);

				boardRigid.push_back(cvMatrix);
			}
		}
	}
	else
	{//所有图像角点提取成功

		//获得手的数据
		for (int i = 0; i < countSample; i++)
		{
			Quaternion<double> q(en.boardRigidData[i].rotation.q0, en.boardRigidData[i].rotation.qx, en.boardRigidData[i].rotation.qy, en.boardRigidData[i].rotation.qz);
			mist::matrix<double> m_qtomatrix = q.ConvertToMatrix();
			m_qtomatrix(0, 3) = en.transformationData[i].translation.x;
			m_qtomatrix(1, 3) = en.transformationData[i].translation.y;
			m_qtomatrix(2, 3) = en.transformationData[i].translation.z;

			Mat cvMatrix(4, 4, CV_64FC1);
			MistMatrix2CvMat(cvMatrix, m_qtomatrix);

			boardRigid.push_back(cvMatrix);
		}
	}
	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


	Mat cvX(4, 4, CV_64FC1);
	MistMatrix2CvMat(cvX, X);

	vector<double> vER(0);
	vector<double> vEt(0);

	//计算C-1AXBX1-1 = I--------------------------------------------------------------------------------------------------------------------------------------------------------------
	if (handData.size() != boardRigid.size())
	{
		cout << "样本数量不一致！/n";
		return;
	}//修改2021-7-3
	else
	{
		for (int i = countSample - 1; i < handData.size(); i++)
		{
			Mat temp(4, 4, CV_64FC1);

			Mat cvhand(4, 4, CV_64FC1);
			MistMatrix2CvMat(cvhand, handData[i]);

			Mat boardInv(4, 4, CV_64FC1);
			invert(boardRigid[i], boardInv);

			temp = boardInv * cvhand * cvX * eyeData[i];
			temp = X1inv * boardInv * cvhand * cvX * eyeData[i];

			//测试计算X---------------------------------------------------------------------------
			
			cout << cvX << "\n";
			Mat cvhandInv(4, 4, CV_64FC1);
			invert(cvhand, cvhandInv);
			Mat cveyeInv(4, 4, CV_64FC1);
			invert(eyeData[i], cveyeInv);
			temp = cvhandInv * boardRigid[i] * X1 * cveyeInv;


			//------------------------------------------------------------------------------------

			//分离出temp的R和t--------------------------------------------------------------------
			Mat Rtemp(3, 3, CV_64FC1);
			Mat ttemp(3, 1, CV_64FC1);

			Rtemp = temp(Rect(0, 0, 3, 3));
			ttemp = temp(Rect(3, 0, 1, 3));
			//	Mat t = m(Rect(3, 0, 1, 3));
			////Mat t = m(Range(0, 3), Range(3, 4));
			//Mat r = m(Range(0, 3), Range(0, 3));

			//end-----------------------------------------------------------------------------------

			//旋转误差-------------------------------------------------------------------------------	
			//Mat rV(3, 1, CV_64FC1);
			//Rodrigues(Rtemp, rV);
			matrix<double> mR(3,3);
			CvMat2MistMatrix(Rtemp, mR);
			Quaternion<double> q;
			q.ConvertFromMatrix(mR);
			double rx, ry, rz;
			q.ConvertToEuler(rx, ry,rz);
			double dER = sqrt(rx * rx + ry * ry + rz * rz) / 3;
			vER.push_back(dER);
			//end-------------------------------------------------------------------------------------

			//平移误差--------------------------------------------------------------------------------		
			double dEt = norm(ttemp, NORM_L2);
			vEt.push_back(dEt);

			//end-------------------------------------------------------------------------------------

		}
	}
	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	if (calculateMeanValue(vER, ER) && calculateMeanValue(vEt, Et))
	{
		FILE *ferr;
		ferr = fopen("./Data\\calibrationError\\error1.txt", "w");

		fprintf(ferr, "旋转：%1f\n平移：%1f", ER, Et);

		fclose(ferr);
	}
}

/*=======================================================
time:2022-3-2
验证标定板与刚体组成的三角块的标定矩阵，true表示验证成功，
符合设定规则，false表示验证失败，不符合验证规则
======================================================*/
/******************************************************/

bool HandEye::verifyCalibrationOfTriangular()
{
	//read Calibration Of Triangular----------------
	FILE * fx;

	if ((fx = fopen(".\\Data\\read data\\X.txt", "r")) == NULL)
	{
		printf("cant find the file .\\Data\\read data\\X.txt!");
		return false;
	}

	cv::Mat cv_calOfTri(4, 4, CV_64FC1);

	double value = 0;

	for (int i = 0; i < 4; i++)
	{
		double *temp = cv_calOfTri.ptr<double>(i);
		for (int j = 0; j < 4; j++)
		{
			std::fscanf(fx, "%lf", &value);
			temp[j] = value;
		}
	}	
	fclose(fx);
	//end-------------------------------------------------

	//verify rule----------------------------------------

	double value1 = cv_calOfTri.at<double>(0,0);
	double value5 = cv_calOfTri.at<double>(1,1);
	double value3 = cv_calOfTri.at<double>(0,2);
	double value9 = cv_calOfTri.at<double>(2,2);
	double value10 = cv_calOfTri.at<double>(0,3);
	double value11 = cv_calOfTri.at<double>(1,3);
	double value12 = cv_calOfTri.at<double>(2,3);

	if (((value1 + value5) > 0.79) && ((value1 + value5) < 0.81))
	{
		if (((value3 + value9) > 0.89) && ((value3 + value9) < 0.91))
		{
			if (((value10 + value11 + value12) > 9.9) && ((value10 + value11 + value12) < 10.1))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	//end-----------------------------------------------
}


HandEye::HandEye(int countSample)
{
	this->countSample = countSample;
}

//HandEye::HandEye()
//{
//	Size imageSize(1280, 720);
//	Size boardSize;
//	boardSize.width = 11;
//	boardSize.height = 8;
//	float squareSize = 5;
//
//	en.setImageSize(imageSize);
//	en.setBoardSize(boardSize);
//	en.setSquareSize(squareSize);
//	en.setGridWidth();
//}

HandEye::HandEye(Size imageSize)//添加2021-6-29
{
	Size boardSize;
	boardSize.width = 11;
	boardSize.height = 8;
	float squareSize = 5;

	en.setImageSize(imageSize);
	en.setBoardSize(boardSize);
	en.setSquareSize(squareSize);
	en.setGridWidth();
}




/*
* 当libjpeg-turbo为vs2010编译时，vs2015下静态链接libjpeg-turbo会链接出错:找不到__iob_func,
* 增加__iob_func到__acrt_iob_func的转换函数解决此问题,
* 当libjpeg-turbo用vs2015编译时，不需要此补丁文件
*/
#if _MSC_VER>=1900
#include "stdio.h"
_ACRTIMP_ALT FILE* __cdecl __acrt_iob_func(unsigned);
#ifdef __cplusplus
extern "C"
#endif
FILE* __cdecl __iob_func(unsigned i) {
	return __acrt_iob_func(i);
}
#endif /* _MSC_VER>=1900 */