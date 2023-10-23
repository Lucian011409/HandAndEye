#include "HandEyeCalibration.h"



/*=======================================================================
*
* �� �� ���� getHandData
*
* �Ρ�������
*
*
* ��������:
*			����ֵ����ݣ�������ΪNDI�Ŀռ�任���ݣ���EndoscopeAndNDI��
*           ��transformationData��ã���ȡ�������޳��ڿ���ͼ����δ��ý�
*           ���ͼ���Ӧ��NDI�ռ�任���ݣ�δ����ȡ�ǵ��ͼ�����ű�����
*           noCornerImageID��
*           �ֵ����ݱ����� handData ��
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
*
=========================================================================*/
bool HandEye::getHandData(vector<QuatTransformation>  &transformationData, vector<int> &noCornerImageID)
{
	handData.resize(0);

	//��δ��ȡ�ǵ��ͼ��
	if (noCornerImageID.size() != 0)               //noCornerImageID��δ����ȡ�ǵ��ͼ�����ţ���1��ʼ
	{

		int flagTemp = 0;                          //noCornerImageID�е����
		int itemp = noCornerImageID[flagTemp];     //δ����ȡ�ǵ��ͼ���ID����1��ʼ

		//����ֵ�����
		for (int i = 0; i < (countSample + noCornerImageID.size()); i++)  //countSample�֡����������ݵ�����
		{
			//�޳�δ��ȡ�ǵ��ͼ���Ӧ���ֵ�����
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
				Quaternion<double> q(transformationData[i].rotation.q0, transformationData[i].rotation.qx, transformationData[i].rotation.qy, transformationData[i].rotation.qz);
				mist::matrix<double> m_qtomatrix = q.ConvertToMatrix();
				m_qtomatrix(0, 3) = transformationData[i].translation.x;
				m_qtomatrix(1, 3) = transformationData[i].translation.y;
				m_qtomatrix(2, 3) = transformationData[i].translation.z;
				//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAConvertToMatrix()������Ԫ��ת��Ϊ 4x4 ����AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
				handData.push_back(m_qtomatrix);
			}
		}
	}
	else
	{//����ͼ��ǵ���ȡ�ɹ�

		//����ֵ�����
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

	//��δ��ȡ�ǵ��ͼ��
	if (en.noCornerImageID.size() != 0)
	{

		int flagTemp = 0;                          //noCornerImageID�е����
		int itemp = en.noCornerImageID[flagTemp];     //δ����ȡ�ǵ��ͼ���ID����1��ʼ

		//����ֵ�����
		for (int i = 0; i < (countSample + en.noCornerImageID.size()); i++)
		{
			//�޳�δ��ȡ�ǵ��ͼ���Ӧ���ֵ�����
			if ((i + 1) == itemp)
			{
				flagTemp++;
				//��ֹԽ��
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
	{//����ͼ��ǵ���ȡ�ɹ�

		//����ֵ�����
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
* �� �� ���� getEyeData
*
* �Ρ�������
*
*
* ��������:
*			����۵����ݣ�������Ϊ�궨�ڿ�����õ����������EndoscopeAndNDI��
*           ��rvecs �� tvecs��ã�
*           �۵����ݱ����� eyeData �� 
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
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

			//��ת�����ƽ������ת��Ϊ 4*4��ξ���
			Mat t = m(Rect(3, 0, 1, 3));
			//Mat t = m(Range(0, 3), Range(3, 4));
			Mat r = m(Range(0, 3), Range(0, 3));

			//��ת����ת��Ϊ��ת����
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

			//��ת�����ƽ������ת��Ϊ 4*4��ξ���
			Mat t = m(Rect(3, 0, 1, 3));
			//Mat t = m(Range(0, 3), Range(3, 4));
			Mat r = m(Range(0, 3), Range(0, 3));

			//��ת����ת��Ϊ��ת����
			Rodrigues(en.rvecs[k], r);

			CV_Assert(en.tvecs[k].rows == 3 && en.tvecs[k].cols == 1);
			//t = tvecs[k].clone();
			en.tvecs[k].copyTo(t);

			eyeData.push_back(m);
		}//for

		//�����������뵽�ļ�
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
* �� �� ���� CvMat2MistMatrix
*
* �Ρ�������
*
*
* ��������:
*			
*            opencv��Matת��ΪMist��matrix
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
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
* �� �� ���� MistMatrix2CvMat
*
* �Ρ�������
*
*
* ��������:
*			
*            Mist��matrix��ת��Ϊopencv��Mat
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
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
* �� �� ���� getMovementsPair
*
* �Ρ�������
*
*
* ��������:
*			
*            �ֵ�����A= Aj-1Ai
*            �۵�����B= BjBi-1
*            ���������ܹ����� N*��N-1��/2
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
*
=========================================================================*/


void HandEye::getMovementsPair()
{
	int index = 0;
	Transformation<double> aTransformation;

	//int motionCount = countSample * (countSample - 1) / 2;// N eye or hand motion �����γɵ��˶�����
	int motionCount = (countSample - 1) * (countSample - 2) / 2;//Ϊ�˼���ʱ���һ�����ڼ���

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
* �� �� ���� getMovementsPairHtoE
*
* �Ρ�������
*
*
* ��������:
*            hand to eye ��
*            �ֵ�����A= Aj-1Ai
*            �۵�����B= Bj-1Bi
*            ���������ܹ����� N*��N-1��/2
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
*
=========================================================================*/


void HandEye::getMovementsPairHtoE()
{
	int index = 0;
	Transformation<double> aTransformation;
	int motionCount = countSample * (countSample - 1) / 2;// N eye or hand motion �����γɵ��˶�����
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
*description:��1��³�����㷨��
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
*description:��1������robust�㷨����  X
*             (2)��GetXRobust������֮ͬ����GetXRobust1�����е�����GetXRobust����
*
*�ο����ģ�//Abed malti and Joao P.Barreto method.
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
*description:���þ������۱궨�㷨����  X
*
*������const std::vector<Transformation<T>> & As, 	const std::vector<Transformation<T>>  & Bs,
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
*description:��1��to solve the equation Tq=0
*            ��2��solve the equation(35) and
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
* �� �� ���� runHandEyeCalibration
*
* �Ρ�������
*
*
* ��������:
*			1������ֺ��۵�����
*           2������A��B
*           3������X
*           4�����Խ��AX-XB
*           5��
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
*
=========================================================================*/
bool HandEye::runHandEyeCalibration(vector<QuatTransformation>  &transformationData, vector<int> &noCornerImageID, vector<Mat> &rvecs, vector<Mat> &tvecs)
{
	//��������۵�����
	if (!getEyeData(rvecs, tvecs))
	{
		cout << "����۵�����ʧ��!\n";
		return false;
	}
	getHandData(transformationData, noCornerImageID);

	//�ж��������������Ƿ����
	if ((handData.size() != eyeData.size()) && (handData.size() != countSample))
	{
		cout << "����ֺ��۵��������������!\n";
		return false;
	}

	//�����۵���������A��B AX=XB
	getMovementsPair();

	//����X
	X = GetXRobust1();

	testResult();

	OutputMatrixToFile("./Data\\resultX\\X.txt", X);

	//�����󷨼������۱궨���
	errorMatrix();

	calculateError();

	return true;
}

bool HandEye::runHandEyeCalibration()
{
	countSample = en.countConerImage;

	//��������۵�����
	if (!getEyeData())
	{
		cout << "����۵�����ʧ��!\n";
		return false;
	}
	getHandData();

	//�ж��������������Ƿ����
	if ((handData.size() != eyeData.size()) || (handData.size() != countSample))//�޸�
	{
		cout << "����ֺ��۵��������������!\n";
		return false;
	}

	//�����۵���������A��B AX=XB
	getMovementsPair();

	//����X
	X = GetXRobust1();
	//X = GetXofAX_XB();

	testResult();

	OutputMatrixToFile("./Data\\resultX\\X.txt", X);

	//�����󷨼������۱궨���
	errorMatrix();

	calculateError();

	return true;
}

/*=======================================================================
*
* �� �� ���� runHandEyeCalibrationHtoE
*
* �Ρ�������
*
*
* ��������:
*			1������ֺ��۵�����
*           2������A��B
*           3������X
*           4�����Խ��AX-XB
*           5��hand to eye ����
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
*
=========================================================================*/
bool HandEye::runHandEyeCalibrationHtoE()
{
	countSample = en.countConerImage;

	//��������۵�����
	if (!getEyeData())
	{
		cout << "����۵�����ʧ��!\n";
		return false;
	}
	getHandData();

	//�ж��������������Ƿ����
	if ((handData.size() != eyeData.size()) && (handData.size() != countSample))
	{
		cout << "����ֺ��۵��������������!\n";
		return false;
	}

	//�����۵���������A��B AX=XB
	getMovementsPairHtoE();

	//����X
	X = GetXRobust1();

	testResult();

	OutputMatrixToFile("./Data\\resultX\\X.txt", X);

	//�����󷨼������۱궨���
	//errorMatrix();

	return true;
}

/*=======================================================================
*
* �� �� ���� testResult
*
* �Ρ�������
*
*
* ��������:
*
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
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

	//���ڼ���AX-XB=Mƽ�����ľ�ֵ
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

		//*������ξ���ƽ�����
		double tempSumT = fabs(error_result(0, 3)) + fabs(error_result(1, 3)) + fabs(error_result(2, 3)) ;
		errorTofM += tempSumT /3 ;		
		//*/

		fprintf(fp, "\n���\n");
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


	//����ƽ������ֵ
	Et = errorTofM / As.size();
	
	fclose(fp);
	//return error_result;
}

bool HandEye::runEndoscopeAndNDI()
{
	//en.CaptureData();//���߳� ����
	//en.OnNDIAndEndoscopeTrackerKey();//���߳� ����
	en.OnNDIAndEndoscopeTracker();//���߳� ʱ��
	if (en.runAndSave())
	{
		return true;
	}
	return false;
}


/*=======================================================================
*
* �� �� ���� errorMatrix
*
* �Ρ�������
*
*
* ��������:
*
*          ���������󷨼�����ת��ƽ�����
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
*
=========================================================================*/
bool HandEye::errorMatrix()
{
	vector<double> vER(As.size());
	vector<double> vEt(As.size());

	//�����X��R��t
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
		//�����A��B��R��t--------------------------------------------------------------------
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

		//��ת���-------------------------------------------------------------------------------
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

		//ƽ�����--------------------------------------------------------------------------------
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

		fprintf(ferr, "��ת��%1f\nƽ�ƣ�%1f", ER, Et);

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
* �� �� ���� calculateMeanValue
*
* �Ρ�������
*
*
* ��������:
*
*            ����vector�����ݵľ�ֵmv
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
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
* �� �� ���� OutputMatrixToFile
*
* �Ρ�������
*
*
* ��������:
*
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
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
* �� �� ���� calculateError
*
* �Ρ�������
*
*
* ��������:
*           ͨ���ڱ궨���ϰ�װ���壬�����ڿ����궨���
*           X1��ʾ�궨�嵽����ı任
*           
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-8
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

	//���boardRigid����----------------------------------------------------------------------------------------------------------------------------------------------------------
	vector<Mat>    boardRigid;

	//��δ��ȡ�ǵ��ͼ��
	if (en.noCornerImageID.size() != 0)
	{

		int flagTemp = 0;                          //noCornerImageID�е����
		int itemp = en.noCornerImageID[flagTemp];     //δ����ȡ�ǵ��ͼ���ID����1��ʼ

		//��ñ궨����������
		for (int i = 0; i < (countSample + en.noCornerImageID.size()); i++)
		{
			//�޳�δ��ȡ�ǵ��ͼ���Ӧ���ֵ�����
			if ((i + 1) == itemp)
			{
				flagTemp++;
				//��ֹԽ��
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
	{//����ͼ��ǵ���ȡ�ɹ�

		//����ֵ�����
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

	//����C-1AXBX1-1 = I--------------------------------------------------------------------------------------------------------------------------------------------------------------
	if (handData.size() != boardRigid.size())
	{
		cout << "����������һ�£�/n";
		return;
	}//�޸�2021-7-3
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

			//���Լ���X---------------------------------------------------------------------------
			
			cout << cvX << "\n";
			Mat cvhandInv(4, 4, CV_64FC1);
			invert(cvhand, cvhandInv);
			Mat cveyeInv(4, 4, CV_64FC1);
			invert(eyeData[i], cveyeInv);
			temp = cvhandInv * boardRigid[i] * X1 * cveyeInv;


			//------------------------------------------------------------------------------------

			//�����temp��R��t--------------------------------------------------------------------
			Mat Rtemp(3, 3, CV_64FC1);
			Mat ttemp(3, 1, CV_64FC1);

			Rtemp = temp(Rect(0, 0, 3, 3));
			ttemp = temp(Rect(3, 0, 1, 3));
			//	Mat t = m(Rect(3, 0, 1, 3));
			////Mat t = m(Range(0, 3), Range(3, 4));
			//Mat r = m(Range(0, 3), Range(0, 3));

			//end-----------------------------------------------------------------------------------

			//��ת���-------------------------------------------------------------------------------	
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

			//ƽ�����--------------------------------------------------------------------------------		
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

		fprintf(ferr, "��ת��%1f\nƽ�ƣ�%1f", ER, Et);

		fclose(ferr);
	}
}

/*=======================================================
time:2022-3-2
��֤�궨���������ɵ����ǿ�ı궨����true��ʾ��֤�ɹ���
�����趨����false��ʾ��֤ʧ�ܣ���������֤����
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

HandEye::HandEye(Size imageSize)//���2021-6-29
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
* ��libjpeg-turboΪvs2010����ʱ��vs2015�¾�̬����libjpeg-turbo�����ӳ���:�Ҳ���__iob_func,
* ����__iob_func��__acrt_iob_func��ת���������������,
* ��libjpeg-turbo��vs2015����ʱ������Ҫ�˲����ļ�
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