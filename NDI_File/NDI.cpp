// NDI.cpp: implementation of the NDI class
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#include "NDI.h"
#include "IniFileRW.h"
#include "Conversions.h"
#include <sstream>
#include <conio.h>


/************************************************************************
*
*�� �ļ����� NDI.cpp
*
*�� �ļ�������	��ѧ��������ؿ��� ���á��������رա���ȡ���ݵȲ���
*
*�� �����ˣ�κ����
*
*�� �汾�ţ�1.0.0
*
*�� �޸ļ�¼��
*
************************************************************************/


// ȫ�ֺ���
//////////////////////////////////////////////////////////////////////////

/*================================================================
*
* �� �� ����NDI
*
* �Ρ�������
*		
*
*
* ��������:
*
*�������� ���캯��
*
* �� �� ֵ��
*
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-06
*
================================================================*/
NDI::NDI()
{
	pCommandHandling = new CCommandHandling;
	//m_nTrakingState1 = 0;
	//m_nTrakingState2 = 0;
	m_bInterference = FALSE;
	m_bUse0x0800Option = FALSE;
	m_bUseEulerAngles = FALSE;
	m_nTrackingMode = 0;
	m_bPortEnabled = FALSE;
	m_bPortInitialized = FALSE;
	m_bStopTracking = FALSE;
	m_bIsTracking = FALSE;
	m_bResetHardware = FALSE;
	m_bWireless = FALSE;
	m_bSystemInitialized = FALSE;
	m_bPortsActivated = FALSE;
	m_nCOMPort = 0;
	m_nSysMode = MODE_PRE_RESET;

	//m_szFrameNumber = "";
	//m_szManufID = "";
	//m_szSerialNo = "";
	//m_szToolRev = "";
	//m_szToolType = "";
	//m_szPartNumber = "";

}

/*================================================================
*
* �� �� ����~NDI
*
* �Ρ�������
*
*
*
* ��������:
*�������� ��������
*
* �� �� ֵ��
*
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-06
*
================================================================*/
NDI::~NDI()
{
	delete pCommandHandling;
}

/*================================================================
*
* �� �� ����ResetSystem
*
* �Ρ�������
*
*
*
* ��������:
*			���ø���ϵͳ
*
* �� �� ֵ��
*			 1.�ɹ� 
*			-1.comδ����
*			-2.Ӳ������ʧ��
*			-3.���ö˿ڲ���ʧ��
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-06
*
================================================================*/
int NDI::ResetSystem()
{
	//read the COM port parameters from the ini file
	ReadINIParm( _T("Communication"), _T("Reset Hardware"), _T("0"), &m_bResetHardware	);
	ReadINIParm( _T("Communication"), _T("COM Port"), _T("0"), &m_nCOMPort		);
	ReadINIParm( _T("Communication"), _T("Wireless"), _T("0"), &m_bWireless		);

	pCommandHandling->nCloseComPorts();
	if (!pCommandHandling->nOpenComPort( m_nCOMPort ))
	{
		return -1;
	} //if
	//��Ӳ������ʱ
	if ( m_bResetHardware )
	{
		if( !pCommandHandling->nHardWareReset(m_bWireless) )
			return -2;		

		if ( !pCommandHandling->nSetSystemComParms( 0, 0, 0, 0, 0))
			return -3;
	} //if
	m_nSysMode = MODE_PRE_INIT;
	return 1;
}

/*================================================================
*
* �� �� ����Initialize
*
* �Ρ�������
*
*
*
* ��������:
*			��ʼ������ϵͳ ������Ҫ �Ƚ��и���ϵͳ���ã�
*
* �� �� ֵ��
*			 1.�ɹ� 
*			-1.comδ���� 
*			-2.Ӳ������ʧ�� 
*			-3.��ȡϵͳ��Ϣʧ��(Polaris, Polaris Accedo, and Aurora)
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-06
*
================================================================*/
int NDI::Initialize()
{
	int
		nBaudRate = 0, //������
		nStopBits = 0, 
		nParity   = 0,
		nDataBits = 0,
		nHardware = 0,
		nWireless = 0;

	//read the COM port parameters from the ini file
	ReadINIParm( _T("Communication"), _T("Baud Rate"),	_T("0"),	&nBaudRate		);
	ReadINIParm( _T("Communication"), _T("Stop Bits"),	_T("0"),	&nStopBits		);
	ReadINIParm( _T("Communication"), _T("Parity"),		_T("0"),	&nParity		);
	ReadINIParm( _T("Communication"), _T("Data Bits"),	_T("0"),	&nDataBits		);
	ReadINIParm( _T("Communication"), _T("Hardware"),	_T("0"),	&nHardware		);
	ReadINIParm( _T("Communication"), _T("COM Port"),	_T("0"),	&m_nCOMPort		);
	ReadINIParm( _T("Communication"), _T("Wireless"),	_T("0"),	&m_bWireless	);

	// This feature is useful for debugging only, m_bResetHardware is set to TRUE to disable it. 
	ReadINIParm( _T("Communication"), _T("Reset Hardware"), _T("0"), &m_bResetHardware );
	
	//close, then open the port
	pCommandHandling->nCloseComPorts();
	if (!pCommandHandling->nOpenComPort( m_nCOMPort ))
	{
		return -1;
	} //if

	//if we are supposed to reset, call the reset now
	if ( m_bResetHardware )
	{
		if (!pCommandHandling->nHardWareReset(m_bWireless))
			return -2;
	}//if
	
	// get the timeout values for the commands this will return an error with all other systems, other than Vicra
	pCommandHandling->CreateTimeoutTable();
	
	// set the System COM Port parameters, then the computers COM Port parameters.if that is successful, initialize the system
	if(pCommandHandling->nSetSystemComParms( nBaudRate, nDataBits, nParity, nStopBits, nHardware ))
	{
		if(pCommandHandling->nSetCompCommParms( nBaudRate, nDataBits, nParity, nStopBits, nHardware ))
		{
			if(pCommandHandling->nInitializeSystem())
			{
				//get the system information
				if (!pCommandHandling->nGetSystemInfo())
				{
					//Check system type: Polaris, Polaris Accedo, and Aurora
					return -3;
				} // if 

				//Set firing rate if system type is Polaris or Polaris Accedo.
				if( pCommandHandling->m_dtSystemInformation.nTypeofSystem != AURORA_SYSTEM )
				{
					pCommandHandling->nSetFiringRate(); 
				}//if

				m_nTypeofSystem = pCommandHandling->m_dtSystemInformation.nTypeofSystem;

				m_nSysMode = MODE_INIT;
				m_bSystemInitialized = TRUE;

				return 1;
			} //if 
		}// if 
	}// if 
	return 0;
}

/*================================================================
*
* �� �� ����ActivatePorts
*
* �Ρ�������
*
*
*
* ��������:
*			�������ж˿ڣ���Ҫ��ʼ��ϵͳ��.......���� 0.ʧ�� 1.�ɹ�
*
* �� �� ֵ��
*			1.�ɹ�
*			0.ʧ��
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-06
*
================================================================*/
int NDI::ActivatePorts()
{
	//if we can active the ports, we then fill the port information on the main dialog
	if (pCommandHandling->nActivateAllPorts())
	{
		m_bPortsActivated = TRUE;
		m_bPortEnabled = FALSE;
		m_bPortInitialized = FALSE;
		//m_szManufID = "";
		//m_szSerialNo = "";
		//m_szToolRev = "";
		//m_szToolType = "";
		//m_szPartNumber = "";
		if (!m_bIsTracking)
		m_nSysMode = MODE_ACTIVATED;

		std::vector<HandleInformation>().swap(m_vecHandleInformation); //	���vector m_vecHandleInformation
		std::vector<int>().swap(m_vecPortID); // ���vector m_vecPortID

		for ( int i = 0; i < NO_HANDLES; i++)
		{
			if ( pCommandHandling->m_dtHandleInformation[i].HandleInfo.bInitialized == 1 ) 
			{
				if( pCommandHandling->m_dtHandleInformation[i].szToolType[1] != '8' )
				{
					HandleInformation tempHandInfo =  pCommandHandling->m_dtHandleInformation[i];
					m_vecHandleInformation.push_back(tempHandInfo);
					int tempPortID = i;
					m_vecPortID.push_back(tempPortID);
				}/* if */
			} /* if */
		} /* for */

		return 1;
	} //if

	m_nSysMode = MODE_PRE_INIT;
	m_bPortsActivated = FALSE;

	return 0;
}

/*================================================================
*
* �� �� ����StartTracking
*
* �Ρ�������
*
*
*
* ��������:
*			��ʼ����(��Ҫ�������ж˿ں�)
*
* �� �� ֵ��
*			1.�ɹ�
*			0.ʧ��
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-06
*
================================================================*/
int NDI::StartTracking()
{
	int nCheckResponse = pCommandHandling->nStartTracking();
	if (nCheckResponse)
	{
		this->m_bIsTracking = TRUE;
		this->m_bStopTracking = FALSE;
		m_nSysMode = MODE_TRACKING;
		
	}
	return nCheckResponse;
}

/*================================================================
*
* �� �� ����StopTracking
*
* �Ρ�������
*
*
*
* ��������:
*			��������
*
* �� �� ֵ��
*			1.�ɹ�
*			0.ʧ��
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-06
*
================================================================*/
int NDI::StopTracking()
{
	int nCheckResponse = pCommandHandling->nStopTracking();
	if (nCheckResponse)
	{
		this->m_bIsTracking = FALSE;  
		this->m_bStopTracking = TRUE; 
		m_nSysMode = MODE_INIT;
	} 
	return nCheckResponse;
}

/*================================================================
*
* �� �� ����GetSysTransformData
*
* �Ρ�������
*
*
*
* ��������:
*			�õ���������
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-06
*
================================================================*/
int NDI::GetSysTransformData()
{
	if (!m_bIsTracking)
		return 0;
	
	//char pszTemp[256];	
	Rotation dtEulerRot;

	
	//if tracking mode is 0, we are asking for TX data, else we are asking for BX data.
	if ( m_nTrackingMode == 0 )
	{
		if ( !pCommandHandling->nGetTXTransforms( m_bUse0x0800Option ? TRUE : FALSE ) ) //�õ�λ�����ݣ�
			return 0;
	} // if 
	else if ( m_nTrackingMode == 1 )
	{
		if ( !pCommandHandling->nGetBXTransforms( m_bUse0x0800Option ? TRUE : FALSE ) ) //�õ�λ�����ݣ�
			return 0;
	} // else if
	//pCommandHandling->m_dtSystemInformation.bTemperatureOutOfRange;
	//pCommandHandling->m_dtSystemInformation.bDiagnosticsPending; //��ϴ���??
	/* 
	 * if a new port has become occupied we do the following:
	 * 1) Stop tracking
	 * 2) Activate Ports
	 * 3) Start Tracking
	 */
	if ( pCommandHandling->m_dtSystemInformation.bPortOccupied ) //���˿ڱ�ռ�� ���Լ���˿����¸���
	{
		if (pCommandHandling->nStopTracking() && ActivatePorts() && pCommandHandling->nStartTracking())
		{
			return 1;
		}//if
		
		 // We don't want the tracking thread to track if activating the ports failed!
		m_bStopTracking = TRUE;
		m_bIsTracking = FALSE;
		return -1;
	} /* if */
	//���²��������������
	for (int i =0;i<m_vecHandleInformation.size();i++)
	{
		// only update the frame if the handle isn't disabled
		m_vecHandleInformation[i] = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]];
		if (m_vecHandleInformation[i].Xfrms.ulFlags == TRANSFORM_VALID )
		{
			if( m_bUseEulerAngles )
			{
				CvtQuatToEulerRotation(&m_vecHandleInformation[i].Xfrms.rotation,&dtEulerRot);
				m_vecHandleInformation[i].Xfrms.rotation.q0 = dtEulerRot.fYaw;
				m_vecHandleInformation[i].Xfrms.rotation.qx = dtEulerRot.fPitch;
				m_vecHandleInformation[i].Xfrms.rotation.qy = dtEulerRot.fRoll;
				m_vecHandleInformation[i].Xfrms.rotation.qz = 0;
			}
		}
	}//for	
	return 1;
}

/*================================================================
*
* �� �� ����GetSysTransformData
*
* �Ρ�������
*            vector<QuatTransformation> &tranformationData     
*            
*
* ��������:
*			�õ��������ݿռ�任������Ԫ����ƽ������
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
*
================================================================*/
int NDI::GetSysTransformData(std::vector<QuatTransformation> &transformationData,bool &stopCapture,bool &flagCaptureSampleEToN, bool &flagCaptureSampleNToE)
{
	if (!m_bIsTracking)
		return 0;

	flagCaptureSampleNToE = false;

	//if tracking mode is 0, we are asking for TX data, else we are asking for BX data.
	if (m_nTrackingMode == 0)
	{
		while (!stopCapture)
		{
			//�õ�λ�����ݣ�
			if (!pCommandHandling->nGetTXTransforms(m_bUse0x0800Option ? TRUE : FALSE)) 
			{
				return 0;
			}	

			//����λ�����ݣ���¼�ڿ�������ͼ��ʱ������
			for (int i = 0; i < m_vecHandleInformation.size(); i++)
			{
				//��������Ƿ񳬳�����Χ
				/*if ((-1000000000000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x) ||
					(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x > 1000000000000.0f))
				{
					if ((-1000000000000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y) ||
						(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y > 1000000000000.0f))
					{
						if ((-1000000000000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z) ||
							(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z > 1000000000000.0f))
						{
							if ((-10000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0) ||
								(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0 > 10000.0f))
							{
								if ((-10000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx) ||
									(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx > 10000.0f))
								{
									if ((-10000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy) ||
										(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy > 10000.0f))
									{
										if ((-10000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz) ||
											(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz > 10000.0f))
										{
											std::cout << "\nδ׷�ٵ�����\n";
											break;
										}
									}
								}
							}
						}
					}

				}*/

				if (pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.ulFlags == TRANSFORM_VALID)
				{
					//�����ڿ������͵Ĳ�����Ϣ��NDI����λ������
					if (flagCaptureSampleEToN == true && flagCaptureSampleNToE == false)
					{
						//��¼λ������
						QuatTransformation qt_temp;
						qt_temp.rotation.q0 = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0;
						qt_temp.rotation.qx = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx;
						qt_temp.rotation.qy = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy;
						qt_temp.rotation.qz = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz;
						qt_temp.translation.x = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x;
						qt_temp.translation.y = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y;
						qt_temp.translation.z = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z;
						transformationData.push_back(qt_temp);

						//NDI���ڿ������ز���ɹ�����Ϣ
						flagCaptureSampleNToE = true;

						//��ԭ�ڿ���������Ϣ�ı�־
						flagCaptureSampleEToN = false;
					}
				}
				else
				{
					std::cout << "\nδ׷�ٵ�����\n";
					
				}
				/*std::cout << "qw:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0 << std::endl
					<< "qx:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx << std::endl
					<< "qy:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy << std::endl
					<< "qz:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz << std::endl
					<< "tx:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x << std::endl
					<< "ty:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y << std::endl
					<< "tz:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z << std::endl;*/
			}//for
		}//while		
	} // if 

	return 1;

}

/*================================================================
*
* �� �� ����GetSysTransformData
*
* �Ρ�������
*            vector<QuatTransformation> &tranformationData
*
*
* ��������:
*			�õ��������ݿռ�任������Ԫ����ƽ�����������������
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-4-26
*
================================================================*/
int NDI::GetSysTransformData(std::vector<QuatTransformation> &transformationData,std::vector<std::vector<MarkersInformation>> &markersData, bool &stopCapture, bool &flagCaptureSampleEToN, bool &flagCaptureSampleNToE)
{
	if (!m_bIsTracking)
		return 0;

	flagCaptureSampleNToE = false;

	//if tracking mode is 0, we are asking for TX data, else we are asking for BX data.
	if (m_nTrackingMode == 0)
	{
		while (!stopCapture)
		{
			//�õ�λ�����ݣ�
			if (!pCommandHandling->nGetTXTransforms(m_bUse0x0800Option ? TRUE : FALSE))
			{
				return 0;
			}

			std::vector<MarkersInformation> mi_temp;

			if (!GetPointsPosition(mi_temp))
			{
				return 0;
				
			}

			//����λ�����ݣ���¼�ڿ�������ͼ��ʱ������
			for (int i = 0; i < m_vecHandleInformation.size(); i++)
			{
				//��������Ƿ񳬳�����Χ
				if ((-1000000000000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x) ||
					(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x > 1000000000000.0f))
				{
					if ((-1000000000000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y) ||
						(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y > 1000000000000.0f))
					{
						if ((-1000000000000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z) ||
							(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z > 1000000000000.0f))
						{
							if ((-10000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0) ||
								(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0 > 10000.0f))
							{
								if ((-10000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx) ||
									(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx > 10000.0f))
								{
									if ((-10000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy) ||
										(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy > 10000.0f))
									{
										if ((-10000.0f > pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz) ||
											(pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz > 10000.0f))
										{
											std::cout << "\nδ׷�ٵ�����\n";
											break;
										}
									}
								}
							}
						}
					}

				}

				//�����ڿ������͵Ĳ�����Ϣ��NDI����λ������
				if (flagCaptureSampleEToN == true && flagCaptureSampleNToE == false)
				{
					//��¼�ռ�任����
					QuatTransformation qt_temp;
					qt_temp.rotation.q0 = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0;
					qt_temp.rotation.qx = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx;
					qt_temp.rotation.qy = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy;
					qt_temp.rotation.qz = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz;
					qt_temp.translation.x = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x;
					qt_temp.translation.y = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y;
					qt_temp.translation.z = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z;
					transformationData.push_back(qt_temp);

					//��¼�����λ������
					markersData.push_back(mi_temp);

					//NDI���ڿ������ز���ɹ�����Ϣ
					flagCaptureSampleNToE = true;

					//��ԭ�ڿ���������Ϣ�ı�־
					flagCaptureSampleEToN = false;
				}

				/*std::cout << "qw:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0 << std::endl
					<< "qx:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx << std::endl
					<< "qy:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy << std::endl
					<< "qz:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz << std::endl
					<< "tx:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x << std::endl
					<< "ty:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y << std::endl
					<< "tz:" << pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z << std::endl;*/
			}//for
		}//while
	} // if 

	return 1;

}

/*======================================================================================
*
* �� �� ����GetSysTransformData
*
* �Ρ�������
*            
*
*
* ��������:
*			�õ� �ڿ����ϵĸ���ͱ궨���ϵĸ���  �������ݿռ�任������Ԫ����ƽ������
*           flagEndoscopeRigid��¼�ڿ����ϸ������ݵ�portID
*           flagBoardRigid    ��¼�궨���ϸ������ݵ�portID
*
* �� �� ֵ��
*           0  δ�������
*           -1 δ����ڿ����ϸ��������
*           -2 δ��ñ궨���ϸ��������
*           1  ������ݳɹ�
*
* �׳��쳣��
*
* �������ߣ� ���� 2021-5-27
*
=======================================================================================*/
int NDI::GetSysTransformData(QuatTransformation &qtransformationData, QuatTransformation &qboardRigidData,int flagEndoscopeRigid, int flagBoardRigid)
{
	if (!m_bIsTracking)
		return 0;

	//if tracking mode is 0, we are asking for TX data, else we are asking for BX data.
	if (m_nTrackingMode == 0)
	{
		//�õ�λ�����ݣ�
		if (!pCommandHandling->nGetTXTransforms(m_bUse0x0800Option ? TRUE : FALSE))
		{
			return 0;
		}
		
		//�ڿ����ϸ���
		if (flagEndoscopeRigid < m_vecHandleInformation.size())
		{
			if (pCommandHandling->m_dtHandleInformation[m_vecPortID[flagEndoscopeRigid]].Xfrms.ulFlags == TRANSFORM_VALID)
			{
				qtransformationData.rotation.q0 = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagEndoscopeRigid]].Xfrms.rotation.q0;
				qtransformationData.rotation.qx = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagEndoscopeRigid]].Xfrms.rotation.qx;
				qtransformationData.rotation.qy = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagEndoscopeRigid]].Xfrms.rotation.qy;
				qtransformationData.rotation.qz = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagEndoscopeRigid]].Xfrms.rotation.qz;
				qtransformationData.translation.x = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagEndoscopeRigid]].Xfrms.translation.x;
				qtransformationData.translation.y = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagEndoscopeRigid]].Xfrms.translation.y;
				qtransformationData.translation.z = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagEndoscopeRigid]].Xfrms.translation.z;
			}
			else
			{
				std::cout << "\nδ׷�ٵ��ڿ�������\n";
				return -1;
			}
		}//if�ڿ����ϸ���
		else
		{
			return -1;
		}
	
		//�궨���ϸ���
		if (flagBoardRigid < m_vecHandleInformation.size())
		{
			if (pCommandHandling->m_dtHandleInformation[m_vecPortID[flagBoardRigid]].Xfrms.ulFlags == TRANSFORM_VALID)
			{
				qboardRigidData.rotation.q0 = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagBoardRigid]].Xfrms.rotation.q0;
				qboardRigidData.rotation.qx = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagBoardRigid]].Xfrms.rotation.qx;
				qboardRigidData.rotation.qy = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagBoardRigid]].Xfrms.rotation.qy;
				qboardRigidData.rotation.qz = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagBoardRigid]].Xfrms.rotation.qz;
				qboardRigidData.translation.x = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagBoardRigid]].Xfrms.translation.x;
				qboardRigidData.translation.y = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagBoardRigid]].Xfrms.translation.y;
				qboardRigidData.translation.z = pCommandHandling->m_dtHandleInformation[m_vecPortID[flagBoardRigid]].Xfrms.translation.z;
			}
			else
			{
				std::cout << "\nδ׷�ٵ��궨�����\n";
				return -2;
			}
		}//if�궨���ϸ���
		else
		{
			return -2;
		}

		std::cout << "\nsuccessed\n";

	} // if 

	return 1;

}

/*================================================================
*
* �� �� ����DirectStartTracking
*
* �Ρ�������
*			�����&strReturnMessage   ���ز��������Ϣ
*
*
* ��������:
*			ֱ�ӽ��и��� ��������á���ʼ��������ڡ��������ٲ��� 
*
* �� �� ֵ��
*			1.�ɹ�
*			2.ʧ��
*
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-09
*
================================================================*/
int NDI::DirectStartTracking(std::string &strReturnMessage)
{
	int returnInfo = 0; //����ֵ

	//����ϵͳ����
	returnInfo = ResetSystem();
	if (returnInfo != 1)
	{
		if (returnInfo == 0)
		{
			strReturnMessage = "Seneor Reset System Failed!";
		}
		else if (returnInfo == -1)
		{
			strReturnMessage = "COM Port could not be opened";
		}
		else if (returnInfo == -2)
		{
			strReturnMessage = "Hard Ware Reset Failed!";
		}
		else if (returnInfo == -3)
		{
			strReturnMessage = "Set System Com parameters Failed!";
		}
		return 0;
	}
	else
	{
		std::cout << "\n����ϵͳ���óɹ���\n";
	}

	//����ϵͳ��ʼ��
	returnInfo = Initialize();
	if (returnInfo != 1)
	{
		if (returnInfo == 0)
		{
			strReturnMessage = "Seneor Initialize Failed!";
		}
		else if (returnInfo == -1)
		{
			strReturnMessage = "COM Port could not be opened";
		}
		else if (returnInfo == -2)
		{
			strReturnMessage = "Hard Ware Reset Failed!";
		}
		else if (returnInfo == -3)
		{
			strReturnMessage = "Could not determine type of system!";
		}
		return 0;
	}
	else
	{
		std::cout << "����ϵͳ��ʼ���ɹ���\n";
	}

	//�����ϵͳ
	returnInfo = ActivatePorts();
	if (returnInfo == 0)
	{
		strReturnMessage = "Seneor Activate Ports Failed!";
		return 0;
	}
	else
	{
		std::cout << "�����ϵͳ��\n";
	}
	
	returnInfo = StartTracking();
	if (returnInfo == 0)
	{
		strReturnMessage = " Start Tracking Failed!";
		return 0;
	}
	else
	{
		std::cout << "����ϵͳ��ʼ׷�٣�\n";
	}

	strReturnMessage = " Start Tracking Succeed!";
	return 1;
}

int NDI::GetPointsPosition(std::vector<MarkersInformation> &vecMarkersInfo)
{
	return pCommandHandling->nGetPointsPosition(vecMarkersInfo);
}


//bool NDI::SetComPort(int com_port_num)
//{
//	m_nCOMPort = com_port_num;
//	WriteINIParm("Communication", "COM Port", m_nCOMPort);
//	return  true;
//}

//bool NDI::SetROMFile(int nPortID, char * pszROMFileName)
//{
//	if ( pszROMFileName[0] == '\0' )
//	{
//		return false;
//	} /* if */	
//	else
//	{
//		////_access(m_szROMFile, 0 ) < 0
//		char pszPortID[64];
//		sprintf(pszPortID,"Wireless Tool %02d",nPortID);
//		WriteINIParm ("POLARIS SROM Image Files", pszPortID, pszROMFileName );
//	}
//	return true;
//}

//void NDI::SetComPortInformation(const ComPortInformation &comPortInformation)
//{
//	WriteINIParm( "Communication", "Baud Rate",      comPortInformation.nBaudRate ); //GetCurSel()
//	WriteINIParm( "Communication", "Stop Bits",      comPortInformation.nStopBits ); //GetCurSel()
//	WriteINIParm( "Communication", "Parity",	     comPortInformation.nParity   ); //GetCurSel()
//	WriteINIParm( "Communication", "Data Bits",      comPortInformation.nDataBits ); //GetCurSel()
//	WriteINIParm( "Communication", "COM Port",	     comPortInformation.nCOMPort  ); //GetCurSel()
//	WriteINIParm( "Communication", "Reset Hardware", comPortInformation.bReset    );
//	WriteINIParm( "Communication", "Hardware",       comPortInformation.bHardware );
//	WriteINIParm( "Communication", "Wireless",       comPortInformation.bWireless );
//
//}

//void NDI::SetUseEulerAngles(bool bused)
//{
//	m_bUseEulerAngles = bused;
//} 

/*================================================================
*
* �� �� ����StopTracking
*
* �Ρ�������
*			strPortHandle
*
*
* ��������:
*			��Ҫ��ʾ���ٵ�portHandles��������
*
* �� �� ֵ��
*
*
* �׳��쳣��
*
* �������ߣ� κ���� 2018-05-06
*
================================================================*/
//int NDI::SelchangePortHandles(std::string strPortHandle) 
//{
//	int nPortHandle = 0;
//
//	nPortHandle = uASCIIToHex( (char*)strPortHandle.data(), 2 );
//
//	/* fill the form with the info that pertains to the selected handle */
//	m_szManufID = pCommandHandling->m_dtHandleInformation[nPortHandle].szManufact;
//	m_bPortEnabled = pCommandHandling->m_dtHandleInformation[nPortHandle].HandleInfo.bEnabled;
//	m_bPortInitialized = pCommandHandling->m_dtHandleInformation[nPortHandle].HandleInfo.bInitialized;
//	m_szSerialNo = pCommandHandling->m_dtHandleInformation[nPortHandle].szSerialNo;
//	m_szToolRev = pCommandHandling->m_dtHandleInformation[nPortHandle].szRev;
//	m_szToolType = pCommandHandling->m_dtHandleInformation[nPortHandle].szToolType;
//	m_szPartNumber = pCommandHandling->m_dtHandleInformation[nPortHandle].szPartNumber;
//	return 0;
//
//} //SelchangePortHandles