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
*　 文件名： NDI.cpp
*
*　 文件描述：	光学跟踪器相关控制 重置、开启、关闭、获取数据等操作
*
*　 创建人：魏国栋
*
*　 版本号：1.0.0
*
*　 修改记录：
*
************************************************************************/


// 全局函数
//////////////////////////////////////////////////////////////////////////

/*================================================================
*
* 函 数 名：NDI
*
* 参　　数：
*		
*
*
* 功能描述:
*
*　　　　 构造函数
*
* 返 回 值：
*
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-06
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
* 函 数 名：~NDI
*
* 参　　数：
*
*
*
* 功能描述:
*　　　　 析构函数
*
* 返 回 值：
*
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-06
*
================================================================*/
NDI::~NDI()
{
	delete pCommandHandling;
}

/*================================================================
*
* 函 数 名：ResetSystem
*
* 参　　数：
*
*
*
* 功能描述:
*			重置跟踪系统
*
* 返 回 值：
*			 1.成功 
*			-1.com未被打开
*			-2.硬件重置失败
*			-3.设置端口参数失败
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-06
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
	//当硬件更新时
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
* 函 数 名：Initialize
*
* 参　　数：
*
*
*
* 功能描述:
*			初始化跟踪系统 （如需要 先进行跟踪系统重置）
*
* 返 回 值：
*			 1.成功 
*			-1.com未被打开 
*			-2.硬件重置失败 
*			-3.获取系统信息失败(Polaris, Polaris Accedo, and Aurora)
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-06
*
================================================================*/
int NDI::Initialize()
{
	int
		nBaudRate = 0, //波特率
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
* 函 数 名：ActivatePorts
*
* 参　　数：
*
*
*
* 功能描述:
*			激活所有端口（需要初始化系统后）.......返回 0.失败 1.成功
*
* 返 回 值：
*			1.成功
*			0.失败
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-06
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

		std::vector<HandleInformation>().swap(m_vecHandleInformation); //	清空vector m_vecHandleInformation
		std::vector<int>().swap(m_vecPortID); // 清空vector m_vecPortID

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
* 函 数 名：StartTracking
*
* 参　　数：
*
*
*
* 功能描述:
*			开始跟踪(需要激活所有端口后)
*
* 返 回 值：
*			1.成功
*			0.失败
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-06
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
* 函 数 名：StopTracking
*
* 参　　数：
*
*
*
* 功能描述:
*			结束跟踪
*
* 返 回 值：
*			1.成功
*			0.失败
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-06
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
* 函 数 名：GetSysTransformData
*
* 参　　数：
*
*
*
* 功能描述:
*			得到跟踪数据
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-06
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
		if ( !pCommandHandling->nGetTXTransforms( m_bUse0x0800Option ? TRUE : FALSE ) ) //得到位置数据；
			return 0;
	} // if 
	else if ( m_nTrackingMode == 1 )
	{
		if ( !pCommandHandling->nGetBXTransforms( m_bUse0x0800Option ? TRUE : FALSE ) ) //得到位置数据；
			return 0;
	} // else if
	//pCommandHandling->m_dtSystemInformation.bTemperatureOutOfRange;
	//pCommandHandling->m_dtSystemInformation.bDiagnosticsPending; //诊断待定??
	/* 
	 * if a new port has become occupied we do the following:
	 * 1) Stop tracking
	 * 2) Activate Ports
	 * 3) Start Tracking
	 */
	if ( pCommandHandling->m_dtSystemInformation.bPortOccupied ) //若端口被占用 尝试激活端口重新跟踪
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
	//如下操作获得所有数据
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
* 函 数 名：GetSysTransformData
*
* 参　　数：
*            vector<QuatTransformation> &tranformationData     
*            
*
* 功能描述:
*			得到跟踪数据空间变换矩阵，四元数和平移向量
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
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
			//得到位置数据；
			if (!pCommandHandling->nGetTXTransforms(m_bUse0x0800Option ? TRUE : FALSE)) 
			{
				return 0;
			}	

			//遍历位置数据，记录内窥镜捕获图像时的数据
			for (int i = 0; i < m_vecHandleInformation.size(); i++)
			{
				//检验刚体是否超出捕获范围
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
											std::cout << "\n未追踪到刚体\n";
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
					//根据内窥镜发送的捕获消息，NDI捕获位置数据
					if (flagCaptureSampleEToN == true && flagCaptureSampleNToE == false)
					{
						//记录位置数据
						QuatTransformation qt_temp;
						qt_temp.rotation.q0 = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0;
						qt_temp.rotation.qx = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx;
						qt_temp.rotation.qy = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy;
						qt_temp.rotation.qz = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz;
						qt_temp.translation.x = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x;
						qt_temp.translation.y = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y;
						qt_temp.translation.z = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z;
						transformationData.push_back(qt_temp);

						//NDI向内窥镜返回捕获成功成消息
						flagCaptureSampleNToE = true;

						//还原内窥镜捕获消息的标志
						flagCaptureSampleEToN = false;
					}
				}
				else
				{
					std::cout << "\n未追踪到刚体\n";
					
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
* 函 数 名：GetSysTransformData
*
* 参　　数：
*            vector<QuatTransformation> &tranformationData
*
*
* 功能描述:
*			得到跟踪数据空间变换矩阵，四元数和平移向量，标记球坐标
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 许毅 2021-4-26
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
			//得到位置数据；
			if (!pCommandHandling->nGetTXTransforms(m_bUse0x0800Option ? TRUE : FALSE))
			{
				return 0;
			}

			std::vector<MarkersInformation> mi_temp;

			if (!GetPointsPosition(mi_temp))
			{
				return 0;
				
			}

			//遍历位置数据，记录内窥镜捕获图像时的数据
			for (int i = 0; i < m_vecHandleInformation.size(); i++)
			{
				//检验刚体是否超出捕获范围
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
											std::cout << "\n未追踪到刚体\n";
											break;
										}
									}
								}
							}
						}
					}

				}

				//根据内窥镜发送的捕获消息，NDI捕获位置数据
				if (flagCaptureSampleEToN == true && flagCaptureSampleNToE == false)
				{
					//记录空间变换数据
					QuatTransformation qt_temp;
					qt_temp.rotation.q0 = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.q0;
					qt_temp.rotation.qx = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qx;
					qt_temp.rotation.qy = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qy;
					qt_temp.rotation.qz = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.rotation.qz;
					qt_temp.translation.x = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.x;
					qt_temp.translation.y = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.y;
					qt_temp.translation.z = pCommandHandling->m_dtHandleInformation[m_vecPortID[i]].Xfrms.translation.z;
					transformationData.push_back(qt_temp);

					//记录标记球位置坐标
					markersData.push_back(mi_temp);

					//NDI向内窥镜返回捕获成功成消息
					flagCaptureSampleNToE = true;

					//还原内窥镜捕获消息的标志
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
* 函 数 名：GetSysTransformData
*
* 参　　数：
*            
*
*
* 功能描述:
*			得到 内窥镜上的刚体和标定板上的刚体  跟踪数据空间变换矩阵，四元数和平移向量
*           flagEndoscopeRigid记录内窥镜上刚体数据的portID
*           flagBoardRigid    记录标定板上刚体数据的portID
*
* 返 回 值：
*           0  未获得数据
*           -1 未获得内窥镜上刚体的数据
*           -2 未获得标定板上刚体的数据
*           1  获得数据成功
*
* 抛出异常：
*
* 作　　者： 许毅 2021-5-27
*
=======================================================================================*/
int NDI::GetSysTransformData(QuatTransformation &qtransformationData, QuatTransformation &qboardRigidData,int flagEndoscopeRigid, int flagBoardRigid)
{
	if (!m_bIsTracking)
		return 0;

	//if tracking mode is 0, we are asking for TX data, else we are asking for BX data.
	if (m_nTrackingMode == 0)
	{
		//得到位置数据；
		if (!pCommandHandling->nGetTXTransforms(m_bUse0x0800Option ? TRUE : FALSE))
		{
			return 0;
		}
		
		//内窥镜上刚体
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
				std::cout << "\n未追踪到内窥镜刚体\n";
				return -1;
			}
		}//if内窥镜上刚体
		else
		{
			return -1;
		}
	
		//标定板上刚体
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
				std::cout << "\n未追踪到标定板刚体\n";
				return -2;
			}
		}//if标定板上刚体
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
* 函 数 名：DirectStartTracking
*
* 参　　数：
*			输出：&strReturnMessage   返回操作结果信息
*
*
* 功能描述:
*			直接进行跟踪 会进行重置、初始化、激活窗口、开启跟踪操作 
*
* 返 回 值：
*			1.成功
*			2.失败
*
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-09
*
================================================================*/
int NDI::DirectStartTracking(std::string &strReturnMessage)
{
	int returnInfo = 0; //返回值

	//导航系统重置
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
		std::cout << "\n导航系统重置成功！\n";
	}

	//导航系统初始化
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
		std::cout << "导航系统初始化成功！\n";
	}

	//激活导航系统
	returnInfo = ActivatePorts();
	if (returnInfo == 0)
	{
		strReturnMessage = "Seneor Activate Ports Failed!";
		return 0;
	}
	else
	{
		std::cout << "激活导航系统！\n";
	}
	
	returnInfo = StartTracking();
	if (returnInfo == 0)
	{
		strReturnMessage = " Start Tracking Failed!";
		return 0;
	}
	else
	{
		std::cout << "导航系统开始追踪！\n";
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
* 函 数 名：StopTracking
*
* 参　　数：
*			strPortHandle
*
*
* 功能描述:
*			需要显示跟踪的portHandles参数设置
*
* 返 回 值：
*
*
* 抛出异常：
*
* 作　　者： 魏国栋 2018-05-06
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