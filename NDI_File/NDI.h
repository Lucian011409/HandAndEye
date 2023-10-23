////////////////////////////////////////////////////////////////
////  NO_HANDLES = 16  支持16个handls  demo 可256
////
////
////////////////////////////////////////////////////////////////

#pragma once
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif
#ifndef NULL
#define NULL 0
#endif

/************************************************************************
*
*　 文件名： NDI.h
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
#include "CommandHandling.h"
#include <string>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <Eigen\dense>
//#include <tchar.h>

#define MODE_PRE_RESET		0x00
#define MODE_PRE_INIT		0x01
#define MODE_INIT			0x02
#define MODE_ACTIVATED		0x03
#define MODE_TRACKING		0x04


#ifdef _DEBUG
#define new DEBUG_NEW
#endif


class NDI
{
public:
	//构造函数
	NDI();
	//析构函数
	~NDI();
public:

	//记录硬件返回的所有信息
	std::vector<HandleInformation>		m_vecHandleInformation; 
	//记录对应的m_vecHandleInformation的portID
	std::vector<int>					m_vecPortID;
	//告诉线程停止工作
	bool								m_bStopTracking;	 
	//标记当前跟踪状态
	bool								m_bIsTracking;
	////跟踪状态 1.全丢失 2.半丢失 3.正常 
	//int									m_nTrakingState1;
	////跟踪状态 1.丢失   2.不可用
	//int									m_nTrakingState2;
	//跟踪模式
	int									m_nTrackingMode; 
	//系统模式
	int									m_nSysMode;      
	//冲突	
	bool								m_bInterference;
	//
	bool								m_bUse0x0800Option;
	//使用欧拉角表示
	bool								m_bUseEulerAngles;
	//
	bool								m_bPortEnabled;
	//
	bool								m_bPortInitialized;
	//是否重置硬件变量	（reset hardware variable） 
	bool								m_bResetHardware;
	//是否采用无线兼容设置（uses the wireless compatible settings） 
	bool								m_bWireless;
	//是否系统被初始化 	（is the system initialized） 
	bool								m_bSystemInitialized;
	//是否端口被激活		（are ports activated）  
	bool								m_bPortsActivated;
	//当前端口号			（the current com port number） 
	int									m_nCOMPort;				
	//命令处理类	 
	CCommandHandling					*pCommandHandling;		

	//系统类型
	int									m_nTypeofSystem;
	////帧数
	//std::string							m_szFrameNumber;
	////
	//std::string							m_szManufID;
	////
	//std::string							m_szSerialNo;
	////
	//std::string							m_szToolRev;
	////
	//std::string							m_szToolType;
	////
	//std::string							m_szPartNumber;
public:
	//重置跟踪系统									返回 0 失败 1.成功 -1.com未被打开 -2.硬件重置失败 -3.设置端口参数失败
	int ResetSystem();		
	//初始化跟踪系统 （如需要 先进行跟踪系统重置）		返回 0.失败 1.成功 -1.com未被打开 -2.硬件重置失败 -3.获取系统信息失败(Polaris, Polaris Accedo, and Aurora)
	int Initialize();	
	//激活所有端口（需要初始化系统后）					返回 0.失败 1.成功
	int ActivatePorts();	
	//开始跟踪 (需要激活所有端口后)
	int StartTracking();
	//结束跟踪
	int StopTracking();	

	//得到跟踪数据
	int GetSysTransformData();
	int GetSysTransformData(std::vector<QuatTransformation>& tranformationData,bool &stopCapture, bool &flagCaptureSampleEToN, bool &flagCaptureSampleNToE);

	int GetSysTransformData(std::vector<QuatTransformation>& transformationData, std::vector<std::vector<MarkersInformation>>& markersData, bool & stopCapture, bool & flagCaptureSampleEToN, bool & flagCaptureSampleNToE);

	int GetSysTransformData(QuatTransformation & qtransformationData, QuatTransformation & qboardRigidData, int flagEndoscopeRigid, int flagBoardRigid);

	//直接开始跟踪
	int DirectStartTracking(std::string &strReturnMessage);
	int GetPointsPosition(std::vector<MarkersInformation> &vecMarkersInfo);
	//bool SetComPort(int com_port_num);        //设置串口参数 0-8 com1-com9
	//void SetComPortInformation(const ComPortInformation &comPortInformation); //设置com口参数
	//bool SetROMFile(int nPortID, char * pszROMFileName);      //设置rom文件 跟踪刚体文件 nPortID配置文件中的第几个
	//void SetUseEulerAngles(bool bused); //设置是否使用欧拉角表示旋转
	//int SelchangePortHandles(std::string strPortHandle);//需要显示跟踪的portHandles参数设置;

protected:
private:
};