////////////////////////////////////////////////////////////////
////  NO_HANDLES = 16  ֧��16��handls  demo ��256
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
*�� �ļ����� NDI.h
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
	//���캯��
	NDI();
	//��������
	~NDI();
public:

	//��¼Ӳ�����ص�������Ϣ
	std::vector<HandleInformation>		m_vecHandleInformation; 
	//��¼��Ӧ��m_vecHandleInformation��portID
	std::vector<int>					m_vecPortID;
	//�����߳�ֹͣ����
	bool								m_bStopTracking;	 
	//��ǵ�ǰ����״̬
	bool								m_bIsTracking;
	////����״̬ 1.ȫ��ʧ 2.�붪ʧ 3.���� 
	//int									m_nTrakingState1;
	////����״̬ 1.��ʧ   2.������
	//int									m_nTrakingState2;
	//����ģʽ
	int									m_nTrackingMode; 
	//ϵͳģʽ
	int									m_nSysMode;      
	//��ͻ	
	bool								m_bInterference;
	//
	bool								m_bUse0x0800Option;
	//ʹ��ŷ���Ǳ�ʾ
	bool								m_bUseEulerAngles;
	//
	bool								m_bPortEnabled;
	//
	bool								m_bPortInitialized;
	//�Ƿ�����Ӳ������	��reset hardware variable�� 
	bool								m_bResetHardware;
	//�Ƿ�������߼������ã�uses the wireless compatible settings�� 
	bool								m_bWireless;
	//�Ƿ�ϵͳ����ʼ�� 	��is the system initialized�� 
	bool								m_bSystemInitialized;
	//�Ƿ�˿ڱ�����		��are ports activated��  
	bool								m_bPortsActivated;
	//��ǰ�˿ں�			��the current com port number�� 
	int									m_nCOMPort;				
	//�������	 
	CCommandHandling					*pCommandHandling;		

	//ϵͳ����
	int									m_nTypeofSystem;
	////֡��
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
	//���ø���ϵͳ									���� 0 ʧ�� 1.�ɹ� -1.comδ���� -2.Ӳ������ʧ�� -3.���ö˿ڲ���ʧ��
	int ResetSystem();		
	//��ʼ������ϵͳ ������Ҫ �Ƚ��и���ϵͳ���ã�		���� 0.ʧ�� 1.�ɹ� -1.comδ���� -2.Ӳ������ʧ�� -3.��ȡϵͳ��Ϣʧ��(Polaris, Polaris Accedo, and Aurora)
	int Initialize();	
	//�������ж˿ڣ���Ҫ��ʼ��ϵͳ��					���� 0.ʧ�� 1.�ɹ�
	int ActivatePorts();	
	//��ʼ���� (��Ҫ�������ж˿ں�)
	int StartTracking();
	//��������
	int StopTracking();	

	//�õ���������
	int GetSysTransformData();
	int GetSysTransformData(std::vector<QuatTransformation>& tranformationData,bool &stopCapture, bool &flagCaptureSampleEToN, bool &flagCaptureSampleNToE);

	int GetSysTransformData(std::vector<QuatTransformation>& transformationData, std::vector<std::vector<MarkersInformation>>& markersData, bool & stopCapture, bool & flagCaptureSampleEToN, bool & flagCaptureSampleNToE);

	int GetSysTransformData(QuatTransformation & qtransformationData, QuatTransformation & qboardRigidData, int flagEndoscopeRigid, int flagBoardRigid);

	//ֱ�ӿ�ʼ����
	int DirectStartTracking(std::string &strReturnMessage);
	int GetPointsPosition(std::vector<MarkersInformation> &vecMarkersInfo);
	//bool SetComPort(int com_port_num);        //���ô��ڲ��� 0-8 com1-com9
	//void SetComPortInformation(const ComPortInformation &comPortInformation); //����com�ڲ���
	//bool SetROMFile(int nPortID, char * pszROMFileName);      //����rom�ļ� ���ٸ����ļ� nPortID�����ļ��еĵڼ���
	//void SetUseEulerAngles(bool bused); //�����Ƿ�ʹ��ŷ���Ǳ�ʾ��ת
	//int SelchangePortHandles(std::string strPortHandle);//��Ҫ��ʾ���ٵ�portHandles��������;

protected:
private:
};