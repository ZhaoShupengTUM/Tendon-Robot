
// TendonRobot.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CTendonRobotApp:
// �йش����ʵ�֣������ TendonRobot.cpp
//

class CTendonRobotApp : public CWinApp
{
public:
	CTendonRobotApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CTendonRobotApp theApp;