
// TendonRobotDlg.cpp : 实现文件
//
#include<windows.h>
#include "stdafx.h"
#include "TendonRobot.h"
#include "TendonRobotDlg.h"
#include "afxdialogex.h"
#include<stdlib.h>
#include "afxmt.h" // MultiThread Synchronization
#include "time.h"
#include "math.h"
#include "robot.h" 
#include "ECanVci.h" // USBCAN Libarary
#include<Eigen/Dense> // Matrix Libarary
using namespace Eigen;

#pragma region declaration 
/********* Self Defined Function ****************/
void ErrorExplain(P_ERR_INFO perror_info);//释错函数
void StopMotor();//停止电机
int Getdata(void);//输出数据

/********* USB-CAN *********/
int USBCANDevType = 3;
int USBCANDevIndex = 0;
int CANIndex = 0;
int nReserved =0;
INIT_CONFIG InitConfig;
P_ERR_INFO perror_info;
DWORD dwRel,TdwRel; 
UINT error;
BYTE kerror[3];
BYTE lerror;
CAN_OBJ SendMotor[6],ReceiveMotor[6],SendDAQ[6],SendStepper[1],ReceiveDAQ[6],Receivebuff[1000000];
BYTE initData[8]={0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55},initData2[8]={0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // 初始化的电机发送数据和数据采集卡请求数据
BYTE release[4]={0xB8,0x0B,0x00,0x00}, grasp[4]={0x48,0xF4,0xFF,0xFF}; // 步进电机释放/抓取速度

/********* Motor Parameter *********/
bool isgrasp=false;
int MotorMode=2;
signed short PWM=5000;
signed int Length_Record[6];// 位置记录(用脉冲数记录) 
volatile double ALength[6]={460,460,460,920,920,920};// 驱动器长度(控制过程中应用)
volatile double initialALength[6];//开机时驱动器长度
volatile double ShowLength[6];//用于在界面上显示驱动器长度
volatile double DAQvolatge[6];
FILE *writetxt=NULL, *readtxt=NULL; // 文本文件读写变量
/********Robot*********/
double BackboneLength=460, PlateRadius=15;
Robot tendoncontinuum(BackboneLength,PlateRadius,0,0,0,0);
int ControlVariable=2;
int T=30;
int F=10;
MatrixXd Path;
double t1=M_PI/2, t2=M_PI/2, d1=0, d2=0;
CWnd* mypWnd; 
CCriticalSection critical_section, critical_section1,critical_section2;//线程同步变量


#pragma endregion

#ifdef _DEBUG
#define new DEBUG_NEW
#endif



// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CTendonRobotDlg 对话框




CTendonRobotDlg::CTendonRobotDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CTendonRobotDlg::IDD, pParent)
	, m_EditReceive(_T(""))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_Length = _T("");
	m_Radius = _T("");
	v_theta1 = _T("");
	v_theta2 = _T("");
	v_delta2 = _T("");
	v_delta1 = _T("");
	v_L11 = _T("");
	v_L12 = _T("");
	v_L13 = _T("");
	v_L21 = _T("");
	v_L22 = _T("");
	v_L23 = _T("");
	v_V1 = _T("");
	v_V2 = _T("");
	v_V3 = _T("");
	v_V4 = _T("");
	v_V6 = _T("");
	v_V5 = _T("");
}

void CTendonRobotDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO1, m_ControlMode);
	DDX_Control(pDX, IDC_COMBO2, m_ControlVariable);
	DDX_Text(pDX, IDC_EDIT1, m_Length);
	DDX_Text(pDX, IDC_EDIT2, m_Radius);
	DDX_Control(pDX, IDC_SLIDER1, m_theta1);
	DDX_Control(pDX, IDC_SLIDER2, m_delta1);
	DDX_Control(pDX, IDC_SLIDER3, m_theta2);
	DDX_Control(pDX, IDC_SLIDER4, m_delta2);
	DDX_Control(pDX, IDC_SLIDER5, m_x);
	DDX_Control(pDX, IDC_SLIDER6, m_y);
	DDX_Control(pDX, IDC_SLIDER7, m_z);
	DDX_Text(pDX, IDC_STATICt1, v_theta1);
	DDX_Text(pDX, IDC_STATICt2, v_theta2);
	DDX_Text(pDX, IDC_STATICd2, v_delta2);
	DDX_Text(pDX, IDC_STATICd1, v_delta1);
	DDX_Text(pDX, IDC_EDIT3, v_L11);
	DDX_Text(pDX, IDC_EDIT4, v_L12);
	DDX_Text(pDX, IDC_EDIT5, v_L13);
	DDX_Text(pDX, IDC_EDIT6, v_L21);
	DDX_Text(pDX, IDC_EDIT7, v_L22);
	DDX_Text(pDX, IDC_EDIT8, v_L23);
	DDX_Text(pDX, IDC_EDIT9, v_V1);
	DDX_Text(pDX, IDC_EDIT10, v_V2);
	DDX_Text(pDX, IDC_EDIT11, v_V3);
	DDX_Text(pDX, IDC_EDIT12, v_V4);
	DDX_Text(pDX, IDC_EDIT14, v_V6);
	DDX_Text(pDX, IDC_EDIT13, v_V5);
	DDX_Text(pDX, IDC_EDIT_Rev, m_EditReceive);
	DDX_Control(pDX, IDC_COMBO3, m_comb1);
	DDX_Control(pDX, IDC_COMBO4, m_comb2);
	DDX_Control(pDX, IDC_MSCOMM1, m_mscom);
}

BEGIN_MESSAGE_MAP(CTendonRobotDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_EN_CHANGE(IDC_EDIT1, &CTendonRobotDlg::OnEnChangeEdit1)
	ON_EN_CHANGE(IDC_EDIT2, &CTendonRobotDlg::OnEnChangeEdit2)
	ON_BN_CLICKED(IDC_BUTTON1, &CTendonRobotDlg::OnBnClickedButton1)
	ON_CBN_SELCHANGE(IDC_COMBO1, &CTendonRobotDlg::OnCbnSelchangeCombo1)
	ON_CBN_SELCHANGE(IDC_COMBO2, &CTendonRobotDlg::OnCbnSelchangeCombo2)
	ON_BN_CLICKED(IDC_BUTTON2, &CTendonRobotDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CTendonRobotDlg::OnBnClickedButton3)
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER7, &CTendonRobotDlg::OnNMCustomdrawSlider7)
	//添加一个事件处理函数 当调节滑块位置的时候能得到相应的数据
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER5, &CTendonRobotDlg::OnNMCustomdrawSlider5)
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER6, &CTendonRobotDlg::OnNMCustomdrawSlider6)
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER1, &CTendonRobotDlg::OnNMCustomdrawSlider1)
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER2, &CTendonRobotDlg::OnNMCustomdrawSlider2)
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER3, &CTendonRobotDlg::OnNMCustomdrawSlider3)
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER4, &CTendonRobotDlg::OnNMCustomdrawSlider4)

	ON_MESSAGE(WM_MYMESSAGE1,OnMyMssage1)/*自定义消息*/

	ON_BN_CLICKED(IDC_BUTTON4_OPEN, &CTendonRobotDlg::OnBnClickedButton4Open)
	ON_BN_CLICKED(IDC_BUTTON5_CLEAN, &CTendonRobotDlg::OnBnClickedButton5Clean)
END_MESSAGE_MAP()


// CTendonRobotDlg 消息处理程序

BOOL CTendonRobotDlg::OnInitDialog()/**************************************窗口初始化函数中打开CAN口*****************************************/
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标


	// TODO: 在此添加额外的初始化代码
	// 串口选择组合框
	CString str;
	int i;
	for(i=0;i<15;i++)
	{
		str.Format(_T("com %d"),i+1);
		m_comb1.InsertString(i,str);
	}
	m_comb1.SetCurSel(2);//预置COM口

	//波特率选择组合框
	CString str1[]={_T("300"),_T("600"),_T("1200"),_T("2400"),_T("4800"),_T("38400"),
		            _T("19200"),_T("9600"),_T("43000"),_T("56000"),_T("57600"),_T("115200")};
	for(int i=0;i<12;i++)
	{
		int judge_tf=m_comb2.AddString(str1[i]);
		if((judge_tf==CB_ERR)||(judge_tf==CB_ERRSPACE))
			MessageBox(_T("build baud error!"));
	}
	m_comb2.SetCurSel(5);//预置波特率为"9600"


#pragma region set USB-CAN 
	dwRel = OpenDevice(USBCANDevType, USBCANDevIndex, nReserved);

	if (dwRel != STATUS_OK)
	{
		MessageBox(TEXT("USB-CAN Open Failure"),TEXT("Error"), MB_OK|MB_ICONERROR);
		PostQuitMessage(0);
		
	}	
	/*************************************************************/
	InitConfig.Filter=0; /// Inable Filter
	InitConfig.AccCode=0x0000;
	InitConfig.AccMask=0xffff;
	InitConfig.Mode=0;   /// Normal Mode
    InitConfig.Timing0=0x00;/// CAN BaudRate 1000Kbps
	InitConfig.Timing1=0x14;/// CAN BaudRate 1000Kbps

	dwRel = InitCAN (USBCANDevType, USBCANDevIndex, CANIndex, &InitConfig);

	if (dwRel == STATUS_ERR)
	{
		MessageBox(TEXT("USB-CAN Initialization Failure"),TEXT("Error"), MB_OK|MB_ICONERROR);
		CloseDevice(USBCANDevType, USBCANDevIndex);
		PostQuitMessage(0);
	}	   
	/*************************************************************/
	dwRel = StartCAN (USBCANDevType, USBCANDevIndex, CANIndex);

	if (dwRel == STATUS_ERR)
	{
		MessageBox(TEXT("USB-CAN Start Failure"),TEXT("Error"), MB_OK|MB_ICONERROR);
		CloseDevice(USBCANDevType, USBCANDevIndex);
		PostQuitMessage(0);
	}	   
	/*************************************************************/
	dwRel = ClearBuffer(USBCANDevType, USBCANDevIndex, CANIndex);
		if (dwRel == STATUS_ERR)
	{
		MessageBox(TEXT("USB-CAN Clean Buffer Failure"),TEXT("Error"), MB_OK|MB_ICONERROR);
		CloseDevice(USBCANDevType, USBCANDevIndex);
		PostQuitMessage(0);
	}	   
	/*************************************************************/

#pragma endregion

	SendStepper[0].RemoteFlag=0;
	SendStepper[0].SendType=0;
	SendStepper[0].ExternFlag=1; // 步进电机驱动是CAN扩展数据帧

	// 一些控件变量值得初始化
    m_ControlMode.SetCurSel(1);
	m_ControlVariable.SetCurSel(1);
	m_theta1.SetRange(0,100);
	m_delta1.SetRange(0,100);
	m_theta2.SetRange(0,100);
	m_delta2.SetRange(0,100);
	m_x.SetRange(0,100);
	m_y.SetRange(0,100);
	m_z.SetRange(0,100);

	m_theta1.SetTicFreq(20);
	m_delta1.SetTicFreq(20);
	m_theta2.SetTicFreq(20);
	m_delta2.SetTicFreq(20);

	m_theta1.SetPos(100);  //滑块控件的默认初始位置
	m_delta1.SetPos(50);
	m_theta2.SetPos(100);
	m_delta2.SetPos(50);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CTendonRobotDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CTendonRobotDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CTendonRobotDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CTendonRobotDlg::OnEnChangeEdit1()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(true);
	if (!m_Length.IsEmpty())
	{
		
		BackboneLength=_wtof(m_Length.GetBuffer());
		
	}
	UpdateData(false);	
}



void CTendonRobotDlg::OnEnChangeEdit2()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(true);
	if (!m_Radius.IsEmpty())
	{
		
		PlateRadius=_wtof(m_Radius.GetBuffer());
		
	}
	UpdateData(false);

	

}


void CTendonRobotDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	if(GetDlgItem(IDC_COMBO1)->IsWindowEnabled())
	{

	#pragma region Initialize Motor & DAQ

		SendMotor[0].DataLen=8;SendMotor[1].DataLen=8;SendMotor[2].DataLen=8;
		SendMotor[3].DataLen=8;SendMotor[4].DataLen=8;SendMotor[5].DataLen=8;

		SendMotor[0].SendType=0;SendMotor[1].SendType=0;SendMotor[2].SendType=0;
		SendMotor[3].SendType=0;SendMotor[4].SendType=0;SendMotor[5].SendType=0;

		SendMotor[0].RemoteFlag=0;SendMotor[1].RemoteFlag=0;SendMotor[2].RemoteFlag=0;
		SendMotor[3].RemoteFlag=0;SendMotor[4].RemoteFlag=0;SendMotor[5].RemoteFlag=0;

		SendMotor[0].ExternFlag=0;SendMotor[1].ExternFlag=0;SendMotor[2].ExternFlag=0;
		SendMotor[3].ExternFlag=0;SendMotor[4].ExternFlag=0;SendMotor[5].ExternFlag=0;

		SendDAQ[0].DataLen=8;SendDAQ[1].DataLen=8;SendDAQ[2].DataLen=8;
		SendDAQ[3].DataLen=8;SendDAQ[4].DataLen=8;SendDAQ[5].DataLen=8;

		SendDAQ[0].SendType=0;SendMotor[1].SendType=0;SendMotor[2].SendType=0;
		SendDAQ[3].SendType=0;SendMotor[4].SendType=0;SendMotor[5].SendType=0;

		SendDAQ[0].RemoteFlag=0;SendMotor[1].RemoteFlag=0;SendMotor[2].RemoteFlag=0;
		SendDAQ[3].RemoteFlag=0;SendMotor[4].RemoteFlag=0;SendMotor[5].RemoteFlag=0;

		SendDAQ[0].ExternFlag=0;SendDAQ[1].ExternFlag=0;SendDAQ[2].ExternFlag=0;
		SendDAQ[3].ExternFlag=0;SendDAQ[4].ExternFlag=0;SendDAQ[5].ExternFlag=0;
		

		/*Initial Data*/
		memcpy(SendMotor[0].Data,initData,sizeof(initData));
		memcpy(SendMotor[1].Data,initData,sizeof(initData));
		memcpy(SendMotor[2].Data,initData,sizeof(initData));
		memcpy(SendMotor[3].Data,initData,sizeof(initData));
		memcpy(SendMotor[4].Data,initData,sizeof(initData));
		memcpy(SendMotor[5].Data,initData,sizeof(initData));

		memcpy(SendDAQ[0].Data,initData2,sizeof(initData2));
		memcpy(SendDAQ[1].Data,initData2,sizeof(initData2));
		memcpy(SendDAQ[2].Data,initData2,sizeof(initData2));
		memcpy(SendDAQ[3].Data,initData2,sizeof(initData2));
		memcpy(SendDAQ[4].Data,initData2,sizeof(initData2));
		memcpy(SendDAQ[5].Data,initData2,sizeof(initData2));

		SendDAQ[0].ID=0x301;SendDAQ[1].ID=0x301;SendDAQ[2].ID=0x301;
	    SendDAQ[3].ID=0x301;SendDAQ[4].ID=0x301;SendDAQ[5].ID=0x301;

		SendDAQ[0].Data[1]=unsigned char(1);SendDAQ[1].Data[1]=unsigned char(2);SendDAQ[2].Data[1]=unsigned char(3);// 将传感器信号接入DAQ的0-5号AD采集通道（一共19个通道）
	    SendDAQ[3].Data[1]=unsigned char(4);SendDAQ[4].Data[1]=unsigned char(5);SendDAQ[5].Data[1]=unsigned char(6);  
		

   		// Reset Motor
		SendMotor[0].ID=0x000;
		dwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendMotor,1);
		Sleep(1000); // Waiting for the resetting 

		// Receive Setting
		SendMotor[0].ID=0x00A;
		SendMotor[0].Data[0]=0x0A;// 数据上传周期 10ms 
		SendMotor[0].Data[1]=0x00;// 关闭左右限位功能
		critical_section2.Lock();
		dwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendMotor,1);
		critical_section2.Unlock();
		Sleep(1000); 
		/*****************************************************************************/

		if (MotorMode==1)
		{
			SendMotor[0].ID=0x001;
			SendMotor[0].Data[0]=0x02;
			SendMotor[0].Data[1]=0x55;// 电流模式
			critical_section2.Lock();
			dwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendMotor,1);
			critical_section2.Unlock();
			Sleep(1000); 

			SendMotor[0].ID=0x013;SendMotor[1].ID=0x023;SendMotor[2].ID=0x033;
			SendMotor[3].ID=0x043;SendMotor[4].ID=0x053;SendMotor[5].ID=0x063;
		}

		else
		{
			SendMotor[0].ID=0x001;
			SendMotor[0].Data[0]=0x03;
			SendMotor[0].Data[1]=0x55;// 速度模式
			critical_section2.Lock();
			dwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendMotor,1);
			critical_section2.Unlock();
			Sleep(1000); 

			SendMotor[0].ID=0x014;SendMotor[1].ID=0x024;SendMotor[2].ID=0x034;
			SendMotor[3].ID=0x044;SendMotor[4].ID=0x054;SendMotor[5].ID=0x064;

		}
		#pragma endregion 

		tendoncontinuum.SetRobot(BackboneLength,PlateRadius,0,0,0,0);

		GetDlgItem(IDC_COMBO1)->EnableWindow(FALSE);
		GetDlgItem(IDC_COMBO2)->EnableWindow(FALSE);
		GetDlgItem(IDC_EDIT1)->EnableWindow(FALSE);
		GetDlgItem(IDC_EDIT2)->EnableWindow(FALSE);
		GetDlgItem(IDC_BUTTON1)->SetWindowText(TEXT("Recharge and ReInitialization"));
		
	}
	else
	{
		GetDlgItem(IDC_COMBO1)->EnableWindow(TRUE);
		GetDlgItem(IDC_COMBO2)->EnableWindow(TRUE);
		GetDlgItem(IDC_EDIT1)->EnableWindow(TRUE);
		GetDlgItem(IDC_EDIT2)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON1)->SetWindowText(TEXT("Initialization"));
	}

	if(ControlVariable==1)
	{
		GetDlgItem(IDC_SLIDER1)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER2)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER3)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER4)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER5)->EnableWindow(TRUE);
		GetDlgItem(IDC_SLIDER6)->EnableWindow(TRUE);
		GetDlgItem(IDC_SLIDER7)->EnableWindow(TRUE);	
	}
	else
	{
		GetDlgItem(IDC_SLIDER1)->EnableWindow(TRUE);
		GetDlgItem(IDC_SLIDER2)->EnableWindow(TRUE);
		GetDlgItem(IDC_SLIDER3)->EnableWindow(TRUE);
		GetDlgItem(IDC_SLIDER4)->EnableWindow(TRUE);
		GetDlgItem(IDC_SLIDER5)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER6)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER7)->EnableWindow(FALSE);
	}


	if((readtxt=fopen("C:/Users/lenovo/Desktop/Length_Record.txt","r"))==NULL) //a.用反斜杠？？//b.全地址or文件名根地址
	//if((readtxt=fopen("Length_Record.txt","r"))==NULL) //a.用反斜杠？？//b.全地址or文件名根地址
	{
		writetxt=fopen("C:/Users/lenovo/Desktop/Length_Record.txt","w");  
		//writetxt=fopen("Length_Record.txt","w"); 
		fprintf(writetxt,"%lf %lf %lf %lf %lf %lf",BackboneLength,BackboneLength,BackboneLength,2*BackboneLength,2*BackboneLength,2*BackboneLength);
		fclose(writetxt);
		ALength[0]=BackboneLength;ALength[1]=BackboneLength;ALength[2]=BackboneLength;
		ALength[3]=2*BackboneLength;ALength[4]=2*BackboneLength;ALength[5]=2*BackboneLength;
	}
	else
	{
		fscanf(readtxt,"%lf %lf %lf %lf %lf %lf",&ALength[0],&ALength[1],&ALength[2],&ALength[3],&ALength[4],&ALength[5]);
	    fclose(readtxt);
	}

	int i;
	for (i=0;i<6;i++)
	{
        initialALength[i]=ALength[i];
	}

}


void CTendonRobotDlg::OnCbnSelchangeCombo1()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	if(m_ControlMode.GetCurSel()==0)
	{
		MotorMode=1;
	}
	else 
	{
		MotorMode=2;
	}
	UpdateData(FALSE);

	
}


void CTendonRobotDlg::OnCbnSelchangeCombo2()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	if(m_ControlVariable.GetCurSel()==0)
	{
		ControlVariable=1;
	}
	else 
	{
		ControlVariable=2;
	}
	UpdateData(FALSE);

}


void CTendonRobotDlg::OnBnClickedButton3()
{
	// TODO: 在此添加控件通知处理程序代码
	pThread2=AfxBeginThread(GraspThread,NULL);
}


void CTendonRobotDlg::OnNMCustomdrawSlider7(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;
}


void CTendonRobotDlg::OnNMCustomdrawSlider5(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;
}


void CTendonRobotDlg::OnNMCustomdrawSlider6(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;
}


void CTendonRobotDlg::OnNMCustomdrawSlider1(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;

	n_1= _ttoi(m_2);

	UpdateData(TRUE);  //将控件的值赋值给成员变量,外部输入值交给内部变量
	
	//t1=M_PI/6+double(m_theta1.GetPos())*M_PI/300;  //把0-100换算成对应弧度制
	
	 t1= M_PI/6+double(n_1)*M_PI/3069;  //把0-1023换算成对应弧度制

	v_theta1.Format(_T("Theta_1: %.2lf"),t1);  //编辑框显示弧度值
	UpdateData(FALSE);    //将成员变量的值赋值给控件，拷贝变量值到控件显示
} 


void CTendonRobotDlg::OnNMCustomdrawSlider2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;

	n_3= _ttoi(m_4);

	UpdateData(TRUE);
	
	//d1=-M_PI+double(m_delta1.GetPos())*M_PI/50;
	d1=-M_PI+double(n_3)*2*M_PI/1023;  //把0-1023换算成对应弧度制

	v_delta1.Format(_T(" Delta_1: %.2lf"),d1);
	UpdateData(FALSE);
}


void CTendonRobotDlg::OnNMCustomdrawSlider3(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;

	n_2= _ttoi(m_3);

	UpdateData(TRUE);

	//t2=M_PI/6+double(m_theta2.GetPos())*M_PI/300;

	t2= M_PI/6+double(n_2)*M_PI/3069;  //把0-1023换算成对应弧度制

	v_theta2.Format(_T("Theta_2: %.2lf"),t2);
	UpdateData(FALSE);
}


void CTendonRobotDlg::OnNMCustomdrawSlider4(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;

	n_4= _ttoi(m_5);

	UpdateData(TRUE);

	//d2=-M_PI+double(m_delta2.GetPos())*M_PI/50;

	d2=-M_PI+double(n_4)*2*M_PI/1023;  //把0-1023换算成对应弧度制

	v_delta2.Format(_T(" Delta_2: %.2lf"),d2);
	UpdateData(FALSE);
}

void CTendonRobotDlg::OnBnClickedButton2()
{
	// TODO: 在此添加控件通知处理程序代码

	GetDlgItem(IDC_BUTTON2)->EnableWindow(FALSE);

	if(ControlVariable==2&&GetDlgItem(IDC_SLIDER1)->IsWindowEnabled())
	{
		GetDlgItem(IDC_SLIDER1)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER2)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER3)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER4)->EnableWindow(FALSE);
	}
	if(ControlVariable==1&&GetDlgItem(IDC_SLIDER5)->IsWindowEnabled())
	{
		GetDlgItem(IDC_SLIDER5)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER6)->EnableWindow(FALSE);
		GetDlgItem(IDC_SLIDER7)->EnableWindow(FALSE);	
	}
	
	pThread=AfxBeginThread(MotionThread,NULL); 
	

}


LRESULT CTendonRobotDlg::OnMyMssage1(WPARAM wParam,LPARAM lParam)
{
	 

	 switch(wParam)
	 {
	 case 1:// updata data
		 {
			 
			 /*double* L = (double*)lParam;*/
			 critical_section.Lock();
			 v_L11.Format(_T("%.2lf"),*ShowLength);
			 v_L12.Format(_T("%.2lf"),*(ShowLength+1));
			 v_L13.Format(_T("%.2lf"),*(ShowLength+2));
			 v_L21.Format(_T("%.2lf"),*(ShowLength+3));
			 v_L22.Format(_T("%.2lf"),*(ShowLength+4));
			 v_L23.Format(_T("%.2lf"),*(ShowLength+5));
			 critical_section.Unlock();
			 //critical_section1.Lock();
			 //v_V1.Format(_T("%.2lf"),*DAQvolatge);
			 //v_V2.Format(_T("%.2lf"),*(DAQvolatge+1));
			 //v_V3.Format(_T("%.2lf"),*(DAQvolatge+2));
			 //v_V4.Format(_T("%.2lf"),*(DAQvolatge+3));
			 //v_V5.Format(_T("%.2lf"),*(DAQvolatge+4));
			 //v_V6.Format(_T("%.2lf"),*(DAQvolatge+5));
			 //critical_section1.Unlock();
			 UpdateData(FALSE);
			 break;
		 }
	 case 2: // Transmit Error
		 {
			MessageBox(TEXT("CAN控制器发送错误"),TEXT("Error"), MB_OK|MB_ICONERROR);
			WaitForSingleObject(pThread->m_hThread, -1);
			PostQuitMessage(0);
			break;
		 }
	 case 3: // Receive Error1
		 {
			CString errinfo;
			errinfo.Format(_T("CAN控制器消极错误\n错误代码：0x%X 0x%X 0x%X "),*kerror,*(kerror+1),*(kerror+2));
			/*字符转换,：Unicode环境下CString到LPCSTR再到LPCWSTR*/
			USES_CONVERSION; 
			LPCWSTR lpcwStr = A2CW((LPCSTR)T2A(errinfo));
			/*字符转换结束*/
			MessageBox(lpcwStr,TEXT("Error"), MB_OK|MB_ICONERROR);
			CloseDevice(USBCANDevType, USBCANDevIndex);
			WaitForSingleObject(pThread->m_hThread, -1);
			PostQuitMessage(0);
			break;
		 }
	case 4: // Receive Error2
		{
			CString errinfo;
			errinfo.Format(_T("CAN控制器仲裁丢失\n错误代码：0x%X "),lerror);
			/*字符转换,：Unicode环境下CString到LPCSTR再到LPCWSTR*/
			USES_CONVERSION; 
			LPCWSTR lpcwStr = A2CW((LPCSTR)T2A(errinfo));
			/*字符转换结束*/
			MessageBox(lpcwStr,TEXT("Error"), MB_OK|MB_ICONERROR);
			CloseDevice(USBCANDevType, USBCANDevIndex);
			WaitForSingleObject(pThread->m_hThread,-1);
			PostQuitMessage(0);
			break;
		}
	case 5: // Receive Error3
		{
			CString errinfo;
			errinfo.Format(_T("CAN控制器一般错误\n错误代码：0x%X "),error);
			/*字符转换,：Unicode环境下CString到LPCSTR再到LPCWSTR*/
			USES_CONVERSION; 
			LPCWSTR lpcwStr = A2CW((LPCSTR)T2A(errinfo));
			/*字符转换结束*/
			MessageBox(lpcwStr,TEXT("Error"), MB_OK|MB_ICONERROR);
			CloseDevice(USBCANDevType, USBCANDevIndex);
			WaitForSingleObject(pThread->m_hThread, -1);
			PostQuitMessage(0);	
			break;
		}
	case 6: // Enable Slider
		{
			GetDlgItem(IDC_BUTTON2)->EnableWindow(TRUE);

			if(ControlVariable==2&&!(GetDlgItem(IDC_SLIDER1)->IsWindowEnabled()))
			{
				GetDlgItem(IDC_SLIDER1)->EnableWindow(TRUE);
				GetDlgItem(IDC_SLIDER2)->EnableWindow(TRUE);
				GetDlgItem(IDC_SLIDER3)->EnableWindow(TRUE);
				GetDlgItem(IDC_SLIDER4)->EnableWindow(TRUE);
			}
			if(ControlVariable==1&&!(GetDlgItem(IDC_SLIDER5)->IsWindowEnabled()))
			{
				GetDlgItem(IDC_SLIDER5)->EnableWindow(TRUE);
				GetDlgItem(IDC_SLIDER6)->EnableWindow(TRUE);
				GetDlgItem(IDC_SLIDER7)->EnableWindow(TRUE);	
			}
				break;
		}
	 }

     return 0;
}

UINT MotionThread(LPVOID lpParam)
{
	int i;
	signed short vc[6];
	Vector3d displacement_P;
	Vector4d displacement_C, C;
	VectorXd Velocity;
	MatrixXd Path;
	
	C=tendoncontinuum.Configuration(ALength[0],ALength[1],ALength[2],ALength[3],ALength[4],ALength[5]);

	if (ControlVariable==1)
	{
		Path=tendoncontinuum.TrajectoryGeneration(displacement_P(0),displacement_P(1),displacement_P(2),T,F);
	}

	else
	{
		displacement_C(0)=t1-C(0);
		displacement_C(1)=d1-C(1);
		displacement_C(2)=t2-C(2);
		displacement_C(3)=d2-C(3);
		Path=tendoncontinuum.TrajectoryGeneration(displacement_C(0),displacement_C(1),displacement_C(2),displacement_C(3),T,F);
	}

	int k=0;
	clock_t time;

#pragma region loop

	Vector4d dP,P;

	while(k<T*F)
	{
		C=tendoncontinuum.Configuration(ALength[0],ALength[1],ALength[2],ALength[3],ALength[4],ALength[5]);
		if (k==0)
        P << C(0),C(1),C(2),C(3);
		else
		{
		dP<< Path(0,k)/F, Path(1,k)/F, Path(2,k)/F, Path(3,k)/F;
		P=P+dP;
		}

		Velocity=tendoncontinuum.JacobianCL(C(0),C(1),C(2),C(3))*(Path.col(k)+5*(P-C));
		Velocity=-Velocity/2*33.25*60;//负号：电机正反向与丝杠正方向相反
		vc[0]=(signed short)Velocity(0);
		vc[1]=(signed short)Velocity(1);
		vc[2]=(signed short)Velocity(2);
		vc[3]=(signed short)Velocity(3);
		vc[4]=(signed short)Velocity(4);
		vc[5]=(signed short)Velocity(5);
				 
		
		for (i=0;i<6;i++)
		{
			SendMotor[i].Data[0]=unsigned char((PWM>>8)&0xff);
			SendMotor[i].Data[1]=unsigned char(PWM&0xff);
			SendMotor[i].Data[2]=unsigned char((vc[i]>>8)&0xff);
			SendMotor[i].Data[3]=unsigned char(vc[i]&0xff);
		}

		
		critical_section2.Lock();
		TdwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendMotor,6);//// Transmit
		critical_section2.Unlock();

		if(TdwRel!=6)
		{
			if(Getdata())
			{
				writetxt=fopen("Length_Record.txt","w+");
				fprintf(writetxt,"%lf %lf %lf %lf %lf %lf",ALength[0],ALength[1],ALength[2],ALength[3],ALength[4],ALength[5]);
				fclose(writetxt);
			}
			mypWnd= AfxGetMainWnd();
			PostMessage((HWND)(mypWnd->GetSafeHwnd()),WM_MYMESSAGE1,2,NULL);
			break;
		}
		else
		{
			time=clock();
			k++;
			while((clock()-time)<1000/F){};// 时钟计时
			if(Getdata())
			{
				writetxt=fopen("C:/Users/lenovo/Desktop/Length_Record.txt","w+");
				//writetxt=fopen("Length_Record.txt","w+");
				fprintf(writetxt,"%lf %lf %lf %lf %lf %lf",ALength[0],ALength[1],ALength[2],ALength[3],ALength[4],ALength[5]);
				fclose(writetxt);
				mypWnd= AfxGetMainWnd();

			critical_section.Lock();
				for (i=0;i<6;i++)
				{
					ShowLength[i]=ALength[i];
				}
			critical_section.Unlock();
			PostMessage((HWND)(mypWnd->GetSafeHwnd()),WM_MYMESSAGE1,1,NULL);
			}

			else break;
		}

	}
#pragma endregion

	 if((TdwRel==6))
	 {
		 StopMotor();
	 }

	 PostMessage((HWND)(mypWnd->GetSafeHwnd()),WM_MYMESSAGE1,6,NULL);
	 return 0; 
}

void StopMotor()
{
     signed short vc[6], PWM=5000;
	 vc[0]=0;vc[1]=0;vc[2]=0;vc[3]=0;vc[4]=0;vc[5]=0;

	 int i;

	 for (i=0;i<6;i++)///!!!!!!!!!!!!!!!!!!!!!!!!!??????????????????????????????????
	 {
		 SendMotor[i].Data[0]=unsigned char((PWM>>8)&0xff);
		 SendMotor[i].Data[1]=unsigned char(PWM&0xff);
		 SendMotor[i].Data[2]=unsigned char((vc[i]>>8)&0xff);
		 SendMotor[i].Data[3]=unsigned char(vc[i]&0xff);
	 }
	 critical_section2.Lock();
	 TdwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendMotor,6);//// Transmit
	 critical_section2.Unlock();
	 Sleep(20);
	/*记录电机位置*/
}

int Getdata(void)
{
	     signed int Length_Record[6];// 位置记录(用脉冲数记录)
	     //double c,v;
		 int CanReceiveNum;
		 UINT  motorcount=0,  DAQcount=0; // 用count的后6位01状态来判别，0未读，1已读
		 int i=0,mi=0;

		 //for (i=0;i<6;i++)///
		 //{
			//SendDAQ[i].Data[0]=unsigned char(5);
		 //   SendDAQ[i].Data[1]=unsigned char(i+1);
			//dwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendDAQ+i,1);//// Transmit
			//Sleep(5); //DAQ的最小数据请求周期4ms
		 //}
		 //i=0;
		 critical_section2.Lock();
		 CanReceiveNum = GetReceiveNum(USBCANDevType, USBCANDevIndex, CANIndex);
		 dwRel = Receive(USBCANDevType, USBCANDevIndex, CANIndex,Receivebuff,CanReceiveNum,1000);/// Receive
		 critical_section2.Unlock();

		 if(dwRel!=0xFFFFFFFF)
		 {
			    /************************ Get Motor Data *********************************/


				while ((motorcount!=63)&&(DAQcount!=63))//count!=63 i<2x6 for six motors
				 { 
					 //if (Receivebuff[dwRel-1-i].ID==0x301)
					 //{
						//if ((DAQcount>>((Receivebuff[dwRel-1-i].Data[1])-1))&1) // 判断该DAQ是否已读
						//{
						//	i++;
						//	continue;
					 //   }
						//else
						//{
						//	ReceiveDAQ[(Receivebuff[dwRel-1-i].Data[1])-1]=Receivebuff[dwRel-1-i];
						//	DAQcount=(((DAQcount>>((Receivebuff[dwRel-1-i].Data[1])-1))+1)<<((Receivebuff[dwRel-1-i].Data[1])-1))|DAQcount;
						//	i++;
						//	continue; 
						//}
					 //}
					 if (mi<6)
					 {
						 if ((motorcount>>((Receivebuff[dwRel-1-i].ID>>4)-1))&1) // 判断该电机是否已读
						 {
							 i++;
							 continue;
						 }
						 else
						 {
							 ReceiveMotor[(Receivebuff[dwRel-1-i].ID>>4)-1]=Receivebuff[dwRel-1-i]; // 按顺序储存未读的电机数据
							 motorcount=(((motorcount>>((Receivebuff[dwRel-1-i].ID>>4)-1))+1)<<((Receivebuff[dwRel-1-i].ID>>4)-1))|motorcount; // 标志已读电机号
							 mi++;
							 i++;
							 continue;
						 }
					 }
					 //else if (mi==6)
					 //{
						// i++;
						// continue;
					 //}

				}

				for(i=0;i<6;i++)// i<6 for six motors
				{
					//c=(double)(ReceiveMotor[i].Data[0]<<8|ReceiveMotor[i].Data[1]);
					//v=(double)(ReceiveMotor[i].Data[2]<<8|ReceiveMotor[i].Data[3]);
					//v=-2*v/33.25/60;
					Length_Record[i]=ReceiveMotor[i].Data[4]<<24|ReceiveMotor[i].Data[5]<<16|ReceiveMotor[i].Data[6]<<8|ReceiveMotor[i].Data[7];
					ALength[i]=initialALength[i]-(double)(2*Length_Record[i])/2000/33.25; //螺距2mm//减号：电机正反向与丝杠正方向相反/2000=500线*4分频

					//// 转换成DAQ电压值（0V-3.3V）
					//critical_section1.Lock();
					//DAQvolatge[i]=(double)(ReceiveDAQ[i].Data[2]<<8|ReceiveDAQ[i].Data[3])*3.3/4096; 
					//critical_section1.Unlock();					
				}


				return 1;						 	 	
		 }

		 else
		 {
			 critical_section2.Lock();
			 ReadErrInfo(USBCANDevType, USBCANDevIndex, CANIndex,perror_info);
			 critical_section2.Unlock();
			 ErrorExplain(perror_info);	
			 return 0;
		 }
}

void ErrorExplain(P_ERR_INFO perror_info)
{

	error=perror_info->ErrCode;
	if (error==0x00000004)
	{
		kerror[0]=perror_info->Passive_ErrData[0];
		kerror[1]=perror_info->Passive_ErrData[1];
		kerror[2]=perror_info->Passive_ErrData[2];
		mypWnd= AfxGetMainWnd();
		PostMessage((HWND)(mypWnd->GetSafeHwnd()),WM_MYMESSAGE1,3,NULL);
	}
	else if (error==0x00000008)
    {
		lerror=perror_info->ArLost_ErrData;
		mypWnd= AfxGetMainWnd();
		PostMessage((HWND)(mypWnd->GetSafeHwnd()),WM_MYMESSAGE1,4,NULL);

	}
    else  
    {
		mypWnd= AfxGetMainWnd();
		PostMessage((HWND)(mypWnd->GetSafeHwnd()),WM_MYMESSAGE1,5,NULL);
	}
}/*释错函数*/

UINT GraspThread(LPVOID lpParam)
{
	// Enable Stepper
	SendStepper[0].DataLen=0;
	SendStepper[0].ID=0x1D086001;
	critical_section2.Lock();
	dwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendStepper,1);
	critical_section2.Unlock();
	Sleep(50);
	// Grasp or Release
	if (isgrasp)
	{
		SendStepper[0].DataLen=4;
		SendStepper[0].ID=0x1D086007;
		memcpy(SendStepper[0].Data,release,sizeof(release));
		critical_section2.Lock();
		dwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendStepper,1);
		critical_section2.Unlock();
		Sleep(3000);
		isgrasp=false;
	}
	else
	{
		SendStepper[0].DataLen=4;
		SendStepper[0].ID=0x1D086007;
		memcpy(SendStepper[0].Data,grasp,sizeof(grasp));
		critical_section2.Lock();
		dwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendStepper,1);
		critical_section2.Unlock();
		Sleep(3000);
		isgrasp=true;
	}
	// Disable Stepper
	SendStepper[0].DataLen=0;
	SendStepper[0].ID=0x1D086002;
	dwRel=Transmit(USBCANDevType, USBCANDevIndex, CANIndex,SendStepper,1);
	critical_section2.Unlock();
	Sleep(50);

   return 0;
}



void CTendonRobotDlg::OnBnClickedButton4Open()
{
	// TODO: 在此添加控件通知处理程序代码
    CString str,str1,n;					//定义字符串
	GetDlgItemText(IDC_BUTTON4_OPEN,str);
	CWnd *h1;
	h1=GetDlgItem(IDC_BUTTON4_OPEN);		//指向控件的caption
	
	if(!m_mscom.get_PortOpen())
	{
		m_comb2.GetLBText(m_comb2.GetCurSel(),str1);//取得所选的字符串，并存放在str1里面
		str1=str1+','+'n'+','+'8'+','+'1';			//这句话很关键
		
		m_mscom.put_CommPort((m_comb1.GetCurSel()+1));	//选择串口
		m_mscom.put_InputMode(1);			//设置输入方式为二进制方式
		m_mscom.put_Settings(str1);		//波特率为（波特率组Á合框）无校验，8数据位，1个停止位
		m_mscom.put_InputLen(1024);		//设置当前接收区数据长度为1024
		m_mscom.put_RThreshold(1);			//缓冲区一个字符引发事件
		m_mscom.put_RTSEnable(1);			//设置RT允许
			
		m_mscom.put_PortOpen(true);		//打开串口
		if(m_mscom.get_PortOpen())
		{
			str=_T("Close");
			UpdateData(true);
			h1->SetWindowText(str);			//改变按钮名称为“关闭串口”
		}		
	}	
	
	else 
	{
		m_mscom.put_PortOpen(false);
		if(str!=_T("Open"))
		{
			str=_T("Open");
			UpdateData(true);
			h1->SetWindowText(str);			//改变按钮名称为打开串口
		}
	}
}


void CTendonRobotDlg::OnBnClickedButton5Clean()
{
	// TODO: 在此添加控件通知处理程序代码
	m_EditReceive=_T("");	//给接收编辑框发送空格符
	UpdateData(false);		//更新数据
}


BEGIN_EVENTSINK_MAP(CTendonRobotDlg, CDialogEx)
	ON_EVENT(CTendonRobotDlg, IDC_MSCOMM1, 1, CTendonRobotDlg::OnCommMscomm1, VTS_NONE)
END_EVENTSINK_MAP()


void CTendonRobotDlg::OnCommMscomm1()
{
	// TODO: 在此处添加消息处理程序代码
	if(m_mscom.get_CommEvent()==2)
	{
	
		char str[1024]={0};
		//char str1[1024]={0};
		//char n='n';
		long k;

		VARIANT InputData=m_mscom.get_Input();	//读缓冲区
		COleSafeArray fs;
		fs=InputData;	//VARIANT型变À量转换为COleSafeArray型变量 
		
		for(k=0;k<fs.GetOneDimSize();k++)
		fs.GetElement(&k,str+k);	//转换为BYTE型数组
		
		//for(m=0;str[m]!=n;m++)
			//str1[m]=str[m];
		
		m_1+=str;                 //	接收到m_1变量中 完整的数据帧

		//提取下关节圆心角数据
		m_2= m_1.SpanExcluding(_T("y"));  //遇到参数里面有的字符即停下，返回停下之前的字符串
       
		//提取上关节圆心角数据
		int m1=m_1.FindOneOf(_T("y"));
		int m2=m_1.FindOneOf(_T("x"));
		m_3=m_1.Mid(m1+1,m2-m1-1);
		
		//提取下关节偏转角数据
		int m3=m_1.FindOneOf(_T("w"));
		m_4=m_1.Mid(m2+1,m3-m2-1);
        

		//提取下关节偏转角数据
		int m4=m_1.FindOneOf(_T("e"));
		m_5=m_1.Mid(m3+1,m4-m3-1);

		//显示所有数据
        m_EditReceive=m_2+" "+m_3+" "+m_4+" "+m_5;

		 //SetTimer(1,1000,NULL);		//延时10ms
		UpdateData(false);
	}
}
