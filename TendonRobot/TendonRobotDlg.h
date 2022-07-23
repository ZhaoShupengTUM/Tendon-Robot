
// TendonRobotDlg.h : 头文件
//

#pragma once
#include "afxwin.h"
#include "mscomm1.h"


// CTendonRobotDlg 对话框

#define WM_MYMESSAGE1 WM_USER+102 /*自定义消息*/

UINT MotionThread(LPVOID lpParam);//运动线程
UINT GraspThread(LPVOID lpParam);//抓取线程

class CTendonRobotDlg : public CDialogEx
{
// 构造
public:
	CTendonRobotDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_TENDONROBOT_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	CWinThread* pThread;//控制线程指针
	CWinThread* pThread2;//抓持线程指针

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
	afx_msg LRESULT OnMyMssage1(WPARAM wParam,LPARAM lParam);/*自定义消息*/
public:
	afx_msg void OnEnChangeEdit1();
	afx_msg void OnEnChangeEdit2();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnCbnSelchangeCombo1();
	afx_msg void OnCbnSelchangeCombo2();
	afx_msg void OnBnClickedButton2();
	CComboBox m_ControlMode;
	CComboBox m_ControlVariable;
	afx_msg void OnBnClickedButton3();
	CString m_Length;
	CString m_Radius;
	afx_msg void OnNMCustomdrawSlider7(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnNMCustomdrawSlider5(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnNMCustomdrawSlider6(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnNMCustomdrawSlider1(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnNMCustomdrawSlider2(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnNMCustomdrawSlider3(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnNMCustomdrawSlider4(NMHDR *pNMHDR, LRESULT *pResult);
	CSliderCtrl m_theta1;
	CSliderCtrl m_delta1;
	CSliderCtrl m_theta2;
	CSliderCtrl m_delta2;
	CSliderCtrl m_x;
	CSliderCtrl m_y;
	CSliderCtrl m_z;
	CString v_theta1;
	CString v_theta2;
	CString v_delta2;
	CString v_delta1;
	CString v_L11;
	CString v_L12;
	CString v_L13;
	CString v_L21;
	CString v_L22;
	CString v_L23;
	CString v_V1;
	CString v_V2;
	CString v_V3;
	CString v_V4;
	CString v_V6;
	CString v_V5;

	afx_msg void OnStnClickedStatict1();
	afx_msg void OnStnClickedStaticd1();
	CString m_EditReceive;
private:
	CComboBox m_comb1;
	CComboBox m_comb2;
public:
	CMscomm1 m_mscom;
	afx_msg void OnBnClickedButton4Open();
	afx_msg void OnBnClickedButton5Clean();
	DECLARE_EVENTSINK_MAP()
	void OnCommMscomm1();
	CString m_1;
	CString m_2;
	CString m_3;
	CString m_4;
	CString m_5;
	int n_1;
	int n_2;
	int n_3;
	int n_4;
};
