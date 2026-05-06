// --------------------------------------------------------------
//
//  Thinkpad Fan Control
//
// --------------------------------------------------------------
//
//	This program and source code is in the public domain.
//
//	The author claims no copyright, copyleft, license or
//	whatsoever for the program itself (with exception of
//	WinIO driver).  You may use, reuse or distribute it's 
//	binaries or source code in any desired way or form,  
//	Useage of binaries or source shall be entirely and 
//	without exception at your own risk. 
// 
// --------------------------------------------------------------

#ifndef FANCONTROL_H
#define FANCONTROL_H

#include "_prec.h"

#pragma once

#include "winstuff.h"
#include "TaskbarTextIcon.h"

constexpr const char* FANCONTROLVERSIONS = "2.3.13 Single Fan";
constexpr const char* FANCONTROLVERSIOND = "2.3.13 Dual Fan";

//Pipe name format - \\.\pipe\pipename
#define g_szPipeName "\\\\.\\Pipe\\TPFanControl01"  //Name given to the pipe

#define WM__DISMISSDLG (WM_USER+5)
#define WM__GETDATA    (WM_USER+6)
#define WM__NEWDATA    (WM_USER+7)
#define WM__TASKBAR    (WM_USER+8)

#define setzero(adr, size) memset((void*)(adr), (char)0x00, (size))
#define ARRAYMAX(tab) (sizeof(tab)/sizeof((tab)[0]))
#define NULLSTRUCT    { 0, }

constexpr auto BUFFER_SIZE = 1024; // 1k
constexpr auto ACK_MESG_RECV = "Message received successfully";

class FANCONTROL {
protected:
	HINSTANCE hinstapp;
	HINSTANCE m_hinstapp;
	HWND hwndDialog;

	UINT_PTR m_fanTimer;
	UINT_PTR m_titleTimer;
	UINT_PTR m_iconTimer;
	UINT_PTR m_renewTimer;

	struct FCSTATE {

		char FanCtrl,
			Fan1SpeedLo,
			Fan1SpeedHi,
			Fan2SpeedLo,
			Fan2SpeedHi;

		char Sensors[12];
		int SensorAddr[12];
		const char* SensorName[12];

	} State;

	struct SMARTENTRY {
		int temp, fan, hystUp, hystDown;
	} SmartLevels[32];

	struct SMARTENTRY1 {
		int temp1, fan1, hystUp1, hystDown1;
	} SmartLevels1[32];

	struct SMARTENTRY2 {
		int temp2, fan2, hystUp2, hystDown2;
	} SmartLevels2[32];

	struct FSMARTENTRY {        //fahrenheit values
		int ftemp, ffan;
	} FSmartLevels[32];

	struct SENSOROFFSET {
		int offs, hystMin, hystMax; // min and max temp values that offs takes effect. -1 to disable
	} SensorOffset[16];

	int IconLevels[3];    // temp levels for coloring the icon
	int FIconLevels[3];    // fahrenheit temp levels for coloring the icon
	int CurrentIcon;
	int IndSmartLevel;
	int FSensorOffset[16];
	int iFarbeIconB;
	int iFontIconB;
	int icontemp;
	int Cycle;
	int IconCycle;
	int ReIcCycle;
	int NoExtSensor;
	int FanSpeedLowByte;
	int ActiveMode;
	int SingleFan;
	int PowerSuspendMode;
	int ModernS0Mode;
	int UseTWR,
		ManFanSpeed,
		FinalSeen;
	int CurrentMode, fanctrl2,
		PreviousMode;
	int TaskbarNew;
	int MaxTemp;
	int iMaxTemp;
	int fan1speed, lastfan1speed, fan2speed, lastfan2speed;
	int FanBeepFreq, FanBeepDura;
	int MinimizeToSysTray,
		Lev64Norm,
		IconColorFan,
		Fahrenheit,
		MinimizeOnClose,
		StartMinimized,
		NoWaitMessage,
		Runs_as_service;
	int ReadErrorCount;
	int MaxReadErrors;
	int SecWinUptime;
	int SecStartDelay;
	int SlimDialog;
	int NoBallons,
		HK_BIOS_Method,
		HK_Manual_Method,
		HK_Smart_Method,
		HK_SM1_Method,
		HK_SM2_Method,
		HK_TG_BS_Method,
		HK_TG_BM_Method,
		HK_TG_MS_Method,
		HK_TG_12_Method,
		HK_BIOS,
		HK_Manual,
		HK_Smart,
		HK_SM1,
		HK_SM2,
		HK_TG_BS,
		HK_TG_BM,
		HK_TG_MS,
		HK_TG_12;
	int BluetoothEDR;
	int ManModeExit;
	int ManModeExitInternal;
	int ShowBiasedTemps;
	char gSensorNames[17][4];
	int Log2File;
	int Log2csv;
	int StayOnTop;
	int ShowAll;
	int ShowTempIcon;
	char IgnoreSensors[256];
	char MenuLabelSM1[32];
	char MenuLabelSM2[32];
	HANDLE hThread;
	static constexpr int NUM_PIPES = 8;
	HANDLE hPipes[NUM_PIPES];
	HANDLE hLock;
	HANDLE hLockS;
	BOOL Closing;
	MUTEXSEM EcAccess;
	bool m_needClose;

	char Title[128];
	char Title2[128];
	char LastTitle[128];
	char LastTooltip[128];
	char CurrentStatus[256];
	char CurrentStatuscsv[256];

	// dialog.cpp

	int CurrentModeFromDialog();

	int ShowAllFromDialog();

	void ModeToDialog(int mode) const;

	void ShowAllToDialog(int mode) const;

	ULONG DlgProc(HWND hwnd, ULONG msg, WPARAM mp1, LPARAM mp2);

	static ULONG CALLBACK
		BaseDlgProc(HWND
			hwnd,
			ULONG msg, WPARAM
			mp1,
			LPARAM mp2
		);

	//The default app-icon with changing colors
	TASKBARICON* pTaskbarIcon;
	//
	CTaskbarTextIcon** ppTbTextIcon;
	MUTEXSEM* pTextIconMutex;

	static int _stdcall
		FANCONTROL_Thread(ULONG
			parm) \
	{ return ((FANCONTROL*)parm)->WorkThread(); }

	int WorkThread();

	// fancontrol.cpp

	bool LockECAccess();

	void FreeECAccess();

	bool SampleMatch(FCSTATE* smp1, FCSTATE* smp2);

	bool ReadEcStatus(FCSTATE* pfcstate);

	bool ReadEcRaw(FCSTATE* pfcstate);

	bool HandleData();

	void SmartControl();

	bool SetFan(const char* source, int level, bool final = false);

	bool SetHdw(const char* source, int hdwctrl, int HdwOffset, int AnyWayBit);

	HPOWERNOTIFY hPowerNotify;

	EVT_HANDLE hEventSubscription;
	static DWORD WINAPI EventLogCallback(EVT_SUBSCRIBE_NOTIFY_ACTION action, PVOID pContext, EVT_HANDLE hEvent);
	void HandleModernStandbyEvent(EVT_HANDLE hEvent);

	// saved mode for power state changes
	int savedMode;
	// power suspend and modern standby state tracking
	bool isPowerSuspendState;
	bool isModernS0State;
	bool isLidClosed;

	// Constructor helpers
	void InitSensorNames();
	void InitSmartLevels();
	void InitDialogWindow();
	void HandleStartupDelay();
	void SubscribePowerEvents();
	void SetupTaskbarAndTimers();

	// DlgProc handlers
	ULONG OnHotKey(WPARAM mp1);
	ULONG OnTimer(WPARAM timerId);
	ULONG OnCommand(WPARAM mp1);
	ULONG OnPowerBroadcast(WPARAM mp1, LPARAM mp2);
	ULONG OnEndSession();
	ULONG OnNewData(WPARAM mp1);
	ULONG OnTaskbarNotify(LPARAM mp2);

	// Shared helpers
	bool TryClose();
	void SwitchSmartLevel(int level);
	void CreateAllNamedPipes();
	void WriteAllNamedPipes(const char* data);
	void CloseAllNamedPipes();
	void UpdateTemperatureDisplay();

	// misc.cpp

	int ReadConfig(const char* filename);

	void Tracecsv(const char* textcsv);

	void Tracecsvod(const char* textcsv);

	bool IsMinimized(void) const;

	void CurrentDateTimeLocalized(char* result, size_t sizeof_result);

	void CurrentTimeLocalized(char* result, size_t sizeof_result);

	HANDLE CreateThread(int(_stdcall
		* fnct)(ULONG),
		ULONG p
	);

	// portio.cpp

	bool ReadByteFromEC(int offset, char* pdata);

	bool WriteByteToEC(int offset, char data);

public:
	int EC_CTRL, EC_DATA;

	FANCONTROL(HINSTANCE hinstapp);

	~FANCONTROL();

	int ProcessDialog() const;

	HWND GetDialogWnd() const { return hwndDialog; }

	HANDLE GetWorkThread() const { return hThread; }

	// The texticons will be shown depending on variables
	void ProcessTextIcons(void);

	void RemoveTextIcons(void);

	void Trace(const char* text);

	bool PollECByte(char offset, char* out, int expected, int timeoutMs);
};

#endif // FANCONTROL_H
