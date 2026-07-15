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
#include "_prec.h"
#include "fancontrol.h"
#include "taskbartexticon.h"
#include <vector>
#include <string>
#include <winevt.h>
#pragma comment(lib, "wevtapi.lib")

DEFINE_GUID(GUID_LIDSWITCH_STATE_CHANGE,
	0xba3e0f4d, 0xb817, 0x4094,
	0xa2, 0xd1, 0xd5, 0x63, 0x79, 0xe6, 0xa0, 0xf3);

//-------------------------------------------------------------------------
//  constructor
//-------------------------------------------------------------------------
FANCONTROL::FANCONTROL(HINSTANCE hinstapp)
	: 
	hinstapp(hinstapp),
	m_hinstapp(hinstapp),
	hwndDialog(NULL),
	hEventSubscription(NULL),
	CurrentMode(-1),
	PreviousMode(-1),
	Cycle(5),
	IconCycle(1),
	ReIcCycle(0),
	NoExtSensor(0),
	FanSpeedLowByte(0x84),
	CurrentIcon(-1),
	hThread(NULL),
	FanBeepFreq(440),
	FanBeepDura(50),
	ReadErrorCount(0),
	MaxReadErrors(10),
	NoBallons(0),
	HK_BIOS_Method(0),
	HK_BIOS(0),
	HK_Manual_Method(0),
	HK_Manual(0),
	HK_Smart_Method(0),
	HK_Smart(0),
	HK_SM1_Method(0),
	HK_SM1(0),
	HK_SM2_Method(0),
	HK_SM2(0),
	HK_TG_BS_Method(0),
	HK_TG_BS(0),
	HK_TG_BM_Method(0),
	HK_TG_BM(0),
	HK_TG_MS_Method(0),
	HK_TG_MS(0),
	HK_TG_12_Method(0),
	HK_TG_12(0),
	EC_DATA(0),
	EC_CTRL(0),
	BluetoothEDR(0),
	ManModeExit(80),
	ManModeExitInternal(80),
	ShowBiasedTemps(0),
	SecWinUptime(0),
	SecStartDelay(0),
	SlimDialog(0),
	Log2File(0),
	StayOnTop(0),
	Log2csv(0),
	ShowAll(0),
	ShowTempIcon(1),
	pTaskbarIcon(NULL),
	Fahrenheit(FALSE),
	MinimizeToSysTray(TRUE),
	IconColorFan(FALSE),
	Lev64Norm(FALSE),
	StartMinimized(FALSE),
	NoWaitMessage(TRUE),
	MinimizeOnClose(TRUE),
	Runs_as_service(FALSE),
	ActiveMode(false),
	ManFanSpeed(7),
	UseTWR(0),
	FinalSeen(false),
	SingleFan(0),
	PowerSuspendMode(1), // 0 Disable, 1 BIOS(default), 2 Auto, 3 Manual(OFF), 4 Manual on any suspend
	ModernS0Mode(0),  // 0 Disable(default), 1 BIOS, 2 Auto, 3 Manual(OFF)
	savedMode(-1),
	isPowerSuspendState(false),
	isModernS0State(false),
	isLidClosed(false),
	m_fanTimer(NULL),
	m_titleTimer(NULL),
	m_iconTimer(NULL),
	m_renewTimer(NULL),
	m_needClose(false),
	ppTbTextIcon(NULL),
	pTextIconMutex(new MUTEXSEM(0, "Global\\TPFanControl_ppTbTextIcon")) {

	InitSensorNames();
	InitSmartLevels();
	this->ReadConfig("TPFanControl.ini");
	InitDialogWindow();
	HandleStartupDelay();
	SubscribePowerEvents();
	SetupTaskbarAndTimers();
}

//-------------------------------------------------------------------------
//  initialize sensor names, titles, and icon levels
//-------------------------------------------------------------------------
void FANCONTROL::InitSensorNames() {
	// SensorNames
	// 78-7F (state index 0-7)
	strcpy_s(this->gSensorNames[0], sizeof(this->gSensorNames[0]), "cpu"); // main processor
	strcpy_s(this->gSensorNames[1], sizeof(this->gSensorNames[1]), "aps"); // harddisk protection gyroscope
	strcpy_s(this->gSensorNames[2], sizeof(this->gSensorNames[2]), "crd"); // under PCMCIA slot (front left)
	strcpy_s(this->gSensorNames[3], sizeof(this->gSensorNames[3]), "gpu"); // graphical processor
	strcpy_s(this->gSensorNames[4], sizeof(this->gSensorNames[4]), "bat"); // inside T43 battery
	strcpy_s(this->gSensorNames[5], sizeof(this->gSensorNames[5]), "x7d"); // usually n/a
	strcpy_s(this->gSensorNames[6], sizeof(this->gSensorNames[6]), "bat"); // inside T43 battery
	strcpy_s(this->gSensorNames[7], sizeof(this->gSensorNames[7]), "x7f"); // usually n/a
	// C0-C4 (state index 8-11)
	strcpy_s(this->gSensorNames[8], sizeof(this->gSensorNames[8]), "bus"); // unknown
	strcpy_s(this->gSensorNames[9], sizeof(this->gSensorNames[9]), "pci"); // mini-pci, WLAN, southbridge area
	strcpy_s(this->gSensorNames[10], sizeof(this->gSensorNames[10]), "pwr"); // power supply (get's hot while charging battery)
	strcpy_s(this->gSensorNames[11], sizeof(this->gSensorNames[11]), "xc3"); // usually n/a
	// future
	for (int i = 12; i <= 16; i++)
		strcpy_s(this->gSensorNames[i], sizeof(this->gSensorNames[i]), "");

	// clear title strings
	setzero(this->Title, sizeof(this->Title));
	setzero(this->Title2, sizeof(this->Title2));
	setzero(this->LastTitle, sizeof(this->LastTitle));
	setzero(this->CurrentStatus, sizeof(this->CurrentStatus));
	setzero(this->CurrentStatuscsv, sizeof(this->CurrentStatuscsv));
	setzero(this->IgnoreSensors, sizeof(this->IgnoreSensors));

	this->IconLevels[0] = 50;    // yellow icon level
	this->IconLevels[1] = 55;    // orange icon level
	this->IconLevels[2] = 60;    // red icon level
}

//-------------------------------------------------------------------------
//  initialize smart fan control tables
//-------------------------------------------------------------------------
void FANCONTROL::InitSmartLevels() {
	const int defaultTemps[] = { 50, 55, 60, 65, 70, -1 };
	const int defaultFans[]  = {  0,  3,  5,  7, 128, 0 };
	const int numEntries = 6;

	setzero(this->SmartLevels, sizeof(this->SmartLevels));
	setzero(this->SmartLevels1, sizeof(this->SmartLevels1));
	setzero(this->SmartLevels2, sizeof(this->SmartLevels2));

	for (int i = 0; i < numEntries; i++) {
		this->SmartLevels[i].temp  = defaultTemps[i];
		this->SmartLevels[i].fan   = defaultFans[i];
		this->SmartLevels1[i].temp1 = defaultTemps[i];
		this->SmartLevels1[i].fan1  = defaultFans[i];
	}

	// SmartLevels2 uses 0 instead of 50 for the first temp entry
	for (int i = 0; i < numEntries; i++) {
		this->SmartLevels2[i].temp2 = (i == 0) ? 0 : defaultTemps[i];
		this->SmartLevels2[i].fan2  = defaultFans[i];
	}
}

//-------------------------------------------------------------------------
//  set up dialog window and controls
//-------------------------------------------------------------------------
void FANCONTROL::InitDialogWindow() {
	if (!this->hwndDialog)
		return;

	char buf[256] = "";

	::GetWindowText(this->hwndDialog, this->Title, sizeof(this->Title));
	strcat_s(this->Title, sizeof(this->Title), " V");
	if (SingleFan)
		strcat_s(this->Title, sizeof(this->Title), FANCONTROLVERSIONS);
	else
		strcat_s(this->Title, sizeof(this->Title), FANCONTROLVERSIOND);

	::SetWindowText(this->hwndDialog, this->Title);

	::SetWindowLongPtr(this->hwndDialog, GWLP_USERDATA, (LONG_PTR)this);

	::SendDlgItemMessage(this->hwndDialog, 8112, EM_LIMITTEXT, 256, 0);
	::SendDlgItemMessage(this->hwndDialog, 9200, EM_LIMITTEXT, 4096, 0);

		// Init temperature ListView columns (normal mode only)
		{
			HWND hLV = ::GetDlgItem(this->hwndDialog, 8101);
			if (hLV) {
				ListView_SetExtendedListViewStyle(hLV, LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);
				LVCOLUMNA lvc = {0};
				lvc.mask = LVCF_TEXT | LVCF_WIDTH | LVCF_FMT;
				lvc.fmt = LVCFMT_LEFT;
				lvc.cx = 28; lvc.pszText = (LPSTR)"#"; ListView_InsertColumn(hLV, 0, &lvc);
				lvc.cx = 48; lvc.pszText = (LPSTR)"Name"; ListView_InsertColumn(hLV, 1, &lvc);
				lvc.cx = 50; lvc.pszText = (LPSTR)"Temp"; ListView_InsertColumn(hLV, 2, &lvc);
				lvc.cx = 42; lvc.pszText = (LPSTR)"EC"; ListView_InsertColumn(hLV, 3, &lvc);
			}
		}

		// Init manual mode ComboBox (editable, normal mode only, supports 0-255)
		{
			HWND hCB = ::GetDlgItem(this->hwndDialog, 8310);
			if (hCB) {
				SendMessageA(hCB, CB_ADDSTRING, 0, (LPARAM)"0");
				SendMessageA(hCB, CB_ADDSTRING, 0, (LPARAM)"1");
				SendMessageA(hCB, CB_ADDSTRING, 0, (LPARAM)"2");
				SendMessageA(hCB, CB_ADDSTRING, 0, (LPARAM)"3");
				SendMessageA(hCB, CB_ADDSTRING, 0, (LPARAM)"4");
				SendMessageA(hCB, CB_ADDSTRING, 0, (LPARAM)"5");
				SendMessageA(hCB, CB_ADDSTRING, 0, (LPARAM)"6");
				SendMessageA(hCB, CB_ADDSTRING, 0, (LPARAM)"7");
				char mbuf[16];
				_itoa_s(this->ManFanSpeed, mbuf, 10);
				SetWindowTextA(hCB, mbuf);
			}
		}

	if (SlimDialog == 1) {
		// Fix: destroy the old dialog before replacing
		if (this->hwndDialog)
			::DestroyWindow(this->hwndDialog);
		if (this->StayOnTop)
			this->hwndDialog = ::CreateDialogParam(hinstapp,
				MAKEINTRESOURCE(9001),
				HWND_DESKTOP,
				(DLGPROC)BaseDlgProc,
				(LPARAM)
				this);
		else
			this->hwndDialog = ::CreateDialogParam(hinstapp,
				MAKEINTRESOURCE(9003),
				HWND_DESKTOP,
				(DLGPROC)BaseDlgProc,
				(LPARAM)
				this);
	}
}

//-------------------------------------------------------------------------
//  perform startup delay if needed
//-------------------------------------------------------------------------
void FANCONTROL::HandleStartupDelay() {
	// If the windows fast start feature is turned on, the system uptime
	// WILL NOT BE RESET ON POWER ON, thus making this logic fail. Turn this
	// feature off to make this work.
	DWORD tickCount = GetTickCount();

	char bufsec[1024] = "";

	sprintf_s(bufsec, sizeof(bufsec), "Windows uptime since boot %d sec., SecWinUptime= %d sec.", tickCount / 1000, SecWinUptime);
	this->Trace(bufsec);

	if ((SecStartDelay > 0) && ((tickCount / 1000) <= (DWORD)SecWinUptime)) {
		sprintf_s(bufsec, sizeof(bufsec), "Delay startup to allow windows to settle, SecStartDelay= %d sec.", SecStartDelay);
		this->Trace(bufsec);

		if (!NoWaitMessage) {
			sprintf_s(bufsec, sizeof(bufsec),
				"TPFanControl delayed %d sec. after\nboot time (SecWinUptime= %d sec.)\n\nto prevent missing systray icons\nand communication errors between\nTPFanControl and embedded controller\n\n\nTo avoid this message box set\nNoWaitMessage=1 in TPFanControl.ini",
				SecStartDelay, SecWinUptime);

			// Don't show message box when as service in Vista
			OSVERSIONINFOEX os = { sizeof(os) };
			VerifyVersionInfoA(&os, VER_MAJORVERSION, 1);
			if (os.dwMajorVersion >= 6 && Runs_as_service == TRUE)
				;
			else
				MessageBox(NULL, bufsec, "TPFanControl delaying startup", MB_ICONEXCLAMATION);
		}

		// sleep until start time + delay time
		while ((DWORD)(tickCount + SecStartDelay * 1000) >= GetTickCount())
			Sleep(200);
	}
}

//-------------------------------------------------------------------------
//  subscribe to power and modern standby events
//-------------------------------------------------------------------------
void FANCONTROL::SubscribePowerEvents() {
	if (this->PowerSuspendMode) {
		this->hPowerNotify = RegisterPowerSettingNotification(this->hwndDialog, &GUID_LIDSWITCH_STATE_CHANGE, DEVICE_NOTIFY_WINDOW_HANDLE);
		if (this->hPowerNotify == NULL) {
			char errBuf[128];
			sprintf_s(errBuf, sizeof(errBuf), "Failed to subscribe to PowerSetting events, error: %lu", GetLastError());
			this->Trace(errBuf);
		}
		else {
			this->Trace("Subscribed to PowerSetting events");
		}
	}

	if (this->ModernS0Mode) {
		// Subscribe to Modern Standby events (EventId 506 = Entry, 507 = Exit)
		const wchar_t* query = L"*[System[Provider[@Name='Microsoft-Windows-Kernel-Power'] and (EventID=506 or EventID=507)]]";
		this->hEventSubscription = EvtSubscribe(
			NULL,                           // Session (local)
			NULL,                           // SignalEvent
			L"System",                      // Channel path
			query,                          // Query
			NULL,                           // Bookmark
			this,                           // Context (pass FANCONTROL pointer)
			(EVT_SUBSCRIBE_CALLBACK)FANCONTROL::EventLogCallback,
			EvtSubscribeToFutureEvents      // Flags
		);

		if (this->hEventSubscription == NULL) {
			char errBuf[128];
			sprintf_s(errBuf, sizeof(errBuf), "Failed to subscribe to ModernS0 events, error: %lu", GetLastError());
			this->Trace(errBuf);
		}
		else {
			this->Trace("Subscribed to ModernS0 events");
		}
	}
}

//-------------------------------------------------------------------------
//  set up taskbar icon, hotkeys, timers, and initial window state
//-------------------------------------------------------------------------
void FANCONTROL::SetupTaskbarAndTimers() {
	// taskbar icon
	if (this->MinimizeToSysTray) {
		if (this->ShowTempIcon)
			this->pTaskbarIcon = NULL;
		else
			this->pTaskbarIcon = new TASKBARICON(this->hwndDialog, 10, "TPFanControl");
	}

	// read current fan control status and set mode buttons accordingly
	this->CurrentMode = this->ActiveMode;
	this->ModeToDialog(this->CurrentMode);
	this->PreviousMode = 1;

	if (HK_BIOS_Method) RegisterHotKey(this->hwndDialog, 1, HK_BIOS_Method, HK_BIOS);
	if (HK_Smart_Method) RegisterHotKey(this->hwndDialog, 2, HK_Smart_Method, HK_Smart);
	if (HK_Manual_Method) RegisterHotKey(this->hwndDialog, 3, HK_Manual_Method, HK_Manual);
	if (HK_SM1_Method) RegisterHotKey(this->hwndDialog, 4, HK_SM1_Method, HK_SM1);
	if (HK_SM2_Method) RegisterHotKey(this->hwndDialog, 5, HK_SM2_Method, HK_SM2);
	if (HK_TG_BS_Method) RegisterHotKey(this->hwndDialog, 6, HK_TG_BS_Method, HK_TG_BS);
	if (HK_TG_BM_Method) RegisterHotKey(this->hwndDialog, 7, HK_TG_BM_Method, HK_TG_BM);
	if (HK_TG_MS_Method) RegisterHotKey(this->hwndDialog, 8, HK_TG_MS_Method, HK_TG_MS);
	if (HK_TG_12_Method) RegisterHotKey(this->hwndDialog, 9, HK_TG_12_Method, HK_TG_12);

	// enable/disable mode radiobuttons
	::EnableWindow(::GetDlgItem(this->hwndDialog, 8300), this->ActiveMode);
	::EnableWindow(::GetDlgItem(this->hwndDialog, 8301), this->ActiveMode);
	::EnableWindow(::GetDlgItem(this->hwndDialog, 8302), this->ActiveMode);
	::EnableWindow(::GetDlgItem(this->hwndDialog, 8310), this->ActiveMode);

	// make it call HandleControl initially
	::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);

	m_fanTimer = ::SetTimer(this->hwndDialog, 1, this->Cycle * 1000, NULL);           // fan update
	m_titleTimer = ::SetTimer(this->hwndDialog, 2, 500, NULL);                        // title update
	m_iconTimer = ::SetTimer(this->hwndDialog, 3, this->IconCycle * 1000, NULL);      // Vista icon update
	if (this->ReIcCycle)
		m_renewTimer = ::SetTimer(this->hwndDialog, 4, this->ReIcCycle * 1000, NULL); // Vista icon update

	if (this->StartMinimized)
		::ShowWindow(this->hwndDialog, SW_MINIMIZE);
	else
		::ShowWindow(this->hwndDialog, TRUE);
}

//-------------------------------------------------------------------------
//  destructor
//-------------------------------------------------------------------------
FANCONTROL::~FANCONTROL() {
	if (this->hThread) {
		::WaitForSingleObject(this->hThread, 2000);
		this->hThread = NULL;
	}

	if (this->ModernS0Mode && this->hEventSubscription) {
		EvtClose(this->hEventSubscription);
		this->hEventSubscription = NULL;
	}

	if (this->PowerSuspendMode) {
		UnregisterPowerSettingNotification(this->hPowerNotify);
	}

	if (this->pTaskbarIcon) {
		delete this->pTaskbarIcon;
		this->pTaskbarIcon = NULL;
	}

	if (this->ppTbTextIcon) {
		delete ppTbTextIcon[0];
		delete[] ppTbTextIcon;
		ppTbTextIcon = NULL;
	}

	if (this->hwndDialog)
		::DestroyWindow(this->hwndDialog);

	if (pTextIconMutex)
		delete pTextIconMutex;
}

//-------------------------------------------------------------------------
//  Event log callback for Modern Standby events
//-------------------------------------------------------------------------
DWORD WINAPI FANCONTROL::EventLogCallback(EVT_SUBSCRIBE_NOTIFY_ACTION action, PVOID pContext, EVT_HANDLE hEvent) {
	FANCONTROL* pThis = static_cast<FANCONTROL*>(pContext);

	if (action == EvtSubscribeActionDeliver && pThis != NULL) {
		pThis->HandleModernStandbyEvent(hEvent);
	}

	return ERROR_SUCCESS;
}

void FANCONTROL::HandleModernStandbyEvent(EVT_HANDLE hEvent) {
	DWORD bufferSize = 0;
	DWORD bufferUsed = 0;
	DWORD propertyCount = 0;

	// First call to get required buffer size
	EvtRender(NULL, hEvent, EvtRenderEventXml, bufferSize, NULL, &bufferUsed, &propertyCount);
	bufferSize = bufferUsed;

	std::vector<wchar_t> buffer(bufferSize / sizeof(wchar_t) + 1);
	if (EvtRender(NULL, hEvent, EvtRenderEventXml, bufferSize, buffer.data(), &bufferUsed, &propertyCount)) {
		// Parse for EventID - simple string search
		std::wstring xml(buffer.data());

		if (xml.find(L"<EventID>506</EventID>") != std::wstring::npos) {
			this->isModernS0State = true;
			this->Trace("Detected Modern S0 Entry");
			this->savedMode = this->CurrentMode;

			if (this->ModernS0Mode == 1) {
				this->ModeToDialog(1);
				if (this->SetFan("Switched to BIOS mode", 0x80)) ::Sleep(1000);
			}
			else if (this->ModernS0Mode == 2 || this->ModernS0Mode >= 4) {
				this->Trace("Continuing current mode");
			}
			else if (this->ModernS0Mode == 3) {
				this->ModeToDialog(3);
				if (this->SetFan("Switched fans off and to manual mode", 0x00))	::Sleep(1000);
			}
		}
		else if (xml.find(L"<EventID>507</EventID>") != std::wstring::npos) {
			this->isModernS0State = false;
			this->Trace("Detected Modern S0 Exit, defer access to EC (10s)");
			::Sleep(10000);

			if (this->savedMode != -1 && this->savedMode != this->CurrentMode) {
				this->ModeToDialog(this->savedMode);
				this->Trace("Restored saved mode");
			}
		}
	}
}

//-------------------------------------------------------------------------
//  mode integer from mode radio buttons
//-------------------------------------------------------------------------
int FANCONTROL::CurrentModeFromDialog() {
	BOOL modetpauto = ::SendDlgItemMessage(this->hwndDialog, 8300, BM_GETCHECK, 0L, 0L),
		modefcauto = ::SendDlgItemMessage(this->hwndDialog, 8301, BM_GETCHECK, 0L, 0L),
		modemanual = ::SendDlgItemMessage(this->hwndDialog, 8302, BM_GETCHECK, 0L, 0L);

	if (modetpauto)
		this->CurrentMode = 1;
	else if (modefcauto)
		this->CurrentMode = 2;
	else if (modemanual)
		this->CurrentMode = 3;
	else
		this->CurrentMode = -1;

	return this->CurrentMode;
}

int FANCONTROL::ShowAllFromDialog() {
	BOOL modefcauto = ::SendDlgItemMessage(this->hwndDialog, 7001, BM_GETCHECK, 0L, 0L),
		modemanual = ::SendDlgItemMessage(this->hwndDialog, 7002, BM_GETCHECK, 0L, 0L);

	if (modefcauto)
		this->ShowAll = 1;
	else if (modemanual)
		this->ShowAll = 0;
	else
		this->ShowAll = -1;

	return this->ShowAll;
}

void FANCONTROL::ModeToDialog(int mode) const {
	::SendDlgItemMessage(this->hwndDialog, 8300, BM_SETCHECK, mode == 1, 0L);
	::SendDlgItemMessage(this->hwndDialog, 8301, BM_SETCHECK, mode == 2, 0L);
	::SendDlgItemMessage(this->hwndDialog, 8302, BM_SETCHECK, mode == 3, 0L);
}

void FANCONTROL::ShowAllToDialog(int show) const {
	::SendDlgItemMessage(this->hwndDialog, 7001, BM_SETCHECK, show == 1, 0L);
	::SendDlgItemMessage(this->hwndDialog, 7002, BM_SETCHECK, show == 0, 0L);
}

//-------------------------------------------------------------------------
//  process main dialog
//-------------------------------------------------------------------------
int FANCONTROL::ProcessDialog() const {

	MSG qmsg, qmsg2;
	int dlgrc = -1;

	if (this->hwndDialog) {
		for (;;) {
			BOOL nodlgmsg = FALSE;

			::GetMessage(&qmsg, NULL, 0L, 0L);

			if (qmsg.message != WM__DISMISSDLG && IsDialogMessage(this->hwndDialog, &qmsg))
				continue;

			qmsg2 = qmsg;
			TranslateMessage(&qmsg);
			DispatchMessage(&qmsg);

			if (qmsg2.message == WM__DISMISSDLG && qmsg2.hwnd == this->hwndDialog) {
				dlgrc = qmsg2.wParam;
				break;
			}
		}
	}

	return dlgrc;
}

//-------------------------------------------------------------------------
//  dialog window procedure (map to class method)
//-------------------------------------------------------------------------
ULONG CALLBACK FANCONTROL::BaseDlgProc(HWND hwnd, ULONG msg, WPARAM mp1, LPARAM mp2) {
	ULONG rc = FALSE;

	static UINT s_TaskbarCreated;

	if (msg == WM_INITDIALOG) {
		s_TaskbarCreated = RegisterWindowMessage("TaskbarCreated");
	}

	FANCONTROL* This = (FANCONTROL*)GetWindowLongPtr(hwnd, GWLP_USERDATA);

	if (This) {
		if (msg == s_TaskbarCreated) {
			This->TaskbarNew = 1;

			if (This->pTaskbarIcon)
				This->pTaskbarIcon->RebuildIfNecessary(TRUE);
			else {
				This->RemoveTextIcons();
				This->ProcessTextIcons();
			}
		}
		rc = This->DlgProc(hwnd, msg, mp1, mp2);
	}

	return rc;
}

//-------------------------------------------------------------------------
//  file-scope state (persists across timer ticks)
//-------------------------------------------------------------------------
constexpr auto WANTED_MEM_SIZE = 65536*12;
BOOL dioicon(TRUE);
char szBuffer[BUFFER_SIZE];
char str_value[256];
DWORD cbBytes;
BOOL bResult(FALSE);
BOOL lbResult(FALSE);
int fanspeed;
int fanctrl;
int IconFontSize;
BOOL _piscreated(FALSE);
char obuftd[256] = "", obuftd2[128] = "", templisttd[512];
char obuf[256] = "", obuf2[128] = "", templist2[512];

// Temperature color constants
static constexpr COLORREF COLOR_RED_ORANGE = RGB(255, 69, 0);
static constexpr COLORREF COLOR_ORANGE = RGB(255, 165, 0);
static constexpr COLORREF COLOR_DARK_YELLOW = RGB(210, 160, 0);
static constexpr COLORREF COLOR_BLACK = RGB(0, 0, 0);

// Helper function to determine color based on temperature and thresholds
static inline COLORREF GetTempColor(int temp, const int* iconLevels) {
	if (temp >= iconLevels[2] && iconLevels[2] > 0)
		return COLOR_RED_ORANGE;
	if (temp >= iconLevels[1] && iconLevels[1] > 0)
		return COLOR_ORANGE;
	if (temp >= iconLevels[0] && iconLevels[0] > 0)
		return COLOR_DARK_YELLOW;
	return COLOR_BLACK;
}

//-------------------------------------------------------------------------
//  dialog window procedure — thin dispatcher
//-------------------------------------------------------------------------
ULONG FANCONTROL::DlgProc(HWND hwnd, ULONG msg, WPARAM mp1, LPARAM mp2) {
	switch (msg) {
	case WM_HOTKEY:
		return OnHotKey(mp1);

	case WM_INITDIALOG:
		// placing code here will NOT work!
		// (put it into BaseDlgProc instead)
		break;

	case WM_TIMER:
		return OnTimer(mp1);

	case WM_NOTIFY:
		{
			LPNMHDR pnmh = (LPNMHDR)mp2;
			// Only handle custom draw if control 8101 is a ListView (normal/StayOnTop dialogs)
			// In slim dialogs, control 8101 is an EDITTEXT
			if (!SlimDialog && pnmh->idFrom == 8101 && pnmh->code == NM_CUSTOMDRAW)
			{
				LPNMLVCUSTOMDRAW lplvcd = (LPNMLVCUSTOMDRAW)mp2;
				LRESULT result = CDRF_DODEFAULT;

				switch (lplvcd->nmcd.dwDrawStage)
				{
				case CDDS_PREPAINT:
					result = CDRF_NOTIFYITEMDRAW;
					break;
				case CDDS_ITEMPREPAINT:
					result = CDRF_NOTIFYSUBITEMDRAW;
					break;
				case CDDS_ITEMPREPAINT | CDDS_SUBITEM:
				{
					lplvcd->clrTextBk = CLR_DEFAULT;
					if (lplvcd->iSubItem == 2)
					{
						char tempStr[16];
						ListView_GetItemText(pnmh->hwndFrom, (int)lplvcd->nmcd.dwItemSpec, 2, tempStr, sizeof(tempStr));
						int temp = atoi(tempStr);
						lplvcd->clrText = GetTempColor(temp, this->IconLevels);
					}
					else
					{
						lplvcd->clrText = CLR_DEFAULT;
					}
					result = CDRF_NEWFONT;
					break;
				}
				default:
					result = CDRF_DODEFAULT;
					break;
				}
				SetWindowLongPtr(hwnd, DWLP_MSGRESULT, result);
				return TRUE;
			}
			else if (!SlimDialog && pnmh->code == NM_CUSTOMDRAW)
			{
				HWND hLV = ::GetDlgItem(hwnd, 8101);
				if (hLV)
				{
					HWND hHeader = ListView_GetHeader(hLV);
					if (pnmh->hwndFrom == hHeader)
					{
						LPNMCUSTOMDRAW lpnmcd = (LPNMCUSTOMDRAW)mp2;
						LRESULT result = CDRF_DODEFAULT;

						switch (lpnmcd->dwDrawStage)
						{
						case CDDS_PREPAINT:
							result = CDRF_NOTIFYITEMDRAW;
							break;
						case CDDS_ITEMPREPAINT:
						{
							if (lpnmcd->dwItemSpec == 2)
							{
								COLORREF textColor = GetTempColor(this->MaxTemp, this->IconLevels);
								SetTextColor(lpnmcd->hdc, textColor);
								result = CDRF_NEWFONT;
							}
							break;
						}
						default:
							result = CDRF_DODEFAULT;
							break;
						}
						SetWindowLongPtr(hwnd, DWLP_MSGRESULT, result);
						return TRUE;
					}
				}
			}
		}
		break;

	case WM_COMMAND:
		return OnCommand(mp1);

	case WM_CLOSE:
		::ShowWindow(this->hwndDialog, SW_MINIMIZE);
		return TRUE;

	case WM_POWERBROADCAST:
		return OnPowerBroadcast(mp1, mp2);

	case WM_ENDSESSION:
		return OnEndSession();

	case WM_SIZE:
		if (mp1 == SIZE_MINIMIZED && this->MinimizeToSysTray)
			::ShowWindow(this->hwndDialog, FALSE);
		return TRUE;

	case WM_DESTROY:
		break;

	case WM__GETDATA:
		if (!this->hThread && !this->FinalSeen)
			this->hThread = this->CreateThread(FANCONTROL_Thread, (ULONG)this);
		break;

	case WM__NEWDATA:
		return OnNewData(mp1);

	case WM__TASKBAR:
		return OnTaskbarNotify(mp2);

	default:
		break;
	}

	return 0;
}

//-------------------------------------------------------------------------
//  WM_HOTKEY handler
//-------------------------------------------------------------------------
ULONG FANCONTROL::OnHotKey(WPARAM mp1) {
	switch (mp1) {
	case 1: // BIOS mode
		this->ModeToDialog(1);
		::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
		break;

	case 2: // Smart mode
		this->ModeToDialog(2);
		::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
		break;

	case 3: // Manual mode
		this->ModeToDialog(3);
		::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
		break;

	case 4: // Smart Mode 1
		SwitchSmartLevel(0);
		break;

	case 5: // Smart Mode 2
		SwitchSmartLevel(1);
		break;

	case 6: // Toggle BIOS <-> Smart
		if (this->CurrentMode > 1) {
			this->ModeToDialog(1);
		}
		else {
			this->ModeToDialog(2);
		}
		::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
		break;

	case 7: // Toggle BIOS <-> Manual
		if (this->CurrentMode > 1) {
			this->ModeToDialog(1);
		}
		else {
			this->ModeToDialog(3);
		}
		::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
		break;

	case 8: // Toggle Manual <-> Smart
		if (this->CurrentMode < 3) {
			this->ModeToDialog(3);
		}
		else {
			this->ModeToDialog(2);
		}
		::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
		break;

	case 9: // Toggle Smart Mode 1 <-> 2
		SwitchSmartLevel(this->IndSmartLevel == 0 ? 1 : 0);
		break;
	}

	return 0;
}

//-------------------------------------------------------------------------
//  WM_TIMER handler
//-------------------------------------------------------------------------
ULONG FANCONTROL::OnTimer(WPARAM timerId) {
	switch (timerId) {
	case 1: // update fan state
		::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
		if (this->Log2csv == 1)
			this->Tracecsv(this->CurrentStatuscsv);
		break;

	case 2: { // update window title
		if (this->CurrentMode == 3 && this->MaxTemp > this->ManModeExitInternal) {
			this->ModeToDialog(2);
			::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
		}

		ULONG res = this->IsMinimized();
		if (res && strcmp(this->LastTitle, this->Title2) != 0) {
			strcpy_s(this->LastTitle, sizeof(this->LastTitle), this->Title2);
		}
		else if (!res && strcmp(this->LastTitle, this->Title) != 0) {
			::SetWindowText(this->hwndDialog, this->Title);
			strcpy_s(this->LastTitle, sizeof(this->LastTitle), this->Title);
		}

		if (this->pTaskbarIcon) {
			this->pTaskbarIcon->SetTooltip(this->Title2);
			strcpy_s(this->LastTooltip, sizeof(this->LastTooltip), this->Title2);
			int icon = -1;

			if (this->CurrentModeFromDialog() == 1) {
				icon = 10;    // gray
			}
			else {
				icon = 11;    // blue
				for (int i = 0; i < ARRAYMAX(this->IconLevels); i++) {
					if (this->MaxTemp >= this->IconLevels[i]) {
						icon = 12 + i;    // yellow, orange, red
					}
				}
			}

			if (icon != this->CurrentIcon && icon != -1) {
				this->pTaskbarIcon->SetIcon(icon);
				this->CurrentIcon = icon;
				if (dioicon && !this->NoBallons) {
					this->pTaskbarIcon->SetBalloon(NIIF_INFO, "TPFanControl old symbol icon",
						"shows temperature level by color and state in tooltip, left click on icon shows or hides control window, right click shows menue",
						11);
					dioicon = FALSE;
				}
			}
			this->iFarbeIconB = icon;
		}
		break;
	}

	case 3: { // update vista icon / named pipe
		// Reconnect if previous write failed
		if (bResult == FALSE && lbResult == TRUE) {
			_piscreated = FALSE;
			lbResult = FALSE;
			bResult = FALSE;
			CloseAllNamedPipes();
		}

		if (_piscreated == FALSE) {
			CreateAllNamedPipes();
			_piscreated = TRUE;
		}

		// Build pipe data
		if (fan1speed > 0x1fff)
			fan1speed = lastfan1speed;

		char pipeData[BUFFER_SIZE];
		if (Fahrenheit) {
			sprintf_s(pipeData, sizeof(pipeData), "%d %d %s %d %d %d ",
				this->CurrentMode, (this->MaxTemp * 9 / 5 + 32), this->gSensorNames[iMaxTemp],
				iFarbeIconB, fan1speed, fanctrl2);
		}
		else {
			sprintf_s(pipeData, sizeof(pipeData), "%d %d %s %d %d %d ",
				this->CurrentMode, (this->MaxTemp), this->gSensorNames[iMaxTemp],
				iFarbeIconB, fan1speed, fanctrl2);
		}

		WriteAllNamedPipes(pipeData);
		break;
	}

	case 4: // renew tempicon
		if (ShowTempIcon && ReIcCycle) {
			this->RemoveTextIcons();
			this->ProcessTextIcons();
		}
		break;

	default:
		break;
	}

	if (this->ShowTempIcon == 1)
		this->ProcessTextIcons();
	else
		this->RemoveTextIcons();

	return 0;
}

//-------------------------------------------------------------------------
//  WM_COMMAND handler
//-------------------------------------------------------------------------
ULONG FANCONTROL::OnCommand(WPARAM mp1) {
	if (HIWORD(mp1) != BN_CLICKED && HIWORD(mp1) != EN_CHANGE
		&& HIWORD(mp1) != CBN_SELCHANGE && HIWORD(mp1) != CBN_EDITCHANGE)
		return 0;

	int cmd = LOWORD(mp1);

	if (cmd == 7001 || cmd == 7002) {
		this->ShowAllFromDialog();
		this->UpdateTempDisplay();
	}

	if (cmd >= 8300 && cmd <= 8302 || cmd == 8310) {  // radio button or manual speed entry
		if (cmd == 8310) {  // auto-switch to Manual when user interacts with speed ComboBox
			if (HIWORD(mp1) == CBN_EDITCHANGE)  // ignore per-keystroke, only act on CBN_SELCHANGE
				return 0;
			this->ModeToDialog(3);
		}
		::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
	}
	else {
		switch (cmd) {
		case 5001: // bios
			this->ModeToDialog(1);
			::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
			break;

		case 5002: // smart
			this->ModeToDialog(2);
			::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
			break;

		case 5003: // smart1
			SwitchSmartLevel(0);
			break;

		case 5004: // smart2
			SwitchSmartLevel(1);
			break;

		case 5005: // manual
			this->ModeToDialog(3);
			::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
			break;

		case 5010: // show window
			::ShowWindow(this->hwndDialog, TRUE);
			::SetForegroundWindow(this->hwndDialog);
			break;

		case 5040: // bluetooth toggle
			if (BluetoothEDR)
				this->SetHdw("Bluetooth", 16, 58, 32);
			else
				this->SetHdw("Bluetooth", 32, 59, 16);
			break;

		case 5070: // show old icon
			this->ShowTempIcon = 0;
			this->pTaskbarIcon = new TASKBARICON(this->hwndDialog, 10, "TPFanControl");
			this->pTaskbarIcon->SetIcon(this->CurrentIcon);
			break;

		case 5080: // show temp icon
			delete this->pTaskbarIcon;
			this->pTaskbarIcon = NULL;
			this->ShowTempIcon = 1;
			break;

		case 5030: // hide window
			::ShowWindow(this->hwndDialog, SW_MINIMIZE);
			break;

		case 5020: // end program
			TryClose();
			break;
		}
	}

	return 0;
}

//-------------------------------------------------------------------------
//  WM_POWERBROADCAST handler
//-------------------------------------------------------------------------
ULONG FANCONTROL::OnPowerBroadcast(WPARAM mp1, LPARAM mp2) {
	if (mp1 == PBT_APMSUSPEND) {
		this->isPowerSuspendState = true;
		this->savedMode = this->CurrentMode;
		this->Trace("System suspend detected");

		if (this->PowerSuspendMode == 4) {
			this->ModeToDialog(3);
			if (this->SetFan("Switched fans off and to manual mode", 0x00)) ::Sleep(1000);
		}
	}
	else if (mp1 == PBT_APMRESUMEAUTOMATIC) {
		this->isPowerSuspendState = false;
		this->Trace("System resume detected");

		if (this->PowerSuspendMode == 4) {
			if (this->savedMode != -1 && this->savedMode != this->CurrentMode) {
				::Sleep(5000);
				this->ModeToDialog(this->savedMode);
				::Sleep(1000);
				this->Trace("Restored saved mode");
			}
		}
	}
	else if (mp1 == PBT_POWERSETTINGCHANGE) {
		POWERBROADCAST_SETTING* pbs = (POWERBROADCAST_SETTING*)mp2;
		if (pbs->PowerSetting == GUID_LIDSWITCH_STATE_CHANGE) {
			BYTE state = *(BYTE*)(&pbs->Data);

			if (state == 0) {  // Lid closed
				this->isLidClosed = true;
				this->Trace("Lid close detected");
				this->savedMode = this->CurrentMode;

				if (this->PowerSuspendMode == 1) {
					this->ModeToDialog(1);
					if (this->SetFan("Switched to BIOS mode", 0x80)) ::Sleep(1000);
				}
				else if (this->PowerSuspendMode == 2 || this->PowerSuspendMode >= 5) {
					this->Trace("Continuing current mode");
				}
				else if (this->PowerSuspendMode == 3) {
					this->ModeToDialog(3);
					if (this->SetFan("Switched fans off and to manual mode", 0x00)) ::Sleep(1000);
				}
				else if (this->PowerSuspendMode == 4) {
					// Defer to PBT_APMSUSPEND/PBT_APMRESUMEAUTOMATIC handling
				}
			}
			else { // Lid opened
				this->isLidClosed = false;
				this->Trace("Lid open detected");

				if (this->PowerSuspendMode != 4) {
					if (this->savedMode != -1 && this->savedMode != this->CurrentMode) {
						::Sleep(5000);
						this->ModeToDialog(this->savedMode);
						::Sleep(1000);
						this->Trace("Restored saved mode");
					}
				}
			}
		}
	}

	return 0;
}

//-------------------------------------------------------------------------
//  WM_ENDSESSION handler
//-------------------------------------------------------------------------
ULONG FANCONTROL::OnEndSession() {
	if (!this->Runs_as_service) {
		TryClose();
	}
	return 0;
}

//-------------------------------------------------------------------------
//  WM__NEWDATA handler
//-------------------------------------------------------------------------
ULONG FANCONTROL::OnNewData(WPARAM mp1) {
	if (this->hThread) {
		::WaitForSingleObject(this->hThread, INFINITE);
		if (this->hThread)
			::CloseHandle(this->hThread);
		else {
			this->Trace("Exception detected, closing to BIOS mode");
			::SendMessage(this->hwndDialog, WM_ENDSESSION, 0, 0);
		}
		this->hThread = 0;
	}

	ULONG ok = mp1;  // equivalent of "ok = this->ReadEcStatus(&this->State);" via thread

	if (ok) {
		this->ReadErrorCount = 0;
		this->HandleData();

		if (m_needClose) {
			this->Trace("Program needs to be closed, changing to BIOS mode");
			::Sleep(1000);
			::PostMessage(this->hwndDialog, WM_COMMAND, 5020, 0);
			::SendMessage(this->hwndDialog, WM_ENDSESSION, 0, 0);
			m_needClose = false;
		}
	}
	else {
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Warning: can't read Status, read error count = %d", this->ReadErrorCount);
		this->Trace(buf);
		sprintf_s(buf, sizeof(buf), "We will close to BIOS-Mode after %d consecutive read errors", this->MaxReadErrors);
		this->Trace(buf);
		this->ReadErrorCount++;

		// after so many consecutive read errors, try to switch back to bios mode
		if (this->ReadErrorCount > this->MaxReadErrors) {
			this->ModeToDialog(1);
			ok = this->SetFan("Max. Errors", 0x80);
			if (ok) {
				this->Trace("Set to BIOS Mode, to many consecutive read errors");
				::Sleep(2000);
				::SendMessage(this->hwndDialog, WM_ENDSESSION, 0, 0);
			}
		}
	}

	return 0;
}

//-------------------------------------------------------------------------
//  WM__TASKBAR handler
//-------------------------------------------------------------------------
ULONG FANCONTROL::OnTaskbarNotify(LPARAM mp2) {
	switch (mp2) {
	case WM_LBUTTONDOWN:
		if (!IsWindowVisible(this->hwndDialog)) {
			::ShowWindow(this->hwndDialog, TRUE);
			::SetForegroundWindow(this->hwndDialog);
		}
		else
			::ShowWindow(this->hwndDialog, SW_MINIMIZE);
		break;

	case WM_LBUTTONUP:
	{
		BOOL isshift = ::GetAsyncKeyState(VK_SHIFT) & 0x8000;
		BOOL isctrl = ::GetAsyncKeyState(VK_CONTROL) & 0x8000;

		int action = -1;

		// some fancy key dependent stuff could be done here.
	}
	break;

	case WM_LBUTTONDBLCLK:
		if (!IsWindowVisible(this->hwndDialog)) {
			::ShowWindow(this->hwndDialog, TRUE);
			::SetForegroundWindow(this->hwndDialog);
		}
		else
			::ShowWindow(this->hwndDialog, SW_MINIMIZE);
		break;

	case WM_RBUTTONDOWN: {
		MENU m(5000);

		if (!this->LockECAccess()) break;

		{
			char testpara;
			ULONG ok = this->ReadByteFromEC(59, &testpara);
			if (testpara & 2) m.CheckMenuItem(5060);

			if (this->BluetoothEDR) {
				ok = this->ReadByteFromEC(58, &testpara);
				if (testpara & 16) m.CheckMenuItem(5040);
			}
			else {
				ok = this->ReadByteFromEC(59, &testpara);
				if (testpara & 32) m.CheckMenuItem(5040);
			}
		}

		int mode = this->CurrentModeFromDialog();
		if (mode == 1) {
			m.CheckMenuItem(5001);
			if (this->ActiveMode == 0) {
				m.DisableMenuItem(5002);
				m.DisableMenuItem(5003);
				m.DisableMenuItem(5004);
				m.DisableMenuItem(5005);
			}
		}
		else if (mode == 2)
			m.CheckMenuItem(5002);
		else if (mode == 3)
			m.CheckMenuItem(5005);

		m.InsertItem(this->MenuLabelSM1, 5003, 10);
		m.InsertItem(this->MenuLabelSM2, 5004, 11);

		if (this->SmartLevels2[0].temp2 == 0) {
			m.DeleteMenuItem(5003);
			m.DeleteMenuItem(5004);
		}
		else {
			m.DeleteMenuItem(5002);

			if (mode == 2) {
				if (this->IndSmartLevel == 0)
					m.CheckMenuItem(5003);
				else
					m.CheckMenuItem(5004);
			}
		}

		if (Runs_as_service)
			m.DeleteMenuItem(5020);

		if (IsWindowVisible(this->hwndDialog))
			m.DeleteMenuItem(5010);
		else
			m.DeleteMenuItem(5030);

		if (this->ShowTempIcon == 0)
			m.DeleteMenuItem(5070);
		else
			m.DeleteMenuItem(5080);

		this->FreeECAccess();

		m.Popup(this->hwndDialog);
		break;
	}
	}

	return TRUE;
}

//-------------------------------------------------------------------------
//  attempt graceful shutdown: set fan to BIOS, kill timers, dismiss dialog
//-------------------------------------------------------------------------
bool FANCONTROL::TryClose() {
	// Wait for the work thread to terminate
	if (this->hThread) {
		::WaitForSingleObject(this->hThread, INFINITE);
	}

	if (!this->EcAccess.Lock(100)) {
		// Something is going on, let's do this later
		this->Trace("Delaying close");
		m_needClose = true;
		return false;
	}

	// don't close if we can't set the fan back to bios controlled
	if (!this->ActiveMode || this->SetFan("On close", 0x80, true)) {
		::KillTimer(this->hwndDialog, m_fanTimer);
		::KillTimer(this->hwndDialog, m_titleTimer);
		::KillTimer(this->hwndDialog, m_iconTimer);
		::KillTimer(this->hwndDialog, m_renewTimer);
		BOOL CloHT = CloseHandle(this->hThread);
		this->Trace("Exiting ProcessDialog");
		this->EcAccess.Unlock();
		::PostMessage(this->hwndDialog, WM__DISMISSDLG, IDCANCEL, 0); // exit from ProcessDialog()
		return true;
	}

	m_needClose = true;
	this->EcAccess.Unlock();
	return false;
}

//-------------------------------------------------------------------------
//  switch to smart level 0 (SM1) or 1 (SM2), with logging
//-------------------------------------------------------------------------
void FANCONTROL::SwitchSmartLevel(int level) {
	this->ModeToDialog(2);

	if (level == 0 && this->IndSmartLevel != 0) {
		this->Trace("Activation of Fan Control Profile 'Smart Mode 1'");
	}
	else if (level == 1 && this->IndSmartLevel != 1) {
		this->Trace("Activation of Fan Control Profile 'Smart Mode 2'");
	}

	this->IndSmartLevel = level;

	for (int i = 0; i < 32; i++) {
		if (level == 0) {
			this->SmartLevels[i].temp = this->SmartLevels1[i].temp1;
			this->SmartLevels[i].fan = this->SmartLevels1[i].fan1;
		}
		else {
			this->SmartLevels[i].temp = this->SmartLevels2[i].temp2;
			this->SmartLevels[i].fan = this->SmartLevels2[i].fan2;
		}
	}

	::PostMessage(this->hwndDialog, WM__GETDATA, 0, 0);
}

//-------------------------------------------------------------------------
//  create all named pipes for client GUI communication
//-------------------------------------------------------------------------
void FANCONTROL::CreateAllNamedPipes() {
	for (int i = 0; i < NUM_PIPES; i++) {
		this->hPipes[i] = CreateNamedPipe(
			g_szPipeName,             // pipe name
			PIPE_ACCESS_OUTBOUND,     // write access
			PIPE_TYPE_MESSAGE |       // message type pipe
			PIPE_READMODE_MESSAGE |   // message-read mode
			PIPE_NOWAIT,              // blocking mode
			PIPE_UNLIMITED_INSTANCES, // max. instances
			BUFFER_SIZE,              // output buffer size
			BUFFER_SIZE,              // input buffer size
			NMPWAIT_USE_DEFAULT_WAIT, // client time-out
			NULL);                    // default security attribute

		if (INVALID_HANDLE_VALUE == this->hPipes[i]) {
			char errBuf[128];
			sprintf_s(errBuf, sizeof(errBuf), "Creating Named Pipe %d client GUI was NOT successful.", i);
			this->Trace(errBuf);
			::PostMessage(this->hwndDialog, WM_COMMAND, 5020, 0);
		}
	}
}

//-------------------------------------------------------------------------
//  write data to all named pipes
//-------------------------------------------------------------------------
void FANCONTROL::WriteAllNamedPipes(const char* data) {
	lbResult = bResult;
	for (int i = 0; i < NUM_PIPES; i++) {
		bResult = WriteFile(
			this->hPipes[i],          // handle to pipe
			data,                     // buffer to write from
			(DWORD)(strlen(data) + 1),// number of bytes to write, include the NULL
			&cbBytes,                 // number of bytes written
			NULL);                    // not overlapped I/O
	}
}

//-------------------------------------------------------------------------
//  close all named pipes
//-------------------------------------------------------------------------
void FANCONTROL::CloseAllNamedPipes() {
	for (int i = 0; i < NUM_PIPES; i++) {
		CloseHandle(this->hPipes[i]);
	}
}

//-------------------------------------------------------------------------
//  update the temperature sensor display list
//-------------------------------------------------------------------------
void
FANCONTROL::UpdateTempDisplay(void)
{
	HWND hLV = ::GetDlgItem(this->hwndDialog, 8101);
	if (!hLV) return;

	ListView_DeleteAllItems(hLV);

	LVITEMA lvi = {0};
	lvi.mask = LVIF_TEXT;
	char buf[32];

	for (int i = 0; i < 12; i++)
	{
		int temp = this->State.Sensors[i];
		BOOL show = (temp < 128 && temp != 0) || (this->ShowAll == 1);

		if (show)
		{
			sprintf_s(buf, sizeof(buf), "%d", i + 1);
			lvi.iItem = i;
			lvi.iSubItem = 0;
			lvi.pszText = buf;
			int idx = ListView_InsertItem(hLV, &lvi);

			ListView_SetItemText(hLV, idx, 1, (LPSTR)this->State.SensorName[i]);

			if (temp < 128 && temp != 0)
			{
				if (Fahrenheit)
					sprintf_s(buf, sizeof(buf), "%d F", temp * 9 / 5 + 32);
				else
					sprintf_s(buf, sizeof(buf), "%d C", temp);
			}
			else
			{
				strcpy_s(buf, sizeof(buf), "n/a");
			}
			ListView_SetItemText(hLV, idx, 2, buf);
			sprintf_s(buf, sizeof(buf), "0x%02x", this->State.SensorAddr[i]);
			ListView_SetItemText(hLV, idx, 3, buf);
		}
	}

	this->icontemp = this->State.Sensors[this->iMaxTemp];

	// Invalidate header to update column 2 color based on MaxTemp
	HWND hHeader = ListView_GetHeader(hLV);
	if (hHeader) {
		InvalidateRect(hHeader, NULL, TRUE);
	}
}

//-------------------------------------------------------------------------
//  reading the EC status may take a while, hence do it in a thread
//-------------------------------------------------------------------------
int FANCONTROL::WorkThread() {
	int ok = this->ReadEcStatus(&this->State);

	::PostMessage(this->hwndDialog, WM__NEWDATA, ok, 0);

	return 0;
}

// The texticons will be shown depending on variables
static const int MAX_TEXT_ICONS = 16;
int icon, oldicon;
BOOL dishow(TRUE);
TCHAR myszTip[64];

void FANCONTROL::ProcessTextIcons(void) {
	oldicon = icon;
	if (this->CurrentModeFromDialog() == 1)
		icon = 10;    // gray
	else {
		icon = 11;    // blue
		for (int i = 0; i < ARRAYMAX(this->IconLevels); i++) {
			if (this->MaxTemp >= this->IconLevels[i])
				icon = 12 + i;    // yellow, orange, red
		}
	}

	if (this->IconColorFan) {
		switch (fan1speed / 1000) {
			case 0:
				break;
			case 1:
				icon = 21; //sehr hell grün
				break;
			case 2:
				icon = 22; //hell grün
				break;
			case 3:
				icon = 23; //grün
				break;
			case 4:
				icon = 24; //dunkel grün
				break;
			case 5:
				icon = 25; //sehr dunkel grün
				break;
			case 6:
				icon = 25; //sehr dunkel grün
				break;
			case 7:
				icon = 25; //sehr dunkel grün
				break;
			case 8:
				icon = 25; //sehr dunkel grün
				break;
			default:
				icon = oldicon;
				break;
		};
	}

	this->iFarbeIconB = icon;

	if (lstrcpyn(myszTip, this->Title2, sizeof(myszTip) - 1) == NULL) {
		myszTip[0] = '\0';
	}

	if (pTextIconMutex->Lock(100)) {
		//init ppTbTextIcon
		if (!ppTbTextIcon || this->TaskbarNew) {
			this->TaskbarNew = 0;
			ppTbTextIcon = new CTaskbarTextIcon * [MAX_TEXT_ICONS];
			for (int i = 0; i < MAX_TEXT_ICONS; ++i)
				ppTbTextIcon[i] = NULL;

			ppTbTextIcon[0] = new CTaskbarTextIcon(
				this->m_hinstapp,
				this->hwndDialog, WM__TASKBAR, 0, "", "",  //WM_APP+5000 -> WM__TASKBAR
				this->iFarbeIconB, this->iFontIconB, myszTip);

			if (dishow && !this->NoBallons) {
				if (Fahrenheit) {
					ppTbTextIcon[0]->DiShowballon(
						_T("shows max. temperature in ° F and sensor name, left click on icon shows or hides control window, right click shows menue"),
						_T("TPFanControl new text icon"), NIIF_INFO, 11);
				}
				else {
					ppTbTextIcon[0]->DiShowballon(
						_T("shows max. temperature in ° C and sensor name, left click on icon shows or hides control window, right click shows menue"),
						_T("TPFanControl new text icon"), NIIF_INFO, 11);
				}

				// Input:
				//  szText: [in] Text for the balloon tooltip.
				//  szTitle: [in] Title for the balloon.  This text is shown in bold above
				//           the tooltip text (szText).  Pass "" if you don't want a title.
				//  dwIcon: [in] Specifies an icon to appear in the balloon.  Legal values are:
				//                 NIIF_NONE: No icon
				//                 NIIF_INFO: Information
				//                 NIIF_WARNING: Exclamation
				//                 NIIF_ERROR: Critical error (red circle with X)
				//  uTimeout: [in] Number of seconds for the balloon to remain visible.  Can
				//            be between 10 and 30 inclusive.
				//

				dishow = FALSE;
			}
		}

		char str_value[256];
		for (int i = 0; i < MAX_TEXT_ICONS; ++i) {
			if (ppTbTextIcon[i]) {
				if (Fahrenheit)
					_itoa_s((this->icontemp * 9 / 5) + 32, str_value, sizeof(str_value), 10);
				else
					_itoa_s(this->icontemp, str_value, sizeof(str_value), 10);
				sprintf_s(str_value, sizeof(str_value), "%s", str_value);
				ppTbTextIcon[i]->ChangeText(str_value, this->gSensorNames[iMaxTemp], iFarbeIconB, iFontIconB, myszTip);
			}
		}
		pTextIconMutex->Unlock();
		//this->Trace(LastTooltip); 
	}
}

void FANCONTROL::RemoveTextIcons(void) {
	if (pTextIconMutex->Lock(10000)) {
		if (ppTbTextIcon) {
			for (int i = 0; i < MAX_TEXT_ICONS; ++i) {
				if (ppTbTextIcon[i]) {
					delete ppTbTextIcon[i];
				}
			}
			delete[] ppTbTextIcon;
			ppTbTextIcon = NULL;
		}
		pTextIconMutex->Unlock();
	}
	else {
		_ASSERT(false);//Mutex not av within 10 sec
	}
}
