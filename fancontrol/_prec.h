//systemheaders in one file for using precompiled headers.

// be compatible downto Windows Vista (required for Windows Event Log API)
#define _WIN32_WINNT 0x0600
//only most neccessary things from windows
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winevt.h>
#pragma comment(lib, "wevtapi.lib")

#include <process.h>
#include <commctrl.h>
#pragma comment(lib, "comctl32.lib")
#include <commdlg.h>
#include <shellapi.h>
#include <tchar.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "winuser.h"
#include "windows.h"
