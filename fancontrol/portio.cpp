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

// Docs:
// https://uefi.org/htmlspecs/ACPI_Spec_6_4_html/12_ACPI_Embedded_Controller_Interface_Specification/embedded-controller-command-set.html

#include "_prec.h"
#include "fancontrol.h"
#include "TVicPort.h"

// Registers of the embedded controller
//
// Note: Testing on a Lenovo P53 with EC at 0x1600/0x1604 and a Lenovo T430 with EC at 0x62/0x66 suggests that the EC command set is
//       consistent across these two models, but the port addresses differ.  This code attempts to auto-detect which port layout is 
//       present by trying one, then the other if the first fails.
//
// Note: Testing has established that the embedded controller on the laptop models of interest do not support burst mode, 
//		 so the burst mode command and status bit are not implemented in this code.  If a future model is found that does support burst
//       mode, then the burst enable/disable commands can be issued as needed and the burst status bit can be checked to confirm that 
//       the controller is in burst mode before attempting to read/write multiple bytes in a row. See the ACPI Embedded Controller Interface
//       Specification (link above) for details on burst mode.
// 
// From TPFanControl V0.6.3+ V.2.2.0+
constexpr auto ACPI_EC_TYPE1_CTRLPORT = 0x1604;
constexpr auto ACPI_EC_TYPE1_DATAPORT = 0x1600;
// From TPFanControl V0.6.2 final
constexpr auto ACPI_EC_TYPE2_CTRLPORT = 0x66;
constexpr auto ACPI_EC_TYPE2_DATAPORT = 0x62;

struct EcPortLayout {
	int ctrl;
	int data;
	const char* name;
};

constexpr EcPortLayout EC_PORT_LAYOUTS[] = {
	{ ACPI_EC_TYPE1_CTRLPORT, ACPI_EC_TYPE1_DATAPORT, "ACPI_EC_TYPE1" },
	{ ACPI_EC_TYPE2_CTRLPORT, ACPI_EC_TYPE2_DATAPORT, "ACPI_EC_TYPE2" },
};
constexpr size_t EC_PORT_LAYOUT_COUNT = sizeof(EC_PORT_LAYOUTS) / sizeof(EC_PORT_LAYOUTS[0]);

// Embedded controller status register bits
constexpr auto ACPI_EC_FLAG_OBF     = static_cast<UCHAR>(0x01);	/* Output buffer full */
constexpr auto ACPI_EC_FLAG_IBF     = static_cast<UCHAR>(0x02);	/* Input buffer full */
constexpr auto ACPI_EC_FLAG_CMD     = static_cast<UCHAR>(0x08);	/* Input buffer contains a command */
constexpr auto ACPI_EC_FLAG_BURST   = static_cast<UCHAR>(0x10);	/* Controller is in burst mode */
constexpr auto ACPI_EC_FLAG_SCI_EVT = static_cast<UCHAR>(0x20);	/* Indicates SCI event is pending */
constexpr auto ACPI_EC_FLAG_SMI_EVT = static_cast<UCHAR>(0x40);	/* Indicates SMI event is pending */

// Embedded controller commands
constexpr auto ACPI_EC_COMMAND_READ = static_cast<UCHAR>(0x80);
constexpr auto ACPI_EC_COMMAND_WRITE = static_cast<UCHAR>(0x81);
constexpr auto ACPI_EC_BURST_ENABLE = static_cast<UCHAR>(0x82);
constexpr auto ACPI_EC_BURST_DISABLE = static_cast<UCHAR>(0x83);
constexpr auto ACPI_EC_COMMAND_QUERY = static_cast<UCHAR>(0x84);

constexpr int DEFAULT_SLEEP_TICKS = 10;
constexpr int DEFAULT_TIMEOUT_MS = 1000;
constexpr int RECOVERY_DRAIN_READS = 8;
constexpr int TRACE_BUFFER_SIZE = 160;

//--------------------------------------------------------------------------
// initialize embedded controller ports if not yet selected
//--------------------------------------------------------------------------
static void
InitializeEcPorts(int& ctrlPort, int& dataPort) {
	if (ctrlPort != 0 && dataPort != 0) return;

	ctrlPort = EC_PORT_LAYOUTS[0].ctrl;
	dataPort = EC_PORT_LAYOUTS[0].data;
}

//--------------------------------------------------------------------------
// resolve a human-readable EC layout name for tracing
//--------------------------------------------------------------------------
static const char*
GetEcLayoutName(int ctrlPort, int dataPort) {
	for (const auto& layout : EC_PORT_LAYOUTS) {
		if (layout.ctrl == ctrlPort && layout.data == dataPort) {
			return layout.name;
		}
	}
	return "ACPI_EC_UNKNOWN";
}

//--------------------------------------------------------------------------
// build ordered EC layout attempts: current first, then remaining known ones
//--------------------------------------------------------------------------
static size_t
BuildEcPortAttempts(
	FANCONTROL* pThis,
	EcPortLayout (&attempts)[EC_PORT_LAYOUT_COUNT],
	char* traceText,
	size_t traceSize) {
	size_t attemptCount = 0;

	if (pThis->EC_CTRL == 0 || pThis->EC_DATA == 0) {
		InitializeEcPorts(pThis->EC_CTRL, pThis->EC_DATA);
		sprintf_s(traceText, traceSize,
			"Trying %s (ctrl=0x%04X data=0x%04X) first",
			EC_PORT_LAYOUTS[0].name, EC_PORT_LAYOUTS[0].ctrl, EC_PORT_LAYOUTS[0].data);
		pThis->Trace(traceText);
	}

	// First try currently selected ports
	attempts[attemptCount++] = {
		pThis->EC_CTRL,
		pThis->EC_DATA,
		GetEcLayoutName(pThis->EC_CTRL, pThis->EC_DATA)
	};

	// Then try the other known layouts
	for (const auto& layout : EC_PORT_LAYOUTS) {
		if (layout.ctrl == pThis->EC_CTRL && layout.data == pThis->EC_DATA) continue;
		attempts[attemptCount++] = layout;
	}

	return attemptCount;
}

//--------------------------------------------------------------------------
// wait until all requested bits are clear
//--------------------------------------------------------------------------
static bool
WaitForAllClear(USHORT port, UCHAR flags, int timeout = DEFAULT_TIMEOUT_MS) {
	if (timeout < 0) timeout = DEFAULT_TIMEOUT_MS;

	const DWORD start = ::GetTickCount();

	for (;;) {
		const UCHAR data = ReadPort(port);
		if ((data & flags) == 0) return true;
		if ((::GetTickCount() - start) >= static_cast<DWORD>(timeout)) return false;
		::Sleep(DEFAULT_SLEEP_TICKS);
	}
}

//--------------------------------------------------------------------------
// wait until any requested bit is set
//--------------------------------------------------------------------------
static bool
WaitForAnySet(USHORT port, UCHAR flags, int timeout = DEFAULT_TIMEOUT_MS) {
	if (timeout < 0) timeout = DEFAULT_TIMEOUT_MS;

	const DWORD start = ::GetTickCount();
	const ULONGLONG timeoutMs = static_cast<ULONGLONG>(timeout);

	for (;;) {
		const UCHAR data = ReadPort(port);
		if ((data & flags) != 0) return true;
		if ((::GetTickCount() - start) >= static_cast<DWORD>(timeout)) return false;
		::Sleep(DEFAULT_SLEEP_TICKS);
	}
}

//--------------------------------------------------------------------------
// best-effort drain of a stale output buffer so a new transaction can start
//--------------------------------------------------------------------------
static void
DrainOutputBuffer(USHORT ctrlPort, USHORT dataPort) {
	for (int i = 0; i < RECOVERY_DRAIN_READS; i++) {
		const UCHAR status = ReadPort(ctrlPort);

		if ((status & ACPI_EC_FLAG_OBF) == 0) break;

		(void)ReadPort(dataPort);

		::Sleep(DEFAULT_SLEEP_TICKS);
	}
}

//--------------------------------------------------------------------------
// wait for controller ready state; drains stale output if necessary
//--------------------------------------------------------------------------
static bool
WaitForControllerReady(USHORT ctrlPort, USHORT dataPort, int timeout = DEFAULT_TIMEOUT_MS) {
	if (timeout < 0) timeout = DEFAULT_TIMEOUT_MS;

	const DWORD start = ::GetTickCount();

	for (;;) {
		const UCHAR status = ReadPort(ctrlPort);

		if ((status & ACPI_EC_FLAG_OBF) != 0) {
			(void)ReadPort(dataPort);
		}
		else if ((status & ACPI_EC_FLAG_IBF) == 0) {
			return true;
		}

		if ((::GetTickCount() - start) >= static_cast<DWORD>(timeout)) return false;

		::Sleep(DEFAULT_SLEEP_TICKS);
	}
}

//-------------------------------------------------------------------------
// Helper: Execute EC read operation on given ports
// Returns: true if successful, false if timeout or error
//-------------------------------------------------------------------------
static bool
ExecuteEcRead(USHORT ctrlPort, USHORT dataPort, UCHAR ecOffset, char& outData, char* traceText, size_t traceSize, FANCONTROL* pThis) {
	// wait for IBF and OBF to clear; drain stale OBF if present
	if (!WaitForControllerReady(ctrlPort, dataPort)) {
		const UCHAR status = ReadPort(ctrlPort);
		sprintf_s(traceText, traceSize,
			"readec: timed out #1 (ctrl=0x%04X data=0x%04X offset=0x%02X status=0x%02X)",
			ctrlPort, dataPort, ecOffset, status);
		pThis->Trace(traceText);
		return false;
	}

	// indicate read operation desired
	WritePort(ctrlPort, ACPI_EC_COMMAND_READ);

	// wait for IBF to clear (command byte removed from EC's input queue)
	if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
		DrainOutputBuffer(ctrlPort, dataPort);
		const UCHAR status = ReadPort(ctrlPort);
		sprintf_s(traceText, traceSize,
			"readec: timed out #2 (ctrl=0x%04X data=0x%04X offset=0x%02X status=0x%02X)",
			ctrlPort, dataPort, ecOffset, status);
		pThis->Trace(traceText);
		return false;
	}

	// indicate read operation desired location
	WritePort(dataPort, ecOffset);

	// wait for IBF to clear (address byte removed from EC's input queue)
	if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
		DrainOutputBuffer(ctrlPort, dataPort);
		const UCHAR status = ReadPort(ctrlPort);
		sprintf_s(traceText, traceSize,
			"readec: timed out #3 (ctrl=0x%04X data=0x%04X offset=0x%02X status=0x%02X)",
			ctrlPort, dataPort, ecOffset, status);
		pThis->Trace(traceText);
		return false;
	}

	// wait for OBF to be SET (data ready to read)
	if (!WaitForAnySet(ctrlPort, ACPI_EC_FLAG_OBF)) {
		DrainOutputBuffer(ctrlPort, dataPort);
		const UCHAR status = ReadPort(ctrlPort);
		sprintf_s(traceText, traceSize,
			"readec: timed out #4 (ctrl=0x%04X data=0x%04X offset=0x%02X status=0x%02X)",
			ctrlPort, dataPort, ecOffset, status);
		pThis->Trace(traceText);
		return false;
	}

	outData = static_cast<char>(ReadPort(dataPort));

	return true;
}

//-------------------------------------------------------------------------
// Helper: Execute EC write operation on given ports
// Returns: true if successful, false if timeout or error
//-------------------------------------------------------------------------
static bool
ExecuteEcWrite(USHORT ctrlPort, USHORT dataPort, UCHAR ecOffset, UCHAR ecData, char* traceText, size_t traceSize, FANCONTROL* pThis) {
	// wait for IBF and OBF to clear; drain stale OBF if present
	if (!WaitForControllerReady(ctrlPort, dataPort)) {
		const UCHAR status = ReadPort(ctrlPort);
		sprintf_s(traceText, traceSize,
			"writeec: timed out #1 (ctrl=0x%04X data=0x%04X offset=0x%02X data=0x%02X status=0x%02X)",
			ctrlPort, dataPort, ecOffset, ecData, status);
		pThis->Trace(traceText);
		return false;
	}

	// indicate write operation desired
	WritePort(ctrlPort, ACPI_EC_COMMAND_WRITE);

	// wait for IBF to clear (command byte removed from EC's input queue)
	if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
		DrainOutputBuffer(ctrlPort, dataPort);
		const UCHAR status = ReadPort(ctrlPort);
		sprintf_s(traceText, traceSize,
			"writeec: timed out #2 (ctrl=0x%04X data=0x%04X offset=0x%02X data=0x%02X status=0x%02X)",
			ctrlPort, dataPort, ecOffset, ecData, status);
		pThis->Trace(traceText);
		return false;
	}

	// indicate write operation desired location
	WritePort(dataPort, ecOffset);

	// wait for IBF to clear (address byte removed from EC's input queue)
	if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
		DrainOutputBuffer(ctrlPort, dataPort);
		const UCHAR status = ReadPort(ctrlPort);
		sprintf_s(traceText, traceSize,
			"writeec: timed out #3 (ctrl=0x%04X data=0x%04X offset=0x%02X data=0x%02X status=0x%02X)",
			ctrlPort, dataPort, ecOffset, ecData, status);
		pThis->Trace(traceText);
		return false;
	}

	// perform the write operation
	WritePort(dataPort, ecData);

	// wait for IBF to clear (data byte removed from EC's input queue)
	if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
		DrainOutputBuffer(ctrlPort, dataPort);
		const UCHAR status = ReadPort(ctrlPort);
		sprintf_s(traceText, traceSize,
			"writeec: timed out #4 (ctrl=0x%04X data=0x%04X offset=0x%02X data=0x%02X status=0x%02X)",
			ctrlPort, dataPort, ecOffset, ecData, status);
		pThis->Trace(traceText);
		return false;
	}

	return true;
}

//-------------------------------------------------------------------------
// read a byte from the embedded controller (EC) via port io 
//-------------------------------------------------------------------------
bool
FANCONTROL::ReadByteFromEC(int offset, char* pdata) {
	char traceText[TRACE_BUFFER_SIZE] = "";

	const UCHAR ecOffset = static_cast<UCHAR>(offset);

	EcPortLayout attempts[EC_PORT_LAYOUT_COUNT];
	const size_t attemptCount = BuildEcPortAttempts(this, attempts, traceText, sizeof(traceText));

	for (size_t i = 0; i < attemptCount; i++) {
		const auto& layout = attempts[i];

		if (ExecuteEcRead(layout.ctrl, layout.data, ecOffset, *pdata, traceText, sizeof(traceText), this)) {
			if (i > 0) {
				sprintf_s(traceText, sizeof(traceText),
					"readec: SUCCESS after layout switch to %s (ctrl=0x%04X data=0x%04X)",
					layout.name, layout.ctrl, layout.data);
				this->Trace(traceText);
				this->EC_CTRL = layout.ctrl;
				this->EC_DATA = layout.data;
			}
			return true;
		}

		if (i + 1 < attemptCount) {
			const auto& nextLayout = attempts[i + 1];
			sprintf_s(traceText, sizeof(traceText),
				"readec: switching to %s (ctrl=0x%04X data=0x%04X)",
				nextLayout.name, nextLayout.ctrl, nextLayout.data);
			this->Trace(traceText);
		}
	}

	this->Trace("readec: FAILED - all ACPI_EC port layouts exhausted");
	return false;
}

//-------------------------------------------------------------------------
// write a byte to the embedded controller (EC) via port io
//-------------------------------------------------------------------------
bool
FANCONTROL::WriteByteToEC(int offset, char NewData) {
	char traceText[TRACE_BUFFER_SIZE] = "";

	const UCHAR ecOffset = static_cast<UCHAR>(offset);
	const UCHAR ecData = static_cast<UCHAR>(NewData);

	EcPortLayout attempts[EC_PORT_LAYOUT_COUNT];
	const size_t attemptCount = BuildEcPortAttempts(this, attempts, traceText, sizeof(traceText));

	for (size_t i = 0; i < attemptCount; i++) {
		const auto& layout = attempts[i];

		if (ExecuteEcWrite(layout.ctrl, layout.data, ecOffset, ecData, traceText, sizeof(traceText), this)) {
			if (i > 0) {
				sprintf_s(traceText, sizeof(traceText),
					"writeec: SUCCESS after port switch to %s (ctrl=0x%04X data=0x%04X)",
					layout.name, layout.ctrl, layout.data);
				this->Trace(traceText);
				this->EC_CTRL = layout.ctrl;
				this->EC_DATA = layout.data;
			}
			return true;
		}

		if (i + 1 < attemptCount) {
			const auto& nextLayout = attempts[i + 1];
			sprintf_s(traceText, sizeof(traceText),
				"writeec: switching to %s (ctrl=0x%04X data=0x%04X)",
				nextLayout.name, nextLayout.ctrl, nextLayout.data);
			this->Trace(traceText);
		}
	}

	this->Trace("writeec: FAILED - all ACPI_EC port layouts exhausted");
	return false;
}
