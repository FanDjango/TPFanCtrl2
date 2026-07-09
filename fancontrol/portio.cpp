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

constexpr bool STRICT_EC_READ_OBF_HANDLING = false;

//-------------------------------------------------------------------------
// Context for a single EC operation
//-------------------------------------------------------------------------
struct EcOpContext {
	USHORT ctrlPort;
	USHORT dataPort;
	UCHAR ecOffset;
	int ecData;          // -1 for reads; 0-255 for writes (included in trace output)
	const char* opName;  // "readec" or "writeec"
	char* traceText;
	size_t traceSize;
	FANCONTROL* pThis;
};

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
	EcPortLayout* attempts,
	size_t maxAttempts,
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
	if (attemptCount < maxAttempts) {
		attempts[attemptCount++] = {
			pThis->EC_CTRL,
			pThis->EC_DATA,
			GetEcLayoutName(pThis->EC_CTRL, pThis->EC_DATA)
		};
	}

	// Then try the other known layouts
	for (const auto& layout : EC_PORT_LAYOUTS) {
		if (layout.ctrl == pThis->EC_CTRL && layout.data == pThis->EC_DATA) continue;
		if (attemptCount >= maxAttempts) break;
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
	const DWORD start = ::GetTickCount();

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
// Trace an EC timeout: optionally drain, read status, format and emit trace
//-------------------------------------------------------------------------
static void
TraceEcTimeout(const EcOpContext& ctx, int step, bool drain) {
	if (drain) DrainOutputBuffer(ctx.ctrlPort, ctx.dataPort);

	const UCHAR status = ReadPort(ctx.ctrlPort);

	if (ctx.ecData >= 0) {
		sprintf_s(ctx.traceText, ctx.traceSize,
			"%s: timed out #%d (ctrl=0x%04X data=0x%04X offset=0x%02X data=0x%02X status=0x%02X)",
			ctx.opName, step, ctx.ctrlPort, ctx.dataPort, ctx.ecOffset,
			static_cast<UCHAR>(ctx.ecData), status);
	}
	else {
		sprintf_s(ctx.traceText, ctx.traceSize,
			"%s: timed out #%d (ctrl=0x%04X data=0x%04X offset=0x%02X status=0x%02X)",
			ctx.opName, step, ctx.ctrlPort, ctx.dataPort, ctx.ecOffset, status);
	}

	ctx.pThis->Trace(ctx.traceText);
}

//-------------------------------------------------------------------------
// Send an EC command byte and address byte, with timeout handling
// Handles the common preamble shared by read and write operations:
//   1. Wait for controller ready
//   2. Write command byte, wait for IBF clear
//   3. Write address byte, wait for IBF clear
//-------------------------------------------------------------------------
static bool
SendEcCommandAndAddress(const EcOpContext& ctx, UCHAR command) {
	// wait for IBF and OBF to clear; drain stale OBF if present
	if (!WaitForControllerReady(ctx.ctrlPort, ctx.dataPort)) {
		TraceEcTimeout(ctx, 1, false);
		return false;
	}

	// send command byte
	WritePort(ctx.ctrlPort, command);

	// wait for IBF to clear (command byte removed from EC's input queue)
	if (!WaitForAllClear(ctx.ctrlPort, ACPI_EC_FLAG_IBF)) {
		TraceEcTimeout(ctx, 2, true);
		return false;
	}

	// send address byte
	WritePort(ctx.dataPort, ctx.ecOffset);

	// wait for IBF to clear (address byte removed from EC's input queue)
	if (!WaitForAllClear(ctx.ctrlPort, ACPI_EC_FLAG_IBF)) {
		TraceEcTimeout(ctx, 3, true);
		return false;
	}

	return true;
}

//-------------------------------------------------------------------------
// Helper: Execute EC read operation on given ports
// Returns: true if successful, false if timeout or error
//-------------------------------------------------------------------------
static bool
ExecuteEcRead(const EcOpContext& ctx, char& outData) {
	if (!SendEcCommandAndAddress(ctx, ACPI_EC_COMMAND_READ))
		return false;

	if (STRICT_EC_READ_OBF_HANDLING) {
		// wait for OBF to be SET (data ready to read)
		if (!WaitForAnySet(ctx.ctrlPort, ACPI_EC_FLAG_OBF)) {
			TraceEcTimeout(ctx, 4, true);
			return false;
		}
	}

	outData = static_cast<char>(ReadPort(ctx.dataPort));
	return true;
}

//-------------------------------------------------------------------------
// Helper: Execute EC write operation on given ports
// Returns: true if successful, false if timeout or error
//-------------------------------------------------------------------------
static bool
ExecuteEcWrite(const EcOpContext& ctx) {
	if (!SendEcCommandAndAddress(ctx, ACPI_EC_COMMAND_WRITE))
		return false;

	// perform the write operation
	WritePort(ctx.dataPort, static_cast<UCHAR>(ctx.ecData));

	// wait for IBF to clear (data byte removed from EC's input queue)
	if (!WaitForAllClear(ctx.ctrlPort, ACPI_EC_FLAG_IBF)) {
		TraceEcTimeout(ctx, 4, true);
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

	EcPortLayout attempts[EC_PORT_LAYOUT_COUNT + 1];
	const size_t attemptCount = BuildEcPortAttempts(this, attempts, ARRAYMAX(attempts), traceText, sizeof(traceText));

	for (size_t i = 0; i < attemptCount; i++) {
		const auto& layout = attempts[i];

		const EcOpContext ctx = {
			static_cast<USHORT>(layout.ctrl), static_cast<USHORT>(layout.data),
			ecOffset, -1, "readec", traceText, sizeof(traceText), this
		};

		if (ExecuteEcRead(ctx, *pdata)) {
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

	EcPortLayout attempts[EC_PORT_LAYOUT_COUNT + 1];
	const size_t attemptCount = BuildEcPortAttempts(this, attempts, ARRAYMAX(attempts), traceText, sizeof(traceText));

	for (size_t i = 0; i < attemptCount; i++) {
		const auto& layout = attempts[i];

		const EcOpContext ctx = {
			static_cast<USHORT>(layout.ctrl), static_cast<USHORT>(layout.data),
			ecOffset, static_cast<int>(ecData), "writeec", traceText, sizeof(traceText), this
		};

		if (ExecuteEcWrite(ctx)) {
			if (i > 0) {
				sprintf_s(traceText, sizeof(traceText),
					"writeec: SUCCESS after layout switch to %s (ctrl=0x%04X data=0x%04X)",
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
