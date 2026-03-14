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
// Note: Testing has established that the embedded controller on the laptop models of interest does not support burst mode, 
//		 so the burst mode command and status bit are not implemented in this code.  If a future model is found that does support burst
//       mode, then the burst enable/disable commands can be issued as needed and the burst status bit can be checked to confirm that 
//       the controller is in burst mode before attempting to read/write multiple bytes in a row. See the ACPI Embedded Controller Interface
//       Specification (above) for details on burst mode.
// 
// From TPFanControl V0.6.3+ V.2.2.0+
constexpr auto ACPI_EC_TYPE1_CTRLPORT = 0x1604;
constexpr auto ACPI_EC_TYPE1_DATAPORT = 0x1600;
// From TPFanControl V0.6.2 final
constexpr auto ACPI_EC_TYPE2_CTRLPORT = 0x66;
constexpr auto ACPI_EC_TYPE2_DATAPORT = 0x62;

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

//--------------------------------------------------------------------------
// initialize embedded controller ports if not yet selected
//--------------------------------------------------------------------------
static void
InitializeEcPorts(int& ctrlPort, int& dataPort) {
	if (ctrlPort != 0 && dataPort != 0) return;

	ctrlPort = ACPI_EC_TYPE1_CTRLPORT;
	dataPort = ACPI_EC_TYPE1_DATAPORT;
}

//--------------------------------------------------------------------------
// switch between known embedded controller port layouts
//--------------------------------------------------------------------------
static void
ToggleEcPorts(int& ctrlPort, int& dataPort) {
	if (ctrlPort == ACPI_EC_TYPE1_CTRLPORT) {
		ctrlPort = ACPI_EC_TYPE2_CTRLPORT;
		dataPort = ACPI_EC_TYPE2_DATAPORT;
	}
	else {
		ctrlPort = ACPI_EC_TYPE1_CTRLPORT;
		dataPort = ACPI_EC_TYPE1_DATAPORT;
	}
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
		::Sleep(1);
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
// read a byte from the embedded controller (EC) via port io 
//-------------------------------------------------------------------------
bool
FANCONTROL::ReadByteFromEC(int offset, char* pdata) {
	UCHAR ecOffset = 0;
	char traceText[160] = "";

	if (pdata == NULL) {
		this->Trace("readec: pdata is null");
		return false;
	}

	if (this->EC_CTRL == 0 || this->EC_DATA == 0) {
		InitializeEcPorts(this->EC_CTRL, this->EC_DATA);
		this->Trace("Using ACPI_EC_TYPE1");
	}

	for (int portAttempt = 0; portAttempt < 2; portAttempt++) {
		const USHORT ctrlPort = static_cast<USHORT>(this->EC_CTRL);
		const USHORT dataPort = static_cast<USHORT>(this->EC_DATA);

		// wait for IBF and OBF to clear; drain stale OBF if present
		if (!WaitForControllerReady(ctrlPort, dataPort)) {
			sprintf_s(traceText, sizeof(traceText),
				"readec: timed out #1 (ctrl=0x%04X data=0x%04X offset=0x%02X)",
				ctrlPort, dataPort, ecOffset);
			this->Trace(traceText);

			if (portAttempt == 0) {
				ToggleEcPorts(this->EC_CTRL, this->EC_DATA);
				this->Trace(this->EC_CTRL == ACPI_EC_TYPE1_CTRLPORT ? "Now using ACPI_EC_TYPE1" : "Now using ACPI_EC_TYPE2");
				continue;
			}

			return false;
		}

		// indicate read operation desired
		WritePort(ctrlPort, ACPI_EC_COMMAND_READ);

		// wait for IBF to clear (command byte removed from EC's input queue)
		if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
			DrainOutputBuffer(ctrlPort, dataPort);
			sprintf_s(traceText, sizeof(traceText),
				"readec: timed out #2 (ctrl=0x%04X data=0x%04X offset=0x%02X)",
				ctrlPort, dataPort, ecOffset);
			this->Trace(traceText);
			return false;
		}

		// indicate read operation desired location
		WritePort(dataPort, ecOffset);

		// wait for IBF to clear (address byte removed from EC's input queue)
		if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
			DrainOutputBuffer(ctrlPort, dataPort);
			sprintf_s(traceText, sizeof(traceText),
				"readec: timed out #3 (ctrl=0x%04X data=0x%04X offset=0x%02X)",
				ctrlPort, dataPort, ecOffset);
			this->Trace(traceText);
			return false;
		}

		// wait for OBF to be SET (data ready to read)
		if (!WaitForAnySet(ctrlPort, ACPI_EC_FLAG_OBF)) {
			DrainOutputBuffer(ctrlPort, dataPort);
			sprintf_s(traceText, sizeof(traceText),
				"readec: timed out #4 (ctrl=0x%04X data=0x%04X offset=0x%02X)",
				ctrlPort, dataPort, ecOffset);
			this->Trace(traceText);
			return false;
		}

		*pdata = static_cast<char>(ReadPort(dataPort));
		return true;
	}

	return false;
}

//-------------------------------------------------------------------------
// write a byte to the embedded controller (EC) via port io
//-------------------------------------------------------------------------
bool
FANCONTROL::WriteByteToEC(int offset, char NewData) {
	UCHAR ecOffset = 0;
	const UCHAR ecData = static_cast<UCHAR>(NewData);
	char traceText[160] = "";

	if (this->EC_CTRL == 0 || this->EC_DATA == 0) {
		InitializeEcPorts(this->EC_CTRL, this->EC_DATA);
		this->Trace("Using ACPI_EC_TYPE1");
	}

	for (int portAttempt = 0; portAttempt < 2; portAttempt++) {
		const USHORT ctrlPort = static_cast<USHORT>(this->EC_CTRL);
		const USHORT dataPort = static_cast<USHORT>(this->EC_DATA);

		// wait for IBF and OBF to clear; drain stale OBF if present
		if (!WaitForControllerReady(ctrlPort, dataPort)) {
			sprintf_s(traceText, sizeof(traceText),
				"writeec: timed out #1 (ctrl=0x%04X data=0x%04X offset=0x%02X data=0x%02X)",
				ctrlPort, dataPort, ecOffset, ecData);
			this->Trace(traceText);

			if (portAttempt == 0) {
				ToggleEcPorts(this->EC_CTRL, this->EC_DATA);
				this->Trace(this->EC_CTRL == ACPI_EC_TYPE1_CTRLPORT ? "Now using ACPI_EC_TYPE1" : "Now using ACPI_EC_TYPE2");
				continue;
			}

			return false;
		}

		// indicate write operation desired
		WritePort(ctrlPort, ACPI_EC_COMMAND_WRITE);

		// wait for IBF to clear (command byte removed from EC's input queue)
		if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
			DrainOutputBuffer(ctrlPort, dataPort);
			sprintf_s(traceText, sizeof(traceText),
				"writeec: timed out #2 (ctrl=0x%04X data=0x%04X offset=0x%02X data=0x%02X)",
				ctrlPort, dataPort, ecOffset, ecData);
			this->Trace(traceText);
			return false;
		}

		// indicate write operation desired location
		WritePort(dataPort, ecOffset);

		// wait for IBF to clear (address byte removed from EC's input queue)
		if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
			DrainOutputBuffer(ctrlPort, dataPort);
			sprintf_s(traceText, sizeof(traceText),
				"writeec: timed out #3 (ctrl=0x%04X data=0x%04X offset=0x%02X data=0x%02X)",
				ctrlPort, dataPort, ecOffset, ecData);
			this->Trace(traceText);
			return false;
		}

		// perform the write operation
		WritePort(dataPort, ecData);

		// wait for IBF to clear (data byte removed from EC's input queue)
		if (!WaitForAllClear(ctrlPort, ACPI_EC_FLAG_IBF)) {
			DrainOutputBuffer(ctrlPort, dataPort);
			sprintf_s(traceText, sizeof(traceText),
				"writeec: timed out #4 (ctrl=0x%04X data=0x%04X offset=0x%02X data=0x%02X)",
				ctrlPort, dataPort, ecOffset, ecData);
			this->Trace(traceText);
			return false;
		}

		return true;
	}

	return false;
}
