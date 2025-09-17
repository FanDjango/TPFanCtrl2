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
#include "TVicPort.h"

// Registers of the embedded controller
// V0.6.3+ V.2.2.0+
constexpr auto ACPI_EC_TYPE1_CTRLPORT = 0x1604;
constexpr auto ACPI_EC_TYPE1_DATAPORT = 0x1600;
// V0.6.2 final
constexpr auto ACPI_EC_TYPE2_CTRLPORT = 0x66;
constexpr auto ACPI_EC_TYPE2_DATAPORT = 0x62;

// Embedded controller status register bits
constexpr auto ACPI_EC_FLAG_OBF = 0x01	/* Output buffer full */;
constexpr auto ACPI_EC_FLAG_IBF = 0x02	/* Input buffer full */;
constexpr auto ACPI_EC_FLAG_CMD = 0x08	/* Input buffer contains a command */;

// Embedded controller commands
constexpr auto ACPI_EC_COMMAND_READ = (char)0x80;
constexpr auto ACPI_EC_COMMAND_WRITE = (char)0x81;
constexpr auto ACPI_EC_BURST_ENABLE = (char)0x82;
constexpr auto ACPI_EC_BURST_DISABLE = (char)0x83;
constexpr auto ACPI_EC_COMMAND_QUERY = (char)0x84;

constexpr int DEFAULT_SLEEP_TICKS = 10;
constexpr int DEFAULT_TIMEOUT_MS = 1000;

//--------------------------------------------------------------------------
// wait for the desired status from the embedded controller (EC) via port io 
//--------------------------------------------------------------------------
// 'onoff' is now a bool, and return value is bool
static bool
WaitForFlags(USHORT port, char flags, bool onoff = false, int timeout = DEFAULT_TIMEOUT_MS) {
    int time = 0;
    for (; time < timeout; time += DEFAULT_SLEEP_TICKS) {
        char data = ReadPort(port);
        bool flagState = (data & flags) != 0;
        if (flagState == onoff) return true;
        ::Sleep(DEFAULT_SLEEP_TICKS);
    }
    return false;
}

//-------------------------------------------------------------------------
// read a byte from the embedded controller (EC) via port io 
//-------------------------------------------------------------------------
bool
FANCONTROL::ReadByteFromEC(int offset, char* pdata) {
    if (this->EC_CTRL == 0) {
        this->EC_CTRL = ACPI_EC_TYPE1_CTRLPORT;
        this->EC_DATA = ACPI_EC_TYPE1_DATAPORT;
        this->Trace("Using ACPI_EC_TYPE1");
    }

    // wait for IBF and OBF to clear
    if (!WaitForFlags(this->EC_CTRL, ACPI_EC_FLAG_IBF | ACPI_EC_FLAG_OBF)) {
        this->Trace("readec: timed out #1");
        if (this->EC_CTRL == ACPI_EC_TYPE1_CTRLPORT) {
            this->EC_CTRL = ACPI_EC_TYPE2_CTRLPORT;
            this->EC_DATA = ACPI_EC_TYPE2_DATAPORT;
            this->Trace("Now using ACPI_EC_TYPE2");
        } else {
            this->EC_CTRL = ACPI_EC_TYPE1_CTRLPORT;
            this->EC_DATA = ACPI_EC_TYPE1_DATAPORT;
            this->Trace("Now using ACPI_EC_TYPE1");
        }
        return false;
    }

    // indicate read operation desired
    WritePort(this->EC_CTRL, ACPI_EC_COMMAND_READ);

    // wait for IBF to clear (command byte removed from EC's input queue)
    if (!WaitForFlags(this->EC_CTRL, ACPI_EC_FLAG_IBF)) {
        this->Trace("readec: timed out #2");
        return false;
    }

    // indicate read operation desired location
    WritePort(this->EC_DATA, static_cast<char>(offset));

    // wait for IBF to clear (address byte removed from EC's input queue)
    if (!WaitForFlags(this->EC_CTRL, ACPI_EC_FLAG_IBF)) {
        this->Trace("readec: timed out #3");
        return false;
    }

    *pdata = ReadPort(this->EC_DATA);

    return true;
}

//-------------------------------------------------------------------------
// write a byte to the embedded controller (EC) via port io
//-------------------------------------------------------------------------
bool
FANCONTROL::WriteByteToEC(int offset, char NewData) {
    // wait for IBF and OBF to clear
    if (!WaitForFlags(this->EC_CTRL, ACPI_EC_FLAG_IBF | ACPI_EC_FLAG_OBF)) {
        this->Trace("writeec: timed out #1");
        return false;
    }

    // indicate write operation desired
    WritePort(this->EC_CTRL, ACPI_EC_COMMAND_WRITE);

    // wait for IBF to clear (command byte removed from EC's input queue)
    if (!WaitForFlags(this->EC_CTRL, ACPI_EC_FLAG_IBF)) {
        this->Trace("writeec: timed out #2");
        return false;
    }

    // indicate write operation desired location
    WritePort(this->EC_DATA, static_cast<char>(offset));

    // wait for IBF to clear (address byte removed from EC's input queue)
    if (!WaitForFlags(this->EC_CTRL, ACPI_EC_FLAG_IBF)) {
        this->Trace("writeec: timed out #3");
        return false;
    }

    // perform the write operation
    WritePort(this->EC_DATA, NewData);

    // wait for IBF to clear (data byte removed from EC's input queue)
    if (!WaitForFlags(this->EC_CTRL, ACPI_EC_FLAG_IBF)) {
        this->Trace("writeec: timed out #4");
        return false;
    }

    return true;
}
