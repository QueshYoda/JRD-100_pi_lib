# JRD-100 UHF RFID Reader Library for Raspberry Pi

A comprehensive C++ library for interfacing with the JRD-100 UHF RFID reader on Raspberry Pi. This library provides full control over tag reading, writing, and reader configuration.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi-red.svg)](https://www.raspberrypi.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-11-blue.svg)](https://isocpp.org/)

## Features

- **Multiple Tag Reading** - Scan and inventory multiple RFID tags simultaneously
- **Tag Writing** - Write data to EPC, USER, TID, and RESERVED memory banks
- **Tag Reading** - Read data from any memory bank with access password support
- **Access Password Finder** - Intelligent brute-force tool to discover tag passwords
- **TX Power Control** - Adjust reader transmission power
- **Thread-Safe Operations** - Mutex-protected serial communication
- **Comprehensive Error Handling** - Detailed error codes and status messages
- **Arduino Compatible** - Protocol implementations match Arduino libraries

## Hardware Requirements

- Raspberry Pi (any model with UART support)
- JRD-100 UHF RFID Reader Module
- EPC Gen2 (ISO 18000-6C) compatible RFID tags
- Serial connection (USB or GPIO UART)

### Connection Options

**USB Connection:**
- Connect JRD-100 via USB to Raspberry Pi
- Default port: `/dev/ttyUSB0`

**GPIO UART Connection:**
- JRD-100 TX → Pi RX (GPIO 15)
- JRD-100 RX → Pi TX (GPIO 14)
- JRD-100 GND → Pi GND
- JRD-100 VCC → Pi 5V
- Default port: `/dev/serial0`

## Installation

### Prerequisites

Install required packages on Raspberry Pi.

### Build Steps

1. Clone the repository
2. Create build directory
3. Run CMake configuration
4. Compile with make

### Available Programs

After building, the following executables are created:

- **ReadMultipleTag** - Scan for multiple tags
- **WriteTag** - Write data to tags
- **MultiplePollingIncTx** - Continuous scanning with TX power adjustment
- **FindMemoryAccessPassword** - Discover tag access passwords

## Usage

### Reading Tags

Scan for RFID tags within range and display their EPC codes and signal strength.

### Writing to Tags

Write custom data to tag memory banks. Data must be in multiples of 2 bytes (word-aligned). Supports writing to USER, EPC, TID, and RESERVED banks.

### Reading from Tags

Read stored data from any memory bank. Specify the memory bank, start address, and number of bytes to read.

### Finding Access Passwords

The password finder uses intelligent strategies:

1. **Common Passwords** - Tests default and frequently used passwords
2. **Manufacturer Passwords** - Tests brand-specific defaults
3. **Repeating Patterns** - Tests patterns like 0x11111111, 0x22222222
4. **Incremental Patterns** - Tests sequential patterns
5. **Low Range Scan** - Brute force from 0x0000 to 0xFFFF
6. **Full Brute Force** - Complete scan (not recommended, takes ~500 days)

### Adjusting TX Power

Control the reader's transmission power for optimal range and performance. Power is specified in hundredths of dBm (e.g., 2600 = 26.00 dBm).

## Memory Banks

RFID tags contain 4 memory banks:

| Bank | ID | Description | Access |
|------|----|----|---------|
| **RESERVED** | 0x00 | Kill & Access passwords | Protected |
| **EPC** | 0x01 | Electronic Product Code | Read/Write |
| **TID** | 0x02 | Tag Identification | Read-Only |
| **USER** | 0x03 | User Memory | Read/Write |

### Memory Addressing

- Memory is addressed in **words** (1 word = 2 bytes)
- All read/write operations must be word-aligned
- Data length must be a multiple of 2 bytes

## API Overview

### Core Functions

**Port Management**
- Open and close serial port connections
- Configure communication parameters

**Tag Operations**
- Scan for multiple tags with timeout
- Read data from specific memory banks
- Write data to any accessible memory bank
- Support for access password protected operations

**Reader Configuration**
- Set transmission power level
- Get current power level
- Configure communication parameters

### Data Structures

**TagData** - Contains EPC code and RSSI signal strength for each detected tag

## Error Codes

| Code | Meaning | Common Cause |
|------|---------|--------------|
| **0x03** | Memory protected | Incorrect access password |
| **0x09** | Invalid parameter | Wrong data length or address |
| **0x0F** | Insufficient power | Tag too far or low TX power |
| **0x15** | Tag not found | No tag in range |
| **0x16** | Memory access error | Invalid memory bank or address |

## Troubleshooting

### Port Access Issues
Add user to dialout group for serial port permissions.

### No Tags Detected
- Verify tag compatibility (EPC Gen2 / ISO 18000-6C)
- Increase transmission power
- Reduce distance between reader and tag
- Check for metal shielding interference

### Write Operations Fail
- Tag may be password-protected
- Use FindMemoryAccessPassword tool to discover password
- Default password is usually 0x00000000

### Communication Errors
- Check serial cable connections
- Verify baud rate settings
- Ensure no electromagnetic interference
- Test with different USB port

### Build Problems
- Update CMake to latest version
- Clean build directory completely
- Check for missing dependencies

## Project Structure

- **JRD-100/** - Core library files (header and implementation)
- **examples/** - Sample programs demonstrating library usage
- **build/** - Compiled binaries and build artifacts
- **CMakeLists.txt** - Build configuration

## Technical Details

**Communication Protocol**
- Baud Rate: 115200 (default)
- Data Bits: 8
- Parity: None
- Stop Bits: 1
- Flow Control: None

**Threading**
- Thread-safe serial operations
- Mutex-protected I/O
- Timeout-based frame reading

**Frame Format**
- Start: 0xBB
- Address: 0x00
- Command byte
- Length field (2 bytes)
- Payload data
- Checksum
- End: 0x7E



## Contributing

Contributions are welcome! Please ensure:
- Code follows existing style
- All features are tested on hardware
- Documentation is updated
- Pull requests include clear descriptions

## License

This project is licensed under the MIT License.

## Support

For questions or issues:
- Open an issue on GitHub
- Check documentation in examples folder
- Review troubleshooting section


**Made for Raspberry Pi • C++11 • Thread-Safe • Production Ready**
