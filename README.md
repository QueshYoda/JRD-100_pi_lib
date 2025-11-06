# JRD-100 Pi Lib

A small, lightweight C++ library to operate the JRD-100 UHF RFID reader from a Raspberry Pi (or any Linux system) over a serial port. The library handles framing, checksum validation, simple APIs to read tags, and basic reader configuration.

---

## Table of contents
- Overview
- Features
- Requirements
- Build / Install
- Usage (examples)
- Library API (summary)
- Serial protocol notes
- Troubleshooting
- Contributing
- License

---

## Overview
This project provides:
- A C++ library (libJRD100.a / header files) to communicate with JRD-100 RFID readers.
- Example command-line tools demonstrating common operations (read tags, increase tx power, etc.).
- CMake-based build system for Linux (tested on Raspberry Pi).

---

## Features
- Serial transport abstraction (support /dev/serial0, /dev/ttyUSB0, etc.)
- Automatic frame parsing and checksum validation
- Read multiple tags in a single call (`ReadMultipleTag`)
- Adjust transmit power (`setTxPower`, `getTxPower`)
- Minimal dependencies; uses standard C++17 and POSIX serial APIs
- Example programs and simple error reporting

---

## Requirements
- Linux system (Raspberry Pi recommended)
- JRD-100 (or compatible) UHF RFID reader
- C++17 compiler (g++ or clang++)
- CMake 3.10+
- make
- Permission to access the serial device (sudo or appropriate udev rule)

---

## Build / Install
Clone the repository and build:

git clone https://github.com/QueshYoda/JRD-100_pi_lib.git
cd JRD-100_pi_lib

mkdir build
cd build
cmake ..
make

Binaries and example programs will be placed in `build/bin/`. The static library and headers will be in `build/lib` and `build/include` (depending on project layout).

---

## Usage (examples)
Most example binaries require root (or serial-device access) to open the serial port.

- Read multiple tags (default):
  sudo ./build/bin/ReadMultipleTag /dev/serial0

- Increase transmit power then poll:
  sudo ./build/bin/MultiplePollingIncTx /dev/serial0

- Show usage:
  ./build/bin/ReadMultipleTag --help

Replace `/dev/serial0` with your actual serial device (e.g., `/dev/ttyUSB0`). If you prefer not to run as root, add a udev rule to grant access to your user or add your user to the `dialout` group.

---

## Library API (summary)
(This is a high-level summary — refer to header files in `include/` for exact prototypes.)

- JRD100::Jrd100Serial(port, baudrate)
  - Constructor opens the serial port and configures baud/flow settings.

- bool connect()
  - Open and initialize connection to reader.

- void disconnect()
  - Close serial port.

- std::vector<Tag> readMultipleTags(timeout_ms)
  - Poll the reader and return all tags seen within the timeout.

- bool setTxPower(float dBm)
  - Set transmit power (returns true on success).

- float getTxPower()
  - Query current tx power from the reader.

- Utility functions:
  - buildFrame(...)
  - parseFrame(...)
  - computeChecksum(...)

See `include/` directory and examples for full usage patterns.

---

## Serial protocol notes
- The JRD-100 protocol uses start/end framing bytes and a checksum/CRC per message.
- The library handles:
  - Frame assembly/disassembly
  - Payload length verification
  - Checksum validation and retry logic
- Typical baudrate: check your reader's default (often 115200) — examples assume correct baud is configured.

---

## Troubleshooting
- Permission denied opening serial port:
  - Use sudo or add your user to `dialout` (on Debian-based systems): sudo usermod -aG dialout $USER (then re-login)
  - Or create a udev rule for your device.

- No tags detected:
  - Verify antenna and reader power
  - Increase tx power with `MultiplePollingIncTx`
  - Test with a known-working tag close to the antenna

- Garbled data:
  - Confirm correct baud rate and serial settings (8N1, no flow control unless required)
  - Try `/dev/ttyUSB0` or other adapters if using USB-serial bridges

- Library build errors:
  - Ensure C++17 toolchain is installed and CMake version >= 3.10

---

## Contributing
- Bug reports and pull requests are welcome.
- Preferred process:
  1. Fork the repository
  2. Create a feature branch
  3. Add tests (where appropriate) and update README/docs
  4. Submit a PR with a clear description of the change

---

## TODO
- Implement tag writing (writeTag)
- Add single-tag read mode (singleRead)
- Expose more reader settings (frequency plan, antennas, session)
- Improve diagnostics and error codes
- Add unit tests for frame parsing and checksum functions

---

## License
MIT — see the LICENSE file included in this repository.

---