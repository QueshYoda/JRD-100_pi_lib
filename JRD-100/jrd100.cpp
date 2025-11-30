#include "jrd100.h"
#include "CMD.h"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <iomanip>
#include <algorithm>
#include <sys/ioctl.h>
#include <errno.h>

namespace {
    // Helper: map integer baud rate to termios speed_t
    speed_t baudToSpeed(int baud) {
        switch (baud) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
#ifdef B460800
            case 460800: return B460800;
#endif
#ifdef B921600
            case 921600: return B921600;
#endif
            default: return B115200;
        }
    }

    // Safely print buffer as hex
    void printHexVec(const std::vector<uint8_t>& v) {
        for (auto b : v) std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        std::cout << std::dec << std::setfill(' ');
    }
}

// Constructor
JRD100::JRD100(const std::string& port, int baudrate)
    : portName(port), baudRate(baudrate), serialFd(-1), isOpen(false) {}

// Destructor
JRD100::~JRD100() {
    if (isOpen)
        closePort();
}

// Open the serial port
bool JRD100::openPort() {
    serialFd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd < 0) {
        std::cerr << "[ERROR] Port açılamadı: " << portName << " (" << strerror(errno) << ")\n";
        return false;
    }
    // Set blocking
    fcntl(serialFd, F_SETFL, 0);

    if (!configurePort()) {
        close(serialFd);
        serialFd = -1;
        return false;
    }

    isOpen = true;
    tcflush(serialFd, TCIOFLUSH); // Clear buffers
    std::cout << "[INFO] Port opened: " << portName << " @ " << baudRate << "bps\n";
    return true;
}

// Close the serial port
void JRD100::closePort() {
    std::lock_guard<std::mutex> lk(ioMutex);
    if (isOpen) {
        close(serialFd);
        serialFd = -1;
        isOpen = false;
        std::cout << "[INFO] Port closed.\n";
    }
}

// Configure serial port
bool JRD100::configurePort() {
    struct termios tty{};
    if (tcgetattr(serialFd, &tty) != 0) {
        std::cerr << "[ERROR] tcgetattr failed: " << strerror(errno) << "\n";
        return false;
    }

    speed_t sp = baudToSpeed(baudRate);
    cfsetospeed(&tty, sp);
    cfsetispeed(&tty, sp);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                    // disable break processing
    tty.c_lflag = 0;                           // no signaling chars, no echo
    tty.c_oflag = 0;                           // no remapping, no delays

    tty.c_cc[VMIN]  = 0;   // read returns immediately if no byte
    tty.c_cc[VTIME] = 5;   // 0.5 seconds read timeout (per read())

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off software flow control
    tty.c_cflag |= (CLOCAL | CREAD);       // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);     // no parity
    tty.c_cflag &= ~CSTOPB;                // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;               // no hardware flow control

    if (tcsetattr(serialFd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] tcsetattr failed: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}

bool JRD100::readCard(std::vector<uint8_t>& data, size_t size, uint8_t membank, 
                      uint16_t startAddr, uint32_t accessPassword) {
    if (!isOpen) {
        std::cerr << "[ERROR] Port is closed.\n";
        return false;
    }

    if (size == 0 || size % 2 != 0) {
        std::cerr << "[ERROR] Read size must be non-zero and multiple of 2.\n";
        return false;
    }

    std::cout << "[DEBUG] readCard çağrıldı:\n";
    std::cout << "  Size: " << size << " bytes (" << (size/2) << " words)\n";
    std::cout << "  Memory Bank: 0x" << std::hex << (int)membank << std::dec << "\n";
    std::cout << "  Start Address: 0x" << std::hex << startAddr << std::dec << "\n";

    // Build command - Arduino READ_STORAGE_CMD formatı
    std::vector<uint8_t> buffer;
    
    buffer.push_back(0xBB);  // Start byte
    buffer.push_back(0x00);  // Address
    buffer.push_back(0x39);  // Read command (READ_STORAGE_CMD)
    
    // Length placeholder
    size_t lengthIndex = buffer.size();
    buffer.push_back(0x00);  // Length high
    buffer.push_back(0x00);  // Length low
    
    // Access Password (4 bytes)
    buffer.push_back((accessPassword >> 24) & 0xFF);
    buffer.push_back((accessPassword >> 16) & 0xFF);
    buffer.push_back((accessPassword >> 8) & 0xFF);
    buffer.push_back(accessPassword & 0xFF);
    
    // Memory bank
    buffer.push_back(membank);
    
    // Start address (2 bytes)
    buffer.push_back((startAddr >> 8) & 0xFF);
    buffer.push_back(startAddr & 0xFF);
    
    // Data length in words (2 bytes)
    uint16_t dataWords = static_cast<uint16_t>(size / 2);
    buffer.push_back((dataWords >> 8) & 0xFF);
    buffer.push_back(dataWords & 0xFF);
    
    // Calculate payload length
    uint16_t payloadLen = buffer.size() - 5;
    buffer[lengthIndex] = (payloadLen >> 8) & 0xFF;
    buffer[lengthIndex + 1] = payloadLen & 0xFF;
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 1; i < buffer.size(); i++) {
        checksum += buffer[i];
    }
    buffer.push_back(checksum & 0xFF);
    buffer.push_back(0x7E);

    std::cout << "[DEBUG] Read komutu: ";
    for (auto b : buffer) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    }
    std::cout << std::dec << "\n";

    if (!sendCommand(buffer)) {
        std::cerr << "[ERROR] Read command could not be sent.\n";
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::vector<uint8_t> frame = readFrame();
    if (frame.empty()) {
        std::cerr << "[ERROR] Read response timeout.\n";
        return false;
    }

    std::cout << "[DEBUG] Yanıt: ";
    for (auto b : frame) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    }
    std::cout << std::dec << "\n";

    if (frame.size() < 7) {
        std::cerr << "[ERROR] Response too short.\n";
        return false;
    }

    uint8_t responseCmd = frame[2];
    
    // Error check
    if (responseCmd == 0xFF) {
        if (frame.size() >= 6) {
            uint8_t errorCode = frame[5];
            std::cerr << "[ERROR] Read error, kod: 0x" << std::hex << (int)errorCode << std::dec << "\n";
        }
        return false;
    }
    
    // Success response
    if (responseCmd == 0x39) {
        // Data offset 20'den başlıyor (Arduino kodundaki gibi)
        size_t dataOffset = 20;
        
        if (frame.size() < dataOffset + size + 2) {
            std::cerr << "[WARN] Frame beklenenden kısa, offset ayarlanıyor.\n";
            if (frame.size() >= size + 2) {
                dataOffset = frame.size() - size - 2;
            } else {
                std::cerr << "[ERROR] Yetersiz veri.\n";
                return false;
            }
        }
        
        data.clear();
        data.assign(frame.begin() + dataOffset, frame.begin() + dataOffset + size);
        
        std::cout << "[SUCCESS] Veri okundu: ";
        for (auto b : data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        }
        std::cout << std::dec << "\n";
        
        return true;
    }
    
    std::cerr << "[ERROR] Beklenmeyen yanıt.\n";
    return false;
}

// Compute checksum by summing buf[startIndex..endIndex] inclusive
uint8_t JRD100::calculateChecksumRange(const std::vector<uint8_t>& buf, size_t startIndex, size_t endIndex) {
    uint32_t sum = 0;
    if (buf.empty() || startIndex > endIndex || endIndex >= buf.size()) return 0;
    for (size_t i = startIndex; i <= endIndex; ++i) sum += buf[i];
    return static_cast<uint8_t>(sum & 0xFF);
}

// Thread-safe send command
bool JRD100::sendCommand(const std::vector<uint8_t>& cmd) {
    std::lock_guard<std::mutex> lk(ioMutex);

    if (!isOpen) {
        std::cerr << "[ERROR] Port is closed.\n";
        return false;
    }

    // flush input so old data doesn't confuse responses
    tcflush(serialFd, TCIFLUSH);

    ssize_t bytesWritten = write(serialFd, cmd.data(), cmd.size());
    if (bytesWritten < 0) {
        std::cerr << "[ERROR] write failed: " << strerror(errno) << "\n";
        return false;
    }

    if ((size_t)bytesWritten != cmd.size()) {
        std::cerr << "[ERROR] Partial write. Wrote " << bytesWritten << " of " << cmd.size() << "\n";
        return false;
    }

    std::cout << "[DEBUG] Sent: ";
    printHexVec(cmd);
    std::cout << std::endl;

    return true;
}

// Read a full frame (0xBB ... 0x7E). Returns empty vector on timeout/error.
std::vector<uint8_t> JRD100::readFrame() {
    std::lock_guard<std::mutex> lk(ioMutex);

    if (!isOpen) return {};

    auto startTime = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count() > 1000) {
            // timeout
            // keep buffer small
            if (readBuffer.size() > 4096) readBuffer.clear();
            return {};
        }

        // find start byte
        size_t startPos = std::string::npos;
        for (size_t i = 0; i < readBuffer.size(); ++i) {
            if (readBuffer[i] == 0xBB) {
                startPos = i;
                break;
            }
        }

        if (startPos == std::string::npos) {
            // read more
            uint8_t tempBuf[256];
            ssize_t len = read(serialFd, tempBuf, sizeof(tempBuf));
            if (len > 0) {
                readBuffer.insert(readBuffer.end(), tempBuf, tempBuf + len);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            continue;
        }

        // discard before start
        if (startPos > 0) {
            readBuffer.erase(readBuffer.begin(), readBuffer.begin() + startPos);
        }

        // need at least: BB, ADDR, CMD, LEN_H, LEN_L, ... , CHK, 7E
        if (readBuffer.size() < 7) {
            // read more
            uint8_t tempBuf[256];
            ssize_t len = read(serialFd, tempBuf, sizeof(tempBuf));
            if (len > 0) readBuffer.insert(readBuffer.end(), tempBuf, tempBuf + len);
            else std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // parse length field
        uint16_t dataLen = (static_cast<uint16_t>(readBuffer[3]) << 8) | static_cast<uint16_t>(readBuffer[4]);
        size_t fullFrameLen = 1 /*BB*/ + 1 /*addr*/ + 1 /*cmd*/ + 2 /*len*/ + dataLen + 1 /*chk*/ + 1 /*7E*/;
        if (readBuffer.size() < fullFrameLen) {
            // need more bytes
            uint8_t tempBuf[256];
            ssize_t len = read(serialFd, tempBuf, sizeof(tempBuf));
            if (len > 0) readBuffer.insert(readBuffer.end(), tempBuf, tempBuf + len);
            else std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // Now we have at least one full frame; verify end byte
        if (readBuffer[fullFrameLen - 1] != 0x7E) {
            // malformed; drop this start byte and retry
            readBuffer.erase(readBuffer.begin());
            std::cerr << "[WARN] Frame end byte mismatch, resyncing.\n";
            continue;
        }

        // Extract frame
        std::vector<uint8_t> frame(readBuffer.begin(), readBuffer.begin() + fullFrameLen);
        // remove from buffer
        readBuffer.erase(readBuffer.begin(), readBuffer.begin() + fullFrameLen);

        // checksum index:
        size_t checksumIndex = 1 + 1 + 1 + 2 + dataLen; // start=0 => checksum at this index
        // (calculation range is from index 1 (ADDR) to checksumIndex-1)
        if (checksumIndex >= frame.size()) {
            std::cerr << "[WARN] Unexpected frame size while checksum indexing.\n";
            continue;
        }

        uint8_t expectedChk = frame[checksumIndex];
        uint8_t calcChk = calculateChecksumRange(frame, 1, checksumIndex - 1);
        if (expectedChk != calcChk) {
            std::cerr << "[WARN] Checksum mistake! Expected: 0x" << std::hex << (int)expectedChk
                      << " Calculated: 0x" << (int)calcChk << std::dec << "\n";
            continue;
        }

        std::cout << "[DEBUG] Frame OK: ";
        printHexVec(frame);
        std::cout << std::endl;

        return frame;
    }
}

// Çoklu etiket okuma
std::vector<TagData> JRD100::readMultipleTags(int timeout_ms) {
    std::vector<TagData> tags;
    std::vector<std::vector<uint8_t>> seenEpcs;

    // Send multi read command
    std::vector<uint8_t> cmd(CMD_MULTI_READ, CMD_MULTI_READ + sizeof(CMD_MULTI_READ));
    if (!sendCommand(cmd)) {
        std::cerr << "[ERROR] Çoklu okuma komutu gönderilemedi.\n";
        return tags;
    }

    auto startTime = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count() > timeout_ms) {
            break;
        }

        std::vector<uint8_t> frame = readFrame();
        if (frame.empty()) {
            // no frame in this iteration
            continue;
        }

        if (frame.size() < 7) continue; // at least header

        uint8_t cmd_code = frame[2];

        if (cmd_code == 0x22) { // inventory response
            uint16_t dataLen = (static_cast<uint16_t>(frame[3]) << 8) | static_cast<uint16_t>(frame[4]);
            if (dataLen < 5) {
                std::cerr << "[WARN] Etiket veri paketi çok kısa (dataLen=" << dataLen << ").\n";
                continue;
            }
            // data starts at index 5
            size_t payloadStart = 5;
            if (payloadStart + dataLen > frame.size()) {
                std::cerr << "[WARN] Frame truncated.\n";
                continue;
            }
            const uint8_t* dataPayload = frame.data() + payloadStart;

            TagData t;
            // RSSI - first byte
            t.rssi = static_cast<int>(dataPayload[0]);
            // If you want approximate dBm: t.rssi = -static_cast<int>(dataPayload[0]) / 2;

            // PC (2 bytes) is dataPayload[1], dataPayload[2]
            // EPC starts at dataPayload[3]
            size_t epcBytes = dataLen; // total dataLen
            if (epcBytes <= 3) {
                std::cerr << "[WARN] EPC length zero.\n";
                continue;
            }

            size_t epcLen = epcBytes - 5; // subtract RSSI(1) + PC(2) + CRC(2) => left is EPC
            // If CRC may not always be included, clamp
            if (epcLen > 0 && (3 + epcLen) <= dataLen) {
                size_t epcStartIndex = payloadStart + 3;
                t.epc.assign(frame.begin() + epcStartIndex, frame.begin() + epcStartIndex + epcLen);
            } else {
                // Fallback: try to take up to 12 bytes (as older assumption)
                size_t fallback = std::min<size_t>(12, (dataLen > 3 ? dataLen - 3 : 0));
                size_t epcStartIndex = payloadStart + 3;
                t.epc.assign(frame.begin() + epcStartIndex, frame.begin() + epcStartIndex + fallback);
            }

            // dedupe
            bool seen = false;
            for (const auto& epc : seenEpcs) {
                if (epc == t.epc) {
                    seen = true;
                    break;
                }
            }
            if (!seen && !t.epc.empty()) {
                tags.push_back(t);
                seenEpcs.push_back(t.epc);

                std::cout << "[TAG] EPC: ";
                printHexVec(t.epc);
                std::cout << " | RSSI: " << t.rssi << std::endl;
            }
        } else if (cmd_code == 0xFF) {
            // command ack or error - ignore for now or log
            // optional: parse error code at frame[5]
        }
        // other cmd codes can be handled if needed
    }

    // stop reading
    std::cout << "[INFO] Okuma süresi doldu. Durdurma komutu gönderiliyor.\n";
    std::vector<uint8_t> stopCmd(CMD_STOP_MULTI_READ, CMD_STOP_MULTI_READ + sizeof(CMD_STOP_MULTI_READ));
    sendCommand(stopCmd);

    // optionally read and discard final ack(s)
    // readFrame();

    return tags;
}

// setTxPower (power in hundredths of dBm)
bool JRD100::setTxPower(uint16_t power_dbm) {
    if (!isOpen) {
        std::cerr << "[ERROR] Port is closed.\n";
        return false;
    }

    // build command: BB 00 B6 00 02 [P_H] [P_L] [CHK] 7E
    std::vector<uint8_t> cmd;
    cmd.push_back(0xBB);
    cmd.push_back(0x00);
    cmd.push_back(0xB6);
    cmd.push_back(0x00);
    cmd.push_back(0x02);
    cmd.push_back((power_dbm >> 8) & 0xFF);
    cmd.push_back(power_dbm & 0xFF);
    // placeholder for checksum
    cmd.push_back(0x00);
    cmd.push_back(0x7E);

    // checksum is sum of bytes from index 1 (ADDR) to last payload byte (index 6)
    cmd[7] = calculateChecksumRange(cmd, 1, 6);

    if (!sendCommand(cmd)) {
        std::cerr << "[ERROR] TX power command not send.\n";
        return false;
    }

    std::vector<uint8_t> frame = readFrame();
    if (frame.empty()) {
        std::cerr << "[ERROR] TX güç ayarı onayı gelmedi.\n";
        return false;
    }

    // Expect: BB 00 B6 00 01 00 CHK 7E
    if (frame.size() >= 7 && frame[2] == 0xB6 && frame[4] == 0x01 && frame[5] == 0x00) {
        std::cout << "[INFO] TX Gücü ayarlandı: " << std::fixed << std::setprecision(2) << (power_dbm / 100.0) << " dBm\n";
        return true;
    } else {
        std::cerr << "[ERROR] TX güç ayarı başarısız oldu. Cevap: ";
        printHexVec(frame);
        std::cout << std::endl;
        return false;
    }
}

// getTxPower
int JRD100::getTxPower() {
    if (!isOpen) {
        std::cerr << "[ERROR] Port açık değil, TX gücü okunamıyor.\n";
        return -1;
    }

    // build: BB 00 B7 00 00 [CHK] 7E
    std::vector<uint8_t> cmd = {0xBB, 0x00, 0xB7, 0x00, 0x00, 0x00, 0x7E};
    // checksum at index 5 = sum of index1..4
    cmd[5] = calculateChecksumRange(cmd, 1, 4);

    if (!sendCommand(cmd)) {
        std::cerr << "[ERROR] TX güç okuma komutu gönderilemedi.\n";
        return -1;
    }

    std::vector<uint8_t> frame = readFrame();
    if (frame.empty()) {
        std::cerr << "[ERROR] TX güç okuma cevabı gelmedi.\n";
        return -1;
    }

    // Expect: BB 00 B7 00 02 [P_H] [P_L] [CHK] 7E
    if (frame.size() >= 8 && frame[2] == 0xB7 && frame[4] == 0x02) {
        uint16_t power_dbm = (static_cast<uint16_t>(frame[5]) << 8) | static_cast<uint16_t>(frame[6]);
        std::cout << "[INFO] Mevcut TX Gücü: " << std::fixed << std::setprecision(2) << (power_dbm / 100.0) << " dBm\n";
        return static_cast<int>(power_dbm);
    } else {
        std::cerr << "[ERROR] TX güç okuma başarısız oldu. Cevap: ";
        printHexVec(frame);
        std::cout << std::endl;
        return -1;
    }
}

bool JRD100::writeTag(const std::vector<uint8_t>& epc, const std::vector<uint8_t>& data, 
                       uint8_t membank, uint16_t startAddr, uint32_t accessPassword) {
    (void)epc;
    
    if (!isOpen) {
        std::cerr << "[ERROR] Port is closed.\n";
        return false;
    }

    if (data.empty()) {
        std::cerr << "[ERROR] Write data cannot be empty.\n";
        return false;
    }

    if (data.size() % 2 != 0) {
        std::cerr << "[ERROR] Write data length must be a multiple of 2 bytes (1 word).\n";
        return false;
    }

    std::cout << "[DEBUG] writeTag çağrıldı:\n";
    std::cout << "  Data Length: " << data.size() << " bytes (" << (data.size()/2) << " words)\n";
    std::cout << "  Memory Bank: 0x" << std::hex << (int)membank << std::dec << "\n";
    std::cout << "  Start Address: 0x" << std::hex << startAddr << std::dec << "\n";
    std::cout << "  Access Password: 0x" << std::hex << accessPassword << std::dec << "\n";

    // Build command buffer - Arduino kodundaki gibi
    std::vector<uint8_t> buffer;
    
    // Header
    buffer.push_back(0xBB);  // Start byte
    buffer.push_back(0x00);  // Address
    buffer.push_back(0x49);  // Write command (WRITE_STORAGE_CMD)
    
    // Length placeholder (will be filled later)
    size_t lengthIndex = buffer.size();
    buffer.push_back(0x00);  // Length high
    buffer.push_back(0x00);  // Length low
    
    // Access Password (4 bytes) - Arduino kodundaki sıraya göre
    buffer.push_back((accessPassword >> 24) & 0xFF);
    buffer.push_back((accessPassword >> 16) & 0xFF);
    buffer.push_back((accessPassword >> 8) & 0xFF);
    buffer.push_back(accessPassword & 0xFF);
    
    // Memory bank (1 byte)
    buffer.push_back(membank);
    
    // Start address (2 bytes) - Arduino kodunda SA (start address)
    buffer.push_back((startAddr >> 8) & 0xFF);
    buffer.push_back(startAddr & 0xFF);
    
    // Data length in words (2 bytes)
    uint16_t dataWords = static_cast<uint16_t>(data.size() / 2);
    buffer.push_back((dataWords >> 8) & 0xFF);
    buffer.push_back(dataWords & 0xFF);
    
    // Data bytes
    buffer.insert(buffer.end(), data.begin(), data.end());
    
    // Calculate payload length (everything after length field, before checksum)
    uint16_t payloadLen = buffer.size() - 5; // Subtract header (BB 00 49 LEN_H LEN_L)
    buffer[lengthIndex] = (payloadLen >> 8) & 0xFF;
    buffer[lengthIndex + 1] = payloadLen & 0xFF;
    
    // Calculate checksum - Arduino kodundaki gibi: index 1'den itibaren topla
    uint8_t checksum = 0;
    for (size_t i = 1; i < buffer.size(); i++) {
        checksum += buffer[i];
    }
    buffer.push_back(checksum & 0xFF);
    
    // End byte
    buffer.push_back(0x7E);

    std::cout << "[DEBUG] Komut hazırlandı, toplam: " << buffer.size() << " bytes\n";
    std::cout << "[DEBUG] Gönderilen komut: ";
    for (auto b : buffer) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    }
    std::cout << std::dec << "\n";

    if (!sendCommand(buffer)) {
        std::cerr << "[ERROR] Write command could not be sent.\n";
        return false;
    }

    // Yanıt bekleniyor
    std::cout << "[DEBUG] Yanıt bekleniyor...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::vector<uint8_t> frame = readFrame();
    if (frame.empty()) {
        std::cerr << "[ERROR] Write command acknowledgment not received (timeout).\n";
        return false;
    }

    std::cout << "[DEBUG] Alınan yanıt: ";
    for (auto b : frame) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    }
    std::cout << std::dec << "\n";

    // Response analysis
    if (frame.size() < 7) {
        std::cerr << "[ERROR] Response too short: " << frame.size() << " bytes\n";
        return false;
    }

    uint8_t responseCmd = frame[2];
    
    // Arduino kodunda: WRITE_STORAGE_ERROR[2] == buffer[2] kontrolü var
    // Error response: BB 00 FF ...
    if (responseCmd == 0xFF) {
        if (frame.size() >= 6) {
            uint8_t errorCode = frame[5];
            std::cerr << "[ERROR] Write error, kod: 0x" << std::hex << (int)errorCode << std::dec << "\n";
            
            switch (errorCode) {
                case 0x15: std::cerr << "  -> Etiket bulunamadı\n"; break;
                case 0x16: std::cerr << "  -> Bellek erişim hatası\n"; break;
                case 0x0F: std::cerr << "  -> Yeterli güç yok\n"; break;
                case 0x03: std::cerr << "  -> Bellek korumalı\n"; break;
                case 0x09: std::cerr << "  -> Hatalı parametre\n"; break;
                default: std::cerr << "  -> Bilinmeyen hata\n"; break;
            }
        }
        return false;
    }
    
    // Success: BB 00 49 00 01 00 CHK 7E
    if (responseCmd == 0x49) {
        if (frame.size() >= 6 && frame[5] == 0x00) {
            std::cout << "[SUCCESS] Etikete yazma başarılı!\n";
            return true;
        } else if (frame.size() >= 6) {
            uint8_t statusCode = frame[5];
            std::cerr << "[ERROR] Yazma başarısız, durum: 0x" << std::hex << (int)statusCode << std::dec << "\n";
            return false;
        }
    }
    
    std::cerr << "[ERROR] Beklenmeyen yanıt formatı.\n";
    return false;
}

// Basitleştirilmiş overload - varsayılan parametrelerle
bool JRD100::writeTag(const std::vector<uint8_t>& epc, const std::vector<uint8_t>& data) {
    return writeTag(epc, data, 0x03, 0x0000, 0x00000000);
    // membank=0x03 (USER), startAddr=0x0000, accessPassword=0x00000000
}