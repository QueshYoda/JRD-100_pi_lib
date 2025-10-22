#include "jrd100.h"
#include "CMD.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

JRD100::JRD100(const std::string& port, int baudrate)
    : portName(port), baudRate(baudrate), serialFd(-1), isOpen(false) {}

JRD100::~JRD100() {
    if (isOpen)
        closePort();
}

bool JRD100::openPort() {
    serialFd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serialFd < 0) {
        std::cerr << "[ERROR] Port açılamadı: " << portName << std::endl;
        return false;
    }

    if (!configurePort()) {
        close(serialFd);
        return false;
    }

    isOpen = true;
    std::cout << "[INFO] Port açıldı: " << portName << std::endl;
    return true;
}

void JRD100::closePort() {
    if (isOpen) {
        close(serialFd);
        isOpen = false;
        std::cout << "[INFO] Port kapatıldı." << std::endl;
    }
}

bool JRD100::configurePort() {
    struct termios tty{};
    if (tcgetattr(serialFd, &tty) != 0) {
        std::cerr << "[ERROR] Termios alınamadı.\n";
        return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serialFd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] Port ayarlanamadı.\n";
        return false;
    }
    return true;
}

bool JRD100::sendCommand(const std::vector<uint8_t>& cmd) {
    if (!isOpen) return false;
    ssize_t bytesWritten = write(serialFd, cmd.data(), cmd.size());
    return bytesWritten == (ssize_t)cmd.size();
}

std::vector<uint8_t> JRD100::readResponse() {
    if (!isOpen) return {};
    uint8_t buffer[512];
    ssize_t len = read(serialFd, buffer, sizeof(buffer));
    return std::vector<uint8_t>(buffer, buffer + len);
}

bool JRD100::singleRead() {
    std::vector<uint8_t> cmd(CMD_SINGLE_READ, CMD_SINGLE_READ + sizeof(CMD_SINGLE_READ));
    if (!sendCommand(cmd)) return false;
    auto resp = readResponse();
    std::cout << "[READ] Tekli okuma yanıtı: ";
    for (auto b : resp) std::cout << std::hex << (int)b << " ";
    std::cout << std::dec << std::endl;
    return !resp.empty();
}

bool JRD100::multiRead() {
    std::vector<uint8_t> cmd(CMD_MULTI_READ, CMD_MULTI_READ + sizeof(CMD_MULTI_READ));
    if (!sendCommand(cmd)) return false;
    auto resp = readResponse();
    std::cout << "[READ] Çoklu okuma yanıtı: ";
    for (auto b : resp) std::cout << std::hex << (int)b << " ";
    std::cout << std::dec << std::endl;
    return !resp.empty();
}

std::vector<TagData> JRD100::readMultipleTags() {
    std::vector<TagData> tags;

    std::vector<uint8_t> cmd(CMD_MULTI_READ, CMD_MULTI_READ + sizeof(CMD_MULTI_READ));
    if (!sendCommand(cmd)) return tags;

    auto resp = readResponse();
    if (resp.empty()) return tags;

    size_t i = 0;
    while (i + 1 < resp.size()) { // en az EPC uzunluğu + RSSI olmalı
        TagData t;
        uint8_t epcLen = resp[i];  // İlk byte EPC uzunluğu
        if (i + 1 + epcLen >= resp.size()) break; // taşmayı önle

        t.epc.assign(resp.begin() + i + 1, resp.begin() + i + 1 + epcLen);
        t.rssi = static_cast<int>(resp[i + 1 + epcLen]); // RSSI, EPC’den sonra geliyor
        tags.push_back(t);

        i += 1 + epcLen + 1; // EPC length byte + EPC + RSSI
    }

    // Debug
    for (auto& tag : tags) {
        std::cout << "[TAG] EPC: ";
        for (auto b : tag.epc) std::cout << std::hex << (int)b << " ";
        std::cout << " RSSI: " << std::dec << tag.rssi << std::endl;
    }

    return tags;
}

bool JRD100::writeTag(const std::vector<uint8_t>& epc, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> cmd(CMD_WRITE_TAG, CMD_WRITE_TAG + sizeof(CMD_WRITE_TAG));
    if (!sendCommand(cmd)) return false;

    auto resp = readResponse();
    return !resp.empty();
}
