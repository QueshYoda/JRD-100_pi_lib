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

static const uint8_t CMD_STOP_MULTI_READ[] = {0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E};

// JRD100 Constructor
JRD100::JRD100(const std::string& port, int baudrate)
    : portName(port), baudRate(baudrate), serialFd(-1), isOpen(false) {}

// JRD100 Destructor
JRD100::~JRD100() {
    if (isOpen)
        closePort();
}

// Open the serial port
bool JRD100::openPort() {
    serialFd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd < 0) {
        std::cerr << "[ERROR] Port açılamadı: " << portName << std::endl;
        return false;
    }
    fcntl(serialFd, F_SETFL, 0); // Blocking mode

    if (!configurePort()) {
        close(serialFd);
        return false;
    }

    isOpen = true;
    tcflush(serialFd, TCIOFLUSH); // Clear input and output buffers
    std::cout << "[INFO] Port opened: " << portName << std::endl;
    return true;
}

// Close the serial port
void JRD100::closePort() {
    if (isOpen) {
        close(serialFd);
        isOpen = false;
        std::cout << "[INFO] Port closed." << std::endl;
    }
}

// Configure serial port settings
bool JRD100::configurePort() {
    struct termios tty{};
    if (tcgetattr(serialFd, &tty) != 0) {
        std::cerr << "[ERROR] Termios not found.\n";
        return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                   // disable break processing
    tty.c_lflag = 0;                          // no signaling chars, no echo,
    tty.c_oflag = 0;                          // no remapping, no delays

    tty.c_cc[VMIN]  = 0; 
    tty.c_cc[VTIME] = 5; 

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);       // ignore modem controls,
    tty.c_cflag &= ~(PARENB | PARODD);     // shut off parity
    tty.c_cflag &= ~CSTOPB;                // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;               // no flow control

    if (tcsetattr(serialFd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] Port not configured.\n";
        return false;
    }
    return true;
}

// CRC
uint8_t JRD100::calculateChecksum(const uint8_t* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return sum;
}

// Send command to the reader
bool JRD100::sendCommand(const std::vector<uint8_t>& cmd) {
    if (!isOpen) {
        std::cerr << "[ERROR] Port is closed.\n";
        return false;
    }
    
    tcflush(serialFd, TCIFLUSH); 

    ssize_t bytesWritten = write(serialFd, cmd.data(), cmd.size());
    
    std::cout << "[DEBUG] Sent: ";
    for (auto b : cmd) std::cout << std::hex << (int)b << " ";
    std::cout << std::dec << std::endl;

    return bytesWritten == (ssize_t)cmd.size();
}

std::vector<uint8_t> JRD100::readFrame() {
    if (!isOpen) return {};

    auto startTime = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count() > 1000) {

            readBuffer.clear(); 
            return {};
        }
        // Look for start byte 0xBB
        size_t startPos = std::string::npos;
        for (size_t i = 0; i < readBuffer.size(); ++i) {
            if (readBuffer[i] == 0xBB) {
                startPos = i;
                break;
            }
        }
        // No start byte found yet
        if (startPos == std::string::npos) {
            uint8_t tempBuf[256];
            ssize_t len = read(serialFd, tempBuf, sizeof(tempBuf));
            if (len > 0) {
                readBuffer.insert(readBuffer.end(), tempBuf, tempBuf + len);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            continue; 
        }
        // Discard bytes before start byte
        if (startPos > 0) {
            readBuffer.erase(readBuffer.begin(), readBuffer.begin() + startPos);
        }
        // Now look for end byte 0x7E
        size_t endPos = std::string::npos;
        for (size_t i = 1; i < readBuffer.size(); ++i) { 
            if (readBuffer[i] == 0x7E) {
                endPos = i;
                break;
            }
        }
        
        if (endPos == std::string::npos) {
            uint8_t tempBuf[256];
            ssize_t len = read(serialFd, tempBuf, sizeof(tempBuf));
            if (len > 0) {
                readBuffer.insert(readBuffer.end(), tempBuf, tempBuf + len);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            continue; 
        }
        // Extract full frame
        std::vector<uint8_t> frame(readBuffer.begin(), readBuffer.begin() + endPos + 1);
        
        readBuffer.erase(readBuffer.begin(), readBuffer.begin() + endPos + 1);
        // Validate frame
        if (frame.size() < 7) { // BB ADDR CMD LEN_H LEN_L CHK 7E
            std::cerr << "[WARN] Short package, skipping.\n";
            continue; 
        }

        // Checksum 
        uint8_t expectedChecksum = frame[frame.size() - 2];
                uint8_t calculatedChecksum = calculateChecksum(&frame[1], frame.size() - 3); 
        
        if (expectedChecksum != calculatedChecksum) {
            std::cerr << "[WARN] Checksum mistake! Expected: " << std::hex << (int)expectedChecksum 
                      << " Calculated: " << (int)calculatedChecksum << std::dec << std::endl;
            continue; 
        }
        // Valid frame
        std::cout << "[DEBUG] Frame OK: ";
        for (auto b : frame) std::cout << std::hex << (int)b << " ";
        std::cout << std::dec << std::endl;
        
        return frame;
    }
}


// TAMAMEN YENİDEN YAZILDI: Çoklu etiket okuma
std::vector<TagData> JRD100::readMultipleTags(int timeout_ms) {
    std::vector<TagData> tags;
    // Benzersiz EPC'leri takip etmek için (isteğe bağlı, ama önerilir)
    std::vector<std::vector<uint8_t>> seenEpcs; 

    // 1. Okuma komutunu gönder
    std::vector<uint8_t> cmd(CMD_MULTI_READ, CMD_MULTI_READ + sizeof(CMD_MULTI_READ));
    if (!sendCommand(cmd)) {
        std::cerr << "[ERROR] Çoklu okuma komutu gönderilemedi.\n";
        return tags;
    }

    auto startTime = std::chrono::steady_clock::now();

    // 2. Belirtilen süre boyunca paketleri oku
    while (true) {
        // Zaman aşımı kontrolü
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count() > timeout_ms) {
            break; // Süre doldu
        }

        // Bir tam paket (frame) oku
        std::vector<uint8_t> frame = readFrame();
        if (frame.empty()) {
            continue; // Geçerli bir paket gelmedi, döngüye devam
        }

        // 3. Gelen paketi işle
        uint8_t cmd_code = frame[2];

        if (cmd_code == 0x22) { // 0x22 = Envanter (Etiket Okuma) Cevabı
            // Loglarınızdaki paketi baz alarak ayrıştırma:
            // bb 2 22 0 11 [bf 30 0 30 39 60 61 c0 6c c2 c0 0 0 1a 96 6a e3] f9 7e
            // [DATA] = 17 byte (0x0011)
            // DATA[0] = RSSI (bf)
            // DATA[1-2] = PC Word (30 00)
            // DATA[3-14] = EPC (12 byte) (30 39 ... 1a 96)
            // DATA[15-16] = Tag CRC (6a e3)

            uint16_t dataLen = (frame[3] << 8) | frame[4];
            if (dataLen < 15) { // En az 1 RSSI + 2 PC + 12 EPC olmalı
                std::cerr << "[WARN] Etiket veri paketi çok kısa.\n";
                continue;
            }

            // Veri alanı 5. index'ten başlar (BB,ADDR,CMD,LEN_H,LEN_L'den sonra)
            const uint8_t* dataPayload = &frame[5];

            TagData t;
            t.rssi = static_cast<int>(dataPayload[0]); // İlk byte RSSI
            
            // Sonraki 12 byte EPC (PC Word'ü (2 byte) atlayarak)
            // Not: Bazen PC word de EPC'nin parçası sayılır,
            // Eğer 14 byte EPC'ye ihtiyacınız varsa: dataPayload + 1, dataPayload + 15
            t.epc.assign(dataPayload + 3, dataPayload + 15); // 12-byte EPC

            // (İsteğe bağlı) Bu etiketi daha önce gördük mü?
            bool seen = false;
            for (const auto& epc : seenEpcs) {
                if (epc == t.epc) {
                    seen = true;
                    break;
                }
            }

            if (!seen) {
                tags.push_back(t);
                seenEpcs.push_back(t.epc);

                // Debug: Bulunan etiketi yazdır
                std::cout << "[TAG] EPC: ";
                for (auto b : t.epc) std::cout << std::hex << (int)b << " ";
                std::cout << " | RSSI: " << std::dec << t.rssi << " dBm" << std::endl;
            }

        } else if (cmd_code == 0xFF) {
            // 0xFF = Komut yanıtı (Muhtemelen okumanın başladığına dair onay)
            // Şimdilik görmezden gelebiliriz.
        }
        // Diğer komut kodları...
    }

    // 4. SÜRE DOLDU - Okumayı DURDUR
    std::cout << "[INFO] Okuma süresi doldu. Durdurma komutu gönderiliyor." << std::endl;
    std::vector<uint8_t> stopCmd(CMD_STOP_MULTI_READ, CMD_STOP_MULTI_READ + sizeof(CMD_STOP_MULTI_READ));
    sendCommand(stopCmd);

    // Okuyucudan gelen son "OK" onayını oku ve atla
    // readFrame(); // Bunu yapmak portu temiz tutar

    return tags;
}


// Set TX Power
bool JRD100::setTxPower(uint16_t power_dbm) {
    if (!isOpen) {
        std::cerr << "[ERROR] Port is closed.\n";
        return false;
    }

    // Command: BB 00 B6 00 02 [P_H] [P_L] [CHK] 7E
    std::vector<uint8_t> cmd = {0xBB, 0x00, 0xB6, 0x00, 0x02, 0x00, 0x00, 0x00, 0x7E};
    
    // Güç değerini ata (Big-Endian)
    cmd[5] = (power_dbm >> 8) & 0xFF; // P_H
    cmd[6] = power_dbm & 0xFF;       // P_L

    // Calculate checksum
    cmd[7] = calculateChecksum(&cmd[1], 6); 

    if (!sendCommand(cmd)) {
        std::cerr << "[ERROR] TX power command not send.\n";
        return false;
    }

    // Wait for acknowledgment from reader
    std::vector<uint8_t> frame = readFrame();
    if (frame.empty()) {
        std::cerr << "[ERROR] TX güç ayarı onayı gelmedi.\n";
        return false;
    }

    // Onay paketini doğrula (BB 00 B6 00 01 00 7A 7E)
    // Başarılı cevap: dataLen=1, status=0
    if (frame.size() >= 7 && frame[2] == 0xB6 && frame[4] == 0x01 && frame[5] == 0x00) {
        std::cout << "[INFO] TX Gücü ayarlandı: " << std::fixed << std::setprecision(2) << (power_dbm / 100.0) << " dBm" << std::endl;
        return true;
    } else {
        std::cerr << "[ERROR] TX güç ayarı başarısız oldu. Cevap: ";
        for (auto b : frame) std::cout << std::hex << (int)b << " ";
        std::cout << std::dec << std::endl;
        return false;
    }
}

int JRD100::getTxPower() {
    if (!isOpen) {
        std::cerr << "[ERROR] Port açık değil, TX gücü okunamıyor.\n";
        return -1;
    }

    // Komut: BB 00 B7 00 00 B7 7E (CMD.h içindeki #22)
    std::vector<uint8_t> cmd = {0xBB, 0x00, 0xB7, 0x00, 0x00, 0xB7, 0x7E};

    if (!sendCommand(cmd)) {
        std::cerr << "[ERROR] TX güç okuma komutu gönderilemedi.\n";
        return -1;
    }

    // Okuyucudan cevabı bekle
    std::vector<uint8_t> frame = readFrame();
    if (frame.empty()) {
        std::cerr << "[ERROR] TX güç okuma cevabı gelmedi.\n";
        return -1;
    }

    // Verify package (BB 00 B7 00 02 [P_H] [P_L] [CHK] 7E)
    // dataLen=2
    if (frame.size() >= 8 && frame[2] == 0xB7 && frame[4] == 0x02) {
        uint16_t power_dbm = (frame[5] << 8) | frame[6];
        std::cout << "[INFO] Mevcut TX Gücü: " << std::fixed << std::setprecision(2) << (power_dbm / 100.0) << " dBm" << std::endl;
        return static_cast<int>(power_dbm);
    } else {
        std::cerr << "[ERROR] TX güç okuma başarısız oldu. Cevap: ";
        for (auto b : frame) std::cout << std::hex << (int)b << " ";
        std::cout << std::dec << std::endl;
        return -1;
    }
}


// YENİ İMPLEMENTASYON: writeTag
bool JRD100::writeTag(const std::vector<uint8_t>& epc, const std::vector<uint8_t>& data) {
    if (!isOpen) {
        std::cerr << "[ERROR] Port is closed.\n";
        return false;
    }

    if (data.empty()) {
        std::cerr << "[ERROR] Write data cannot be empty.\n";
        return false;
    }

    // Veri uzunluğu 2'nin katı olmalı (16-bit words)
    if (data.size() % 2 != 0) {
        std::cerr << "[ERROR] Write data length must be a multiple of 2 bytes (1 word).\n";
        return false;
    }

    // Komut yükünü (payload) oluştur
    std::vector<uint8_t> payload;

    // 1. Access Password (2 bytes) - default 0x0000
    payload.push_back(0x00);
    payload.push_back(0x00);

    // 2. EPC Filtre Uzunluğu (2 bytes, Big-Endian) - BYTE cinsinden
    //    Eğer epc vektörü boşsa, filtre uygulanmaz (uzunluk 0 olur).
    payload.push_back((epc.size() >> 8) & 0xFF);
    payload.push_back(epc.size() & 0xFF);

    // 3. Hafıza Bankası (1 byte)
    //    0x01 = EPC, 0x02 = TID, 0x03 = USER
    //    CMD_WRITE_TAG örneğindeki gibi USER bankasına (0x03) yazıyoruz.
    payload.push_back(0x03); 

    // 4. Başlangıç Adresi (2 bytes, Big-Endian) - WORD cinsinden
    //    Adres 0x0000'dan başlıyoruz.
    payload.push_back(0x00);
    payload.push_back(0x00);

    // 5. Veri Uzunluğu (2 bytes, Big-Endian) - WORD cinsinden
    uint16_t dataLenWords = data.size() / 2;
    payload.push_back((dataLenWords >> 8) & 0xFF);
    payload.push_back(dataLenWords & 0xFF);

    // 6. Veri (N bytes)
    payload.insert(payload.end(), data.begin(), data.end());

    // 7. EPC Filtresi (M bytes)
    payload.insert(payload.end(), epc.begin(), epc.end());

    // ---
    // Tam komut çerçevesini (frame) oluştur
    // ---
    std::vector<uint8_t> cmd;
    cmd.push_back(0xBB); // Start
    cmd.push_back(0x00); // Address
    cmd.push_back(0x49); // Command: WRITE_TAG

    // Toplam Yük (Payload) Uzunluğu 
    cmd.push_back((payload.size() >> 8) & 0xFF);
    cmd.push_back(payload.size() & 0xFF);

    // Yükü (Payload) ekle
    cmd.insert(cmd.end(), payload.begin(), payload.end());

    // Checksum (Adres, Komut, Uzunluk ve Yük üzerinden hesaplanır)
    cmd.push_back(calculateChecksum(&cmd[1], cmd.size() - 1));

    // End
    cmd.push_back(0x7E);

    // Komutu gönder
    if (!sendCommand(cmd)) {
        std::cerr << "[ERROR] Write command not send.\n";
        return false;
    }

    // Okuyucudan onayı bekle
    std::vector<uint8_t> frame = readFrame();
    if (frame.empty()) {
        std::cerr << "[ERROR] Write command acknowledgment not received.\n";
        return false;
    }

    // Onayı doğrula
    // Başarılı cevap: BB 00 49 00 01 00 [CHK] 7E (Status 0x00 = OK)
    // (setTxPower fonksiyonundaki gibi benzer bir cevap formatı varsayıyoruz)
    if (frame.size() >= 7 && frame[2] == 0x49 && frame[4] == 0x01 && frame[5] == 0x00) {
        std::cout << "[INFO] Tag write successful." << std::endl;
        return true;
    } 
    // Genel hata cevabı
    else if (frame.size() >= 7 && frame[2] == 0xFF) {
        uint8_t statusCode = frame[5];
        std::cerr << "[ERROR] Tag write failed. Reader returned error code: 0x" << std::hex << (int)statusCode << std::dec << std::endl;
        return false;
    }
    // 0x49 komutuna özel hata cevabı
    else if (frame.size() >= 7 && frame[2] == 0x49 && frame[4] == 0x01 && frame[5] != 0x00) {
            uint8_t statusCode = frame[5];
        std::cerr << "[ERROR] Tag write failed. Reader returned error code: 0x" << std::hex << (int)statusCode << std::dec << std::endl;
        return false;
    }
    // Bilinmeyen cevap
    else {
        std::cerr << "[ERROR] Unknown response after write command: ";
        for (auto b : frame) std::cout << std::hex << (int)b << " ";
        std::cout << std::dec << std::endl;
        return false;
    }
}
