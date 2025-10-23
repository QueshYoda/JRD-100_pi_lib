#include "jrd100.h"
#include "CMD.h" // CMD_MULTI_READ burada olmalı
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono> // Eklendi
#include <thread> // Eklendi

// CMD_STOP_MULTI_READ komutu CMD.h dosyanızda yoksa, onu buraya ekleyin.
// Bu, çoklu okumayı durduran standart bir komuttur.
static const uint8_t CMD_STOP_MULTI_READ[] = {0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E};

// JRD100 Constructor
JRD100::JRD100(const std::string& port, int baudrate)
    : portName(port), baudRate(baudrate), serialFd(-1), isOpen(false) {}

// JRD100 Destructor
JRD100::~JRD100() {
    if (isOpen)
        closePort();
}

// Portu açar
bool JRD100::openPort() {
    serialFd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd < 0) {
        std::cerr << "[ERROR] Port açılamadı: " << portName << std::endl;
        return false;
    }
    fcntl(serialFd, F_SETFL, 0); // Engelleme moduna geri dön

    if (!configurePort()) {
        close(serialFd);
        return false;
    }

    isOpen = true;
    tcflush(serialFd, TCIOFLUSH); // Portu temizle
    std::cout << "[INFO] Port açıldı: " << portName << std::endl;
    return true;
}

// Portu kapatır
void JRD100::closePort() {
    if (isOpen) {
        close(serialFd);
        isOpen = false;
        std::cout << "[INFO] Port kapatıldı." << std::endl;
    }
}

// Port ayarlarını yapar
bool JRD100::configurePort() {
    struct termios tty{};
    if (tcgetattr(serialFd, &tty) != 0) {
        std::cerr << "[ERROR] Termios alınamadı.\n";
        return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                   // disable break processing
    tty.c_lflag = 0;                          // no signaling chars, no echo,
    tty.c_oflag = 0;                          // no remapping, no delays
    
    // --- OKUMA AYARLARI GÜNCELLENDİ ---
    // VMIN = 0, VTIME = 5: Engellemesiz (non-blocking) okuma.
    // read() fonksiyonu hemen döner. Veri varsa veriyi, yoksa 0 döndürür.
    // 0.5 saniye (5 * 100ms) içinde veri gelmezse time-out olur.
    tty.c_cc[VMIN]  = 0; 
    tty.c_cc[VTIME] = 5; 

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);       // ignore modem controls,
    tty.c_cflag &= ~(PARENB | PARODD);     // shut off parity
    tty.c_cflag &= ~CSTOPB;                // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;               // no flow control

    if (tcsetattr(serialFd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] Port ayarlanamadı.\n";
        return false;
    }
    return true;
}

// Kontrol toplamını (checksum) hesaplar
uint8_t JRD100::calculateChecksum(const uint8_t* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return sum;
}

// Komut gönderir
bool JRD100::sendCommand(const std::vector<uint8_t>& cmd) {
    if (!isOpen) {
        std::cerr << "[ERROR] Port açık değil, komut gönderilemiyor.\n";
        return false;
    }
    
    // Göndermeden önce porttaki eski verileri temizle
    tcflush(serialFd, TCIFLUSH); 

    ssize_t bytesWritten = write(serialFd, cmd.data(), cmd.size());
    
    std::cout << "[DEBUG] Sent: ";
    for (auto b : cmd) std::cout << std::hex << (int)b << " ";
    std::cout << std::dec << std::endl;

    return bytesWritten == (ssize_t)cmd.size();
}

// YENİ FONKSİYON: Bir tam 'bb...7e' paketi okur
std::vector<uint8_t> JRD100::readFrame() {
    if (!isOpen) return {};

    auto startTime = std::chrono::steady_clock::now();

    while (true) {
        // 1. Zaman aşımı kontrolü
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count() > 1000) {
            // 1 saniyedir tam bir paket bulamadık
            // std::cerr << "[DEBUG] readFrame timeout." << std::endl;
            readBuffer.clear(); // Hatalı veriyi temizle
            return {};
        }

        // 2. 'bb' (Başlangıç) ara
        size_t startPos = std::string::npos;
        for (size_t i = 0; i < readBuffer.size(); ++i) {
            if (readBuffer[i] == 0xBB) {
                startPos = i;
                break;
            }
        }

        if (startPos == std::string::npos) {
            // 'bb' bulunamadı, daha fazla veri oku
            uint8_t tempBuf[256];
            ssize_t len = read(serialFd, tempBuf, sizeof(tempBuf));
            if (len > 0) {
                readBuffer.insert(readBuffer.end(), tempBuf, tempBuf + len);
            } else {
                // Veri gelmiyorsa kısa bir süre bekle
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            continue; // Döngüye baştan başla
        }

        // 'bb' bulundu, 'bb'den önceki tüm veriyi at
        if (startPos > 0) {
            readBuffer.erase(readBuffer.begin(), readBuffer.begin() + startPos);
        }

        // 3. '7e' (Bitiş) ara
        size_t endPos = std::string::npos;
        for (size_t i = 1; i < readBuffer.size(); ++i) { // 1'den başla (bb'yi atla)
            if (readBuffer[i] == 0x7E) {
                endPos = i;
                break;
            }
        }

        if (endPos == std::string::npos) {
            // '7e' bulunamadı, daha fazla veri oku
            uint8_t tempBuf[256];
            ssize_t len = read(serialFd, tempBuf, sizeof(tempBuf));
            if (len > 0) {
                readBuffer.insert(readBuffer.end(), tempBuf, tempBuf + len);
            } else {
                 std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            continue; // Döngüye baştan başla
        }

        // 4. Paket bulundu!
        std::vector<uint8_t> frame(readBuffer.begin(), readBuffer.begin() + endPos + 1);
        
        // Paketi ara bellekten sil
        readBuffer.erase(readBuffer.begin(), readBuffer.begin() + endPos + 1);

        // 5. Paketi doğrula (Minimum uzunluk ve Checksum)
        if (frame.size() < 7) { // En kısa paket: BB ADDR CMD LEN_H LEN_L CHK 7E
            std::cerr << "[WARN] Kısa paket alındı, atlanıyor.\n";
            continue; // Çok kısa, hatalı paket. Döngüye devam et.
        }

        // Checksum doğrulaması
        uint8_t expectedChecksum = frame[frame.size() - 2];
        // Checksum ADDR'dan DATA'nın sonuna kadardır (BB ve CHK/7E hariç)
        // HATA DÜZELTMESİ: frame.size() - 4 yerine frame.size() - 3 olmalı.
        uint8_t calculatedChecksum = calculateChecksum(&frame[1], frame.size() - 3); 
        
        if (expectedChecksum != calculatedChecksum) {
            std::cerr << "[WARN] Checksum hatası! Beklenen: " << std::hex << (int)expectedChecksum 
                      << " Hesaplanan: " << (int)calculatedChecksum << std::dec << std::endl;
            continue; // Checksum hatalı, paketi atla.
        }

        // 6. Geçerli paket
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


// Henüz implemente edilmedi
bool JRD100::writeTag(const std::vector<uint8_t>& epc, const std::vector<uint8_t>& data) {
    std::cerr << "[WARN] writeTag fonksiyonu henüz implemente edilmedi.\n";
    // TODO: 'CMD_WRITE_TAG' komutunu 'sendCommand' ile gönder
    // ve 'readFrame' ile 'bb...ff...7e' formatındaki onayı bekle.
    return false;
}

/* // Bu fonksiyonlar artık readMultipleTags içinde yönetiliyor
bool JRD100::singleRead() {
    std::cerr << "[WARN] singleRead önerilmiyor. readMultipleTags(100) kullanın.\n";
    auto tags = readMultipleTags(100); // 100ms'lik hızlı bir okuma yap
    return !tags.empty();
}

bool JRD100::multiRead() {
    std::cerr << "[WARN] multiRead önerilmiyor. readMultipleTags(1000) kullanın.\n";
    auto tags = readMultipleTags(1000); // 1 saniyelik okuma yap
    return !tags.empty();
}
*/

