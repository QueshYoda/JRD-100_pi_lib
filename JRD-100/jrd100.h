#ifndef JRD100_H
#define JRD100_H

#include <string>
#include <vector>
#include <cstdint>
#include <chrono>
#include <mutex>

struct TagData {
    std::vector<uint8_t> epc;
    int rssi;
};

class JRD100 {
public:
    explicit JRD100(const std::string& port, int baudrate = 115200);
    ~JRD100();

    bool openPort();
    void closePort();

    bool sendCommand(const std::vector<uint8_t>& cmd);

    /**
     * @brief Çoklu etiket okuma
     * @param timeout_ms süre (ms)
     * @return bulunan etiketler
     */
    std::vector<TagData> readMultipleTags(int timeout_ms = 500);

    /**
     * @brief Etikete veri yazar.
     * @param epc Hedef etiketi filtrelemek için EPC verisi. Boş vektör ({}) filtre uygulanmaz.
     * @param data Yazılacak veri. Uzunluğu 2'nin katı (word) olmalıdır.
     * @return başarılı mı
     */
    bool writeTag(const std::vector<uint8_t>& epc, const std::vector<uint8_t>& data);

    /**
     * @brief TX gücü ayarla (ör: 2300 => 23.00 dBm)
     * @param power_dbm power value in hundredths of dBm
     * @return başarılı mı
     */
    bool setTxPower(uint16_t power_dbm);

    /**
     * @brief TX gücünü oku (hundredths of dBm)
     * @return power value veya -1 hata
     */
    int getTxPower();

private:
    bool configurePort();

    /**
     * @brief Bir tam frame okur (start=0xBB, end=0x7E). Timeout internal (1s).
     * @return frame vector veya boş
     */
    std::vector<uint8_t> readFrame();

    /**
     * @brief hesaplama için yardımcı checksum (verilen buffer içindeki startIndex'ten endIndex'e kadar toplar)
     */
    uint8_t calculateChecksumRange(const std::vector<uint8_t>& buf, size_t startIndex, size_t endIndex);

    std::string portName;
    int baudRate;
    int serialFd;
    bool isOpen;

    std::vector<uint8_t> readBuffer;
    std::mutex ioMutex; // send/read eşzamanlaması için
};

#endif // JRD100_H
