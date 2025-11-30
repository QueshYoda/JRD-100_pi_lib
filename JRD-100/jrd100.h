// jrd100.h - Arduino koduna göre uyarlanmış
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
     * @brief Etikete veri yazar - TAM KONTROL (Arduino uyumlu)
     * @param epc Hedef etiketi filtrelemek için EPC verisi (boş = filtre yok)
     * @param data Yazılacak veri (2'nin katı byte)
     * @param membank Memory bank (0x01=EPC, 0x02=TID, 0x03=USER, 0x00=RESERVED)
     * @param startAddr Başlangıç adresi (word cinsinden)
     * @param accessPassword Access password (varsayılan: 0x00000000)
     * @return başarılı mı
     */
    bool writeTag(const std::vector<uint8_t>& epc, 
                  const std::vector<uint8_t>& data,
                  uint8_t membank,
                  uint16_t startAddr,
                  uint32_t accessPassword);

    /**
     * @brief Etikete veri yazar - BASİT KULLANIM
     * @param epc Hedef EPC (boş = filtre yok)
     * @param data Yazılacak veri
     * @return başarılı mı
     * 
     * Varsayılanlar: USER bank (0x03), adres 0x0000, password 0x00000000
     */
    bool writeTag(const std::vector<uint8_t>& epc, 
                  const std::vector<uint8_t>& data);

    /**
     * @brief Etiketten veri okur (Arduino uyumlu)
     * @param data Okunan veriyi alacak buffer
     * @param size Okunacak byte sayısı (2'nin katı)
     * @param membank Memory bank
     * @param startAddr Başlangıç adresi (word)
     * @param accessPassword Access password
     * @return başarılı mı
     */
    bool readCard(std::vector<uint8_t>& data, 
                  size_t size,
                  uint8_t membank = 0x03,
                  uint16_t startAddr = 0x0000,
                  uint32_t accessPassword = 0x00000000);

    /**
     * @brief TX gücü ayarla
     * @param power_dbm 2600 => 26.00 dBm
     * @return başarılı mı
     */
    bool setTxPower(uint16_t power_dbm);

    /**
     * @brief TX gücünü oku
     * @return power value (hundredths) veya -1
     */
    int getTxPower();

private:
    bool configurePort();
    std::vector<uint8_t> readFrame();
    uint8_t calculateChecksumRange(const std::vector<uint8_t>& buf, size_t startIndex, size_t endIndex);

    std::string portName;
    int baudRate;
    int serialFd;
    bool isOpen;

    std::vector<uint8_t> readBuffer;
    std::mutex ioMutex;
};

#endif // JRD100_H