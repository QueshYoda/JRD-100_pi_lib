#ifndef JRD100_H
#define JRD100_H

#include <string>
#include <vector>
#include <cstdint>
#include <chrono> // Zamanlama için eklendi

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

    // Bu fonksiyonlar artık güncel 'readMultipleTags' tarafından yönetiliyor.
    // bool singleRead();
    // bool multiRead(); 

    /**
     * @brief Belirtilen süre boyunca (milisaniye) etiketleri okur.
     * @param timeout_ms Okuma işleminin ne kadar süreceği (örn: 1000ms = 1 saniye).
     * @return Bulunan etiketlerin bir vektörü.
     */
    std::vector<TagData> readMultipleTags(int timeout_ms = 500);

    bool writeTag(const std::vector<uint8_t>& epc, const std::vector<uint8_t>& data);

    /**
     * @brief Okuyucunun RF çıkış gücünü (TX Power) ayarlar.
     * @param power_dbm Güç (dBm * 100). Örn: 2700 = 27.00 dBm.
     * @return Başarılıysa true.
     */
    bool setTxPower(uint16_t power_dbm);

    /**
     * @brief Okuyucunun mevcut RF çıkış gücünü (TX Power) okur.
     * @return Güç (dBm * 100) veya hata durumunda -1.
     */
    int getTxPower();

private:
    bool configurePort();

    /**
     * @brief Seri porttan bir adet tam 'bb...7e' paketi okur.
     * @return Tam bir paket veya zaman aşımında boş vektör.
     */
    std::vector<uint8_t> readFrame();
    
    /**
     * @brief Bir paketin kontrol toplamını (checksum) hesaplar.
     * @param packet ADDR, CMD, LEN ve DATA içeren bölüm.
     * @return Hesaplanan 8-bit checksum.
     */
    uint8_t calculateChecksum(const uint8_t* data, size_t len);

    std::string portName;
    int baudRate;
    int serialFd;
    bool isOpen;

    /**
     * @brief Parçalı okumalardan gelen veriyi biriktirmek için ara bellek.
     */
    std::vector<uint8_t> readBuffer; 
};

#endif

