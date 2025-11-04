#ifndef JRD100_H
#define JRD100_H

#include <string>
#include <vector>
#include <cstdint>
#include <chrono> 

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

    // bool singleRead();
    // bool multiRead(); 

    /**
     * @brief
     * @param timeout_ms 
     * @return 
     */
    std::vector<TagData> readMultipleTags(int timeout_ms = 500);

    /**
     * @brief Etikete veri yazar.
     * @param epc Hedef etiketi filtrelemek için EPC verisi. Boş vektör ({}) filtre uygulamazi.
     * @param data Yazılacak veri. Uzunluğu 2'nin katı (word) olmalıdır.
     * @return 
     */
    bool writeTag(const std::vector<uint8_t>& epc, const std::vector<uint8_t>& data);

    /**
     * @brief 
     * @param power_dbm 
     * @return 
     */
    bool setTxPower(uint16_t power_dbm);

    /**
     * @brief 
     * @return 
     */
    int getTxPower();

private:
    bool configurePort();

    /**
     * @brief 
     * @return 
     */
    std::vector<uint8_t> readFrame();
    
    /**
     * @brief 
     * @param packet 
     * @return 
     */
    uint8_t calculateChecksum(const uint8_t* data, size_t len);

    std::string portName;
    int baudRate;
    int serialFd;
    bool isOpen;

    /**
     * @brief 
     */
    std::vector<uint8_t> readBuffer; 
};

#endif
