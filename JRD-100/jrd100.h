// jrd100.h - Arduino compatible version
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
     * @brief Read multiple tags
     * @param timeout_ms Duration in milliseconds
     * @return Found tags
     */
    std::vector<TagData> readMultipleTags(int timeout_ms = 500);

    /**
     * @brief Write data to tag - FULL CONTROL (Arduino compatible)
     * @param epc EPC data for filtering target tag (empty = no filter)
     * @param data Data to write (must be multiple of 2 bytes)
     * @param membank Memory bank (0x01=EPC, 0x02=TID, 0x03=USER, 0x00=RESERVED)
     * @param startAddr Start address (in words)
     * @param accessPassword Access password (default: 0x00000000)
     * @return Success status
     */
    bool writeTag(const std::vector<uint8_t>& epc, 
                  const std::vector<uint8_t>& data,
                  uint8_t membank,
                  uint16_t startAddr,
                  uint32_t accessPassword);

    /**
     * @brief Write data to tag - SIMPLE USAGE
     * @param epc Target EPC (empty = no filter)
     * @param data Data to write
     * @return Success status
     * 
     * Defaults: USER bank (0x03), address 0x0000, password 0x00000000
     */
    bool writeTag(const std::vector<uint8_t>& epc, 
                  const std::vector<uint8_t>& data);

    /**
     * @brief Read data from tag (Arduino compatible)
     * @param data Buffer to receive read data
     * @param size Number of bytes to read (must be multiple of 2)
     * @param membank Memory bank
     * @param startAddr Start address (in words)
     * @param accessPassword Access password
     * @return Success status
     */
    bool readCard(std::vector<uint8_t>& data, 
                  size_t size,
                  uint8_t membank = 0x03,
                  uint16_t startAddr = 0x0000,
                  uint32_t accessPassword = 0x00000000);

    /**
     * @brief Set TX power
     * @param power_dbm 2600 => 26.00 dBm
     * @return Success status
     */
    bool setTxPower(uint16_t power_dbm);

    /**
     * @brief Get TX power
     * @return Power value (in hundredths) or -1 on error
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