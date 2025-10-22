#ifndef JRD100_H
#define JRD100_H

#include <string>
#include <vector>
#include <cstdint>

struct TagData {
    std::vector<uint8_t> epc;
    int rssi;
};

class JRD100 {
public:
    explicit JRD100(const std::string& port, int baudrate);
    ~JRD100();

    bool openPort();
    void closePort();
    bool configurePort();

    bool sendCommand(const std::vector<uint8_t>& cmd);
    std::vector<uint8_t> readResponse();

    bool singleRead();
    bool multiRead();

    std::vector<TagData> readMultipleTags();
    bool writeTag(const std::vector<uint8_t>& epc, const std::vector<uint8_t>& data);

private:
    std::string portName;
    int baudRate;
    int serialFd;
    bool isOpen;
};

#endif
