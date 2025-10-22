#ifndef JRD100_H
#define JRD100_H

#include <string>
#include <vector>

class JRD100 {
public:
    explicit JRD100(const std::string& port, int baudrate = 115200);
    ~JRD100();

    bool openPort();
    void closePort();

    bool sendCommand(const std::vector<uint8_t>& cmd);
    std::vector<uint8_t> readResponse();

    bool singleRead();
    bool multiRead();

private:
    std::string portName;
    int baudRate;
    int serialFd;
    bool isOpen;

    bool configurePort();
};

#endif
