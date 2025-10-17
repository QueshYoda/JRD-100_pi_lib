#ifndef JRD100_H
#define JRD100_H

#include <stdint.h>
#include <vector>
#include <string>

class JRD100 {
public:
    JRD100(const std::string& uart_port = "/dev/serial0", int baudrate = 115200, bool debug = true);
    ~JRD100();

    bool begin();                          // Start UART 
    void sendCommand(uint8_t cmd_num);     // Send command to UART
    std::vector<uint8_t> readResponse();   // Read response from UART

    void setDebug(bool debug_flag);

private:
    int fd;                                // UART file descriptor
    std::string uartDevice;
    int baud;
    bool DEBUG;

    void writeByte(uint8_t b);

    static const uint8_t RFID_cmdnub[][26]; // Command byte arrays
};

#endif // JRD100_H
