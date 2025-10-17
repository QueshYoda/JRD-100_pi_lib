#ifndef JRD100_WRITE_H
#define JRD100_WRITE_H

#include "jrd100.h"
#include <vector>
#include <cstdint>

class JRD100Writer : public JRD100 {
public:
    JRD100Writer(const std::string& uart_port, int baudrate, bool debug = false)
        : JRD100(uart_port, baudrate, debug) {}

    // Data processing function for writing tags
    void WriteTag(const std::vector<uint8_t>& data);
};

#endif
