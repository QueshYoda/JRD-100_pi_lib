#include "jrd100_write.h"
#include <vector>
#include <cstdint>

int main() {
    JRD100Writer rfid("/dev/serial0", 115200, true);

    if(!rfid.begin()) {
        return -1;
    }

    // Örnek veri: 8 byte
    std::vector<uint8_t> myData = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

    rfid.WriteTag(myData);

    return 0;
}
