#include "../jrd100.h"
#include <iostream>

int main() {
    JRD100 reader("/dev/serial0", 115200);
    if (!reader.openPort()) return -1;

    auto tags = reader.readMultipleTags();
    for (const auto& tag : tags) {
        std::cout << "EPC: ";
        for (auto b : tag.epc)
            printf("%02X ", b);
        std::cout << " | RSSI: " << tag.rssi << " dBm\n";
    }

    reader.closePort();
    return 0;
}
