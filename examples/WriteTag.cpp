#include "../JRD-100/jrd100.h"
#include <iostream>

int main() {
    JRD100 reader("/dev/serial0", 115200);
    if (!reader.openPort()) return -1;

    std::vector<uint8_t> epc = {0x30, 0x00, 0x11, 0x22};
    std::vector<uint8_t> data = {0xDE, 0xAD, 0xBE, 0xEF};

    if (reader.writeTag(epc, data))
        std::cout << "[INFO] Yazma işlemi başarılı.\n";
    else
        std::cout << "[ERROR] Yazma başarısız.\n";

    reader.closePort();
    return 0;
}
