#include "../JRD-100/jrd100.h"
#include <iostream>
#include <vector>

int main() {
    JRD100 rfid("/dev/serial0", 115200, true);

    if(!rfid.begin()) {
        std::cerr << "RFID modül başlatılamadı!" << std::endl;
        return -1;
    }

    // Çoklu polling başlat
    rfid.sendCommand(4);
    sleep(1);

    auto resp = rfid.readResponse();
    std::cout << "Çoklu Tag Yanıtı: ";
    for(auto v : resp) printf("%02X ", v);
    std::cout << std::endl;

    // RSSI
    rfid.sendCommand(28);
    auto rssi_resp = rfid.readResponse();
    std::cout << "RSSI: ";
    for(auto v : rssi_resp) printf("%02X ", v);
    std::cout << std::endl;

    rfid.sendCommand(5); // polling durdur

    return 0;
}
