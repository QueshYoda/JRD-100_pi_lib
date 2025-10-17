#include "JRD-100/jrd100.h"
#include <iostream>
#include <vector>

int main() {
    JRD100 rfid("/dev/serial0", 115200, true);

    if(!rfid.begin()) {
        std::cerr << "RFID modül başlatılamadı!" << std::endl;
        return -1;
    }

    // Çoklu polling başlat (komut 4)
    rfid.sendCommand(4);
    sleep(1); // Modülün polling yapması için bekle

    // Cevabı oku
    auto resp = rfid.readResponse();

    std::cout << "Çoklu Tag Yanıtı: " << std::endl;
    for(size_t i = 0; i < resp.size(); i++) {
        printf("%02X ", resp[i]);
    }
    std::cout << std::endl;

    // RSSI'yi al (komut 28)
    rfid.sendCommand(28);
    auto rssi_resp = rfid.readResponse();
    std::cout << "RSSI Cevabı: ";
    for(auto v : rssi_resp) printf("%02X ", v);
    std::cout << std::endl;

    // Çoklu polling durdur (komut 5)
    rfid.sendCommand(5);

    return 0;
}
