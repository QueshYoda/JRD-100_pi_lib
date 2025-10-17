#include "jrd100_write.h"
#include <iostream>

void JRD100Writer::WriteTag(const std::vector<uint8_t>& data) {
    if(fd == -1) {
        std::cerr << "UART başlatılmamış!" << std::endl;
        return;
    }

    // 1. SELECT parametreleri (eğer gerekiyorsa) ayarlanabilir
    sendCommand(6); // Örnek: SELECT parametrelerini set et

    // 2. Yazma komutu (10) ve veri ekle
    std::vector<uint8_t> cmd(RFID_cmdnub[10], RFID_cmdnub[10] + 26);

    // Data alanını gönderilecek pakete ekle
    size_t idx = 6; // 10. komutun veri başlangıcı (RFID modül spesifik)
    for(auto b : data) {
        if(idx < cmd.size() - 2) { // 0x7E öncesine ekle
            cmd[idx++] = b;
        }
    }

    // Komutu gönder
    for(auto b : cmd) writeByte(b);
    writeByte(0x7E); // paket sonu

    // Yanıtı oku
    auto resp = readResponse();
    std::cout << "Yazma cevabı: ";
    for(auto v : resp) printf("%02X ", v);
    std::cout << std::endl;
}
