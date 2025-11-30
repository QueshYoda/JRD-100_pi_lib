// main.cpp - İyileştirilmiş versiyon
#include <iostream>
#include <vector>
#include "jrd100.h"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Kullanım: " << argv[0] << " /dev/serial0" << std::endl;
        return 1;
    }
    
    std::string port = argv[1];
    JRD100 reader(port);
    
    if (!reader.openPort()) {
        std::cerr << "Port açılamadı!" << std::endl;
        return 1;
    }

    // ÖNCELİKLE ETİKET OKUYUN
    std::cout << "\n=== ADIM 1: Etiket Tarama ===\n";
    auto tags = reader.readMultipleTags(1000);
    
    if (tags.empty()) {
        std::cerr << "HATA: Hiç etiket bulunamadı!\n";
        std::cerr << "Lütfen bir etiketin menzilde olduğundan emin olun.\n";
        reader.closePort();
        return 1;
    }
    
    std::cout << "Bulunan etiket sayısı: " << tags.size() << "\n";
    
    // İlk etiketi kullan
    std::vector<uint8_t> targetEpc = tags[0].epc;
    std::cout << "Hedef EPC: ";
    for (auto b : targetEpc) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    }
    std::cout << std::dec << "\n";

    // YAZILACAK VERİ
    std::vector<uint8_t> dataToWrite = {0x12, 0x34, 0x56, 0x78}; // 2 word
    
    std::cout << "\n=== ADIM 2: Etikete Yazma ===\n";
    
    // ÖNCE TX GÜCÜNÜ KONTROL EDİN
    int currentPower = reader.getTxPower();
    if (currentPower > 0) {
        std::cout << "Mevcut güç: " << (currentPower/100.0) << " dBm\n";
    }
    
    // Güç düşükse artırın
    if (currentPower < 2000) {  // 20 dBm'den düşükse
        std::cout << "Güç artırılıyor: 25 dBm\n";
        reader.setTxPower(2500);  // 25 dBm
    }
    
    // YAZMA İŞLEMİ
    if (reader.writeTag(targetEpc, dataToWrite)) {
        std::cout << "\n✓ BAŞARILI: Veri yazıldı!\n";
    } else {
        std::cerr << "\n✗ BAŞARISIZ: Yazma hatası!\n";
    }
    
    reader.closePort();
    return 0;
}