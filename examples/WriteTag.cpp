#include <iostream>
#include <iomanip>
#include "jrd100.h"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Kullanım: " << argv[0] << " /dev/serial0\n";
        return 1;
    }

    JRD100 reader(argv[1]);
    if (!reader.openPort()) {
        std::cerr << "Port açılamadı!\n";
        return 1;
    }

    std::cout << "\n=== ÖRNEK 1: BASİT YAZMA (USER bank) ===\n";
    std::vector<uint8_t> data1 = {0x11, 0x22, 0x33, 0x44};
    if (reader.writeTag({}, data1)) {  // Boş EPC = ilk etikete yaz
        std::cout << "✓ Yazıldı!\n";
    }

    std::cout << "\n=== ÖRNEK 2: FARKLI BANK'A YAZMA ===\n";
    std::vector<uint8_t> data2 = {0xAA, 0xBB, 0xCC, 0xDD};
    // EPC bank'a yaz (membank=0x01)
    if (reader.writeTag({}, data2, 0x01, 0x0002, 0x00000000)) {
        std::cout << "✓ EPC bank'a yazıldı!\n";
    }

    std::cout << "\n=== ÖRNEK 3: OKUMA ===\n";
    std::vector<uint8_t> readData;
    if (reader.readCard(readData, 4, 0x03, 0x0000)) {  // 4 byte USER'dan oku
        std::cout << "✓ Okunan veri: ";
        for (auto b : readData) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                      << (int)b << " ";
        }
        std::cout << std::dec << "\n";
    }

    std::cout << "\n=== ÖRNEK 4: ŞİFRELİ YAZMA ===\n";
    // Özel access password ile yazma
    std::vector<uint8_t> secureData = {0xFF, 0xEE, 0xDD, 0xCC};
    if (reader.writeTag({}, secureData, 0x03, 0x0000, 0x12345678)) {
        std::cout << "✓ Şifreli yazma başarılı!\n";
    }

    reader.closePort();
    return 0;
}