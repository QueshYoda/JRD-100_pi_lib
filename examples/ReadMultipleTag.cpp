#include "jrd100.h"
#include <iostream>
#include <iomanip> // std::setw ve std::setfill için

void printTag(const TagData& tag) {
    std::cout << "EPC: ";
    // EPC'yi hex formatında yazdır
    for (const auto& byte : tag.epc) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    // RSSI'yi decimal formatında yazdır
    std::cout << " | RSSI: " << std::dec << tag.rssi << " dBm" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Kullanım: " << argv[0] << " /dev/serial0" << std::endl;
        return 1;
    }

    std::string port = argv[1];
    JRD100 reader(port); // Baudrate varsayılan olarak 115200

    if (!reader.openPort()) {
        std::cerr << "Okuyucuya bağlanılamadı." << std::endl;
        return 1;
    }

    std::cout << "--- Etiketler 2 saniye boyunca okunuyor... ---" << std::endl;

    // 2000 milisaniye (2 saniye) boyunca etiketleri oku
    std::vector<TagData> tags = reader.readMultipleTags(2000);

    std::cout << "--- Okuma tamamlandı ---" << std::endl;

    if (tags.empty()) {
        std::cout << "Hiç etiket bulunamadı." << std::endl;
    } else {
        std::cout << "Toplam " << tags.size() << " adet benzersiz etiket bulundu:" << std::endl;
        for (const auto& tag : tags) {
            printTag(tag);
        }
    }

    reader.closePort();
    return 0;
}
