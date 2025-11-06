#include "jrd100.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

void printTag(const TagData& tag) {
    std::cout << "EPC: ";
    for (const auto& byte : tag.epc) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << " | RSSI: " << std::dec << tag.rssi << " dBm" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Kullanım: " << argv[0] << " /dev/serial0" << std::endl;
        return 1;
    }

    std::string port = argv[1];
    JRD100 reader(port);

    if (!reader.openPort()) {
        std::cerr << "HATA: Reader bulunamadı veya port açılamadı." << std::endl;
        return 1;
    }

    std::cout << "Reader bağlı. TX güç denemesi başlatılıyor..." << std::endl;

    // Güç değerlerini 70 dBm'den 10 dBm'ye doğru 10'ar azaltarak dene
    for (int dbm = 70; dbm >= 10; dbm -= 10) {
        uint16_t power_setting = dbm * 100; // örn: 70 dBm -> 7000
        std::cout << "\nDeneme -> " << dbm << " dBm (" << power_setting << ")\n";

        bool ok = reader.setTxPower(power_setting);
        if (ok) {
            std::cout << "✅ Güç başarıyla ayarlandı: " << dbm << " dBm" << std::endl;
        } else {
            std::cout << "❌ Ayarlama başarısız: " << dbm << " dBm" << std::endl;
        }

        // Donanımın stabilize olması için biraz bekle
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // İsteğe bağlı: tag okuma testi
        std::vector<TagData> tags = reader.readMultipleTags(500);
        if (!tags.empty()) {
            std::cout << "📡 Tag(ler) okundu (" << tags.size() << "):" << std::endl;
            for (const auto& tag : tags) {
                printTag(tag);
            }
        } else {
            std::cout << "No tag detected at " << dbm << " dBm" << std::endl;
        }

        // Aralarda cihazı çok yormamak için bekleme
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    reader.closePort();
    std::cout << "\n--- TX Güç testi tamamlandı ---" << std::endl;

    return 0;
}
