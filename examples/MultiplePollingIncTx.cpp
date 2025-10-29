#include "jrd100.h"
#include <iostream>
#include <iomanip> // std::setw, std::setfill, std::setprecision

void printTag(const TagData& tag) {
    std::cout << "EPC: ";
    // EPC to HEX
    for (const auto& byte : tag.epc) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << " | RSSI: " << std::dec << tag.rssi << " dBm" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << " Serial Port : " << argv[0] << " /dev/serial0" << std::endl;
        return 1;
    }

    std::string port = argv[1];
    JRD100 reader(port); 

    if (!reader.openPort()) {
        std::cerr << "Reader not found." << std::endl;
        return 1;
    }

    // --- TX GÜCÜ AYARLAMA BÖLÜMÜ ---

    
    reader.getTxPower(); 

    // Set new TX power
    uint16_t new_power_setting = 5000; // 50.00 dBm
    
    std::cout << std::fixed << std::setprecision(2) 
              << (new_power_setting / 100.0) 
              << " Setting power..." << std::endl;

    if (!reader.setTxPower(new_power_setting)) {
        std::cerr << "Warning:Setting TX power failed. Turning default settings" << std::endl;
    }



    // Set reading interval 2000 ms
    std::vector<TagData> tags = reader.readMultipleTags(2000);

    std::cout << "--- Reading Done ---" << std::endl;

    if (tags.empty()) {
        std::cout << "Tag No Found." << std::endl;
    } else {
        std::cout << "Total " << tags.size() << " unique tags found." << std::endl;
        for (const auto& tag : tags) {
            printTag(tag);
        }
    }

    reader.closePort();
    return 0;
}
