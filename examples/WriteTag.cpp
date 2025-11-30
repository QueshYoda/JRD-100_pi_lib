#include <iostream>
#include <vector>
#include <string>
#include <cstdint>
#include <iomanip>
#include "jrd100.h" 

int main(int argc, char* argv[]) {
    // 1. Port adını komut satırından al
    if (argc < 2) {
        std::cerr << "Kullanım: " << argv[0] << " /dev/serial0" << std::endl;
        std::cerr << "Lütfen seri port adını argüman olarak belirtin." << std::endl;
        return 1;
    }
    std::string port = argv[1];

    // 2. JRD100 sınıfını başlat ve portu aç
    JRD100 reader(port);
    if (!reader.openPort()) {
        std::cerr << "HATA: Port açılamadı: " << port << std::endl;
        return 1;
    }

    std::cout << "Port başarıyla açıldı: " << port << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    // --- Yazma İşlemi ---

    // 3. YAZILACAK VERİ:
    // 4 byte'lık örnek veri. jrd100.cpp'deki implementasyona göre
    // veri uzunluğu 2'nin katı (word) olmalıdır.
    std::vector<uint8_t> dataToWrite = {0x12, 0x34, 0x56, 0x78 , 0x9A, 0xBC , 0xB6, 0xA8 }; // 8 byte = 4 word

    // 4. EPC FİLTRESİ (İsteğe bağlı):
    // Hangi etikete yazılacağını belirler.
    //
    // DİKKAT: Boş bir vektör ({}), okuyucunun gördüğü İLK etikete yazmaya çalışır.
    // Bu, test için kolaylık sağlar ancak gerçek bir uygulamada tehlikeli olabilir.
    //
    // Gerçek bir uygulamada, yazmak istediğiniz etiketin tam EPC'sini (örn. 12 byte)
    // okuyup buraya girmelisiniz.
    // Örnek: std::vector<uint8_t> epcFilter = {0x30, 0x39, 0x60, ... , 0x96};
    //
    // Şimdilik test için boş filtre kullanıyoruz (ilk gördüğüne yaz):
    std::vector<uint8_t> epcFilter = {}; 

    std::cout << "Etikete yazma denemesi başlatılıyor..." << std::endl;
    std::cout << "  Yazılacak Veri (HEX): ";
    for(auto b : dataToWrite) {
        std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    }
    std::cout << std::dec << std::endl;
    
    if (epcFilter.empty()) {
        std::cout << "  EPC Filtresi: YOK (Algılanan ilk etikete yazılacak)" << std::endl;
        std::cout << "  UYARI: Lütfen okuyucunun menzilinde sadece 1 etiket olduğundan emin olun!" << std::endl;
    } else {
        std::cout << "  EPC Filtresi: Var (Sadece bu EPC'ye sahip etikete yazılacak)" << std::endl;
    }


    // 5. Yazma fonksiyonunu çağır
    if (reader.writeTag(epcFilter, dataToWrite)) {
        std::cout << "\nBAŞARILI: Etikete yazma işlemi tamamlandı." << std::endl;
        std::cout << "Veriyi doğrulamak için okuma programınızı çalıştırabilirsiniz." << std::endl;
    } else {
        std::cerr << "\nHATA: Etikete yazma işlemi başarısız oldu." << std::endl;
        std::cerr << "Okuyucudan gelen cevabı veya port durumunu kontrol edin." << std::endl;
    }

    // 6. Portu kapat
    std::cout << "------------------------------------------" << std::endl;
    reader.closePort();
    std::cout << "Port kapatıldı." << std::endl;

    return 0;
}
