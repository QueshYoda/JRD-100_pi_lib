JRD-100 Pi LibBu, JRD-100 RFID okuyucunu Raspberry Pi'nde çalıştırmak için bi C++ kütüphanesi. Öyle Linux'ta falan da yürür.Just a simple C++ library to get your JRD-100 UHF RFID reader working on a Raspberry Pi (or really any Linux box) using the serial port.🚀 Neler Yapıyo? (Features)Bağlantı Mı? Kolay: Fişe takar gibi /dev/serial0, /dev/ttyUSB0 falan ne varsa bağlanıyo.Paket Çözme Derdi Yok: O BB...7E saçmalığını falan kendi hallediyo, checksum'a (doğrulama) bile bakıyo.Hepsini Oku: readMultipleTags ile ne kadar etiket varsa döküyo.Menzil Ayarı: setTxPower ile gücü artırabiliyon (ya da kısabiliyon). getTxPower da var.Kafa Karıştırmayan Yapı: Sadece bi kütüphane (libJRD100.a) ve örnekler. Kasmıyo.CMake Var: cmake .. && make yap geç.🛠️ Ne Lazım? (Requirements)Bi Raspberry Pi lazım (ya da Linux'lu bi PC/şey)Tabii bi JRD-100 okuyucu (ya da uyumlu bişi)C++17 bilen bi derleyici (g++, clang++ falan)CMake (3.10'dan yeni olsun)make⚙️ Nasıl Kurcan? (Setup & Compile)Git'ten Çek:git clone [https://github.com/QueshYoda/JRD-100_pi_lib.git](https://github.com/QueshYoda/JRD-100_pi_lib.git)
cd JRD-100_pi_lib

Derle Gitsin:mkdir build
cd build
cmake ..
make

Bitti! Çalışan dosyalar build/bin/ klasöründe.🏃‍♂️ Nasıl Çalışıyo? (How to Use)O build/bin'deki dosyaları çalıştırıcan. Portu bilmen lazım (mesela /dev/serial0) ve sudo şart, yoksa çalışmaz.Normal Okuma (ReadMultipleTag)Varsayılan ayarlarla okur, ne bulursa getirir.sudo ./bin/ReadMultipleTag /dev/serial0

Güçlü Okuma (MultiplePollingIncTx)Bu, okumadan önce gücü artırıyo (örn: 30.00 dBm). Menzil için falan oldukça kullanışlı!sudo ./bin/MultiplePollingIncTx /dev/serial0

📝 Yapılacaklar (To-Do)$$ $$ writeTag (etikete yazma) bitsin artık.$$ $$ singleRead (tek tek okuma) gelsin.$$ $$ Diğer ayarlar (frekans falan filan).$$ $$ Hata verince daha net konuşsun, anlasak.📄 Lisans (License)MIT. Yani kafana göre takıl, LICENSE dosyasına bi bakarsın yine de.