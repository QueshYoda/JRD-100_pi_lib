#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdint>
#include <chrono>
#include <thread>
#include <csignal>
#include "jrd100.h"

// Global variable - stop with Ctrl+C
volatile bool keepRunning = true;

void signalHandler(int signum) {
    std::cout << "\n\n[!] Operation stopped (Signal: " << signum << ")\n";
    keepRunning = false;
}

// Common access passwords (priority testing)
const std::vector<uint32_t> COMMON_PASSWORDS = {
    0x00000000,  // Default - most common
    0xFFFFFFFF,  // All bits set
    0x12345678,  // Test password
    0x11111111,  // Repeating 1s
    0x00000001,  // Simple 1
    0x0000FFFF,  // Lower 16 bits set
    0xFFFF0000,  // Upper 16 bits set
    0xAAAAAAAA,  // Alternating bits
    0x55555555,  // Alternating bits
    0x87654321,  // Reverse order
    0xDEADBEEF,  // Debug magic number
    0xCAFEBABE,  // Java magic number
    0xFEEDFACE,  // Mach-O magic
    0x8BADF00D,  // iOS crash report
};

// Manufacturer-specific common passwords
const std::vector<uint32_t> MANUFACTURER_PASSWORDS = {
    // Alien Technology
    0x00000000, 0x01020304,
    // Impinj
    0x00000000, 0x12345678,
    // NXP
    0x00000000, 0xFFFFFFFF,
    // Avery Dennison
    0x00000000,
    // Generic test patterns
    0x01010101, 0x10101010,
    0xABCDEF00, 0x00ABCDEF,
};

class PasswordFinder {
private:
    JRD100& reader;
    std::vector<uint8_t> testData;
    uint8_t membank;
    uint16_t startAddr;
    uint64_t totalAttempts;
    std::chrono::steady_clock::time_point startTime;

public:
    PasswordFinder(JRD100& r, uint8_t mb = 0x03, uint16_t addr = 0x0000) 
        : reader(r), membank(mb), startAddr(addr), totalAttempts(0) {
        // Test data - 2 bytes (1 word)
        testData = {0xAB, 0xCD};
        startTime = std::chrono::steady_clock::now();
    }

    bool testPassword(uint32_t password) {
        totalAttempts++;
        
        // Show progress every 100 attempts
        if (totalAttempts % 100 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
            double rate = elapsed > 0 ? totalAttempts / (double)elapsed : 0;
            
            std::cout << "\r[" << totalAttempts << "] Testing: 0x" 
                      << std::hex << std::setw(8) << std::setfill('0') << password 
                      << std::dec << " | Rate: " << std::fixed << std::setprecision(1) 
                      << rate << " attempts/s" << std::flush;
        }

        // Try write operation
        bool success = reader.writeTag(testData, membank, startAddr, password);
        
        if (success) {
            std::cout << "\n\n";
            std::cout << "╔════════════════════════════════════════╗\n";
            std::cout << "║        ✓ PASSWORD FOUND!               ║\n";
            std::cout << "╚════════════════════════════════════════╝\n";
            std::cout << "\n";
            std::cout << "  Access Password: 0x" << std::hex << std::setw(8) 
                      << std::setfill('0') << password << std::dec << "\n";
            std::cout << "  Decimal: " << password << "\n";
            std::cout << "  Binary: ";
            for (int i = 31; i >= 0; i--) {
                std::cout << ((password >> i) & 1);
                if (i % 8 == 0 && i != 0) std::cout << " ";
            }
            std::cout << "\n";
            std::cout << "  Bytes: [ ";
            for (int i = 3; i >= 0; i--) {
                std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') 
                          << ((password >> (i*8)) & 0xFF) << " ";
            }
            std::cout << std::dec << "]\n";
            std::cout << "\n";
            
            return true;
        }
        
        // Short delay - don't stress the reader
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        return false;
    }

    void printStats() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
        
        std::cout << "\n\n=== STATISTICS ===\n";
        std::cout << "Total Attempts: " << totalAttempts << "\n";
        std::cout << "Elapsed Time: " << elapsed << " seconds\n";
        if (elapsed > 0) {
            std::cout << "Average Rate: " << (totalAttempts / (double)elapsed) << " attempts/s\n";
        }
    }

    // Strategy 1: Try common passwords
    bool findCommonPassword() {
        std::cout << "\n=== STEP 1: COMMON PASSWORDS ===\n";
        std::cout << "Testing: " << COMMON_PASSWORDS.size() << " common passwords\n\n";
        
        for (auto pwd : COMMON_PASSWORDS) {
            if (!keepRunning) return false;
            
            std::cout << "Trying: 0x" << std::hex << std::setw(8) 
                      << std::setfill('0') << pwd << std::dec << "\n";
            
            if (testPassword(pwd)) {
                return true;
            }
        }
        
        std::cout << "\n[!] Common passwords failed.\n";
        return false;
    }

    // Strategy 2: Manufacturer-specific passwords
    bool findManufacturerPassword() {
        std::cout << "\n=== STEP 2: MANUFACTURER PASSWORDS ===\n";
        std::cout << "Testing: " << MANUFACTURER_PASSWORDS.size() << " manufacturer passwords\n\n";
        
        for (auto pwd : MANUFACTURER_PASSWORDS) {
            if (!keepRunning) return false;
            if (testPassword(pwd)) return true;
        }
        
        std::cout << "\n[!] Manufacturer passwords failed.\n";
        return false;
    }

    // Strategy 3: Low range values (0x00000000 - 0x0000FFFF)
    bool findLowRangePassword() {
        std::cout << "\n=== STEP 3: LOW RANGE (0x0000-0xFFFF) ===\n";
        std::cout << "This will test 65,536 combinations (~10 minutes)\n";
        std::cout << "Continue? (y/n): ";
        
        char choice;
        std::cin >> choice;
        if (choice != 'y' && choice != 'Y') {
            return false;
        }
        
        std::cout << "\nScanning started...\n";
        
        for (uint32_t pwd = 0; pwd <= 0xFFFF; pwd++) {
            if (!keepRunning) return false;
            if (testPassword(pwd)) return true;
        }
        
        std::cout << "\n[!] Low range scan failed.\n";
        return false;
    }

    // Strategy 4: Repeating byte patterns
    bool findRepeatingPattern() {
        std::cout << "\n=== STEP 4: REPEATING PATTERNS ===\n";
        std::cout << "Testing 0xXXXXXXXX format: 256 combinations\n\n";
        
        for (uint32_t byte = 0; byte <= 0xFF; byte++) {
            if (!keepRunning) return false;
            
            uint32_t pwd = (byte << 24) | (byte << 16) | (byte << 8) | byte;
            if (testPassword(pwd)) return true;
        }
        
        std::cout << "\n[!] Repeating patterns failed.\n";
        return false;
    }

    // Strategy 5: Incremental patterns
    bool findIncrementalPattern() {
        std::cout << "\n=== STEP 5: INCREMENTAL PATTERNS ===\n";
        
        std::vector<uint32_t> patterns = {
            0x01020304, 0x04030201,  // Sequential
            0x00010203, 0x03020100,  // Zero-based sequential
            0x10203040, 0x40302010,  // Hex sequential
            0x11223344, 0x44332211,  // Paired
        };
        
        for (auto pwd : patterns) {
            if (!keepRunning) return false;
            
            std::cout << "Pattern: 0x" << std::hex << std::setw(8) 
                      << std::setfill('0') << pwd << std::dec << "\n";
            
            if (testPassword(pwd)) return true;
        }
        
        std::cout << "\n[!] Incremental patterns failed.\n";
        return false;
    }

    // FULL BRUTE FORCE (WILL TAKE VERY LONG!)
    bool fullBruteForce() {
        std::cout << "\n=== WARNING: FULL BRUTE FORCE ===\n";
        std::cout << "This will test 4,294,967,296 combinations!\n";
        std::cout << "Estimated time: ~500 days (at 10ms/attempt)\n";
        std::cout << "Not recommended! Continue? (yes/no): ";
        
        std::string choice;
        std::cin >> choice;
        if (choice != "yes") {
            return false;
        }
        
        std::cout << "\n[!] STARTING... (Press Ctrl+C to stop)\n\n";
        
        for (uint64_t pwd = 0; pwd <= 0xFFFFFFFF; pwd++) {
            if (!keepRunning) return false;
            if (testPassword(static_cast<uint32_t>(pwd))) return true;
        }
        
        return false;
    }
};

int main(int argc, char* argv[]) {
    // Ctrl+C handler
    signal(SIGINT, signalHandler);
    
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " /dev/serial0 [membank] [startAddr]\n";
        std::cerr << "\nExamples:\n";
        std::cerr << "  " << argv[0] << " /dev/serial0           # USER bank (0x03)\n";
        std::cerr << "  " << argv[0] << " /dev/serial0 0x01      # EPC bank\n";
        std::cerr << "  " << argv[0] << " /dev/serial0 0x03 0x00 # USER bank, addr 0x00\n";
        return 1;
    }

    std::string port = argv[1];
    uint8_t membank = 0x03;  // Default: USER
    uint16_t startAddr = 0x0000;
    
    if (argc >= 3) {
        membank = std::stoi(argv[2], nullptr, 16);
    }
    if (argc >= 4) {
        startAddr = std::stoi(argv[3], nullptr, 16);
    }

    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║   RFID ACCESS PASSWORD FINDER          ║\n";
    std::cout << "╚════════════════════════════════════════╝\n\n";

    // Open port
    JRD100 reader(port);
    if (!reader.openPort()) {
        std::cerr << "ERROR: Could not open port!\n";
        return 1;
    }

    std::cout << "Port: " << port << "\n";
    std::cout << "Memory Bank: 0x" << std::hex << (int)membank << std::dec << "\n";
    std::cout << "Start Address: 0x" << std::hex << startAddr << std::dec << "\n\n";

    // Check for tag
    std::cout << "Scanning for tag...\n";
    auto tags = reader.readMultipleTags(1000);
    
    if (tags.empty()) {
        std::cerr << "ERROR: No tag found!\n";
        std::cerr << "Please place a tag near the reader.\n";
        reader.closePort();
        return 1;
    }

    std::cout << "✓ Tag found! EPC: ";
    for (auto b : tags[0].epc) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
    }
    std::cout << std::dec << "\n";
    std::cout << "  RSSI: " << tags[0].rssi << "\n\n";

    // Password finding
    PasswordFinder finder(reader, membank, startAddr);

    // Try strategies in order
    if (finder.findCommonPassword()) goto found;
    if (finder.findManufacturerPassword()) goto found;
    if (finder.findRepeatingPattern()) goto found;
    if (finder.findIncrementalPattern()) goto found;
    if (finder.findLowRangePassword()) goto found;
    
    std::cout << "\n[?] All fast methods exhausted.\n";
    std::cout << "Would you like to try full brute force? (Not recommended!)\n";
    
    if (finder.fullBruteForce()) goto found;

    // Not found
    std::cout << "\n\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║     PASSWORD NOT FOUND!                ║\n";
    std::cout << "╚════════════════════════════════════════╝\n\n";
    
    std::cout << "Possible reasons:\n";
    std::cout << "  1. Password is non-standard\n";
    std::cout << "  2. Memory is fully locked (write-protected)\n";
    std::cout << "  3. Tag is damaged or unsupported\n";
    std::cout << "  4. Reader power insufficient\n\n";
    
    finder.printStats();
    reader.closePort();
    return 1;

found:
    finder.printStats();
    
    std::cout << "\n[TIP] Save this password in a secure location!\n";
    std::cout << "[TIP] Run this program again for other tags.\n\n";
    
    reader.closePort();
    return 0;
}