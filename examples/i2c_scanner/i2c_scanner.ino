/**
 * I2C Scanner for ESP32
 * Scans the I2C bus and identifies connected devices
 * Helps verify FRAM and other I2C devices are properly connected
 */

#include <Arduino.h>
#include <Wire.h>

// I2C pins configured to avoid Ethernet PHY conflicts
#define I2C_SDA_PIN 33
#define I2C_SCL_PIN 32

// Known I2C device addresses
struct I2CDevice {
    uint8_t address;
    const char* name;
    const char* description;
};

// Common I2C devices in boiler controller system
const I2CDevice knownDevices[] = {
    {0x50, "MB85RC256V", "FRAM Memory (32KB) - RuntimeStorage"},
    {0x51, "MB85RC256V", "FRAM Memory (Alt Address)"},
    {0x52, "MB85RC256V", "FRAM Memory (Alt Address)"},
    {0x53, "MB85RC256V", "FRAM Memory (Alt Address)"},
    {0x54, "MB85RC256V", "FRAM Memory (Alt Address)"},
    {0x55, "MB85RC256V", "FRAM Memory (Alt Address)"},
    {0x56, "MB85RC256V", "FRAM Memory (Alt Address)"},
    {0x57, "MB85RC256V", "FRAM Memory (Alt Address)"},
    {0x68, "DS3231", "RTC - Real Time Clock"},
    {0x48, "LM75/TMP75", "Temperature Sensor (Flow)"},
    {0x49, "LM75/TMP75", "Temperature Sensor (Return)"},
    {0x4A, "LM75/TMP75", "Temperature Sensor (Outside)"},
    {0x4B, "LM75/TMP75", "Temperature Sensor (Room)"},
    {0x20, "PCF8574", "I/O Expander (8-bit)"},
    {0x21, "PCF8574", "I/O Expander (8-bit)"},
    {0x38, "PCF8574A", "I/O Expander (8-bit)"},
    {0x3C, "SSD1306", "OLED Display (128x64)"},
    {0x3D, "SSD1306", "OLED Display (Alt Address)"},
    {0x27, "PCF8574", "LCD I2C Backpack"},
    {0x3F, "PCF8574", "LCD I2C Backpack (Alt)"},
    {0x76, "BME280", "Temp/Humidity/Pressure Sensor"},
    {0x77, "BME280", "Temp/Humidity/Pressure (Alt)"}
};

const int knownDeviceCount = sizeof(knownDevices) / sizeof(knownDevices[0]);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    
    Serial.println(F("\n=== I2C Scanner for ESP32 ==="));
    Serial.println(F("Scanning for I2C devices..."));
    Serial.printf("Using pins: SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.println(F("----------------------------------------"));
    
    // Initialize I2C with custom pins
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Optional: Set I2C frequency
    Wire.setClock(400000);  // 400kHz
    
    Serial.println(F("\nScanning I2C bus..."));
    delay(100);
}

void loop() {
    static unsigned long lastScan = 0;
    
    // Scan every 5 seconds
    if (millis() - lastScan > 5000) {
        lastScan = millis();
        scanI2CBus();
        
        Serial.println(F("\nPress 'h' for help, 's' to scan again"));
    }
    
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 's':
            case 'S':
                Serial.println(F("\nManual scan requested..."));
                scanI2CBus();
                break;
                
            case 'f':
            case 'F':
                Serial.println(F("\nTesting FRAM at 0x50..."));
                testFRAM();
                break;
                
            case 'r':
            case 'R':
                Serial.println(F("\nTesting RTC at 0x68..."));
                testRTC();
                break;
                
            case 'h':
            case 'H':
            case '?':
                printHelp();
                break;
                
            case '1':
                Wire.setClock(100000);
                Serial.println(F("I2C speed set to 100kHz"));
                break;
                
            case '4':
                Wire.setClock(400000);
                Serial.println(F("I2C speed set to 400kHz"));
                break;
                
            case '0':
                Wire.setClock(1000000);
                Serial.println(F("I2C speed set to 1MHz"));
                break;
        }
        
        // Clear buffer
        while (Serial.available()) Serial.read();
    }
}

void scanI2CBus() {
    uint8_t error, address;
    int deviceCount = 0;
    uint8_t foundAddresses[128];
    
    Serial.println(F("\n     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
    Serial.print(F("00:     "));
    
    for (address = 3; address < 127; address++) {
        // Print row header
        if (address % 16 == 0) {
            Serial.printf("\n%02X: ", address);
        }
        
        // Perform I2C probe
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("%02X ", address);
            foundAddresses[deviceCount++] = address;
        } else if (error == 4) {
            Serial.print("?? ");  // Unknown error
        } else {
            Serial.print("-- ");  // No device
        }
        
        delay(1);  // Small delay between probes
    }
    
    Serial.println(F("\n"));
    
    // Report findings
    if (deviceCount == 0) {
        Serial.println(F("No I2C devices found!"));
        Serial.println(F("Check your connections:"));
        Serial.printf("  SDA -> GPIO %d\n", I2C_SDA_PIN);
        Serial.printf("  SCL -> GPIO %d\n", I2C_SCL_PIN);
        Serial.println(F("  VCC -> 3.3V"));
        Serial.println(F("  GND -> GND"));
        Serial.println(F("  Pull-up resistors (4.7kΩ) on SDA and SCL"));
    } else {
        Serial.printf("Found %d I2C device(s):\n", deviceCount);
        Serial.println(F("----------------------------------------"));
        
        for (int i = 0; i < deviceCount; i++) {
            uint8_t addr = foundAddresses[i];
            Serial.printf("0x%02X: ", addr);
            
            // Check if it's a known device
            bool found = false;
            for (int j = 0; j < knownDeviceCount; j++) {
                if (knownDevices[j].address == addr) {
                    Serial.printf("%-12s - %s", 
                        knownDevices[j].name, 
                        knownDevices[j].description);
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                Serial.print("Unknown device");
            }
            
            Serial.println();
        }
    }
    
    Serial.println(F("----------------------------------------"));
}

void testFRAM() {
    const uint8_t framAddr = 0x50;
    const uint16_t testAddr = 0x7FF0;  // Near end of 32KB
    uint8_t testData[] = {0xAA, 0x55, 0x12, 0x34};
    uint8_t readData[4];
    
    // Check if FRAM exists
    Wire.beginTransmission(framAddr);
    if (Wire.endTransmission() != 0) {
        Serial.println(F("FRAM not found at 0x50!"));
        return;
    }
    
    Serial.println(F("FRAM found at 0x50"));
    
    // Write test data
    Serial.print(F("Writing test data: "));
    for (int i = 0; i < 4; i++) {
        Serial.printf("0x%02X ", testData[i]);
    }
    Serial.println();
    
    Wire.beginTransmission(framAddr);
    Wire.write((uint8_t)(testAddr >> 8));    // Address high byte
    Wire.write((uint8_t)(testAddr & 0xFF));  // Address low byte
    Wire.write(testData, 4);
    if (Wire.endTransmission() != 0) {
        Serial.println(F("Write failed!"));
        return;
    }
    
    delay(5);  // Small delay for FRAM write
    
    // Read back test data
    Wire.beginTransmission(framAddr);
    Wire.write((uint8_t)(testAddr >> 8));
    Wire.write((uint8_t)(testAddr & 0xFF));
    Wire.endTransmission();
    
    Wire.requestFrom(framAddr, (uint8_t)4);
    int bytesRead = 0;
    while (Wire.available() && bytesRead < 4) {
        readData[bytesRead++] = Wire.read();
    }
    
    Serial.print(F("Read back data:    "));
    bool success = true;
    for (int i = 0; i < 4; i++) {
        Serial.printf("0x%02X ", readData[i]);
        if (readData[i] != testData[i]) {
            success = false;
        }
    }
    Serial.println();
    
    if (success) {
        Serial.println(F("FRAM test PASSED! ✓"));
        Serial.println(F("MB85RC256V (32KB) is working correctly"));
    } else {
        Serial.println(F("FRAM test FAILED! ✗"));
    }
}

void testRTC() {
    const uint8_t rtcAddr = 0x68;
    
    // Check if RTC exists
    Wire.beginTransmission(rtcAddr);
    if (Wire.endTransmission() != 0) {
        Serial.println(F("RTC not found at 0x68!"));
        return;
    }
    
    Serial.println(F("RTC found at 0x68"));
    
    // Read seconds register
    Wire.beginTransmission(rtcAddr);
    Wire.write(0x00);  // Seconds register
    Wire.endTransmission();
    
    Wire.requestFrom(rtcAddr, (uint8_t)7);  // Read 7 bytes (time and date)
    
    if (Wire.available() >= 7) {
        uint8_t sec = bcd2dec(Wire.read() & 0x7F);
        uint8_t min = bcd2dec(Wire.read());
        uint8_t hour = bcd2dec(Wire.read());
        uint8_t dow = Wire.read();
        uint8_t date = bcd2dec(Wire.read());
        uint8_t month = bcd2dec(Wire.read());
        uint8_t year = bcd2dec(Wire.read());
        
        Serial.printf("Time: %02d:%02d:%02d\n", hour, min, sec);
        Serial.printf("Date: 20%02d-%02d-%02d\n", year, month, date);
        Serial.println(F("DS3231 RTC is working correctly"));
    } else {
        Serial.println(F("Failed to read RTC data"));
    }
}

uint8_t bcd2dec(uint8_t bcd) {
    return ((bcd / 16) * 10) + (bcd % 16);
}

void printHelp() {
    Serial.println(F("\n=== I2C Scanner Commands ==="));
    Serial.println(F("s - Scan I2C bus"));
    Serial.println(F("f - Test FRAM at 0x50"));
    Serial.println(F("r - Test RTC at 0x68"));
    Serial.println(F("1 - Set I2C speed to 100kHz"));
    Serial.println(F("4 - Set I2C speed to 400kHz"));
    Serial.println(F("0 - Set I2C speed to 1MHz"));
    Serial.println(F("h - Show this help"));
    Serial.println(F(""));
    Serial.println(F("Common I2C Addresses:"));
    Serial.println(F("  0x50-0x57: FRAM (MB85RC256V)"));
    Serial.println(F("  0x68: RTC (DS3231)"));
    Serial.println(F("  0x48-0x4F: Temperature sensors"));
    Serial.println(F("  0x20-0x27: I/O Expanders"));
    Serial.println(F("  0x3C-0x3D: OLED Displays"));
}