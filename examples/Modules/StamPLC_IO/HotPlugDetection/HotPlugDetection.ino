/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 *
 * M5StamPLC IO Hot-Plug Detection Example
 *
 * This example demonstrates hot-plug detection for multiple M5StamPLC IO modules:
 * - Automatically scan I2C bus and detect connected modules (0x20-0x2F)
 * - Display online modules list
 * - Real-time detection of module insertion and removal
 */
#include <Arduino.h>
#include <M5StamPLC.h>

#define MAX_MODULES      16
#define SCAN_INTERVAL_MS 2000

struct ModuleInfo {
    uint8_t address;
    uint8_t firmware_version;
    bool connected;
};

ModuleInfo modules[MAX_MODULES];
uint8_t module_count         = 0;
unsigned long last_scan_time = 0;

void initModules()
{
    for (int i = 0; i < MAX_MODULES; i++) {
        modules[i].address          = M5StamPLC_IO::I2C_ADDR_MIN + i;
        modules[i].firmware_version = 0;
        modules[i].connected        = false;
    }
    module_count = 0;
}

void scanModules()
{
    bool found_map[0x80] = {false};
    m5::In_I2C.scanID(found_map);

    for (int i = 0; i < MAX_MODULES; i++) {
        uint8_t addr = M5StamPLC_IO::I2C_ADDR_MIN + i;
        if (found_map[addr] && !modules[i].connected) {
            M5StamPLC_IO temp_device;
            if (temp_device.begin(addr, false)) {
                modules[i].address          = addr;
                modules[i].firmware_version = temp_device.getFirmwareVersion();
                modules[i].connected        = true;
                module_count++;
                Serial.printf("[+] Module connected: 0x%02X (FW: 0x%02X)\n", addr, modules[i].firmware_version);
            }
        } else if (!found_map[addr] && modules[i].connected) {
            Serial.printf("[-] Module disconnected: 0x%02X\n", modules[i].address);
            modules[i].connected = false;
            module_count--;
        }
    }
}

void displayModules()
{
    M5StamPLC.Display.fillScreen(TFT_BLACK);
    M5StamPLC.Display.setCursor(0, 0);

    M5StamPLC.Display.setTextColor(TFT_GREENYELLOW);
    M5StamPLC.Display.println("=== IO Hot-Plug ===");

    M5StamPLC.Display.setTextColor(TFT_CYAN);
    M5StamPLC.Display.printf("Online: %d\n\n", module_count);

    if (module_count == 0) {
        M5StamPLC.Display.setTextColor(TFT_ORANGE);
        M5StamPLC.Display.println("No modules found");
    } else {
        int displayed = 0;
        for (int i = 0; i < MAX_MODULES && displayed < 8; i++) {
            if (modules[i].connected) {
                M5StamPLC.Display.setTextColor(TFT_GREEN);
                M5StamPLC.Display.printf("%d. Addr: 0x%02X\n", displayed + 1, modules[i].address);
                M5StamPLC.Display.setTextColor(TFT_YELLOW);
                M5StamPLC.Display.printf("   FW: 0x%02X\n", modules[i].firmware_version);
                displayed++;
            }
        }
    }
}

void setup()
{
    M5StamPLC.begin();
    M5StamPLC.Display.setTextScroll(false);
    M5StamPLC.Display.setTextSize(1);
    M5StamPLC.Display.setFont(&fonts::efontCN_16);

    Serial.begin(115200);
    Serial.println("\n=== M5StamPLC IO Hot-Plug Detection ===");

    initModules();

    M5StamPLC.Display.fillScreen(TFT_BLACK);
    M5StamPLC.Display.setCursor(0, 0);
    M5StamPLC.Display.setTextColor(TFT_GREENYELLOW);
    M5StamPLC.Display.println("M5StamPLC IO");
    M5StamPLC.Display.println("Hot-Plug Detection");
    M5StamPLC.Display.setTextColor(TFT_CYAN);
    M5StamPLC.Display.println("\nScanning...");

    delay(1000);
    scanModules();
    displayModules();
    last_scan_time = millis();
}

void loop()
{
    M5StamPLC.update();

    if (millis() - last_scan_time > SCAN_INTERVAL_MS) {
        scanModules();
        displayModules();
        last_scan_time = millis();
    }

    delay(100);
}
