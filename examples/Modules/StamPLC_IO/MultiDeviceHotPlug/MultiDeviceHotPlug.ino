/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 *
 * M5StamPLC IO Multi-Device Hot-Plug Example
 *
 * Demonstrates hot-plug and DIP-switch address change detection
 * for up to 16 M5StamPLC IO modules (I2C address 0x20-0x2F).
 */
#include <Arduino.h>
#include <M5StamPLC.h>

M5StamPLC_IO_Manager io_mgr;

void onConnect(M5StamPLC_IO& dev, uint8_t addr)
{
    Serial.printf("[+] 0x%02X online  FW=0x%02X  DIP=0x%02X\n", addr, dev.getFirmwareVersion(),
                  dev.getExpectedAddress());
}

void onDisconnect(uint8_t addr)
{
    Serial.printf("[-] 0x%02X offline\n", addr);
}

void onAddrChange(uint8_t old_addr, uint8_t new_addr)
{
    Serial.printf("[~] DIP change: 0x%02X -> 0x%02X, address applied\n", old_addr, new_addr);
}

void updateDisplay()
{
    M5StamPLC.Display.fillScreen(TFT_BLACK);
    M5StamPLC.Display.setCursor(0, 0);

    M5StamPLC.Display.setTextColor(TFT_GREENYELLOW);
    M5StamPLC.Display.println("== Multi IO HotPlug ==");

    M5StamPLC.Display.setTextColor(TFT_CYAN);
    M5StamPLC.Display.printf("Online: %d\n\n", io_mgr.count());

    for (uint8_t i = 0; i < io_mgr.count() && i < 6; i++) {
        M5StamPLC_IO* dev = io_mgr.get(i);
        uint8_t addr      = dev->getCurrentAddress();
        M5StamPLC.Display.setTextColor(TFT_GREEN);
        M5StamPLC.Display.printf("0x%02X  FW:0x%02X", addr, dev->getFirmwareVersion());
        M5StamPLC.Display.setTextColor(TFT_YELLOW);
        M5StamPLC.Display.printf("  DIP:0x%02X\n", dev->getExpectedAddress());
    }
}

void setup()
{
    M5StamPLC.begin();
    M5StamPLC.Display.setTextScroll(false);
    M5StamPLC.Display.setTextSize(1);
    M5StamPLC.Display.setFont(&fonts::efontCN_16);

    Serial.begin(115200);
    Serial.println("\n=== M5StamPLC IO Multi-Device Hot-Plug ===");

    io_mgr.onConnect(onConnect);
    io_mgr.onDisconnect(onDisconnect);
    io_mgr.onAddrChange(onAddrChange);
    io_mgr.begin();

    updateDisplay();
}

void loop()
{
    M5StamPLC.update();
    io_mgr.update();
    updateDisplay();
    delay(1000);
}
