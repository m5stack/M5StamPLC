/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <M5StamPLC.h>

M5Canvas canvas(&M5StamPLC.Display);
M5StamPLC_IO stamplc_io;

void setup()
{
    M5StamPLC.begin();
    canvas.createSprite(M5StamPLC.Display.width(), M5StamPLC.Display.height());
    canvas.setTextScroll(true);
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextSize(1);
    canvas.setFont(&fonts::efontCN_16);
    canvas.println("Try to find M5StamPLC IO");

    while (!stamplc_io.begin()) {
        canvas.println("M5StamPLC_IO not found, retry in 1s...");
        canvas.pushSprite(0, 0);
        delay(1000);
    }

    canvas.printf("Found: 0x%02X  FW: 0x%02X\n", stamplc_io.getCurrentAddress(), stamplc_io.getFirmwareVersion());

    uint8_t sys_status = stamplc_io.getSystemStatus();
    canvas.setTextColor(sys_status == 0 ? TFT_GREEN : TFT_RED);
    canvas.printf("System: %s\n", sys_status == 0 ? "Normal" : "Error");
    if (sys_status & (1 << M5StamPLC_IO::SYS_CH1_INA226_ERROR)) canvas.println("- CH1 INA226 Error");
    if (sys_status & (1 << M5StamPLC_IO::SYS_CH2_INA226_ERROR)) canvas.println("- CH2 INA226 Error");

    canvas.setTextColor(TFT_CYAN);
    canvas.println("INA226 Config:");
    for (uint8_t ch = 1; ch <= 2; ch++) {
        uint16_t cfg;
        if (stamplc_io.readINA226Config(ch, &cfg) == ESP_OK) {
            uint8_t vs, vb, avg;
            stamplc_io.getINA226ConversionTime(ch, &vs, &vb);
            stamplc_io.getINA226Averaging(ch, &avg);
            canvas.printf("CH%d: 0x%04X VS=%d VB=%d AVG=%d\n", ch, cfg, vs, vb, avg);
        }
    }

    canvas.setTextColor(TFT_YELLOW);
    canvas.println("press BtnC to start monitoring");
    canvas.pushSprite(0, 0);
    while (!M5StamPLC.BtnC.isPressed()) {
        M5StamPLC.update();
        delay(10);
    }
    canvas.clear();
}

void loop()
{
    M5StamPLC.update();

    static unsigned long last_update = 0;
    if (millis() - last_update > 1000) {
        last_update = millis();

        int16_t v1, v2;
        int32_t i1, i2;
        stamplc_io.readAllChannelsData(&v1, &i1, &v2, &i2);

        uint8_t io_ctrl    = stamplc_io.readRegister(M5StamPLC_IO::REG_IO_CONTROL);
        uint8_t sys_status = stamplc_io.getSystemStatus();

        canvas.fillScreen(TFT_BLACK);
        canvas.setCursor(0, 0);

        canvas.setTextColor(sys_status & (1 << M5StamPLC_IO::SYS_CH1_INA226_ERROR) ? TFT_RED : TFT_GREEN);
        canvas.printf("CH1: %d.%02dV %duA\n", v1 / 1000, abs(v1 % 1000) / 10, i1);

        canvas.setTextColor(sys_status & (1 << M5StamPLC_IO::SYS_CH2_INA226_ERROR) ? TFT_RED : TFT_GREEN);
        canvas.printf("CH2: %d.%02dV %duA\n", v2 / 1000, abs(v2 % 1000) / 10, i2);

        canvas.setTextColor(TFT_YELLOW);
        canvas.printf("Pull-up: CH1=%s CH2=%s\n", (io_ctrl & (1 << M5StamPLC_IO::BIT_CH1_PU_EN)) ? "ON" : "OFF",
                      (io_ctrl & (1 << M5StamPLC_IO::BIT_CH2_PU_EN)) ? "ON" : "OFF");

        canvas.setTextColor(TFT_MAGENTA);
        canvas.printf("Addr: 0x%02X  DIP: 0x%02X\n", stamplc_io.getCurrentAddress(), stamplc_io.getExpectedAddress());

        canvas.setTextColor(sys_status == 0 ? TFT_GREEN : TFT_RED);
        canvas.printf("System: %s\n", sys_status == 0 ? "Normal" : "Error");
    }

    if (M5StamPLC.BtnA.wasClicked()) stamplc_io.toggleIOBit(M5StamPLC_IO::BIT_CH1_PU_EN);
    if (M5StamPLC.BtnB.wasClicked()) stamplc_io.toggleIOBit(M5StamPLC_IO::BIT_CH2_PU_EN);

    if (stamplc_io.syncAddress()) {
        canvas.printf("Address changed to 0x%02X\n", stamplc_io.getCurrentAddress());
    }

    canvas.pushSprite(0, 0);
    delay(10);
}
