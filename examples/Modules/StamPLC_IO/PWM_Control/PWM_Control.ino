/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 *
 * M5StamPLC IO PWM Control Example
 *
 * Button A:        Toggle PWM mode on/off
 * Button B:        CH1 duty +10%
 * Long press B:    CH1 duty -10%
 * Button C:        CH2 duty +10%
 * Long press C:    CH2 duty -10%
 */
#include <Arduino.h>
#include <M5StamPLC.h>

M5StamPLC_IO stamplc_io;

bool pwm_mode         = false;
uint8_t pwm_freq      = 50;
uint16_t ch1_duty     = 0;
uint16_t ch2_duty     = 0;
bool btnB_longPressed = false;
bool btnC_longPressed = false;

void updateDisplay()
{
    M5StamPLC.Display.fillScreen(TFT_BLACK);
    M5StamPLC.Display.setCursor(0, 0);

    M5StamPLC.Display.setTextColor(TFT_GREENYELLOW);
    M5StamPLC.Display.println("=== PWM Control ===");

    M5StamPLC.Display.setTextColor(TFT_CYAN);
    M5StamPLC.Display.printf("Mode: %s  Freq: %dHz\n", pwm_mode ? "PWM" : "IO", pwm_freq);

    M5StamPLC.Display.setTextColor(TFT_YELLOW);
    M5StamPLC.Display.printf("CH1: %d.%d%% (%d/1000)\n", ch1_duty / 10, ch1_duty % 10, ch1_duty);
    M5StamPLC.Display.printf("CH2: %d.%d%% (%d/1000)\n", ch2_duty / 10, ch2_duty % 10, ch2_duty);

    M5StamPLC.Display.setTextColor(TFT_DARKGREY);
    M5StamPLC.Display.println("A: Toggle PWM mode");
    M5StamPLC.Display.println("B/C: duty+  Long: duty-");
}

void setup()
{
    M5StamPLC.begin();
    M5StamPLC.Display.setTextScroll(false);
    M5StamPLC.Display.setTextSize(1);
    M5StamPLC.Display.setFont(&fonts::efontCN_16);
    M5StamPLC.Display.println("Try to find M5StamPLC IO module...");

    while (!stamplc_io.begin()) {
        M5StamPLC.Display.println("Not found, retry in 1s...");
        delay(1000);
    }

    M5StamPLC.Display.printf("Found: 0x%02X  FW: 0x%02X\n", stamplc_io.getCurrentAddress(),
                             stamplc_io.getFirmwareVersion());

    pwm_mode = stamplc_io.getPWMMode();
    pwm_freq = stamplc_io.getPWMFrequency();
    ch1_duty = stamplc_io.getChannelDuty(1);
    ch2_duty = stamplc_io.getChannelDuty(2);

    if (pwm_freq == 0) {
        pwm_freq = 50;
        stamplc_io.setPWMFrequency(pwm_freq);
    }

    updateDisplay();
}

void loop()
{
    M5StamPLC.update();

    bool needUpdate = false;

    if (M5StamPLC.BtnA.wasClicked()) {
        pwm_mode = !pwm_mode;
        stamplc_io.setPWMMode(pwm_mode);
        needUpdate = true;
    }

    if (M5StamPLC.BtnB.wasClicked()) {
        ch1_duty = (ch1_duty + 100 > 1000) ? 1000 : ch1_duty + 100;
        stamplc_io.setChannelDuty(1, ch1_duty);
        needUpdate = true;
    } else if (M5StamPLC.BtnB.pressedFor(500) && !btnB_longPressed) {
        btnB_longPressed = true;
        ch1_duty         = (ch1_duty >= 100) ? ch1_duty - 100 : 0;
        stamplc_io.setChannelDuty(1, ch1_duty);
        needUpdate = true;
    } else if (!M5StamPLC.BtnB.isPressed()) {
        btnB_longPressed = false;
    }

    if (M5StamPLC.BtnC.wasClicked()) {
        ch2_duty = (ch2_duty + 100 > 1000) ? 1000 : ch2_duty + 100;
        stamplc_io.setChannelDuty(2, ch2_duty);
        needUpdate = true;
    } else if (M5StamPLC.BtnC.pressedFor(500) && !btnC_longPressed) {
        btnC_longPressed = true;
        ch2_duty         = (ch2_duty >= 100) ? ch2_duty - 100 : 0;
        stamplc_io.setChannelDuty(2, ch2_duty);
        needUpdate = true;
    } else if (!M5StamPLC.BtnC.isPressed()) {
        btnC_longPressed = false;
    }

    if (stamplc_io.syncAddress()) needUpdate = true;

    if (needUpdate) updateDisplay();

    delay(10);
}
