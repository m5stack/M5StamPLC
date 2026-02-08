/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 *
 * M5StamPLC IO PWM Control Example
 *
 * This example demonstrates how to use the PWM control function of the M5StamPLC IO module:
 * - Switch between IO mode and PWM mode
 * - Set PWM frequency (1-100 Hz)
 * - Adjust CH1 and CH2 duty cycle (0-1000 thousandths)
 *
 * Button functions:
 * - Button A: Toggle PWM mode on/off
 * - Button B: Increase CH1 duty cycle (+10%)
 * - Long press B: Decrease CH1 duty cycle (-10%)
 * - Button C: Increase CH2 duty cycle (+10%)
 * - Long press C: Decrease CH2 duty cycle (-10%)
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

unsigned long lastUpdateTime = 0;

void updateDisplay()
{
    M5StamPLC.Display.fillScreen(TFT_BLACK);
    M5StamPLC.Display.setCursor(0, 0);

    M5StamPLC.Display.setTextColor(TFT_GREENYELLOW);
    M5StamPLC.Display.println("=== PWM Control ===");

    M5StamPLC.Display.setTextColor(TFT_CYAN);
    M5StamPLC.Display.printf("Mode: %s\n", pwm_mode ? "PWM" : "IO");
    M5StamPLC.Display.printf("Frequency: %d Hz\n", pwm_freq);

    M5StamPLC.Display.setTextColor(TFT_YELLOW);
    M5StamPLC.Display.printf("CH1 Duty: %d.%d%% (%d/1000)\n", ch1_duty / 10, ch1_duty % 10, ch1_duty);
    M5StamPLC.Display.printf("CH2 Duty: %d.%d%% (%d/1000)\n", ch2_duty / 10, ch2_duty % 10, ch2_duty);

    M5StamPLC.Display.setTextColor(TFT_DARKGREY);
    M5StamPLC.Display.println("A: Toggle PWM mode");
    M5StamPLC.Display.println("B: CH1+ Long press: CH1-");
    M5StamPLC.Display.println("C: CH2+ Long press: CH2-");
}

void setup()
{
    M5StamPLC.begin();
    M5StamPLC.Display.setTextScroll(false);
    M5StamPLC.Display.setTextSize(1);
    M5StamPLC.Display.setFont(&fonts::efontCN_16);
    M5StamPLC.Display.println("Try to find M5StamPLC IO module...");

    /* Init M5StamPLC IO */
    while (!stamplc_io.begin()) {
        M5StamPLC.Display.println("Not found, retry in 1s...");
        delay(1000);
    }

    M5StamPLC.Display.printf("Found IO module: 0x%02X\n", stamplc_io.getCurrentAddress());
    M5StamPLC.Display.printf("Firmware Version: 0x%02X\n", stamplc_io.getFirmwareVersion());

    delay(2000);

    pwm_mode = stamplc_io.getPWMMode();
    pwm_freq = stamplc_io.getPWMFrequency();
    ch1_duty = stamplc_io.getChannelDuty(1);
    ch2_duty = stamplc_io.getChannelDuty(2);

    if (pwm_freq == 0) {
        pwm_freq = 50;
        stamplc_io.setPWMFrequency(pwm_freq);
    }

    updateDisplay();
    lastUpdateTime = millis();
}

void loop()
{
    M5StamPLC.update();

    bool needUpdate = false;

    // Button A: Toggle PWM mode
    if (M5StamPLC.BtnA.wasClicked()) {
        pwm_mode = !pwm_mode;
        stamplc_io.setPWMMode(pwm_mode);
        needUpdate = true;
    }

    // Button B: Adjust CH1 duty cycle
    if (M5StamPLC.BtnB.wasClicked()) {
        // Increase 10% (100/1000)
        ch1_duty += 100;
        if (ch1_duty > 1000) ch1_duty = 1000;
        stamplc_io.setChannelDuty(1, ch1_duty);
        needUpdate = true;
    } else if (M5StamPLC.BtnB.pressedFor(500) && !btnB_longPressed) {
        // Decrease 10%
        btnB_longPressed = true;
        if (ch1_duty >= 100) {
            ch1_duty -= 100;
        } else {
            ch1_duty = 0;
        }
        stamplc_io.setChannelDuty(1, ch1_duty);
        needUpdate = true;
    } else if (!M5StamPLC.BtnB.isPressed()) {
        btnB_longPressed = false;
    }

    // Button C: Adjust CH2 duty cycle
    if (M5StamPLC.BtnC.wasClicked()) {
        // Increase 10% (100/1000)
        ch2_duty += 100;
        if (ch2_duty > 1000) ch2_duty = 1000;
        stamplc_io.setChannelDuty(2, ch2_duty);
        needUpdate = true;
    } else if (M5StamPLC.BtnC.pressedFor(500) && !btnC_longPressed) {
        // Decrease 10%
        btnC_longPressed = true;
        if (ch2_duty >= 100) {
            ch2_duty -= 100;
        } else {
            ch2_duty = 0;
        }
        stamplc_io.setChannelDuty(2, ch2_duty);
        needUpdate = true;
    } else if (!M5StamPLC.BtnC.isPressed()) {
        btnC_longPressed = false;
    }

    // Periodically update display
    if (needUpdate || (millis() - lastUpdateTime > 1000)) {
        updateDisplay();
        lastUpdateTime = millis();
    }

    delay(10);
}
