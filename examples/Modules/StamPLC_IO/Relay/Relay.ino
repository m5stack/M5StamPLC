/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <M5StamPLC.h>

M5StamPLC_IO stamplc_io;
bool relay_state[3]   = {false, false, false};
bool btnB_longPressed = false;
bool btnC_longPressed = false;

void setup()
{
    /* Init M5StamPLC*/
    M5StamPLC.begin();
    M5StamPLC.Display.setTextScroll(true);
    M5StamPLC.Display.setTextSize(1);
    M5StamPLC.Display.setFont(&fonts::efontCN_16);
    M5StamPLC.Display.println("Try to find M5StamPLC IO");

    /* Init M5StamPLC IO */
    while (!stamplc_io.begin()) {
        M5StamPLC.Display.println("M5StamPLC_IO not found, retry in 1s...");
        delay(1000);
    }

    M5StamPLC.Display.printf("M5StamPLC IO found in 0x%02X\n", stamplc_io.getCurrentAddress());
    M5StamPLC.Display.printf("Firmware Version: 0x%02X\n", stamplc_io.getFirmwareVersion());
    M5StamPLC.Display.printf("push A/B/C to switch relay\n");
    M5StamPLC.Display.printf("long press B/C to switch pull-up\n");
}

void loop()
{
    M5StamPLC.update();

    if (M5StamPLC.BtnA.wasClicked()) {
        M5StamPLC.Display.printf("BtnA switch relay 0 to %s\n", relay_state[0] ? "OFF" : "ON");
        relay_state[0] = !relay_state[0];
        stamplc_io.setRelayState(M5StamPLC_IO::BIT_RELAY_TRIG, relay_state[0]);
    }

    if (M5StamPLC.BtnB.wasClicked()) {
        M5StamPLC.Display.printf("BtnB switch relay 1 to %s\n", relay_state[1] ? "OFF" : "ON");
        relay_state[1] = !relay_state[1];
        stamplc_io.setRelayState(M5StamPLC_IO::BIT_EX_CTR_1, relay_state[1]);
    } else if (M5StamPLC.BtnB.pressedFor(1000) && !btnB_longPressed) {
        btnB_longPressed = true;
        M5StamPLC.Display.printf("switch CH1 pull-up\n");
        stamplc_io.toggleIOBit(M5StamPLC_IO::BIT_CH1_PU_EN);
    } else if (!M5StamPLC.BtnB.isPressed()) {
        btnB_longPressed = false;
    }

    if (M5StamPLC.BtnC.wasClicked()) {
        M5StamPLC.Display.printf("BtnC switch relay 2 to %s\n", relay_state[2] ? "OFF" : "ON");
        relay_state[2] = !relay_state[2];
        stamplc_io.setRelayState(M5StamPLC_IO::BIT_EX_CTR_2, relay_state[2]);
    } else if (M5StamPLC.BtnC.pressedFor(1000) && !btnC_longPressed) {
        btnC_longPressed = true;
        M5StamPLC.Display.printf("switch CH2 pull-up\n");
        stamplc_io.toggleIOBit(M5StamPLC_IO::BIT_CH2_PU_EN);
    } else if (!M5StamPLC.BtnC.isPressed()) {
        btnC_longPressed = false;
    }

    if (stamplc_io.syncAddress()) {
        M5StamPLC.Display.printf("Address changed to 0x%02X\n", stamplc_io.getCurrentAddress());
    }

    delay(10);
}
