/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <M5StamPLC.h>

void setup()
{
    /* Init M5StamPLC */
    M5StamPLC.begin();

    /* Play tone */
    // M5StamPLC.tone(frequency, duration);

    /* Stop playing tone */
    // M5StamPLC.noTone();
}

void loop()
{
    M5StamPLC.update();

    /* Play tone if button was clicked */
    if (M5StamPLC.BtnA().wasClicked()) {
        M5StamPLC.tone(523, 50);
    }
    if (M5StamPLC.BtnB().wasClicked()) {
        M5StamPLC.tone(659, 50);
    }
    if (M5StamPLC.BtnC().wasClicked()) {
        M5StamPLC.tone(880, 50);
    }

    delay(50);
}
