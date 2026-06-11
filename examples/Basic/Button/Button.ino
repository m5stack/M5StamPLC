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

    M5StamPLC.Display().setTextScroll(true);
    M5StamPLC.Display().setTextColor(TFT_GREENYELLOW);
    M5StamPLC.Display().println("Button example");
    M5StamPLC.Display().setTextColor(TFT_YELLOW);
}

void loop()
{
    /* Update button states */
    M5StamPLC.update();

    /* Check if button was clicked */
    if (M5StamPLC.BtnA().wasClicked()) {
        M5StamPLC.Display().println("Button A was clicked");
    } else if (M5StamPLC.BtnB().wasClicked()) {
        M5StamPLC.Display().println("Button B was clicked");
    } else if (M5StamPLC.BtnC().wasClicked()) {
        M5StamPLC.Display().println("Button C was clicked");
    }

    delay(100);
}
