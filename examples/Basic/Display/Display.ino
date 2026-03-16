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
}

void loop()
{
    /* Fill screen with colors */
    M5StamPLC.Display().fillScreen(TFT_WHITE);
    delay(800);
    M5StamPLC.Display().fillScreen(TFT_RED);
    delay(800);
    M5StamPLC.Display().fillScreen(TFT_GREEN);
    delay(800);
    M5StamPLC.Display().fillScreen(TFT_BLUE);
    delay(800);
    M5StamPLC.Display().fillScreen(TFT_BLACK);
    delay(800);

    /* Set cursor and print text */
    M5StamPLC.Display().setCursor(10, 10);
    M5StamPLC.Display().setTextColor(TFT_WHITE);
    M5StamPLC.Display().setTextSize(1);
    M5StamPLC.Display().printf("Display Test!");
    delay(800);

    /* Draw shapes */
    M5StamPLC.Display().drawRect(100, 50, 50, 50, BLUE);
    delay(800);
    M5StamPLC.Display().fillRect(100, 50, 50, 50, BLUE);
    delay(800);
    M5StamPLC.Display().drawCircle(100, 50, 50, RED);
    delay(800);
    M5StamPLC.Display().fillCircle(100, 50, 50, RED);
    delay(800);
    M5StamPLC.Display().drawTriangle(30, 30, 180, 50, 80, 100, YELLOW);
    delay(800);
    M5StamPLC.Display().fillTriangle(30, 30, 180, 50, 80, 100, YELLOW);
    delay(800);

    /* Draw random triangles */
    for (int i = 0; i < 1000; i++) {
        M5StamPLC.Display().fillTriangle(
            random(M5StamPLC.Display().width() - 1), random(M5StamPLC.Display().height() - 1),
            random(M5StamPLC.Display().width() - 1), random(M5StamPLC.Display().height() - 1),
            random(M5StamPLC.Display().width() - 1), random(M5StamPLC.Display().height() - 1), random(0xfffe));
    }
    delay(800);
}
