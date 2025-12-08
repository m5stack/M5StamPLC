/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <M5StamPLC.h>

#define PIN_IRQ 14

void setup()
{
    /* Init M5StamPLC */
    M5StamPLC.begin();

    /* Enable PI4IOE5V6408 irq */
    auto& ioe = M5StamPLC.getIOExpanderA();
    ioe.enableIrq();

    /* Setup irq pin */
    pinMode(PIN_IRQ, INPUT_PULLUP);

    printf("Press button A/B/C to trigger irq\n");
}

void loop()
{
    /* Check irq pin */
    if (digitalRead(PIN_IRQ) == LOW) {
        printf("PI4IOE5V6408 irq detected\n");

        /* Clear irq */
        auto& ioe = M5StamPLC.getIOExpanderA();
        ioe.resetIrq();

        delay(100);
    }

    delay(20);
}
