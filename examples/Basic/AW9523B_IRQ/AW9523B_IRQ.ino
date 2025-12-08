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

    /* Enable AW9523B input pins irq */
    auto& ioe                   = M5StamPLC.getIOExpanderB();
    std::vector<int> input_pins = {4, 5, 6, 7, 12, 13, 14, 15};
    for (const auto& i : input_pins) {
        ioe.enableInterrupt(i, true);
    }

    /* Setup irq pin */
    pinMode(PIN_IRQ, INPUT_PULLUP);

    printf("Change PLC input state to trigger irq\n");
}

void loop()
{
    /* Check irq pin */
    if (digitalRead(PIN_IRQ) == LOW) {
        printf("AW9523B irq detected\n");

        /* Clear irq */
        auto& ioe = M5StamPLC.getIOExpanderB();
        ioe.resetIrq();

        delay(100);
    }

    delay(20);
}
