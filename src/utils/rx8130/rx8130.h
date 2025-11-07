/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <M5Unified.h>
#include <driver/gpio.h>
// https://download.epsondevice.com/td/pdf/app/RX8130CE_en.pdf
// https://github.com/alexreinert/piVCCU/blob/master/kernel/rtc-rx8130.c

class RX8130_Class : public m5::I2C_Device {
public:
    RX8130_Class(std::uint8_t i2c_addr = 0x32, std::uint32_t freq = 400000, m5::I2C_Class *i2c = &m5::In_I2C)
        : I2C_Device(i2c_addr, freq, i2c)
    {
    }

    bool begin();

    void initBat();
    void setTime(struct tm *time);
    void getTime(struct tm *time);
    void clearIrqFlags();
    void disableIrq();
};
