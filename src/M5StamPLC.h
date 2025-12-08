/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include "pin_config.h"
#include "utils/aw9523/aw9523.h"
#include "utils/lm75b/lm75b.h"
#include "utils/rx8130/rx8130.h"
#include "modules/M5StamPLC_AC.h"
#include <M5GFX.h>
#include <M5Unified.hpp>
#include <driver/twai.h>
#include <mbcontroller.h>

namespace m5 {

class M5_STAMPLC {
public:
    struct Config_t {
        /* Modbus */
        bool enableModbusSlave = false;
        uint8_t modbusSlaveId  = 1;
        long modbusBaudRate    = 115200;

        /* CAN */
        bool enableCan   = false;
        long canBaudRate = 1000000;  // 25000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000

        /* SD Card */
        bool enableSdCard = false;
    };

    Config_t config(void) const
    {
        return _config;
    }
    void config(const Config_t& cfg)
    {
        _config = cfg;
    }

    void begin();
    void update();

    LGFX_Device& Display = M5.Display;
    LGFX_Device& Lcd     = M5.Lcd;

    LM75B_Class LM75B;
    INA226_Class INA226;
    RX8130_Class RX8130;
    Button_Class& BtnA = M5.BtnA;
    Button_Class& BtnB = M5.BtnB;
    Button_Class& BtnC = M5.BtnC;

    /**
     * @brief Set Status Light
     *
     * @param r
     * @param g
     * @param b
     */
    void setStatusLight(const uint8_t& r, const uint8_t& g, const uint8_t& b);

    /**
     * @brief Control the LCD backlight
     *
     * @param on true to turn backlight ON, false to turn it OFF
     */
    void setBacklight(bool on);

    /**
     * @brief Read PLC Input
     *
     * @param channel 0-7
     * @return true
     * @return false
     */
    bool readPlcInput(const uint8_t& channel);

    /**
     * @brief Read PLC Relay state
     *
     * @param channel 0-3
     * @return true if ON, false if OFF
     */
    bool readPlcRelay(const uint8_t& channel);

    /**
     * @brief Write PLC Relay state
     *
     * @param channel 0-3
     * @param state true if ON, false if OFF
     */
    void writePlcRelay(const uint8_t& channel, const bool& state);

    /**
     * @brief Write all PLC relays
     *
     * @param relayState
     */
    void writePlcAllRelay(const uint8_t& relayState);

    /**
     * @brief Get the Rtc Time
     *
     * @param time
     */
    void getRtcTime(struct tm* time);

    /**
     * @brief Set the Rtc Time
     *
     * @param time
     */
    void setRtcTime(struct tm* time);

    /**
     * @brief Get the current temperature measurement of the LM75B
     *
     * @return float The current temperature measurement in °C.
     */
    float getTemp();

    /**
     * @brief Get the current power voltage measurement of the INA226
     *
     * @return float The current power voltage measurement in V.
     */
    float getPowerVoltage();

    /**
     * @brief Get the current output current measurement of the right side io socket
     *
     * @return float The current output current measurement in A.
     */
    float getIoSocketOutputCurrent();

    /**
     * @brief Play a tone on buzzer
     *
     * @param frequency
     * @param duration
     */
    void tone(unsigned int frequency, unsigned long duration = 0UL);

    /**
     * @brief Stop buzzer playing
     *
     */
    void noTone();

    /**
     * @brief Get the IOExpander A, this is the IOExpander that controls the status light and button A/B/C
     *
     * @return m5::IOExpander_Base&
     */
    m5::IOExpander_Base& getIOExpanderA();

    /**
     * @brief Get the IOExpander B, this is the IOExpander that controls the plc relays and plc inputs
     *
     * @return AW9523_Class&
     */
    AW9523_Class& getIOExpanderB();

protected:
    AW9523_Class* _io_expander_b = nullptr;  // Controls plc relays, plc inputs
    Config_t _config;

    void i2c_init();
    void io_expander_a_init();
    void io_expander_b_init();
    void lm75b_init();
    void ina226_init();
    void rx8130_init();
    void modbus_slave_init();
    void can_init();
    void sd_card_init();
};

}  // namespace m5

extern m5::M5_STAMPLC M5StamPLC;
