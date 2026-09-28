/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "M5StamPLC_AC.h"
#include "utils/make_unique_compat.h"
#include <esp_log.h>
#include <memory>

static const char* _tag = "M5StamPLC_AC";

static const uint8_t _ioe_address        = 0x44;
static const uint8_t _pin_relay          = 2;
static const uint8_t _pin_status_light_r = 5;
static const uint8_t _pin_status_light_g = 6;
static const uint8_t _pin_status_light_b = 7;

bool M5StamPLC_AC::begin()
{
    if (_ioe) {
        ESP_LOGW(_tag, "IOExpander already initialized");
        return true;
    }

    _ioe = std::make_unique<m5::PI4IOE5V6408_Class>(_ioe_address, 400000, &m5::In_I2C);
    if (!_ioe->begin()) {
        _ioe.reset();
        ESP_LOGE(_tag, "IOExpander initialization failed");
        return false;
    }

    // AC relay
    _ioe->setDirection(_pin_relay, true);
    _ioe->setPullMode(_pin_relay, m5::IOExpander_Base::pull_down);
    _ioe->setHighImpedance(_pin_relay, false);

    // Status light
    auto setup_status_light_pin = [](m5::PI4IOE5V6408_Class* ioe, uint8_t pin) {
        ioe->setDirection(pin, true);
        ioe->setPullMode(pin, m5::IOExpander_Base::pull_up);
        ioe->setHighImpedance(pin, false);
        ioe->digitalWrite(pin, true);
    };

    setup_status_light_pin(_ioe.get(), _pin_status_light_r);
    setup_status_light_pin(_ioe.get(), _pin_status_light_g);
    setup_status_light_pin(_ioe.get(), _pin_status_light_b);

    return true;
}

bool M5StamPLC_AC::readRelay()
{
    if (!is_ioe_valid()) {
        return false;
    }

    return _ioe->getWriteValue(_pin_relay);
}

void M5StamPLC_AC::writeRelay(const bool& state)
{
    if (!is_ioe_valid()) {
        return;
    }

    _ioe->digitalWrite(_pin_relay, state);
}

void M5StamPLC_AC::setStatusLight(const uint8_t& r, const uint8_t& g, const uint8_t& b)
{
    if (!is_ioe_valid()) {
        return;
    }

    auto set_channel = [](m5::PI4IOE5V6408_Class* ioe, uint8_t pin, uint8_t value) {
        if (value == 0) {
            ioe->setHighImpedance(pin, true);
        } else {
            ioe->setHighImpedance(pin, false);
            ioe->digitalWrite(pin, false);
        }
    };

    set_channel(_ioe.get(), _pin_status_light_r, r);
    set_channel(_ioe.get(), _pin_status_light_g, g);
    set_channel(_ioe.get(), _pin_status_light_b, b);
}

bool M5StamPLC_AC::is_ioe_valid()
{
    if (!_ioe) {
        ESP_LOGE(_tag, "IOExpander not initialized");
        return false;
    }
    return true;
}
