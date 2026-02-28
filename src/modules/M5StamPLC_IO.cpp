/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "M5StamPLC_IO.h"
#include <esp_log.h>

static const char* _tag = "M5StamPLC_IO";

bool M5StamPLC_IO::begin(uint8_t addr, bool debug)
{
    esp_log_level_set(_tag, debug ? ESP_LOG_INFO : ESP_LOG_WARN);

    if (addr == 0) {
        _current_addr = scanI2CDevices();
        if (_current_addr == 0) {
            ESP_LOGE(_tag, "No I2C device found in range 0x%02X-0x%02X", I2C_ADDR_MIN, I2C_ADDR_MAX);
            return false;
        }
    } else {
        if (addr < I2C_ADDR_MIN || addr > I2C_ADDR_MAX) {
            ESP_LOGE(_tag, "Invalid I2C address: 0x%02X", addr);
            return false;
        }
        _current_addr = addr;
    }

    ESP_LOGI(_tag, "Found I2C device at address: 0x%02X", _current_addr);

    if (!waitForSystemReady()) {
        ESP_LOGW(_tag, "System ready timeout, but continuing");
    }

    uint8_t firmware_version = getFirmwareVersion();
    ESP_LOGI(_tag, "Firmware version: 0x%02X", firmware_version);

    uint8_t io_state      = readRegister(REG_IO_CONTROL);
    uint8_t desired_state = io_state & ~((1 << BIT_CH1_PU_EN) | (1 << BIT_CH2_PU_EN));
    if (desired_state != io_state) {
        ESP_LOGI(_tag, "Disabling CH1 and CH2 pull-ups: 0x%02X -> 0x%02X", io_state, desired_state);
        writeRegister(REG_IO_CONTROL, desired_state);
    }

    return true;
}

uint8_t M5StamPLC_IO::scanI2CDevices()
{
    bool i2c_devices[0x80] = {false};
    m5::In_I2C.scanID(i2c_devices);

    for (uint8_t addr = I2C_ADDR_MIN; addr <= I2C_ADDR_MAX; addr++) {
        if (i2c_devices[addr]) {
            ESP_LOGI(_tag, "Found I2C device: 0x%02X", addr);
            return addr;
        }
    }

    ESP_LOGW(_tag, "No I2C device found in range 0x%02X-0x%02X", I2C_ADDR_MIN, I2C_ADDR_MAX);
    return 0;
}

bool M5StamPLC_IO::waitForSystemReady(uint32_t timeout_ms)
{
    unsigned long start_time = millis();

    while (millis() - start_time < timeout_ms) {
        uint8_t status = getSystemStatus();

        if (status == 0) {
            ESP_LOGI(_tag, "System ready, INA226 initialized");
            return true;
        }

        if ((status & (1 << SYS_CH1_INA226_ERROR)) != 0) {
            ESP_LOGD(_tag, "Waiting for CH1 INA226 initialization...");
        }
        if ((status & (1 << SYS_CH2_INA226_ERROR)) != 0) {
            ESP_LOGD(_tag, "Waiting for CH2 INA226 initialization...");
        }

        delay(100);
    }

    ESP_LOGW(_tag, "System ready timeout, final status: 0x%02X", getSystemStatus());
    return false;
}

uint8_t M5StamPLC_IO::readRegister(uint8_t reg)
{
    if (_current_addr == 0) {
        ESP_LOGE(_tag, "Device not initialized");
        return 0;
    }

    uint8_t value = m5::In_I2C.readRegister8(_current_addr, reg, 400000);

    if (value == 0x00 || value == 0xFF) {
        delay(10);
        uint8_t retry_value = m5::In_I2C.readRegister8(_current_addr, reg, 100000);
        if (retry_value != value) {
            ESP_LOGD(_tag, "Retry read register 0x%02X: 0x%02X -> 0x%02X", reg, value, retry_value);
            value = retry_value;
        }
    }

    return value;
}

void M5StamPLC_IO::writeRegister(uint8_t reg, uint8_t value)
{
    if (_current_addr == 0) {
        ESP_LOGE(_tag, "Device not initialized");
        return;
    }

    bool success = m5::In_I2C.writeRegister8(_current_addr, reg, value, 400000);
    if (!success) {
        ESP_LOGW(_tag, "Failed to write register 0x%02X = 0x%02X", reg, value);
    }
}

int16_t M5StamPLC_IO::readVoltage(uint8_t channel)
{
    if (_current_addr == 0) {
        ESP_LOGE(_tag, "Device not initialized");
        return 0;
    }

    uint8_t data[2];
    uint8_t start_reg;

    if (channel == 1) {
        start_reg = REG_V_CH1_LSB;
    } else if (channel == 2) {
        start_reg = REG_V_CH2_LSB;
    } else {
        ESP_LOGW(_tag, "Invalid channel: %d", channel);
        return 0;
    }

    if (m5::In_I2C.readRegister(_current_addr, start_reg, data, 2, 400000)) {
        uint16_t raw_value = (data[1] << 8) | data[0];
        return (int16_t)raw_value;
    } else {
        ESP_LOGW(_tag, "Failed to read CH%d voltage", channel);
        return 0;
    }
}

int32_t M5StamPLC_IO::readCurrent(uint8_t channel)
{
    if (_current_addr == 0) {
        ESP_LOGE(_tag, "Device not initialized");
        return 0;
    }

    uint8_t data[4];
    uint8_t start_reg;

    if (channel == 1) {
        start_reg = REG_I_CH1_LSB;
    } else if (channel == 2) {
        start_reg = REG_I_CH2_LSB;
    } else {
        ESP_LOGW(_tag, "Invalid channel: %d", channel);
        return 0;
    }

    if (m5::In_I2C.readRegister(_current_addr, start_reg, data, 4, 400000)) {
        uint32_t raw_value = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
        return (int32_t)raw_value;
    } else {
        ESP_LOGW(_tag, "Failed to read CH%d current", channel);
        return 0;
    }
}

void M5StamPLC_IO::readAllChannelsData(int16_t* v1, int32_t* i1, int16_t* v2, int32_t* i2)
{
    if (_current_addr == 0) {
        ESP_LOGE(_tag, "Device not initialized");
        *v1 = *i1 = *v2 = *i2 = 0;
        return;
    }

    uint8_t data[12];

    if (m5::In_I2C.readRegister(_current_addr, REG_V_CH1_LSB, data, 12, 400000)) {
        uint16_t v1_raw = (data[1] << 8) | data[0];
        *v1             = (int16_t)v1_raw;

        uint32_t i1_raw = ((uint32_t)data[5] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[3] << 8) | data[2];
        *i1             = (int32_t)i1_raw;

        uint16_t v2_raw = (data[7] << 8) | data[6];
        *v2             = (int16_t)v2_raw;

        uint32_t i2_raw = ((uint32_t)data[11] << 24) | ((uint32_t)data[10] << 16) | ((uint32_t)data[9] << 8) | data[8];
        *i2             = (int32_t)i2_raw;

        ESP_LOGD(_tag, "Batch read: CH1=%dmV/%duA, CH2=%dmV/%duA", *v1, *i1, *v2, *i2);
    } else {
        ESP_LOGW(_tag, "Failed to batch read");
        *v1 = *i1 = *v2 = *i2 = 0;
    }
}

uint8_t M5StamPLC_IO::getSystemStatus()
{
    return readRegister(REG_SYSTEM_STATUS);
}

uint8_t M5StamPLC_IO::getFirmwareVersion()
{
    return readRegister(REG_FIRMWARE_VER);
}

uint8_t M5StamPLC_IO::getExpectedAddress()
{
    uint8_t config = readRegister(REG_ADDR_CONFIG);
    return config & 0x7F;
}

void M5StamPLC_IO::setNewAddress(uint8_t newAddr)
{
    if (newAddr < I2C_ADDR_MIN || newAddr > I2C_ADDR_MAX) {
        ESP_LOGW(_tag, "Invalid address: 0x%02X, must be in range 0x%02X-0x%02X", newAddr, I2C_ADDR_MIN, I2C_ADDR_MAX);
        return;
    }

    uint8_t config = newAddr | 0x80;
    writeRegister(REG_ADDR_CONFIG, config);

    ESP_LOGI(_tag, "Set new address: 0x%02X", newAddr);
    _current_addr = newAddr;
}

bool M5StamPLC_IO::syncAddress()
{
    uint8_t dip_addr = getExpectedAddress();

    if (dip_addr == _current_addr || dip_addr < I2C_ADDR_MIN || dip_addr > I2C_ADDR_MAX) {
        return false;
    }

    ESP_LOGI(_tag, "DIP address changed: 0x%02X -> 0x%02X, applying", _current_addr, dip_addr);
    setNewAddress(dip_addr);
    return true;
}

void M5StamPLC_IO::toggleIOBit(uint8_t bit)
{
    uint8_t oldState = readRegister(REG_IO_CONTROL);
    uint8_t newState = oldState ^ (1 << bit);

    ESP_LOGI(_tag, "Toggle IO bit%d: %s -> %s", bit, (oldState & (1 << bit)) ? "ON" : "OFF",
             (newState & (1 << bit)) ? "ON" : "OFF");

    writeRegister(REG_IO_CONTROL, newState);
}

void M5StamPLC_IO::setRelayState(uint8_t bit, bool state)
{
    uint8_t currentState = readRegister(REG_IO_CONTROL);
    uint8_t newState;

    if (state) {
        newState = currentState | (1 << bit);
    } else {
        newState = currentState & ~(1 << bit);
    }

    if (newState != currentState) {
        ESP_LOGI(_tag, "Set relay bit%d to %s", bit, state ? "ON" : "OFF");
        writeRegister(REG_IO_CONTROL, newState);
    }
}

void M5StamPLC_IO::setAllRelays(bool state)
{
    uint8_t currentState = readRegister(REG_IO_CONTROL);
    uint8_t relay_mask   = (1 << BIT_RELAY_TRIG) | (1 << BIT_EX_CTR_1) | (1 << BIT_EX_CTR_2);
    uint8_t newState;

    if (state) {
        newState = currentState | relay_mask;
        ESP_LOGI(_tag, "Turn ON all relays");
    } else {
        newState = currentState & ~relay_mask;
        ESP_LOGI(_tag, "Turn OFF all relays");
    }

    if (newState != currentState) {
        ESP_LOGI(_tag, "Relay state: 0x%02X -> 0x%02X", currentState, newState);
        writeRegister(REG_IO_CONTROL, newState);
    }
}

esp_err_t M5StamPLC_IO::readINA226Config(uint8_t channel, uint16_t* config)
{
    if (_current_addr == 0) {
        ESP_LOGE(_tag, "Device not initialized");
        return ESP_FAIL;
    }

    uint8_t data[2];
    uint8_t start_reg;

    if (channel == 1) {
        start_reg = REG_INA226_CONFIG_CH1_LSB;
    } else if (channel == 2) {
        start_reg = REG_INA226_CONFIG_CH2_LSB;
    } else {
        ESP_LOGW(_tag, "Invalid channel: %d", channel);
        return ESP_FAIL;
    }

    if (m5::In_I2C.readRegister(_current_addr, start_reg, data, 2, 400000)) {
        *config = (data[1] << 8) | data[0];
        ESP_LOGI(_tag, "Read CH%d INA226 config: 0x%04X", channel, *config);
        return ESP_OK;
    } else {
        ESP_LOGW(_tag, "Failed to read CH%d INA226 config", channel);
        return ESP_FAIL;
    }
}

void M5StamPLC_IO::writeINA226Config(uint8_t channel, uint16_t config)
{
    if (_current_addr == 0) {
        ESP_LOGE(_tag, "Device not initialized");
        return;
    }

    uint8_t data[2];
    data[0] = config & 0xFF;
    data[1] = (config >> 8) & 0xFF;

    uint8_t start_reg;
    if (channel == 1) {
        start_reg = REG_INA226_CONFIG_CH1_LSB;
    } else if (channel == 2) {
        start_reg = REG_INA226_CONFIG_CH2_LSB;
    } else {
        ESP_LOGW(_tag, "Invalid channel: %d", channel);
        return;
    }

    bool success = m5::In_I2C.writeRegister(_current_addr, start_reg, data, 2, 400000);
    if (!success) {
        ESP_LOGW(_tag, "Failed to write CH%d INA226 config", channel);
        return;
    }

    ESP_LOGI(_tag, "Write CH%d INA226 config: 0x%04X", channel, config);
}

esp_err_t M5StamPLC_IO::setINA226ConversionTime(uint8_t channel, uint8_t vshct, uint8_t vbusct)
{
    uint16_t config;
    esp_err_t err = readINA226Config(channel, &config);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    config &= ~(INA226_VSHCT_MASK | INA226_VBUSCT_MASK);
    config |= ((vshct & 0x07) << 3);
    config |= ((vbusct & 0x07) << 6);

    writeINA226Config(channel, config);
    ESP_LOGI(_tag, "Set CH%d INA226 conversion time: VSHCT=%d, VBUSCT=%d", channel, vshct, vbusct);
    return ESP_OK;
}

esp_err_t M5StamPLC_IO::setINA226Averaging(uint8_t channel, uint8_t avg)
{
    uint16_t config;
    esp_err_t err = readINA226Config(channel, &config);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    config &= ~INA226_AVG_MASK;
    config |= ((avg & 0x07) << 9);

    writeINA226Config(channel, config);
    ESP_LOGI(_tag, "Set CH%d INA226 averaging: %d", channel, avg);
    return ESP_OK;
}

esp_err_t M5StamPLC_IO::getINA226ConversionTime(uint8_t channel, uint8_t* vshct, uint8_t* vbusct)
{
    uint16_t config;
    esp_err_t err = readINA226Config(channel, &config);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    *vshct  = (config & INA226_VSHCT_MASK) >> 3;
    *vbusct = (config & INA226_VBUSCT_MASK) >> 6;
    return ESP_OK;
}

esp_err_t M5StamPLC_IO::getINA226Averaging(uint8_t channel, uint8_t* avg)
{
    uint16_t config;
    esp_err_t err = readINA226Config(channel, &config);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    *avg = (config & INA226_AVG_MASK) >> 9;
    return ESP_OK;
}

void M5StamPLC_IO::setPWMMode(bool enable)
{
    uint8_t currentState = readRegister(REG_IO_CONTROL);
    uint8_t newState;

    if (enable) {
        newState = currentState | (1 << BIT_PWM_MODE);
        ESP_LOGI(_tag, "Enable PWM mode");
    } else {
        newState = currentState & ~(1 << BIT_PWM_MODE);
        ESP_LOGI(_tag, "Disable PWM mode");
    }

    if (newState != currentState) {
        writeRegister(REG_IO_CONTROL, newState);
    }
}

bool M5StamPLC_IO::getPWMMode()
{
    uint8_t state = readRegister(REG_IO_CONTROL);
    return (state & (1 << BIT_PWM_MODE)) != 0;
}

void M5StamPLC_IO::setPWMFrequency(uint8_t freq)
{
    if (freq < 1 || freq > 100) {
        ESP_LOGW(_tag, "Invalid PWM frequency: %d, must be 1-100 Hz", freq);
        return;
    }

    writeRegister(REG_PWM_FREQ, freq);
    ESP_LOGI(_tag, "Set PWM frequency: %d Hz", freq);
}

uint8_t M5StamPLC_IO::getPWMFrequency()
{
    return readRegister(REG_PWM_FREQ);
}

void M5StamPLC_IO::setChannelDuty(uint8_t channel, uint16_t duty)
{
    if (_current_addr == 0) {
        ESP_LOGE(_tag, "Device not initialized");
        return;
    }

    if (duty > 1000) {
        ESP_LOGW(_tag, "Invalid duty cycle: %d, must be 0-1000", duty);
        return;
    }

    uint8_t data[2];
    data[0] = duty & 0xFF;
    data[1] = (duty >> 8) & 0xFF;

    uint8_t start_reg;
    if (channel == 1) {
        start_reg = REG_CH1_DUTY_LSB;
    } else if (channel == 2) {
        start_reg = REG_CH2_DUTY_LSB;
    } else {
        ESP_LOGW(_tag, "Invalid channel: %d", channel);
        return;
    }

    bool success = m5::In_I2C.writeRegister(_current_addr, start_reg, data, 2, 400000);
    if (!success) {
        ESP_LOGW(_tag, "Failed to write CH%d duty cycle", channel);
        return;
    }

    ESP_LOGI(_tag, "Set CH%d duty cycle: %d/1000 (%d%%)", channel, duty, duty / 10);
}

uint16_t M5StamPLC_IO::getChannelDuty(uint8_t channel)
{
    uint8_t data[2];
    uint8_t start_reg;

    if (channel == 1) {
        start_reg = REG_CH1_DUTY_LSB;
    } else if (channel == 2) {
        start_reg = REG_CH2_DUTY_LSB;
    } else {
        ESP_LOGW(_tag, "Invalid channel: %d", channel);
        return 0;
    }

    if (m5::In_I2C.readRegister(_current_addr, start_reg, data, 2, 400000)) {
        uint16_t duty = (data[1] << 8) | data[0];
        return duty;
    } else {
        ESP_LOGW(_tag, "Failed to read CH%d duty cycle", channel);
        return 0;
    }
}

/* M5StamPLC_IO_Manager */

void M5StamPLC_IO_Manager::begin(uint32_t interval_ms)
{
    _interval_ms = interval_ms;
    _scan();
    _last_ms = millis();
}

void M5StamPLC_IO_Manager::update()
{
    if (millis() - _last_ms >= _interval_ms) {
        _last_ms = millis();  // update before scan so _scan() can override to 0 for fast retry
        _scan();
    }
}

M5StamPLC_IO* M5StamPLC_IO_Manager::get(uint8_t index)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (_slots[i].connected) {
            if (n == index) return &_slots[i].io;
            n++;
        }
    }
    return nullptr;
}

M5StamPLC_IO* M5StamPLC_IO_Manager::getByAddr(uint8_t addr)
{
    if (addr < M5StamPLC_IO::I2C_ADDR_MIN || addr > M5StamPLC_IO::I2C_ADDR_MAX) return nullptr;
    uint8_t idx = addr - M5StamPLC_IO::I2C_ADDR_MIN;
    return _slots[idx].connected ? &_slots[idx].io : nullptr;
}

void M5StamPLC_IO_Manager::_scan()
{
    bool found[0x80] = {false};
    bool changed     = false;
    for (uint8_t addr = M5StamPLC_IO::I2C_ADDR_MIN; addr <= M5StamPLC_IO::I2C_ADDR_MAX; addr++) {
        if (m5::In_I2C.scanID(addr)) {
            ESP_LOGI(_tag, "Found I2C device: 0x%02X", addr);
            found[addr] = true;
        }
    }

    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        uint8_t addr = M5StamPLC_IO::I2C_ADDR_MIN + i;
        Slot& s      = _slots[i];

        if (found[addr] && !s.connected) {
            if (s.io.begin(addr, false)) {
                s.connected = true;
                s.dip_addr  = s.io.getExpectedAddress();
                s.fw_ver    = s.io.getFirmwareVersion();
                _count++;
                changed = true;
                if (_on_connect) _on_connect(s.io, addr);
            }
        } else if (!found[addr] && s.connected) {
            s.connected = false;
            if (_count > 0) _count--;
            changed = true;
            if (_on_disconnect) _on_disconnect(addr);
        } else if (found[addr] && s.connected) {
            uint8_t new_dip = s.io.getExpectedAddress();
            if (new_dip != s.dip_addr && new_dip >= M5StamPLC_IO::I2C_ADDR_MIN &&
                new_dip <= M5StamPLC_IO::I2C_ADDR_MAX) {
                _applyAddrChange(i, new_dip);
                changed = true;
            }
        }
    }

    // Any topology change triggers an immediate follow-up scan on the next update() call.
    if (changed) {
        _last_ms = 0;
    }
}

void M5StamPLC_IO_Manager::_applyAddrChange(uint8_t old_idx, uint8_t new_dip)
{
    uint8_t new_idx  = new_dip - M5StamPLC_IO::I2C_ADDR_MIN;
    uint8_t old_addr = M5StamPLC_IO::I2C_ADDR_MIN + old_idx;

    if (_slots[new_idx].connected) {
        ESP_LOGE(_tag, "Address conflict! Dev at 0x%02X wants to move to 0x%02X, but slot is busy.", old_addr, new_dip);
        return;
    }

    // Write (new_dip | 0x80): firmware validates bit6:0 == DIP, then updates I2C address.
    // After this call, _slots[old_idx].io._current_addr == new_dip internally.
    _slots[old_idx].io.setNewAddress(new_dip);

    _slots[new_idx]           = _slots[old_idx];
    _slots[new_idx].dip_addr  = new_dip;
    _slots[old_idx].connected = false;

    if (_on_addr_change) _on_addr_change(old_addr, new_dip);
}
