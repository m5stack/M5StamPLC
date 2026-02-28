/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <M5GFX.h>
#include <M5Unified.hpp>
#include <esp_err.h>

class M5StamPLC_IO {
public:
    // I2C ADDRESS RANGE
    static constexpr uint8_t I2C_ADDR_MIN = 0x20;
    static constexpr uint8_t I2C_ADDR_MAX = 0x2F;

    // REGISTER ADDRESS DEFINITION
    static constexpr uint8_t REG_V_CH1_LSB     = 0x00;
    static constexpr uint8_t REG_V_CH1_MSB     = 0x01;
    static constexpr uint8_t REG_I_CH1_LSB     = 0x02;
    static constexpr uint8_t REG_I_CH1_MSB     = 0x03;
    static constexpr uint8_t REG_I_CH1_EXT1    = 0x04;
    static constexpr uint8_t REG_I_CH1_EXT2    = 0x05;
    static constexpr uint8_t REG_V_CH2_LSB     = 0x06;
    static constexpr uint8_t REG_V_CH2_MSB     = 0x07;
    static constexpr uint8_t REG_I_CH2_LSB     = 0x08;
    static constexpr uint8_t REG_I_CH2_MSB     = 0x09;
    static constexpr uint8_t REG_I_CH2_EXT1    = 0x0A;
    static constexpr uint8_t REG_I_CH2_EXT2    = 0x0B;
    static constexpr uint8_t REG_IO_CONTROL    = 0x10;
    static constexpr uint8_t REG_PWM_FREQ      = 0x30;
    static constexpr uint8_t REG_CH1_DUTY_LSB  = 0x31;
    static constexpr uint8_t REG_CH1_DUTY_MSB  = 0x32;
    static constexpr uint8_t REG_CH2_DUTY_LSB  = 0x33;
    static constexpr uint8_t REG_CH2_DUTY_MSB  = 0x34;
    static constexpr uint8_t REG_SYSTEM_STATUS = 0xFB;
    static constexpr uint8_t REG_FIRMWARE_VER  = 0xFE;
    static constexpr uint8_t REG_ADDR_CONFIG   = 0xFF;

    // INA226 CONFIG REGISTER ADDRESS DEFINITION
    static constexpr uint8_t REG_INA226_CONFIG_CH1_LSB = 0x20;
    static constexpr uint8_t REG_INA226_CONFIG_CH1_MSB = 0x21;
    static constexpr uint8_t REG_INA226_CONFIG_CH2_LSB = 0x22;
    static constexpr uint8_t REG_INA226_CONFIG_CH2_MSB = 0x23;

    // INA226 CONFIG BIT DEFINITION
    static constexpr uint16_t INA226_VSHCT_MASK  = 0x38;
    static constexpr uint16_t INA226_VBUSCT_MASK = 0x1C0;
    static constexpr uint16_t INA226_AVG_MASK    = 0xE00;

    // IO CONTROL BIT DEFINITION
    static constexpr uint8_t BIT_EX_CTR_1   = 0;
    static constexpr uint8_t BIT_EX_CTR_2   = 1;
    static constexpr uint8_t BIT_CH1_PU_EN  = 2;
    static constexpr uint8_t BIT_CH2_PU_EN  = 3;
    static constexpr uint8_t BIT_RELAY_TRIG = 4;
    static constexpr uint8_t BIT_PWM_MODE   = 5;

    // SYSTEM STATUS BIT DEFINITION
    static constexpr uint8_t SYS_CH1_INA226_ERROR = 0;
    static constexpr uint8_t SYS_CH2_INA226_ERROR = 1;

    // INA226 CONVERSION TIME DEFINITION
    enum INA226_ConversionTime : uint8_t {
        TIME_140US   = 0x00,
        TIME_204US   = 0x01,
        TIME_332US   = 0x02,
        TIME_588US   = 0x03,
        TIME_1_1MS   = 0x04,
        TIME_2_116MS = 0x05,
        TIME_4_156MS = 0x06,
        TIME_8_244MS = 0x07
    };

    // INA226 AVERAGING DEFINITION
    enum INA226_Averaging : uint8_t {
        AVG_1    = 0x00,
        AVG_4    = 0x01,
        AVG_16   = 0x02,
        AVG_64   = 0x03,
        AVG_128  = 0x04,
        AVG_256  = 0x05,
        AVG_512  = 0x06,
        AVG_1024 = 0x07
    };

    /**
     * @brief Initialize M5StamPLC-IO
     *
     * @param addr I2C address (0x20-0x2F), if 0 will auto-scan
     * @param debug Enable debug logging (default: false)
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool begin(uint8_t addr = 0, bool debug = false);

    /**
     * @brief Scan for I2C devices
     *
     * @return I2C address if found, 0 if not found
     */
    uint8_t scanI2CDevices();

    /**
     * @brief Wait for system ready
     *
     * @param timeout_ms timeout in milliseconds
     * @return true if system is ready
     * @return false if timeout
     */
    bool waitForSystemReady(uint32_t timeout_ms = 5000);

    /**
     * @brief Read I2C register
     *
     * @param reg register address
     * @return register value
     */
    uint8_t readRegister(uint8_t reg);

    /**
     * @brief Write I2C register
     *
     * @param reg register address
     * @param value value to write
     */
    void writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief Read voltage (mV)
     *
     * @param channel 1 or 2
     * @return voltage in mV
     */
    int16_t readVoltage(uint8_t channel);

    /**
     * @brief Read current (uA)
     *
     * @param channel 1 or 2
     * @return current in uA
     */
    int32_t readCurrent(uint8_t channel);

    /**
     * @brief Read all channels data at once
     *
     * @param v1 pointer to CH1 voltage
     * @param i1 pointer to CH1 current
     * @param v2 pointer to CH2 voltage
     * @param i2 pointer to CH2 current
     */
    void readAllChannelsData(int16_t* v1, int32_t* i1, int16_t* v2, int32_t* i2);

    /**
     * @brief Get system status
     *
     * @return system status register value
     */
    uint8_t getSystemStatus();

    /**
     * @brief Get firmware version
     *
     * @return firmware version
     */
    uint8_t getFirmwareVersion();

    /**
     * @brief Get expected I2C address from configuration
     *
     * @return expected address
     */
    uint8_t getExpectedAddress();

    /**
     * @brief Set new I2C address
     *
     * @param newAddr new address (0x20-0x2F)
     */
    void setNewAddress(uint8_t newAddr);

    /**
     * @brief Check if the DIP-switch address has changed and apply it if so.
     *
     * Reads REG_ADDR_CONFIG bit6:0 (DIP-switch position) and compares with the
     * current I2C address. If different, calls setNewAddress() to apply the change.
     * Call this periodically in loop() when DIP-switch hot-swap is needed.
     *
     * @return true  if the address was updated
     * @return false if no change detected
     */
    bool syncAddress();

    /**
     * @brief Toggle IO control bit
     *
     * @param bit bit number to toggle
     */
    void toggleIOBit(uint8_t bit);

    /**
     * @brief Set relay state
     *
     * @param bit bit number
     * @param state true for ON, false for OFF
     */
    void setRelayState(uint8_t bit, bool state);

    /**
     * @brief Set all relays state
     *
     * @param state true for ON, false for OFF
     */
    void setAllRelays(bool state);

    /**
     * @brief Read INA226 configuration
     *
     * @param channel 1 or 2
     * @param config pointer to configuration value
     * @return ESP_OK on success
     */
    esp_err_t readINA226Config(uint8_t channel, uint16_t* config);

    /**
     * @brief Write INA226 configuration
     *
     * @param channel 1 or 2
     * @param config configuration value
     */
    void writeINA226Config(uint8_t channel, uint16_t config);

    /**
     * @brief Set INA226 conversion time
     *
     * @param channel 1 or 2
     * @param vshct VSHCT conversion time
     * @param vbusct VBUSCT conversion time
     * @return ESP_OK on success
     */
    esp_err_t setINA226ConversionTime(uint8_t channel, uint8_t vshct, uint8_t vbusct);

    /**
     * @brief Set INA226 averaging
     *
     * @param channel 1 or 2
     * @param avg averaging value
     * @return ESP_OK on success
     */
    esp_err_t setINA226Averaging(uint8_t channel, uint8_t avg);

    /**
     * @brief Get INA226 conversion time
     *
     * @param channel 1 or 2
     * @param vshct pointer to VSHCT conversion time
     * @param vbusct pointer to VBUSCT conversion time
     * @return ESP_OK on success
     */
    esp_err_t getINA226ConversionTime(uint8_t channel, uint8_t* vshct, uint8_t* vbusct);

    /**
     * @brief Get INA226 averaging
     *
     * @param channel 1 or 2
     * @param avg pointer to averaging value
     * @return ESP_OK on success
     */
    esp_err_t getINA226Averaging(uint8_t channel, uint8_t* avg);

    /**
     * @brief Get current I2C address
     *
     * @return current I2C address
     */
    uint8_t getCurrentAddress() const
    {
        return _current_addr;
    }

    /**
     * @brief Set PWM mode
     *
     * @param enable true for PWM mode, false for IO mode
     */
    void setPWMMode(bool enable);

    /**
     * @brief Get PWM mode
     *
     * @return true if PWM mode enabled
     */
    bool getPWMMode();

    /**
     * @brief Set PWM frequency
     *
     * @param freq frequency in Hz (1-100), default 50
     */
    void setPWMFrequency(uint8_t freq);

    /**
     * @brief Get PWM frequency
     *
     * @return frequency in Hz
     */
    uint8_t getPWMFrequency();

    /**
     * @brief Set channel duty cycle
     *
     * @param channel 1 or 2
     * @param duty duty cycle in permille (0-1000), default 0
     */
    void setChannelDuty(uint8_t channel, uint16_t duty);

    /**
     * @brief Get channel duty cycle
     *
     * @param channel 1 or 2
     * @return duty cycle in permille (0-1000)
     */
    uint16_t getChannelDuty(uint8_t channel);

protected:
    uint8_t _current_addr = 0;
};

/**
 * @brief Multi-device hot-plug manager for M5StamPLC IO modules.
 *
 * Scans the I2C bus periodically (0x20-0x2F), detects module insertion /
 * removal, and applies DIP-switch address changes on-the-fly.
 *
 * Address-change protocol (REG_ADDR_CONFIG = 0xFF):
 *   Read  bit6:0  - current DIP-switch address (hardware, read-only)
 *   Write bit6:0 + bit7=1 - firmware validates written value == DIP reading,
 *                           on match I2C address is updated, bit7 auto-clears
 *
 * Usage:
 *   M5StamPLC_IO_Manager mgr;
 *   mgr.onConnect([](M5StamPLC_IO& dev, uint8_t addr) { ... });
 *   mgr.onDisconnect([](uint8_t addr) { ... });
 *   mgr.onAddrChange([](uint8_t old_addr, uint8_t new_addr) { ... });
 *   mgr.begin();   // call once in setup()
 *   mgr.update();  // call in loop()
 */
class M5StamPLC_IO_Manager {
public:
    static constexpr uint8_t MAX_DEVICES = 16;

    struct Slot {
        M5StamPLC_IO io;
        bool connected   = false;
        uint8_t dip_addr = 0;
        uint8_t fw_ver   = 0;
    };

    using ConnectCb    = void (*)(M5StamPLC_IO& dev, uint8_t addr);
    using DisconnectCb = void (*)(uint8_t addr);
    using AddrChangeCb = void (*)(uint8_t old_addr, uint8_t new_addr);

    /**
     * @brief Initial scan, call once in setup()
     *
     * @param interval_ms Scan interval in milliseconds (default: 2000)
     */
    void begin(uint32_t interval_ms = 2000);

    /**
     * @brief Periodic scan, call in loop()
     */
    void update();

    /**
     * @brief Register callback invoked when a module is connected
     *
     * @param cb Callback function pointer: void cb(M5StamPLC_IO& dev, uint8_t addr)
     */
    void onConnect(ConnectCb cb)
    {
        _on_connect = cb;
    }

    /**
     * @brief Register callback invoked when a module is disconnected
     *
     * @param cb Callback function pointer: void cb(uint8_t addr)
     */
    void onDisconnect(DisconnectCb cb)
    {
        _on_disconnect = cb;
    }

    /**
     * @brief Register callback invoked when a module's DIP-switch address changes
     *
     * @param cb Callback function pointer: void cb(uint8_t old_addr, uint8_t new_addr)
     */
    void onAddrChange(AddrChangeCb cb)
    {
        _on_addr_change = cb;
    }

    /**
     * @brief Get number of currently connected modules
     *
     * @return Number of connected modules
     */
    uint8_t count() const
    {
        return _count;
    }

    /**
     * @brief Get a connected module by sequential index
     *
     * @param index Index in range 0 to count()-1
     * @return Pointer to M5StamPLC_IO, or nullptr if out of range
     */
    M5StamPLC_IO* get(uint8_t index);

    /**
     * @brief Get a connected module by its I2C address
     *
     * @param addr I2C address (0x20-0x2F)
     * @return Pointer to M5StamPLC_IO, or nullptr if not connected
     */
    M5StamPLC_IO* getByAddr(uint8_t addr);

private:
    Slot _slots[MAX_DEVICES];
    uint8_t _count               = 0;
    uint32_t _interval_ms        = 2000;
    unsigned long _last_ms       = 0;
    ConnectCb _on_connect        = nullptr;
    DisconnectCb _on_disconnect  = nullptr;
    AddrChangeCb _on_addr_change = nullptr;

    void _scan();
    void _applyAddrChange(uint8_t old_idx, uint8_t new_dip);
};
