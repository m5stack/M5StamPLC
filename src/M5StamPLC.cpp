/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "M5StamPLC.h"
#include "pin_config.h"
#include "utils/modbus_params/modbus_params.h"
#include <cstring>
#include <mbcontroller.h>
#include <SD.h>

using namespace m5;

M5_STAMPLC M5StamPLC;

static const char* TAG = "M5StamPLC";

void M5_STAMPLC::begin()
{
    M5.begin();

    i2c_init();
    io_expander_a_init();
    io_expander_b_init();
    lm75b_init();
    ina226_init();
    rx8130_init();

    if (_config.enableModbusSlave) {
        modbus_slave_init();
    }
    if (_config.enableCan) {
        can_init();
    }
    if (_config.enableSdCard) {
        sd_card_init();
    }
}

void M5_STAMPLC::update()
{
    M5.update();
}

/* -------------------------------------------------------------------------- */
/*                                     I2C                                    */
/* -------------------------------------------------------------------------- */
void M5_STAMPLC::i2c_init()
{
    m5::In_I2C.release();
    m5::In_I2C.begin(I2C_NUM_0, STAMPLC_PIN_I2C_INTER_SDA, STAMPLC_PIN_I2C_INTER_SCL);
}

/* -------------------------------------------------------------------------- */
/*                                  IO EXT A (0)                              */
/* -------------------------------------------------------------------------- */
void M5_STAMPLC::io_expander_a_init()
{
    auto& ioe = M5.getIOExpander(0);

    // Status light init
    ioe.setDirection(4, true);
    ioe.setPullMode(4, false);
    ioe.setHighImpedance(4, true);

    ioe.setDirection(5, true);
    ioe.setPullMode(5, false);
    ioe.setHighImpedance(5, true);

    ioe.setDirection(6, true);
    ioe.setPullMode(6, false);
    ioe.setHighImpedance(6, true);
}

void M5_STAMPLC::setBacklight(bool on)
{
    auto& ioe = M5.getIOExpander(0);

    ioe.setHighImpedance(7, !on);
    ioe.digitalWrite(7, !on);  // backlight is active low
}

void M5_STAMPLC::setStatusLight(const uint8_t& r, const uint8_t& g, const uint8_t& b)
{
    auto& ioe = M5.getIOExpander(0);

    if (r == 0) {
        ioe.setHighImpedance(6, true);
    } else {
        ioe.setHighImpedance(6, false);
        ioe.digitalWrite(6, false);
    }

    if (g == 0) {
        ioe.setHighImpedance(5, true);
    } else {
        ioe.setHighImpedance(5, false);
        ioe.digitalWrite(5, false);
    }

    if (b == 0) {
        ioe.setHighImpedance(4, true);
    } else {
        ioe.setHighImpedance(4, false);
        ioe.digitalWrite(4, false);
    }
}

/* -------------------------------------------------------------------------- */
/*                                  IO EXT B                                  */
/* -------------------------------------------------------------------------- */
static std::vector<int> _in_pin_list  = {4, 5, 6, 7, 12, 13, 14, 15};
static std::vector<int> _out_pin_list = {0, 1, 2, 3};

void M5_STAMPLC::io_expander_b_init()
{
    _io_expander_b = new AW9523_Class;
    if (!_io_expander_b->begin()) {
        ESP_LOGE(TAG, "io expander b init failed");
    } else {
        _io_expander_b->configureDirection(0x0);  // all inputs!
        _io_expander_b->openDrainPort0(false);    // push pull default
        _io_expander_b->interruptEnableGPIO(0);   // no interrupt

        // Outputs init
        for (const auto& i : _out_pin_list) {
            _io_expander_b->pinMode(i, AW9523_Class::AW_OUTPUT);
            _io_expander_b->digitalWrite(i, false);
        }

        // Inputs init
        for (const auto& i : _in_pin_list) {
            _io_expander_b->pinMode(i, AW9523_Class::AW_INPUT);
        }

        _io_expander_b->disableIrq();
    }
}

bool M5_STAMPLC::readPlcInput(const uint8_t& channel)
{
    if (_io_expander_b == nullptr) {
        return false;
    }
    if (channel >= _in_pin_list.size()) {
        return false;
    }
    return _io_expander_b->digitalRead(_in_pin_list[channel]);
}

bool M5_STAMPLC::readPlcRelay(const uint8_t& channel)
{
    if (_io_expander_b == nullptr) {
        return false;
    }
    if (channel >= _out_pin_list.size()) {
        return false;
    }
    return _io_expander_b->digitalRead(_out_pin_list[channel]);
}

void M5_STAMPLC::writePlcRelay(const uint8_t& channel, const bool& state)
{
    if (_io_expander_b == nullptr) {
        return;
    }
    if (channel >= _out_pin_list.size()) {
        return;
    }
    _io_expander_b->digitalWrite(_out_pin_list[channel], state);
}

void M5_STAMPLC::writePlcAllRelay(const uint8_t& relayState)
{
    if (_io_expander_b == nullptr) {
        return;
    }
    for (int i = 0; i < _out_pin_list.size(); i++) {
        _io_expander_b->digitalWrite(_out_pin_list[i], (relayState & (1 << i)));
    }
}

/* -------------------------------------------------------------------------- */
/*                                    LM75B                                   */
/* -------------------------------------------------------------------------- */
void M5_STAMPLC::lm75b_init()
{
    if (!LM75B.begin()) {
        ESP_LOGE(TAG, "lm75b init failed");
    }
}

float M5_STAMPLC::getTemp()
{
    return LM75B.temp();
}

/* -------------------------------------------------------------------------- */
/*                                   INA226                                   */
/* -------------------------------------------------------------------------- */
void M5_STAMPLC::ina226_init()
{
    if (!INA226.begin()) {
        ESP_LOGE(TAG, "ina226 init failed");
    } else {
        INA226_Class::config_t cfg;
        cfg.sampling_rate         = INA226_Class::Sampling::Rate16;
        cfg.bus_conversion_time   = INA226_Class::ConversionTime::US_1100;
        cfg.shunt_conversion_time = INA226_Class::ConversionTime::US_1100;
        cfg.mode                  = INA226_Class::Mode::ShuntAndBus;
        cfg.shunt_res             = 0.01f;
        cfg.max_expected_current  = 2.0f;
        INA226.config(cfg);
    }
}

float M5_STAMPLC::getPowerVoltage()
{
    return INA226.getBusVoltage();
}

float M5_STAMPLC::getIoSocketOutputCurrent()
{
    return INA226.getShuntCurrent();
}

/* -------------------------------------------------------------------------- */
/*                                   RX8130                                   */
/* -------------------------------------------------------------------------- */
void M5_STAMPLC::rx8130_init()
{
    if (!RX8130.begin()) {
        ESP_LOGE(TAG, "rx8130 init failed!");
    } else {
        RX8130.initBat();
        RX8130.disableIrq();
        RX8130.clearIrqFlags();
    }
}

/* -------------------------------------------------------------------------- */
/*                                   SD Card                                  */
/* -------------------------------------------------------------------------- */
void M5_STAMPLC::sd_card_init()
{
    if (!SD.begin(STAMPLC_PIN_SD_CS, SPI, 4000000)) {
        ESP_LOGE(TAG, "sd card init failed");
    }
}

void M5_STAMPLC::setRtcTime(struct tm* time)
{
    RX8130.setTime(time);
}

void M5_STAMPLC::getRtcTime(struct tm* time)
{
    RX8130.getTime(time);
}

/* -------------------------------------------------------------------------- */
/*                                   Buzzer                                   */
/* -------------------------------------------------------------------------- */
void M5_STAMPLC::tone(unsigned int frequency, unsigned long duration)
{
    ::tone(STAMPLC_PIN_BUZZ, frequency, duration);
}

void M5_STAMPLC::noTone()
{
    ::noTone(STAMPLC_PIN_BUZZ);
}

/* -------------------------------------------------------------------------- */
/*                                Modbus Slave                                */
/* -------------------------------------------------------------------------- */
// https://github.com/espressif/esp-idf/blob/v4.4.8/examples/protocols/modbus/serial/mb_slave

// Defines below are used to define register start address for each type of Modbus registers
#define HOLD_OFFSET(field)          ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
#define INPUT_OFFSET(field)         ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
#define MB_REG_DISCRETE_INPUT_START (0x0000)
#define MB_REG_COILS_START          (0x0000)
#define MB_REG_INPUT_START_AREA0    (INPUT_OFFSET(input_data0))  // register offset input area 0
#define MB_REG_INPUT_START_AREA1    (INPUT_OFFSET(input_data4))  // register offset input area 1
#define MB_REG_HOLDING_START_AREA0  (HOLD_OFFSET(holding_data0))
#define MB_REG_HOLDING_START_AREA1  (HOLD_OFFSET(holding_data4))

#define MB_PAR_INFO_GET_TOUT (10)  // Timeout for get parameter info
#define MB_CHAN_DATA_MAX_VAL (6)
#define MB_CHAN_DATA_OFFSET  (0.2f)
#define MB_READ_MASK         (MB_EVENT_INPUT_REG_RD | MB_EVENT_HOLDING_REG_RD | MB_EVENT_DISCRETE_RD | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK        (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK   (MB_READ_MASK | MB_WRITE_MASK)

// static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;

static void float32_2_uint16(float value, uint16_t& high, uint16_t& low)
{
    // Convert float to uint32_t
    uint32_t temp;
    std::memcpy(&temp, &value, sizeof(float));

    // Extract high 16 bits and low 16 bits
    high = (temp >> 16) & 0xFFFF;
    low  = temp & 0xFFFF;
}

static void setup_reg_data(void)
{
    // Define initial state of parameters
    coil_reg_params.coils_port0 = 0x00;
    coil_reg_params.coils_port1 = 0x00;

    input_reg_params.input_data0 = 0;
    input_reg_params.input_data1 = 0;
    input_reg_params.input_data2 = 0;
    input_reg_params.input_data3 = 0;
    input_reg_params.input_data4 = 0;
    input_reg_params.input_data5 = 0;
    input_reg_params.input_data6 = 0;
    input_reg_params.input_data7 = 0;

    float32_2_uint16(0.0f, input_reg_params.input_temp_0, input_reg_params.input_temp_1);
    float32_2_uint16(0.0f, input_reg_params.input_voltage_0, input_reg_params.input_voltage_1);
    float32_2_uint16(0.0f, input_reg_params.input_current_0, input_reg_params.input_current_1);
}

void modbus_handle_update_plc_inputs()
{
    input_reg_params.input_data0 = M5StamPLC.readPlcInput(0);
    input_reg_params.input_data1 = M5StamPLC.readPlcInput(1);
    input_reg_params.input_data2 = M5StamPLC.readPlcInput(2);
    input_reg_params.input_data3 = M5StamPLC.readPlcInput(3);
    input_reg_params.input_data4 = M5StamPLC.readPlcInput(4);
    input_reg_params.input_data5 = M5StamPLC.readPlcInput(5);
    input_reg_params.input_data6 = M5StamPLC.readPlcInput(6);
    input_reg_params.input_data7 = M5StamPLC.readPlcInput(7);
}

void modbus_handle_update_temp()
{
    float32_2_uint16(M5StamPLC.LM75B.temp(), input_reg_params.input_temp_0, input_reg_params.input_temp_1);
}

void modbus_handle_update_voltage()
{
    float32_2_uint16(M5StamPLC.INA226.getBusVoltage(), input_reg_params.input_voltage_0,
                     input_reg_params.input_voltage_1);
}

void modbus_handle_update_current()
{
    float32_2_uint16(M5StamPLC.INA226.getShuntCurrent(), input_reg_params.input_current_0,
                     input_reg_params.input_current_1);
}

void modbus_handle_update_coils()
{
    M5StamPLC.writePlcAllRelay(coil_reg_params.coils_port0);
}

static void modbus_daemon(void* param)
{
    delay(2000);

    mb_param_info_t reg_info;

    while (1) {
        // Check for read/write events of Modbus master for certain events
        (void)mbc_slave_check_event((mb_event_group_t)MB_READ_WRITE_MASK);
        ESP_ERROR_CHECK_WITHOUT_ABORT(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));

        // Filter events and process them accordingly
        if (reg_info.type & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_HOLDING_REG_RD)) {
            // Pass
        } else if (reg_info.type & MB_EVENT_INPUT_REG_RD) {
            // portENTER_CRITICAL(&param_lock);
            if (reg_info.address == (uint8_t*)&input_reg_params.input_temp_0) {
                modbus_handle_update_temp();
            } else if (reg_info.address == (uint8_t*)&input_reg_params.input_voltage_0) {
                modbus_handle_update_voltage();
            } else if (reg_info.address == (uint8_t*)&input_reg_params.input_current_0) {
                modbus_handle_update_current();
            } else {
                modbus_handle_update_plc_inputs();
            }
            // portEXIT_CRITICAL(&param_lock);
        } else if (reg_info.type & MB_EVENT_DISCRETE_RD) {
            // Pass
        } else if (reg_info.type & (MB_EVENT_COILS_RD | MB_EVENT_COILS_WR)) {
            // portENTER_CRITICAL(&param_lock);
            modbus_handle_update_coils();
            // portEXIT_CRITICAL(&param_lock);
        }
    }
}

void M5_STAMPLC::modbus_slave_init()
{
    // Using UART_NUM_1 for RS485
    Serial1.end();

    mb_communication_info_t comm_info;       // Modbus communication parameters
    mb_register_area_descriptor_t reg_area;  // Modbus register area descriptor structure

    void* mbc_slave_handler = NULL;
    ESP_ERROR_CHECK(mbc_slave_init(MB_PORT_SERIAL_SLAVE, &mbc_slave_handler));  // Initialization of Modbus controller

    // Setup communication parameters and start stack
    comm_info.mode       = MB_MODE_RTU;
    comm_info.slave_addr = _config.modbusSlaveId;
    comm_info.port       = UART_NUM_1;
    comm_info.baudrate   = _config.modbusBaudRate;
    comm_info.parity     = MB_PARITY_NONE;
    ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));

    // Inputs
    reg_area.type         = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START_AREA0;
    reg_area.address      = (void*)&input_reg_params.input_data0;
    reg_area.size         = 16;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    // Temp
    reg_area.type         = MB_PARAM_INPUT;
    reg_area.start_offset = INPUT_OFFSET(input_temp_0);
    reg_area.address      = (void*)&input_reg_params.input_temp_0;
    reg_area.size         = 4;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    // Voltage
    reg_area.type         = MB_PARAM_INPUT;
    reg_area.start_offset = INPUT_OFFSET(input_voltage_0);
    reg_area.address      = (void*)&input_reg_params.input_voltage_0;
    reg_area.size         = 4;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    // Current
    reg_area.type         = MB_PARAM_INPUT;
    reg_area.start_offset = INPUT_OFFSET(input_current_0);
    reg_area.address      = (void*)&input_reg_params.input_current_0;
    reg_area.size         = 4;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    // Initialization of Coils register area
    reg_area.type         = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_COILS_START;
    reg_area.address      = (void*)&coil_reg_params;
    reg_area.size         = sizeof(coil_reg_params);
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    setup_reg_data();  // Set values into known state

    // Starts of modbus controller and stack
    ESP_ERROR_CHECK(mbc_slave_start());

    // Set UART pin numbers
    ESP_ERROR_CHECK(
        uart_set_pin(UART_NUM_1, STAMPLC_PIN_485_TX, STAMPLC_PIN_485_RX, STAMPLC_PIN_485_DIR, UART_PIN_NO_CHANGE));

    // Set UART driver mode to Half Duplex
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX));

    xTaskCreate(modbus_daemon, "mb", 4000, NULL, 15, NULL);
}

/* -------------------------------------------------------------------------- */
/*                                     CAN                                    */
/* -------------------------------------------------------------------------- */
// https://github.com/m5stack/M5Unit-Roller/blob/main/src/unit_rollercan.cpp

void M5_STAMPLC::can_init()
{
    static twai_timing_config_t t_config;
    switch (_config.canBaudRate) {
        case 25000:
            t_config = TWAI_TIMING_CONFIG_25KBITS();
            break;
        case 50000:
            t_config = TWAI_TIMING_CONFIG_50KBITS();
            break;
        case 100000:
            t_config = TWAI_TIMING_CONFIG_100KBITS();
            break;
        case 125000:
            t_config = TWAI_TIMING_CONFIG_125KBITS();
            break;
        case 250000:
            t_config = TWAI_TIMING_CONFIG_250KBITS();
            break;
        case 500000:
            t_config = TWAI_TIMING_CONFIG_500KBITS();
            break;
        case 800000:
            t_config = TWAI_TIMING_CONFIG_800KBITS();
            break;
        case 1000000:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
        default:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
    }

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)STAMPLC_PIN_CAN_TX, (gpio_num_t)STAMPLC_PIN_CAN_RX, TWAI_MODE_NORMAL);

    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}
