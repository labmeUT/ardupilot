#include "MCP9600.h"

#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

static const uint8_t MCP9600_CMD_READ_ADC    = 0x00;

MCP9600::MCP9600() :
    _dev(nullptr),
    _temperature(0)
{
}

bool MCP9600::init()
{
    _dev = std::move(hal.i2c_mgr->get_device(1, MCP9600_ADDR));
    if (!_dev) {
        printf("MCP9600 device is null!");
        return false;
    }

    if (!_dev->get_semaphore()->take(0)) {
        AP_HAL::panic("PANIC: MCP9600: failed to take serial semaphore for init");
    }

    _dev->set_retries(10);
/*
    if (!_reset()) {
        printf("MCP9600 reset failed");
        _dev->get_semaphore()->give();
        return false;
    }
*/
    hal.scheduler->delay(4);
/*
    if (!_read_prom()) {
        printf("MCP9600 prom read failed");
        _dev->get_semaphore()->give();
        return false;
    }
*/
/*
    _convert();
*/
    // lower retries for run

    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    // Request 20Hz update
    // Max conversion time is 9.04 ms
/*
    _dev->register_periodic_callback( (50 * USEC_PER_MSEC),
                                     FUNCTOR_BIND_MEMBER(&MCP9600::_temperature, void));
*/
    return true;
}

/*bool TSYS01::_reset()
{
    return _dev->transfer(&TSYS01_CMD_RESET, 1, nullptr, 0);
}

// Register map
// prom word	Address
//      0       0xA0 -> unused
//      1       0xA2 -> _k[4]
//      2       0xA4 -> _k[3]
//      3       0xA6 -> _k[2]
//      4       0xA8 -> _k[1]
//      5       0xAA -> _k[0]
//      6       0xAC -> unused
//      7       0xAE -> unused
bool TSYS01::_read_prom()
{
    bool success = false;
    for (int i = 0; i < 5; i++) {
        // Read only the prom values that we use
        _k[i] = _read_prom_word(5-i);
        success |= _k[i] != 0;
    }
    return success;
}*/

/*// Borrowed from MS Baro driver
uint16_t TSYS01::_read_prom_word(uint8_t word)
{
    const uint8_t reg = TSYS01_CMD_READ_PROM + (word << 1);
    uint8_t val[2];
    if (!_dev->transfer(&reg, 1, val, 2)) {
        return 0;
    }
    return (val[0] << 8) | val[1];
}*/

/*bool TSYS01::_convert()
{
    return _dev->transfer(&TSYS01_CMD_CONVERT, 1, nullptr, 0);
}*/

float MCP9600::temperature(void)
{
    uint8_t val[2];
    uint32_t adc;
    if (!_dev->transfer(&MCP9600_CMD_READ_ADC, 1, val, 2)) {
        return 0;
    }
    adc = val[0] << 8 |  val[1];
    _temperature = adc * 0.0625;
    return (_temperature);
}

/*uint32_t MCP9600::_read_adc()
{
    uint8_t val[2];
    uint32_t adc
    if (!_dev->transfer(&MCP9600_CMD_READ_ADC, 1, val, 2)) {
        return 0;
    }
    _temperature = adc * 0.0625;
    return (val[0] << 8) |  val[1];
}*/
/*
void MCP9600::_timer(void)
{
    uint32_t adc = _read_adc();
    _healthy = adc != 0;

    if (_healthy) {
        _calculate(adc);
    } else {
        _temperature = 0;
    }

    //printf("\nTemperature: %.2lf C", _temperature);

    _convert();
}
*/

/*void MCP9600::_calculate(uint32_t adc)
{
    //float adc16 = adc/256;
    _temperature = adc * 0.0625;
       // -2   * _k[4] * powf(10, -21) * powf(adc16, 4) +
       // 4    * _k[3] * powf(10, -16) * powf(adc16, 3) +
       // -2   * _k[2] * powf(10, -11) * powf(adc16, 2) +
       // 1    * _k[1] * powf(10, -6)  * adc16 +
       // -1.5 * _k[0] * powf(10, -2);
}*/
