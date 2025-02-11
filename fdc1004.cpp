//////////////////////////////////////////////////////////////////////////////////////////
//
//    Based on Arduino library for the FDC1004 capacitance sensor breakout board
//    Author: Ashwin Whitchurch
//    Copyright (c) 2018 ProtoCentral
//    For more information, visit https://github.com/protocentral/ProtoCentral_fdc1004_breakout
//
//    Based on original code written by Benjamin Shaya
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include <esp_log.h>

#include "fdc1004.h"

static const uint8_t MEAS_CONFIG[] = {0x08, 0x09, 0x0A, 0x0B};
static const uint8_t MEAS_MSB[] = {0x00, 0x02, 0x04, 0x06};
static const uint8_t MEAS_LSB[] = {0x01, 0x03, 0x05, 0x07};
static const uint8_t GAIN_CAL[] = {0x11, 0x12, 0x13, 0x14};
static const uint8_t SAMPLE_DELAY[] = {11,11,6,3};
static const bool ACK_CHECK_EN = true;   /*!< I2C master will check ack from slave*/
static const bool ACK_CHECK_DIS = false; 
static const int I2C_TIMEOUT = 50;

FDC1004::FDC1004(i2c_port_t port, uint16_t rate)
{
    this->_port = port;
    this->_addr = 0b1010000;
    this->_rate = rate;
}

esp_err_t FDC1004::write16(uint8_t reg, uint16_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (this->_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t) (data >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t) data, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(this->_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);
    if( ret != ESP_OK ) {
        ESP_LOGE(this->TAG, "Write16 reg: %d (error %d)", reg, ret);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t FDC1004::read16(uint8_t reg, uint16_t* value)
{
    uint8_t buf[2] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (this->_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(this->_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);
    if( ret != ESP_OK ) {
        ESP_LOGE(this->TAG, "Read16 reg: %d (error %d)", reg, ret);
        return ESP_FAIL;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (this->_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &buf[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[0], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(this->_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);
    if( ret != ESP_OK ) {
        ESP_LOGE(this->TAG, "Read16 reg: %d (error %d)", reg, ret);
        return ESP_FAIL;
    }

    *value = buf[0] | (buf[1] << 8);

    return ESP_OK;
}

//configure a measurement
esp_err_t FDC1004::configureMeasurementSingle(uint8_t measurement, uint8_t channel, uint8_t capdac)
{
    //Verify data
    if (!FDC1004_IS_MEAS(measurement) || !FDC1004_IS_CHANNEL(channel) || capdac > FDC1004_CAPDAC_MAX) {
        ESP_LOGE(this->TAG, "Bad configuration");
        return ESP_ERR_INVALID_ARG;
    }

    //build 16 bit configuration
    uint16_t configuration_data = 0;
    configuration_data = ((uint16_t)channel) << 13; //CHA
    configuration_data |=  ((uint16_t)0x04) << 10; //CHB disable / CAPDAC enable
    configuration_data |= ((uint16_t)capdac) << 5; //CAPDAC value
    esp_err_t res = write16(MEAS_CONFIG[measurement], configuration_data);

    return res;
}

//configure a channel
esp_err_t FDC1004::configureChannelGain(uint8_t channel, uint8_t gain_int, uint16_t gain_dec)
{
    //Verify data
    if (!FDC1004_IS_CHANNEL(channel)) {
        ESP_LOGE(this->TAG, "Bad configuration");
        return ESP_ERR_INVALID_ARG;
    }

    //build 16 bit gain
    uint16_t gain_data = 0;
    gain_data = ((uint16_t)gain_int) << 14; //integer part
    gain_data |=  ((uint16_t)gain_dec) & 0x3FF; //decimal part
    esp_err_t res = write16(GAIN_CAL[channel], gain_data);
    
    return res;
}

esp_err_t FDC1004::triggerSingleMeasurement(uint8_t measurement, uint8_t rate)
{
  //verify data
    if (!FDC1004_IS_MEAS(measurement) || !FDC1004_IS_RATE(rate)) {
        ESP_LOGE(this->TAG, "Bad trigger request");
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t trigger_data = 0;
    trigger_data = ((uint16_t)rate) << 10; // sample rate
    trigger_data |= 0 << 8; //repeat disabled
    trigger_data |= (1 << (7-measurement)); // 0 > bit 7, 1 > bit 6, etc
    return write16(FDC1004_FDC_REGISTER, trigger_data);
}

/**
 * Check if measurement is done, and read the measurement into value if so.
  * value should be at least 4 bytes long (24 bit measurement)
 */
esp_err_t FDC1004::readMeasurement(uint8_t measurement)
{
    if (!FDC1004_IS_MEAS(measurement)) {
        ESP_LOGE(this->TAG, "Bad request");
        return ESP_ERR_INVALID_ARG;
    }

    //check if measurement is complete
    uint16_t fdc_register = 0;
    esp_err_t err = read16(FDC1004_FDC_REGISTER, &fdc_register);
    if (err) {
        return ESP_FAIL;
    }
    if (! (fdc_register & ( 1 << (3-measurement)))) {
        ESP_LOGE(this->TAG, "Measurement not completed");
        return ESP_ERR_INVALID_STATE;
    }

    //read the value
    err = read16(MEAS_MSB[measurement], &_last_raw_values[0]);
    if (err) {
        return ESP_FAIL;
    }
    err = read16(MEAS_LSB[measurement], &_last_raw_values[1]);
    if (err) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * take a measurement, uses the measurement register equal to the channel number
 */
esp_err_t FDC1004::measureChannel(uint8_t channel, uint8_t capdac, float * value)
{
    uint8_t measurement = channel; //4 measurement configs, 4 channels, seems fair
    if (configureMeasurementSingle(measurement, channel, capdac)) return ESP_FAIL;
    if (triggerSingleMeasurement(measurement, this->_rate)) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY[this->_rate]));
    if (readMeasurement(measurement)) return ESP_FAIL;
    *value = calculateCapacitance(capdac);
    return ESP_OK;
}

/**
 *  function to get the capacitance from a channel.
  */
int32_t FDC1004::getCapacitance(uint8_t channel)
{
    fdc1004_measurement_t value;
    uint8_t result = getRawCapacitance(channel, &value);
    if (result) return 0x80000000;

    int32_t capacitance = ((int32_t)FDC1004_ATTOFARADS_UPPER_WORD) * ((int32_t)value.value); //attofarads
    capacitance /= 1000; //femtofarads
    capacitance += ((int32_t)FDC1004_FEMTOFARADS_CAPDAC) * ((int32_t)value.capdac);
    return capacitance;
}

/**
 * function to get the raw capacitance from a channel
 */
esp_err_t FDC1004::getRawCapacitance(uint8_t channel, fdc1004_measurement_t * value) {
    if (!FDC1004_IS_CHANNEL(channel)) return 1;
    value->value = 0x7FFF;
    value->capdac = this->_last_capdac[channel]; //load last capdac as starting point
    float cap = 0.0f;

    //sample until we get a good result
    while(value->value > 0x7E00 || value->value < 0x8100) {
        if (measureChannel(channel, value->capdac, &cap)) {
            ESP_LOGE(this->TAG, "Measurement not obtained");
            return ESP_ERR_INVALID_STATE;
        }
        value->value = (int16_t)_last_raw_values[0];

        //adjust capdac if necessary
        if (value->value > FDC1004_UPPER_BOUND && value->capdac < FDC1004_CAPDAC_MAX) {
            value->capdac++;
        } else if (value->value < FDC1004_LOWER_BOUND && value->capdac > 0) {
            value->capdac--;
        } else {
            //out of range, but capdac is already maxed (or minned). Return.
            this->_last_capdac[channel] = value->capdac;
            return ESP_OK;
        }
    }
    this->_last_capdac[channel] = value->capdac;
    return ESP_OK;

}

/**
 *  function to calculate capacitance from raw values.
  */
float FDC1004::calculateCapacitance(int capdac)
{
    int32_t capacitance_int = ((int32_t)FDC1004_ATTOFARADS_UPPER_WORD) * ((int32_t)_last_raw_values[0]); //attofarads
    float capacitance = capacitance_int;
    capacitance /= 1000; //femtofarads
    capacitance += ((int32_t)FDC1004_FEMTOFARADS_CAPDAC) * ((int32_t)capdac);
    capacitance /= 1000; //picofarads
    return capacitance;
}