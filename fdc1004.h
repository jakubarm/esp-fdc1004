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

#ifndef _FDC1004
#define _FDC1004

#include <stdint.h>
#include "driver/i2c.h"

//Constants and limits for FDC1004
#define FDC1004_100HZ (0x01)
#define FDC1004_200HZ (0x02)
#define FDC1004_400HZ (0x03)
#define FDC1004_IS_RATE(x) (x == FDC1004_100HZ || \
                            x == FDC1004_200HZ || \
                            x == FDC1004_400HZ)

#define FDC1004_CAPDAC_MAX (0x1F)

#define FDC1004_CHANNEL_MAX (0x03)
#define FDC1004_IS_CHANNEL(x) (x >= 0 && x <= FDC1004_CHANNEL_MAX)

#define FDC1004_MEAS_MAX (0x03)
#define FDC1004_IS_MEAS(x) (x >= 0 && x <= FDC1004_MEAS_MAX)

#define FDC1004_FDC_REGISTER (0x0C)

#define FDC1004_ATTOFARADS_UPPER_WORD (457) //number of attofarads for each 8th most lsb (lsb of the upper 16 bit half-word)
#define FDC1004_FEMTOFARADS_CAPDAC (3028) //number of femtofarads for each lsb of the capdac

#define FDC1004_UPPER_BOUND ((int16_t) 0x4000)
#define FDC1004_LOWER_BOUND (-1 * FDC1004_UPPER_BOUND)

/********************************************************************************************************
 * typedefs
 *******************************************************************************************************/
typedef struct fdc1004_measurement_t
{
    int16_t value;
    uint8_t capdac;
}fdc1004_measurement_t;

/******************************************************************************************
 * Function Declarations
 ******************************************************************************************/
class FDC1004
{
 public:
    FDC1004(i2c_port_t port, uint16_t rate = FDC1004_100HZ);
    int32_t getCapacitance(uint8_t channel = 1);
    esp_err_t getRawCapacitance(uint8_t channel, fdc1004_measurement_t * value);
    esp_err_t configureMeasurementSingle(uint8_t measurement, uint8_t channel, uint8_t capdac);
    esp_err_t configureChannelGain(uint8_t channel, uint8_t gain_int, uint16_t gain_dec);
    esp_err_t triggerSingleMeasurement(uint8_t measurement, uint8_t rate);
    esp_err_t readMeasurement(uint8_t measurement);
    esp_err_t measureChannel(uint8_t channel, uint8_t capdac, float * value);
    float calculateCapacitance(int capdac);

    static const int16_t UPPER_BOUND = 0X4000; // max readout capacitance
    static const int16_t LOWER_BOUND = (-1 * UPPER_BOUND);
    static const uint16_t MEASUREMENT_DELAY = 20;

 private:
    const char *TAG = "FDC1004";
    i2c_port_t _port;    /**< I2C_NUM_0 or I2C_NUM_1 */
    uint8_t _addr;
    uint8_t _rate;
    uint8_t _last_capdac[4];
    uint16_t _last_raw_values[2] = {0};
    esp_err_t write16(uint8_t reg, uint16_t data);
    esp_err_t read16(uint8_t reg, uint16_t* value);

};

#endif
