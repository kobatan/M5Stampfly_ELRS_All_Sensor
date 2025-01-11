/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <Arduino.h>
#include "flight_control.hpp"
#include "pid.hpp"
#include <INA3221.h>
#include <MadgwickAHRS.h>
#include "imu.hpp"
#include "BMI270.h"
#include <stdint.h>
#include "alt_kalman.hpp"
#include <driver/spi_master.h>
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <Bitcraze_PMW3901.h>
#include <Adafruit_BMP280.h>
#include "BMM150Compass.hpp"
#include "tof.hpp"

#define SDA_PIN				(3)
#define SCL_PIN				(4)
#define PIN_NUM_MOSI	(14)
#define PIN_NUM_MISO	(43)
#define PIN_NUM_CLK		(44)
#define PIN_CS				(46)
#define PIN_CS2				(12)
#define PIN_INT1		 	(11)
#define INT_BOTTOM    (6)
#define XSHUT_BOTTOM  (7)
#define INT_FRONT     (8)
#define XSHUT_FRONT   (9)
#define GROVE_RX 		 	(1)
#define GROVE_TX 			(2)
#define PIN_LED_ONBORD (39)
#define PIN_LED_ESP    (21)
// Motor PWM Pin
#define PIN_MT_FL 		(5) 
#define PIN_MT_FR 		(42)
#define PIN_MT_RL 		(10)
#define PIN_MT_RR 		(41) 
//#define INA3221_ADDRESS		0x40	// 電流電圧計 (I2C)
//#define VL53L0X_ADDRESS1	0x29	// 距離センサー1 (I2C)
//#define VL53L0X_ADDRESS2	0x2A	// 距離センサー2 (I2C)
//#define BMP280_ADDRESS	0x76	// 気圧計 (I2C) <Adafruit_BMP280.h>で定義済み
#define BMM150_ADDRESS		0x10	// コンパス (I2C)

typedef struct {
    spi_host_device_t host;  ///< The SPI host used, set before calling `spi_eeprom_init()`
    gpio_num_t cs_io;        ///< CS gpio number, set before calling `spi_eeprom_init()`
    gpio_num_t miso_io;      ///< MISO gpio number, set before calling `spi_eeprom_init()`
    bool intr_used;  ///< Whether to use polling or interrupt when waiting for write to be done. Set before calling
                     ///< `spi_eeprom_init()`.
} eeprom_config_t;

typedef struct eeprom_context_t* eeprom_handle_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quat_t;

typedef struct {
    uint16_t distance;
    uint16_t cnt;
} distance_t;


// Sensor data
// extern volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
extern volatile float Roll_angle, Pitch_angle, Yaw_angle;
extern volatile float Roll_rate, Pitch_rate, Yaw_rate;
extern volatile float Accel_x_raw, Accel_y_raw, Accel_z_raw;
extern volatile float Accel_x, Accel_y, Accel_z;
extern volatile float Accel_z_d;
extern volatile int16_t RawRange;
extern volatile int16_t Range;
extern volatile float Altitude;
extern volatile float Altitude2;
extern volatile float Alt_velocity;
extern volatile uint8_t Alt_control_ok;
extern volatile float Voltage;
extern float Acc_norm;
extern quat_t Quat;
extern float Over_g, Over_rate;
extern uint8_t OverG_flag;
extern uint8_t Range0flag;
extern volatile uint8_t Under_voltage_flag;
extern volatile uint8_t ToF_bottom_data_ready_flag;
extern volatile float Az;
extern volatile float Az_bias;
extern Alt_kalman EstimatedAltitude;
extern volatile int16_t RawRangeFront;
extern volatile int16_t RangeFront;
extern volatile float compass_deg;	// コンパス
extern int16_t deltaX, deltaY;  		// オプティカルフロー値
extern volatile float Altitude3;		// 気圧計の高度


void sensor_init(void);
void sensor_init2(void);
float sensor_read(void);
void sensor_read2(void);
void sensor_reset_offset(void);
uint16_t sensor_calc_offset_avarage(void);
void ahrs_reset(void);
uint8_t scan_i2c(void);

#endif