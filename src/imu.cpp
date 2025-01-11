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

#include <Arduino.h>
//#include "common.h"
//#include "bmi2.h"
#include "imu.hpp"
#include "BMI270.h"

static BMI270 imu = BMI270( PIN_CS , BMI270::ACCEL_RANGE_8_G, BMI270::ACCEL_ODR_400_HZ, BMI270::GYRO_RANGE_2000_DPS, BMI270::GYRO_ODR_400_HZ);
//struct bmi2_dev Bmi270;
//struct bmi2_dev *pBmi270=&Bmi270;

float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
struct bmi2_sens_data imu_data;


static bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

void imu_init(void) {
    int8_t st;
    uint8_t data = 0;

    USBSerial.printf("Start IMU Initialize!\n\r");
		imu.begin();
		
		attachInterrupt(PIN_INT1, handleInterrupt, RISING);
}

float lsb_to_mps2(int16_t val, float g_range)					// val = +32767 ~ -32768 
{
    float half_scale = (float)((pow(2, 16) / 2.0f));	//  = 32767 (0x7fff)
    return 	(val / half_scale) * g_range;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 * range 2.181661565, 4.36332313, 8.72664626, 17.45329252, 34.90658504
 */
float lsb_to_rps(int16_t val, float rps)						// val = +32767 ~ -32768
{
    float half_scale = (float)((pow(2,16) / 2.0f));	//  = 32767 (0x7fff)
    return 	(val / half_scale) * rps;
}

bool imu_update(void) {
	if(gotInterrupt) {
		gotInterrupt = false;
  	imu.readSensor();
		return true;
	}else
		return false;
}

float imu_get_acc_x(void) {
//return imu.getRawAccelX();
    return lsb_to_mps2(imu.getRawAccelX(), 8.0);	// +8g ~ -8g
}

float imu_get_acc_y(void) {
//	return imu.getRawAccelY();
 return lsb_to_mps2(imu.getRawAccelY(), 8.0);
}

float imu_get_acc_z(void) {
//	return imu.getRawAccelZ();
    return lsb_to_mps2(imu.getRawAccelZ(), 8.0);
}

float imu_get_gyro_x(void) {
//	return imu.getRawGyroX();
  return lsb_to_rps(imu.getRawGyroX(), DPS20002RAD);	// 角速度をラジアンに変換
}

float imu_get_gyro_y(void) {
//	return imu.getRawGyroY();	
 return lsb_to_rps(imu.getRawGyroY(), DPS20002RAD);
}

float imu_get_gyro_z(void) {
//	return imu.getRawGyroZ();	
 return lsb_to_rps(imu.getRawGyroZ(), DPS20002RAD);
}


