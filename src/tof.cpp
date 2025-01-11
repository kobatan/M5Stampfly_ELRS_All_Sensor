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
#include "tof.hpp"

VL53LX tof_bottom = VL53LX( &Wire1, XSHUT_BOTTOM);
VL53LX tof_front  = VL53LX( &Wire1, XSHUT_FRONT);

volatile uint8_t ToF_bottom_data_ready_flag = 0;
volatile uint8_t ToF_front_data_ready_flag = 0;

void IRAM_ATTR tof_bottom_int() {
  ToF_bottom_data_ready_flag = 1;
}

void IRAM_ATTR tof_front_int() {
  ToF_front_data_ready_flag = 1;
}

int16_t tof_bottom_get_range() {
	int16_t range = -1;
	if(ToF_bottom_data_ready_flag == 1) {
	 	ToF_bottom_data_ready_flag = 0;
		range = tof_range_get(&tof_bottom);
	}
	return range;
}

int16_t tof_front_get_range() {
	int16_t range = -1;
	if(ToF_front_data_ready_flag == 1) {
	 	ToF_front_data_ready_flag = 0;
		range = tof_range_get(&tof_front);
	}
	return range;
}

void tof_init(void) {	
	pinMode(XSHUT_BOTTOM, OUTPUT);
	pinMode(XSHUT_FRONT, OUTPUT);	
	tof_bottom.begin();
	tof_front.begin();

	tof_front.VL53LX_Off();
	tof_bottom.InitSensor(0x54);	// ボトム側のI2Cアドレスを変更する(0x2A)

	tof_front.VL53LX_On();
	tof_front.InitSensor(0x52);
//	tof_front.VL53LX_WaitDeviceBooted();
//	tof_front.Init();

	pinMode(INT_BOTTOM, INPUT);
	attachInterrupt(INT_BOTTOM, &tof_bottom_int, FALLING);
	pinMode(INT_FRONT, INPUT);
	attachInterrupt(INT_FRONT, &tof_front_int, FALLING);

	tof_bottom.VL53LX_ClearInterruptAndStartMeasurement();
	delay(11);	
	tof_front.VL53LX_ClearInterruptAndStartMeasurement();
}

int16_t tof_range_get(VL53LX *dev) {
	int16_t range;
	int16_t range_min;
	int16_t range_max;
	int16_t range_ave;
	uint8_t count;
	VL53LX_MultiRangingData_t MultiRangingData;
	uint8_t NewDataReady = 0;
	int8_t no_of_object_found = 0;
	
	int status = dev->VL53LX_GetMeasurementDataReady(&NewDataReady);
	if( status == 0 && NewDataReady != 0){
		range_min = 10000;
		range_max = 0;
		range_ave = 0;
		status = dev->VL53LX_GetMultiRangingData(&MultiRangingData);
		no_of_object_found = MultiRangingData.NumberOfObjectsFound;
		if (no_of_object_found == 0) {
			range_min = 9999;
			range_max = -1;
		} else {
			count = 0;
			for (uint8_t j = 0; j < no_of_object_found; j++) {
				if (MultiRangingData.RangeData[j].RangeStatus == VL53LX_RANGESTATUS_RANGE_VALID) {
					count++;
					range = MultiRangingData.RangeData[j].RangeMilliMeter;
					if (range_min > range) range_min = range;
					if (range_max < range) range_max = range;
					range_ave = range_ave + range;
				}
			}
			if (count != 0) range_ave = range_ave / count;
		}

	}	
	dev->VL53LX_ClearInterruptAndStartMeasurement();
	return range_max;
}