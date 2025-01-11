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

#include "telemetry.hpp"

#include "rc.hpp"
#include "led.hpp"
#include "sensor.hpp"
#include "flight_control.hpp"

uint8_t Telem_mode     = 0;
uint8_t Telem_cnt      = 0;

void set_telemetry_header_data(telemetry_header_data_t *t){
	t->head[0] = 99;
	t->head[1] = 99;
	t->Roll_rate_kp = Roll_rate_kp;
	t->Roll_rate_ti = Roll_rate_ti;
	t->Roll_rate_td = Roll_rate_td;
	t->Roll_rate_eta = Roll_rate_eta;

	t->Pitch_rate_kp = Pitch_rate_kp;
	t->Pitch_rate_ti = Pitch_rate_ti;
	t->Pitch_rate_td = Pitch_rate_td;
	t->Pitch_rate_eta = Pitch_rate_eta;
	
	t->Yaw_rate_kp = Yaw_rate_kp;
	t->Yaw_rate_ti = Yaw_rate_ti;
	t->Yaw_rate_td = Yaw_rate_td;
	t->Yaw_rate_eta =Yaw_rate_eta;

	t->Roll_angle_kp = Roll_angle_kp;
	t->Roll_angle_ti = Roll_angle_ti;
	t->Roll_rate_td = Roll_angle_td;
	t->Roll_angle_eta = Roll_angle_eta;

	t->Pitch_angle_kp = Pitch_angle_kp;
	t->Pitch_angle_ti = Pitch_angle_ti; 
	t->Pitch_angle_td = Pitch_angle_td;
	t->Pitch_angle_eta = Pitch_angle_eta;
}

void set_telemetry_data(telemetry_data_t *t){

	t->head[0] = 88;
	t->head[1] = 88;
	t->Elapsed_time = Elapsed_time;
	t->Interval_time = Interval_time;
	t->roll_angle = (Roll_angle - 	Roll_angle_offset) * 180 / PI;
	t->pitch_angle =(Pitch_angle - Pitch_angle_offset) * 180 / PI;
	t->yaw_angle = 	(Yaw_angle - 	   Yaw_angle_offset) * 180 / PI;
	t->roll_rate = 	(Roll_rate) 	* 180 / PI;
	t->pitch_rate = (Pitch_rate) 	* 180 / PI;
	t->yaw_rate = 	(Yaw_rate) 		* 180 / PI;
	t->roll_angle_reference =  Roll_angle_reference 	* 180 / PI;
	t->pitch_angle_reference = Pitch_angle_reference 	* 180 / PI;
	t->roll_rate_reference =   Roll_rate_reference 		* 180 / PI;
	t->pitch_rate_reference =  Pitch_rate_reference 	* 180 / PI;
	t->yaw_rate_reference =  	 Yaw_rate_reference 		* 180 / PI;
	t->thrust_command = Thrust_command / BATTERY_VOLTAGE;
	t->voltage = Voltage;
	t->accel_x_raw = Accel_x_raw;
	t->accel_y_raw = Accel_y_raw;
	t->accel_z_raw = Accel_z_raw;
	t->Alt_velocity = Alt_velocity;
	t->Z_dot_ref = Z_dot_ref;
	t->FrontLeft_motor_duty = FrontLeft_motor_duty;
	t->RearRight_motor_duty = RearRight_motor_duty;
	t->Alt_ref = Alt_ref;
	t->Altitude2 = Altitude2;
	t->Altitude = Altitude;
	t->Az = Az;
	t->Az_bias = Az_bias;
	t->Alt_flag = Throttle_control_mode;	//Alt_flag;
	t->Mode = Mode;
	t->RangeFront = RangeFront;
	t->cnt_mode = Control_mode;
	t->compass_deg = compass_deg;
	t->deltaX = deltaX;
	t->deltaY = deltaY;
	t->Altitude3 = Altitude3;
}

void telemetry_header(void){
	telemetry_header_data_t senddata;

	set_telemetry_header_data(&senddata);
	telemetry_send((uint8_t *)&senddata, sizeof(senddata));
}

void telemetry_sequence(void) {
	telemetry_data_t senddata;

	set_telemetry_data(&senddata);
	if (telemetry_send((uint8_t *)&senddata, sizeof(senddata)) == 1)
		esp_led(0x110000, 1);  // Telemetory Reciver OFF
	else
		esp_led(0x001100, 1);  // Telemetory Reciver ON
}

void telemetry(void) {

	if (Telem_mode == 0) {	// 最初の１回だけ送る
		Telem_mode = 1;
		telemetry_header();
	}else{
		if (Telem_cnt == 0){
			telemetry_sequence();			// 10回に一度送信
		}
		Telem_cnt++;
		if (Telem_cnt > 9) Telem_cnt = 0;
	}
}

