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

#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP
#include <Arduino.h>
#include <stdint.h>
typedef struct __attribute__((__packed__)) {
  uint8_t head[2] = {99, 99};
	float Roll_rate_kp;
  float Roll_rate_ti;
  float Roll_rate_td;
  float Roll_rate_eta;
  float Pitch_rate_kp;
  float Pitch_rate_ti;
  float Pitch_rate_td;
  float Pitch_rate_eta;
  float Yaw_rate_kp;
  float Yaw_rate_ti;
  float Yaw_rate_td;
  float Yaw_rate_eta;
  float Roll_angle_kp;
  float Roll_angle_ti;
  float Roll_angle_td;
  float Roll_angle_eta;
  float Pitch_angle_kp;
  float Pitch_angle_ti;
  float Pitch_angle_td;
  float Pitch_angle_eta;
}telemetry_header_data_t;

typedef struct __attribute__((__packed__)) {
	uint8_t head[2] = {88, 88};
  float Elapsed_time;					// Time
  float Interval_time;				// delta Time
  float roll_angle;						// Roll_angle
  float pitch_angle;					// Pitch_angle
  float yaw_angle;						// Yaw_angle
  float roll_rate;    				// rool rate
  float pitch_rate;						// pitch rate
  float yaw_rate;							// yaw rate
  float roll_angle_reference;	// Roll_angle_reference
  float pitch_angle_reference;// Pitch_angle_reference
  float roll_rate_reference;	// roll rate ref
  float pitch_rate_reference;	// pitch rate ref
  float yaw_rate_reference;		// yaw rate ref
  float thrust_command;				// T ref
  float voltage;							// Voltage
  float accel_x_raw;					// Accel_x_raw
  float accel_y_raw;					// Accel_y_raw
  float accel_z_raw;					// Accel_z_raw
  float Alt_velocity;					// Alt Velocity
  float Z_dot_ref;						// Z_dot_ref
  float FrontLeft_motor_duty;	// FrontLeft_motor_duty
  float RearRight_motor_duty;	// RearRight_motor_duty
  float Alt_ref;							// Alt_ref
  float Altitude2;						// Altitude2
  float Altitude;							// Sense_Alt
  float Az;										// Az
  float Az_bias;							// Az_bias
  uint8_t Alt_flag;						// Alt_flag 高度維持制御モード　AUTO・MANUAL
  uint8_t Mode;								// fly mode	機体の状態
	int16_t RangeFront;					// tof front　前方の距離
	
	uint8_t cnt_mode;						// コントロールモード
	float compass_deg;					// コンパス
	int16_t deltaX;  						// オプティカルフローX値
	int16_t deltaY;  						// オプティカルフローY値
	float Altitude3;						// 気圧計での高度
} telemetry_data_t;

void telemetry(void);

#endif