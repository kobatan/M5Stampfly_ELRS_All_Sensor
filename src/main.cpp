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
#include <FastLED.h>
#include "flight_control.hpp"
#include "CrsfSerial.h"
#include "rc.hpp"
#include "sensor.hpp"
#include "buzzer.h"

CrsfSerial crsf(Serial1, CRSF_BAUDRATE);
volatile uint8_t Loop_flag2;
hw_timer_t* timer2 = NULL;

void packetChannels(void)		// ELRS受信チャンネルをスティックに振り分ける
{
    Stick[AILERON]	= 2.0 * (float)(crsf.getChannel(1) - AILERON_MID)/(float)(AILERON_MAX - AILERON_MIN);			// -1 ~ 1
    Stick[ELEVATOR]	= 2.0 * (float)(crsf.getChannel(2) - ELEVATOR_MID)/(float)(ELEVATOR_MAX - ELEVATOR_MIN);		// -1 ~ 1 
    Stick[THROTTLE]	= 1.0 * (float)(crsf.getChannel(3) - THROTTLE_MID)/(float)(THROTTLE_MAX - THROTTLE_MIN);		// 0 ~ 1 に変更
    Stick[RUDDER]		= 2.0 * (float)(crsf.getChannel(4) - RUDDER_MID)/(float)(RUDDER_MAX - RUDDER_MIN);					// -1 ~ 1
    Stick[BUTTON_ARM] = (uint8_t)(crsf.getChannel(9)>1600);			// AtomJoyの左スティック押し込み　ARM・(高度制御ONの時は自動離陸・自動着陸)    (モーメンタリSWに割り当てる)
    Stick[CONTROLMODE] = (uint8_t)(crsf.getChannel(6)>1600);		// AtomJoyのRボタン　Stable/Sports
    Stick[ALTCONTROLMODE] = (uint8_t)(crsf.getChannel(7)>1600);	// AtomJoyのLボタン　高度制御
    Stick[BUTTON_FLIP] = (uint8_t)(crsf.getChannel(8)>1600);		// AtomJoyの右スティック押し込み　フリップする　(モーメンタリSWに割り当てる)
		Stick[LOG] = (uint8_t)(crsf.getChannel(13)>1600);						// 

/*
    USBSerial.printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f  %6.3f\n\r", 
                                                Stick[THROTTLE],
                                                Stick[AILERON],
                                                Stick[ELEVATOR],
                                                Stick[RUDDER],
                                                Stick[BUTTON_ARM],
                                                Stick[BUTTON_FLIP],
                                                Stick[CONTROLMODE],
                                                Stick[ALTCONTROLMODE],
                                                Stick[LOG]);
*/
}

void IRAM_ATTR onTimer2() {
  Loop_flag2 = 1;
}

void loop1()
{
	crsf.loop();					// ELRS 受信

	if(Loop_flag2 == 1){	// 2.5ms周期
		Loop_flag2 = 0;
		sensor_read2();			// I2C関連センサー読み込み
	}
}

void setup1(void *arg)
{
	Serial1.begin(CRSF_BAUDRATE,SERIAL_8N1, GROVE_RX, GROVE_TX);	// Rx = G1, Tx = G2

	crsf.begin(CRSF_BAUDRATE);	//Initialize ELRS
	crsf.onPacketChannels = &packetChannels;
	
	// Initialize intrupt
	timer2 = timerBegin(1, 80, true);
	timerAttachInterrupt(timer2, &onTimer2, true);
	timerAlarmWrite(timer2, 2500, true);							// 2.5ms 間隔（400Hz）loop1内で使う
	timerAlarmEnable(timer2);

	sensor_init2();		// I2C関連の初期化

	USBSerial.printf("Finish StampFly init!\r\n");
	USBSerial.printf("Enjoy Flight!\r\n");
	start_tone();		// 起動音を鳴らす
	
	while(1){
		loop1();			// ELRS受信 と　I2C接続センサーの読み込み
	}
}

void setup() {
	init_copter();
	
	xTaskCreateUniversal(setup1, "core0", 8192, NULL, 1, NULL, 0 );			// Core0でマルチコア起動
	delay(1000);	
}

void loop() {			// Core1 ループ
	loop_400Hz();		// SPI接続センサー　(IMU と　オプティカルフロー)
}
