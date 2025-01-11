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

#include "sensor.hpp"
#include "flight_control.hpp"

Madgwick Drone_ahrs;					// マッジウィック　フィルターによる推定姿勢
Alt_kalman EstimatedAltitude;	// カルマンフィルターでの推定高度

Adafruit_BMP280 bmp(&Wire1);					// 気圧計
INA3221 ina3221(INA3221_ADDR40_GND);  // 電流電圧計　Set I2C address to 0x40 (A0 pin -> GND)
BMM150Compass compass;
Bitcraze_PMW3901 flow(PIN_CS2) ;		// オプティカルフロー　センサー
char flow_frame[35*35];								//array to hold the framebuffer

Filter acc_filter;
Filter az_filter;
Filter voltage_filter;
Filter raw_ax_filter;
Filter raw_ay_filter;
Filter raw_az_filter;
Filter raw_az_d_filter;
Filter raw_gx_filter;
Filter raw_gy_filter;
Filter raw_gz_filter;
Filter alt_filter;

// Sensor data
volatile float Roll_angle = 0.0f, Pitch_angle = 0.0f, Yaw_angle = 0.0f;
volatile float Roll_rate, Pitch_rate, Yaw_rate;
volatile float Roll_rate_offset = 0.0f, Pitch_rate_offset = 0.0f, Yaw_rate_offset = 0.0f;
volatile float Accel_z_d;
volatile float Accel_z_offset = 0.0f;
volatile float Accel_x_raw, Accel_y_raw, Accel_z_raw;
volatile float Accel_x, Accel_y, Accel_z;
volatile float Roll_rate_raw, Pitch_rate_raw, Yaw_rate_raw;
volatile float Mx, My, Mz, Mx0, My0, Mz0, Mx_ave, My_ave, Mz_ave;
volatile int16_t RawRange      = 0;
volatile int16_t Range         = 0;
volatile int16_t RawRangeFront = 0;
volatile int16_t RangeFront    = 0;
volatile float Altitude        = 0.0f;
volatile float Altitude2       = 0.0f;
volatile float Alt_velocity    = 0.0f;
volatile float Az              = 0.0;
volatile float Az_bias         = 0.0;
volatile float compass_deg;			// コンパス角度
int16_t deltaX, deltaY;					// オプティカルフロー値
volatile float Altitude3;				// 気圧計の高度
static uint8_t preMode         = 0;

volatile uint16_t Offset_counter = 0;
volatile float Voltage;
float Acc_norm = 0.0f;
// quat_t Quat;
float Over_g = 0.0f, Over_rate = 0.0f;
uint8_t OverG_flag                  = 0;
uint8_t Range0flag                  = 0;
volatile uint8_t Under_voltage_flag = 0;

uint8_t scan_i2c(TwoWire *i2c) {
    USBSerial.println("I2C scanner. Scanning ...");
    delay(50);
    byte count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        i2c->beginTransmission(i);        // Begin I2C transmission Address (i)
        if (i2c->endTransmission() == 0)  // Receive 0 = success (ACK response)
        {
            USBSerial.printf("Found address: 0x%2X\n",i);
            count++;
        }
    }
    USBSerial.printf("Found %d device(s).",count);
    return count;
}

void sensor_reset_offset(void) {
	Roll_rate_offset  = 0.0f;
	Pitch_rate_offset = 0.0f;
	Yaw_rate_offset   = 0.0f;
	Accel_z_offset    = 0.0f;
	Offset_counter = 0;
}

uint16_t sensor_calc_offset_avarage(void) {
    Roll_rate_offset  = (Offset_counter * Roll_rate_offset + Roll_rate_raw) / (Offset_counter + 1);
    Pitch_rate_offset = (Offset_counter * Pitch_rate_offset + Pitch_rate_raw) / (Offset_counter + 1);
    Yaw_rate_offset   = (Offset_counter * Yaw_rate_offset + Yaw_rate_raw) / (Offset_counter + 1);
    Accel_z_offset    = (Offset_counter * Accel_z_offset + Accel_z_raw) / (Offset_counter + 1);

    Offset_counter++;
		return Offset_counter;
}

void voltage_init(){
	ina3221.begin(&Wire1);		// 電流電圧計
	ina3221.reset();
	voltage_filter.set_parameter(0.005, 0.0025);
}

void volt_read(){
	float filterd_v;	
	// Battery voltage check
	Voltage   = ina3221.getVoltage(INA3221_CH2);
	filterd_v = voltage_filter.update(Voltage, Interval_time2);	// Control_period);

	if (Under_voltage_flag != UNDER_VOLTAGE_COUNT) {
		if (filterd_v < POWER_LIMIT)
			Under_voltage_flag++;
		else
			Under_voltage_flag = 0;
		if (Under_voltage_flag > UNDER_VOLTAGE_COUNT)
			Under_voltage_flag = UNDER_VOLTAGE_COUNT;
	}
}

void test_voltage(void) {
	for (uint16_t i = 0; i < 1000; i++) {
		USBSerial.printf("Voltage[%03d]:%f\n\r", i, ina3221.getVoltage(INA3221_CH2));
	}
}

void ahrs_reset(void) {
//	Drone_ahrs.reset();
}

void imu_read( float interval_time){
	float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
	float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;

//    float h;
//    static float opt_interval = 0.0;
// 以下では航空工学の座標軸の取り方に従って
    // X軸：前後（前が正）左肩上がりが回転の正
    // Y軸：右左（右が正）頭上げが回転の正
    // Z軸：下上（下が正）右回りが回転の正
    // となる様に軸の変換を施しています
    // BMI270の座標軸の撮り方は
    // X軸：右左（右が正）頭上げが回転の正
    // Y軸：前後（前が正）左肩上がりが回転の正
    // Z軸：上下（上が正）左回りが回転の正

//    opt_interval    = opt_interval + sens_interval;
//		USBSerial.printf("Interval:%fms\n", sens_interval * 1000);
	// Get IMU raw data
	if( !imu_update() ) return;  // IMUの値を読む前に必ず実行
	acc_x  = imu_get_acc_x();
	acc_y  = imu_get_acc_y();
	acc_z  = imu_get_acc_z();
	gyro_x = imu_get_gyro_x();	// rad/s
	gyro_y = imu_get_gyro_y();
	gyro_z = imu_get_gyro_z();

//	USBSerial.printf("ax:%8.4f ay:%8.4f az:%8.4f gx:%8.4f gy:%8.4f gz%8.4f\n",acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z);
 
	// Axis Transform
	Accel_x_raw    = acc_y;
	Accel_y_raw    = acc_x;
	Accel_z_raw    = -acc_z;
	Roll_rate_raw  = gyro_y;
	Pitch_rate_raw = gyro_x;
	Yaw_rate_raw   = -gyro_z;

	if ((Mode == PARKING_MODE) && (Mode != preMode))  // モードが遷移した時Static変数を初期化する。外れ値除去のバグ対策
	{
		raw_ax_filter.reset();
  	raw_ay_filter.reset();
		raw_az_filter.reset();
		raw_az_d_filter.reset();

		raw_gx_filter.reset();
		raw_gy_filter.reset();
		raw_gz_filter.reset();

		az_filter.reset();
		alt_filter.reset();

		acc_filter.reset();
	}

	if (Mode > AVERAGE_MODE) {	// Not INIT_MODE & AVERAGE_MODE
		Accel_x   = raw_ax_filter.update(Accel_x_raw, Interval_time);
		Accel_y   = raw_ay_filter.update(Accel_y_raw, Interval_time);
		Accel_z   = raw_az_filter.update(Accel_z_raw, Interval_time);
		Accel_z_d = raw_az_d_filter.update(Accel_z_raw - Accel_z_offset, Interval_time);

		Roll_rate  = raw_gx_filter.update(Roll_rate_raw - Roll_rate_offset, Interval_time);
		Pitch_rate = raw_gy_filter.update(Pitch_rate_raw - Pitch_rate_offset, Interval_time);
		Yaw_rate   = raw_gz_filter.update(Yaw_rate_raw - Yaw_rate_offset, Interval_time);

//		USBSerial.printf("AccX:%5.2f AccY:%5.2f AccZ:%5.2f\n", Accel_x, Accel_y, Accel_z);	
//		USBSerial.printf("Rrate:%5.2f Prate:%5.2f Yrate:%5.2f\n", Roll_rate, Pitch_rate, Yaw_rate);

		Drone_ahrs.updateIMU((Pitch_rate) * (float)RAD_TO_DEG, (Roll_rate) * (float)RAD_TO_DEG, -(Yaw_rate) * (float)RAD_TO_DEG, Accel_y, Accel_x, -Accel_z);
		Roll_angle  =  Drone_ahrs.getPitch()* (float)DEG_TO_RAD;
		Pitch_angle =  Drone_ahrs.getRoll() * (float)DEG_TO_RAD;
		Yaw_angle   = -Drone_ahrs.getYaw()  * (float)DEG_TO_RAD;

//		USBSerial.printf("R:%5.2f P:%5.2f Y:%5.2f\n", Roll_angle, Pitch_angle, Yaw_angle);

		// Get Altitude (30Hz)
		Az = az_filter.update(-Accel_z_d, interval_time);

		// Accel fail safe
		acc_norm = sqrt(Accel_x * Accel_x + Accel_y * Accel_y + Accel_z_d * Accel_z_d);
		Acc_norm = acc_filter.update(acc_norm, Control_period);
		if (Acc_norm > 2.0) {
			OverG_flag = 1;
			if (Over_g == 0.0) Over_g = acc_norm;
    }
	}
}


float zero_pressure = 1013.25;	// 海抜０ｍの気圧 

void bmp_begin(){		// 気圧計
	float pres = 0;
	if (!bmp.begin(BMP280_ADDRESS_ALT)) {
		USBSerial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    while(1){delay(1);};	// 初期化に失敗した場合は無限ループ
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */
	
	for(int i=0;i<10;i++){ 
		pres += bmp.readPressure()/100;	// 現在の気圧を０ｍにする
	}
	zero_pressure = pres/10;
}

void bmp_read(uint16_t n){		// 気圧センサー読み込み
	static uint16_t cnt = 0;
	uint16_t interval = 400/n +1  ;	// 1秒で30回測定 

	if(cnt > interval){
		cnt = 0;
		Altitude3 = bmp.readAltitude( zero_pressure);	// 高度を得る mm。　1m上昇ごとに 0.1hPa下がる
	}
	else cnt++;
}

void compass_init(){	// コンパス初期化
  if (compass.initialize( BMM150_ADDRESS) != BMM150_OK)
  {
    USBSerial.println("BMM150 initialization failed!");
    while(1){delay(1);}; 	// 初期化に失敗した場合は無限ループ
  }
  compass.offset_load();  	// オフセットの読み込み
  compass.calibrate(3000);	// 3秒間 キャリブレーションの実行
  USBSerial.println("BMM150 Calibration complete.");
}

void compass_read(uint16_t n){	// コンパス読み込み
	static uint16_t cnt = 0;
	uint16_t interval = 400/n +1;	// 1秒でn回測定
	double deg = 0;
	if(cnt > interval){
		cnt = 0;
		deg = (double)compass.getHeadingDegrees360();
 		deg += 180;  // Grove Degital Compass v2はv1と比べて180度ずれてるので、180度回転させる
  	if (deg < 0)
  	{
    	deg += 360; // 負の値を修正
  	}
  	deg = fmod(deg, 360); // 360°を超える値を修正
		compass_deg = (float)deg;
	}else cnt++;
}

void flow_init(){
	if (!flow.begin()) {			//　PMW3901 オプティカルフローセンサー初期化
		USBSerial.println("Initialization of the flow sensor failed");
	 	while(1) {delay(1);}
	}
//	flow.enableFrameBuffer(); 
// 	USBSerial.println("framebuffer init");
}

char asciiart(int k){ //converter magic? Higher value shunts more right in char array 
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k>>4]; //return shunted from array value character
}

void flow_read(){
	bool gotMotion;
	flow.readMotionCount(&deltaX, &deltaY);		// オプティカルフロー値
//	USBSerial.printf("flow x:%d y:%d\n",deltaX, deltaY);
/*	
	flow.readFrameBuffer(flow_frame);
  int i,j,k;
  for(i=0, k=0; i<35; i++){ //i is Y pixel pos
    for(j=0; j<35; j++, k++){  //j is X pixel pos
      USBSerial.print(asciiart(flow_frame[k]));
      USBSerial.print(' ');
      }
     USBSerial.println();
  }
  USBSerial.println();
*/
}

void tof_bottom_read(uint16_t n){
	static uint8_t first_flag      = 0;
	static int16_t old_range[4]    = {0};
	int16_t deff;
	static uint8_t outlier_counter = 0;
	const uint16_t interval         = 400 / n + 1;	// n回/s
	static uint16_t dcnt = 0;

	if ((Mode == PARKING_MODE) && (Mode != preMode))  // モードが遷移した時Static変数を初期化する。外れ値除去のバグ対策
	{
		first_flag   = 0;
		old_range[0] = 0;
		old_range[1] = 0;
		old_range[2] = 0;
		old_range[3] = 0;
	}		
	if (dcnt > interval) {	// 33.3ms 間隔以上で距離センサーを読む
		dcnt = 0;
		RawRange = tof_bottom_get_range();
		if (RawRange > 20)
			Range = RawRange;
			// 外れ値処理
			deff = Range - old_range[1];
			if(outlier_counter < 2 && (deff > 500 || deff < -500)){
				Range = old_range[1] + (old_range[1] - old_range[3]) / 2;
				outlier_counter++;
			}
			else {
				outlier_counter = 0;
				old_range[3]    = old_range[2];
				old_range[2]    = old_range[1];
				old_range[1]    = Range;
			}
//			USBSerial.printf("bottom:raw%d range%d\n",RawRange, Range);
	} else
		dcnt++;

	Altitude = alt_filter.update((float)Range / 1000.0, Interval_time2);
	if (first_flag == 1)
		EstimatedAltitude.update(Altitude, Az, Interval_time2);	// 推定高度
	else
		first_flag = 1;

	Altitude2 		= EstimatedAltitude.Altitude;	// 推定高度
	Alt_velocity	= EstimatedAltitude.Velocity;	// 上昇速度
	Az_bias				= EstimatedAltitude.Bias;			// 
//	USBSerial.printf("Range:%d Altitude:%f Altitude2:%f\n",Range, Altitude, Altitude2);

	// MAX_ALTを超えたら高度下げるフラグを立てる（自動着陸）高度維持モード時 または 接地時
	if ((Altitude2 > ALT_LIMIT && Alt_flag >= 1 && Flip_flag == 0) || RawRange == 0)
		Range0flag++;
	else
		Range0flag = 0;
	if (Range0flag > RNAGE0FLAG_MAX)
		Range0flag = RNAGE0FLAG_MAX;
}

void tof_front_read(uint16_t n){
	const uint16_t interval = 400 / n + 1;		// n回/s　測定
	static uint16_t dcnt = interval/2 + 1;		// bottomとタイミングをずらす
	static int16_t old_range[4]    = {0};
	int16_t deff;
	static uint8_t outlier_counter = 0;

	if(dcnt > interval){
		dcnt = 0;
		RawRangeFront = tof_front_get_range();
		if (RawRangeFront > 0) {
			RangeFront = RawRangeFront;
			// 外れ値処理
			deff = RangeFront - old_range[1];
			if(outlier_counter < 2 && (deff > 500 || deff < -500)){
				RangeFront = old_range[1] + (old_range[1] - old_range[3]) / 2;
				outlier_counter++;
			}
			else {
				outlier_counter = 0;
				old_range[3]    = old_range[2];
				old_range[2]    = old_range[1];
				old_range[1]    = RangeFront;
			}
//			USBSerial.printf("front:raw%d range%d\n",RawRange, Range);
		}
	}else dcnt++;
}

void sensor_init(void) {
	SPI.begin(PIN_NUM_CLK, PIN_NUM_MISO, PIN_NUM_MOSI);
	pinMode(PIN_CS, OUTPUT);  	// CSを設定
	digitalWrite(PIN_CS, HIGH); // CSをHIGH (Low Active)
	pinMode(PIN_CS2, OUTPUT); 	// CS2を設定
	digitalWrite(PIN_CS2, HIGH);// CS2をHIGH (Low Active)
	delay(5);
	
	imu_init();				// 6軸IMUセンサ (SPI)
	flow_init();			// オプティカルフロー (SPI)

	Drone_ahrs.begin(400.0);	// マッジウィック　フィルター　400Hzに設定
	// Acceleration filter
	acc_filter.set_parameter(0.005, 0.0025);

	raw_ax_filter.set_parameter(0.003, 0.0025);
	raw_ay_filter.set_parameter(0.003, 0.0025);
	raw_az_filter.set_parameter(0.003, 0.0025);

	raw_gx_filter.set_parameter(0.003, 0.0025);
	raw_gy_filter.set_parameter(0.003, 0.0025);
	raw_gz_filter.set_parameter(0.003, 0.0025);

	raw_az_d_filter.set_parameter(0.1, 0.0025);  // alt158
	az_filter.set_parameter(0.1, 0.0025);        // alt158
	alt_filter.set_parameter(0.005, 0.0025);

	USBSerial.println("Sensor init done");
}

float sensor_read(void) {	// SPI 関連
	static float sensor_time	= 0.0f;	
	float old_sensor_time			= 0.0;
	float sens_interval;
	uint8_t interval = 400 / 30 + 1;	// 33.3mS間隔
	static uint8_t cnt = 0;

	uint32_t st = micros();			// 開始時間
	sensor_time     = (float)st * 1.0e-6;
	sens_interval   = sensor_time - old_sensor_time;
	old_sensor_time = sensor_time;

	imu_read(sens_interval);	// ６軸IMU (SPI)

	if(cnt > interval){				// 1秒間で30回測定
		flow_read();						// オプティカルフロー (SPI)
		cnt = 0;
	}else cnt++;

	preMode = Mode;  // 今のモードを記憶
	return (float)(micros() - st) * 1.0e-6;	// センサー読み取りにかかった時間を返す
}

void sensor_init2(void){		// I2C関連　初期化
	Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);		// I2C設定

	voltage_init();		// 電流電圧計 (I2C)
	tof_init();				// 距離センサー (I2C)		
	compass_init();		// コンパス (I2C)
	bmp_begin();			// 気圧計 (I2C)

	if (scan_i2c(&Wire1) == 0) {
		USBSerial.printf("No I2C device!\r\n");
		while (1){delay(1);}
	}
}

static uint32_t st,et;

void sensor_read2(void){		// I2C関連　読み込み
	et = micros();
	Interval_time2 = (et - st) / 1000000.0f;	// 秒に変換
	st = et;
//	USBSerial.printf("time:%fmS\n",Interval_time2 * 1000);
	
	tof_bottom_read(30);	// 距離計 Bottom (I2C) 測定に約5msもかかる 30回/s
	tof_front_read(30);		// 距離計 Front (I2C)  測定に約5msもかかる 30回/s
	volt_read();					// 電圧 (I2C)	測定 0.2ms~0.5ms 						毎回	
	bmp_read(30);					// 気圧センサー (I2C) 測定 0.6ms					30回/s
	compass_read(10);			// コンパス (I2C) 測定 0.5ms							10回/s
}