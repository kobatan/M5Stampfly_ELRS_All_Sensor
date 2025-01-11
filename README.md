# StampFly
StampFlyのELRS版です。

さらに、搭載されている全てのセンサーを読めるようし、テレメトリーで出力させて見ました。

〇　６軸IMUセンサー（BMI270）SPI

〇　オプティカルフローセンサー（PMW3901MB-TXQT）SPI

〇　前方と下の距離センサー（VL53L3CX）I2C [0x29,0x2A]

〇　コンパス（BMM150）I2C [0x10]

〇　気圧計（BMP280）I2C [0x76]

〇　電流電圧計（INA3221AIRGVR）I2C [0x40]

マルチコアを利用し、センサー読み込みを分散させています。

<img width="300" src="https://github.com/user-attachments/assets/a41d8cbd-0b13-4dff-a2a9-e2a14cd6b905">

[M5Stack用テレメトリーモニターはこちらです。](https://github.com/kobatan/Stampfly_Monitor)

## Framework

Platformio

## Base on project

[M5Fly-kanazawa/M5StampFly (github.com)](https://github.com/M5Fly-kanazawa/M5StampFly/tree/elrs)
## Product introduction

[M5Stampfly](https://docs.m5stack.com/en/app/Stamp%20Fly)

## Third-party libraries

fastled/FastLED

tinyu-zhao/INA3221

mathertel/OneButton

bitcraze/Bitcraze PMW3901@^1.2

adafruit/Adafruit BMP280 Library@^2.6.8

arduino-libraries/Madgwick@^1.2.0

hideakitai/ESP32DMASPI@^0.6.4

https://github.com/m5stack/M5_BMM150

https://github.com/simondlevy/BMI270-SPI

https://github.com/stm32duino/VL53L3CX

