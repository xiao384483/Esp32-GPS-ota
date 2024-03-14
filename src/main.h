#include<ArduinoOTA.h>
#include<Wire.h>
#include<SH1106.h>
#include<WiFi.h>
#include<WiFiClient.h>
#include<WiFiUdp.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include<BLEDevice.h>
#include<BLEService.h>
//#include<BLEUtils.h>
#include<BLE2902.h>
#include<queue>

//pwm设置
const int pwmPin1 = 8;
const int pwmPin2 = 9;
//oled设置
#define OLED_SDA 19
#define OLED_SCL 18
SH1106Wire display(0x3c, OLED_SDA, OLED_SCL); // 创建OLED显示对象
//GPS设置
static const int GPSRXPin = 1, GPSTXPin = 0;//
static const uint32_t GPSBaud = 9600;//GPS默认9600
const int gpioPin13 = 13;//gps归零引脚，拉低可关闭GPS
TinyGPSPlus gps;
SoftwareSerial ss(GPSRXPin, GPSTXPin);
//wifi设置
const char *ssid = "Redmi_E57E";
const char *password = "88889999";
//蓝牙设置
 #define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
 #define CHARACTERISTIC_RX_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
 #define CHARACTERISTIC_TX_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

