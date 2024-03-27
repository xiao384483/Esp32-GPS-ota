//#include<ArduinoOTA.h>
#include<Wire.h>
#include<SH1106.h>
#include<Arduino.h>
// #include<WiFi.h>
// #include<WiFiClient.h>
// #include<WiFiUdp.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include<BLEDevice.h>
#include<BLEService.h>
//#include<BLEUtils.h>
#include<BLE2902.h>
#include<queue>
#include <SimpleKalmanFilter.h>
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
double latitudeCircle = 0; // 获取经度
double longitudeCircle =0; // 获取纬度 
int CircleCount=0;
double origin_x=0;
double origin_y=0;
double NowCircle_x = 0;
double NowCircle_y = 0;
//wifi设置
// const char *ssid = "Redmi_E57E";
// const char *password = "88889999";
//蓝牙设置
 #define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
 #define CHARACTERISTIC_RX_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
 #define CHARACTERISTIC_TX_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

//PID
float Setpoint, Input, Output;
float M1Kp = 1, M1Ki = 0, M1Kd = 0;//激进方案
float M2Kp = 0.8, M2Ki = 0.2, M2Kd = 0.5;//中等增益方案
float M3Kp = 0.5, M3Ki = 0.1, M3Kd = 0.25;//保守方案
float pout = 0, iout = 0, dout = 0;
float Error=0,dt=0.1;//采样时间dt
float Dbuf[3];
float Error1[3];

int StartCount = 0;

// 定义Kalman滤波器
//有没有对经纬度滤波的需求？？？或者使用计算平均值的方法即可？
// SimpleKalmanFilter latFilter(0.1,0.1,0.01);
// SimpleKalmanFilter lngFilter(0.1,0.1,0.01);
SimpleKalmanFilter degFilter(2, 100, 1);
// // 设置Kalman滤波器参数
// const float latMeasurementNoise = 0.1; // GPS纬度测量误差
// const float lngMeasurementNoise = 0.1; // GPS经度测量误差
// const float latProcessNoise = 0.001;   // GPS纬度过程噪声
// const float lngProcessNoise = 0.001;   // GPS经度过程噪声
// const float latEstimateError = 0.1;     // GPS纬度估计误差
// const float lngEstimateError = 0.1;     // GPS经度估计误差
unsigned long lastMethod1Time = 0;
unsigned long lastMethod2Time = 0;