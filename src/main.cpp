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

const float version = 5.11;//请及时更新版本号
#define OLED_SDA 19
#define OLED_SCL 18
SH1106Wire display(0x3c, OLED_SDA, OLED_SCL); // 创建OLED显示对象


static const int RXPin = 1, TXPin = 0;
static const uint32_t GPSBaud = 9600;
const int gpioPin = 13;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

const char *ssid = "Redmi_E57E";
const char *password = "88889999";

BLEServer *pServer;
BLECharacteristic *pCharacteristicRX;
BLECharacteristic *pCharacteristicTX;
//std::queue<String> commandQueue;
//蓝牙设置
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_RX_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_TX_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"


bool deviceConnected = false;
bool oldDeviceConnected = false;
class MyServerCallbacks:public BLEServerCallbacks{
  void onConnect(BLEServer*pServer){
    deviceConnected = true;
    display.clear();
    display.drawString(70, 0, "on");
    display.display();
    // 在设备连接时触发
  }
  void onDisconnect(BLEServer*pServer){
    deviceConnected = false;
    display.clear();
    display.drawString(70, 0, "off");
    display.display();
    // 在设备断开时触发
  }
};
//BLE输入不同类型字符对应的输出
class MyCallbacks:public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic*pCharacteristic){
    std::string rxValue = pCharacteristic->getValue();
   if (!rxValue.empty()) {
  String receivedString = String(rxValue.c_str());
  //commandQueue.push(receivedString);
  // display.clear();
  // display.drawString(0, 20, receivedString);
  }
  }
};

//BLE初始化
void setupBLE(){
BLEDevice::init("esp32c3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristicRX = pService->createCharacteristic(CHARACTERISTIC_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  pCharacteristicRX->setCallbacks((new MyCallbacks()));

  pCharacteristicTX = pService->createCharacteristic(CHARACTERISTIC_TX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  pCharacteristicTX->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("blurtooth device active,waiting for connections...");
 }
void setupOTA(){
      ArduinoOTA
      .onStart([]()
               {
         String type;
         if (ArduinoOTA.getCommand()==U_FLASH)
         type="sketch";
         else type="filesystem";
         Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.println("Progress:%u%%\r,(progress/(total/100))"); })
      .onError([](ota_error_t error)
               { Serial.printf("Error[%u]",error);
               if(error==OTA_AUTH_ERROR)Serial.println("Auth Failed");
               else if(error==OTA_BEGIN_ERROR)Serial.println("Begin Failed");
               else if(error==OTA_CONNECT_ERROR)Serial.println("Connect Failed");
               else if(error==OTA_RECEIVE_ERROR)Serial.println("Receive Failed");
               else if(error==OTA_END_ERROR)Serial.println("End Failed"); });
  // 初始化OTA
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  WiFi.begin(ssid, password);//初始化至STA模式，需要预先输入wifi名称密码
while (WiFi.status()!=WL_CONNECTED)
   {
    delay(1000);
    Serial.println("Connecting to wifi...");
  }
  pinMode(gpioPin, OUTPUT);
  digitalWrite(gpioPin, HIGH);


  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  setupBLE();
  //setupOTA();



  
  
}
void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
     display.clear();
    Serial.print(gps.location.lat(), 6);
    display.drawString(0, 18, "Latitude: " + String(gps.location.lat(), 6));
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    display.drawString(0, 36, "Longitude: " + String(gps.location.lng(), 6));
    display.display();
   
  }
  else
  {
    Serial.print(F("INVALID"));
    display.clear();
    display.drawString(0, 18, "Latitude: INVALID");
    display.drawString(0, 36, "Longitude: INVALID");
  }

  Serial.print(F("  Date/Time: "));
  display.drawString(0, 54,String(gps.date.month()) +"/" +String( gps.date.day() )+ "/" + String(gps.date.year()));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
    display.drawString(0, 54, "Date/Time:  INVALID");
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
void loop() {
  // ArduinoOTA.handle();
   display.drawString(0, 0, "version: "+String(version));//版本号
   display.display();
   // This sketch displays information every time a new sentence is correctly encoded.
   while (ss.available() > 0)
     if (gps.encode(ss.read()))
       displayInfo();

   if (millis() > 5000 && gps.charsProcessed() < 10)
   {
     Serial.println(F("No GPS detected: check wiring."));
     while (true)
       ;
    
  }
}

