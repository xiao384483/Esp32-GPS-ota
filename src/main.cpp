#include<main.h>
#include <regex>
#include <string>
#include <vector>
const float version = 4.16;//请及时更新版本号
/*目前实现的功能：

1.使用WIFI进行OTA升级
2.蓝牙从机模式打开，名称为esp32，可以接收"([^:]+(.+))"形式的字符串，输入speed:[1-100]时显示百分之[1-100]的进度条
3.急停锁。当检测到锁打开以后对应引脚拉高,检测不到锁的时候拉低引脚。
4.GPS实时读取卫星时间信息，目前打印经纬度和日期在oled上，10s后续切换屏幕2，显示Battery soc和voltage,3s后切换到屏幕1
5.
*/
BLEServer *pServer;
BLECharacteristic *pCharacteristicRX;
BLECharacteristic *pCharacteristicTX;
std::queue<String> commandQueue;

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool safeLock = false;

void safelock(String value){
  int safe = value.toInt();
  if (safe==1)
  {
    safeLock = true;
  }else{
    safeLock = false;
  }
}
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
  commandQueue.push(receivedString);
  // display.clear();
  // display.drawString(0, 20, receivedString);
  }
  }
};

// sport:后跟前，后，左，右四个指令
void sport(String value) {
    // 执行 sport 函数的操作
    display.clear();
    display.drawString(0, 20, value);
    pCharacteristicTX->setValue("success");
    pCharacteristicTX->notify();
}

// 定义函数2
void speed(String value) {
    // 执行 speed 函数的操作
    int num = value.toInt();
    display.clear();
    display.drawRect(0, 40, 60, 10);
    display.fillRect(0, 42, (60 * num/ 100), 8);
    pCharacteristicTX->setValue("speed:");
    pCharacteristicTX->notify();
}

// 解析接收到的指令并执行相应操作
std::regex commandPattern("([^:]+):(.+)");
void parseAndExecuteCommand(String command) {
  std::smatch match;
  // 将命令字符串转换为 std::string 类型
  std::string commandStr = command.c_str();

  // 尝试在命令中匹配模式
  if (std::regex_match(commandStr, match, commandPattern)) {
    if (match.size() == 3) {
      String key = match[1].str().c_str();
      String value = match[2].str().c_str();

      // 根据键调用相应函数，并传递值作为参数
      if (key == "sport") {
        sport(value);
      } else if (key == "speed") {
        speed(value);
      } else if (key == "safelock") {
        safelock(value);
      } else {
        display.drawString(0, 54, "NO define");
      }
    }
  } else {
    Serial.println("Invalid command format");
  }
}


// 

//BLE初始化
void setupBLE(){
BLEDevice::init("esp32");
pServer = BLEDevice::createServer();
pServer->setCallbacks(new MyServerCallbacks());

BLEService *pService = pServer->createService(SERVICE_UUID);
pCharacteristicRX = pService->createCharacteristic(CHARACTERISTIC_RX_UUID, BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY|BLECharacteristic::PROPERTY_INDICATE);

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
  pinMode(gpioPin13, OUTPUT);//13号引脚设置为输出
  digitalWrite(gpioPin13, HIGH);//拉高13号引脚，GPS可以正常工作

WiFi.begin(ssid, password);//初始化至STA模式，需要预先输入wifi名称密码
while (WiFi.status()!=WL_CONNECTED)
   {
    delay(1000);
    Serial.println("Connecting to wifi...");
  }
  setupOTA();
  setupBLE();
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear(); // 显示屏初始化
}
//GPS信息传输
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
  display.display();
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
   ArduinoOTA.handle();
   display.drawString(0, 0, "version: "+String(version));//版本号
 
   
   if (deviceConnected)
   {
     display.drawString(70, 0, "on");
   }else{
     display.drawString(70, 0, "off");
   }
   if (safeLock)
   {
     display.drawString(90, 0, "safe");
   }else{
     display.drawString(90, 0, "no");
   }

   //This sketch displays information every time a new sentence is correctly encoded.
   while (ss.available() > 0)
     if (gps.encode(ss.read()))
       displayInfo();
//GPS喂数据检错
   if (millis() > 5000 && gps.charsProcessed() < 10)
   {
     Serial.println(F("No GPS detected: check wiring."));
     while (true);
  }
   if (!commandQueue.empty()) {
      String command = commandQueue.front();
      commandQueue.pop();
      parseAndExecuteCommand(command); // 解析并执行蓝牙指令
   }
display.display();
}

  