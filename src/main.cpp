#include<main.h>
#include <regex>
#include <string>
#include <vector>
#include<map>
#include <iostream>
#include <iomanip> 
const float version = 3.3;//请及时更新版本号
/*目前实现的功能：

1.使用WIFI进行OTA升级（但加入蓝牙后很容易冲突导致OTA失败，得再试试蓝牙OTA升级）
2.蓝牙名称为RG_Pulse，可以接收"([^;:]+):([^:;]+)"形式的字符串，后对应不同函数
3.急停锁。当检测到锁打开以后对应引脚拉高,检测不到锁的时候拉低引脚。
4.GPS实时读取卫星信息：经纬度，时间日期，航向角等。GPS开关：发送：GPS：true来拉高GPS引脚使它工作。
5.打印连接设备数以及连接状态，还有完成指令的时间
6.连接成功，断开以后重新广播，方便重连
7.实时检测两个航向角，使用PID调节后输出一个角度信号。目前Pid被注释了。后续再用
8.锚定模式：
9.目前对航向角做了卡尔曼滤波，三个滤波参数是2，100，1，暂定，2指GPS精度误差，100一直在随系统改变，1指系统噪音，需要根据实际来调节。
*/
unsigned long delayTime;
unsigned long StartTime;
unsigned long EndTime;//这仨是为了计算延迟用
BLEServer *pServer;
BLECharacteristic *pCharacteristicRX;
BLECharacteristic *pCharacteristicTX;
std::queue<String> commandQueue;//该队列防止堵塞，波特率9600的话可以删掉，再往高可能比较有用
std::queue<double> latti;
std::queue<double> longgi;
std::queue<float> NowDeg;
std::queue<float> FilteredNowDeg;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool safeLock = false;
bool wait = false;
bool deeg = false;
int connectionCount = 0;
// struct PolarCoordinate {
//     double direction; // 航向角
//     double distance;  // 距离
// };

 // // 定义经纬度结构体
 // struct LatLng {
//     double latitude;
//     double longitude;
// };

// struct Point {
//     double x;
//     double y;
// };
// std::queue<Point> points;

// LatLng Me = {Me_lati,Me_long};//当前定位的GPS位置
// 定义三个点的经纬度坐标
  // LatLng pointA = {22.688044, 114.064332};
  // LatLng pointB = {22.689064, 114.067758};
  // LatLng pointC = {22.689567, 114.072458};
class MyServerCallbacks:public BLEServerCallbacks{
  void onConnect(BLEServer*pServer){
    deviceConnected = true;
    oldDeviceConnected = true;
    display.clear();
    connectionCount += 1; // 在设备连接时触发
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
  }
  void onDisconnect(BLEServer*pServer){
    deviceConnected = false;
    display.clear();// 在设备断开时触发
    connectionCount -= 1;
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
  }
};
//BLE输入不同类型字符对应的输出
class MyCallbacks:public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic*pCharacteristic){
    std::string rxValue = pCharacteristic->getValue();
   if (!rxValue.empty()) {
  String receivedString = String(rxValue.c_str());
  commandQueue.push(receivedString);
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
    wait = false;
}
// 定义速度函数
void speed(String value) {
    // 执行 speed 函数的操作
    int num = value.toInt();
    display.clear();
    display.drawRect(0, 40, 60, 10);
    display.fillRect(0, 42, (60 * num/ 100), 8);
    EndTime = millis();
    delayTime = EndTime - StartTime;
    pCharacteristicTX->setValue("speed:");
    pCharacteristicTX->notify();
    wait = false;
}
void safelock(String value){
  if (value=="on")
  {
    safeLock = true;
    display.clear();
  }else{
    safeLock = false;
    display.clear();
  }
}
void GPS(String value){
  if (value=="off")
  {
   digitalWrite(gpioPin13, LOW);//拉高13号引脚，GPS可以正常工作
  }else{digitalWrite(gpioPin13, HIGH);}//拉高13号引脚，GPS可以正常工作
  wait = false;
  EndTime = millis();
  delayTime = EndTime - StartTime;
}
void Navigation(String value){
  float Nav1 = value.toFloat();
    std::regex pattern(R"(([\d.]+),([\d.]+))");
    std::smatch match;
    std::string valuer = value.c_str();
    if (std::regex_match(valuer, match, pattern)) {
      //double latitu = m.toDouble();//接收到的还是小数点后两位
      double x = std::stod(match[1].str());
      double y = std::stod(match[2].str());//必须用stod，此处用stof会导致最后两位小数有偏差
      Serial.println(x,6);
      Serial.println(y,6);
      latti.push(x);
      longgi.push(y);


      //   Point p;
      //   p.x = x;
      //   p.y = y;
      // // 将 Point 对象添加到队列中
      //   points.push(p);
      // x = distanceBetween(pointA, pointB);
      // Point p = {x, y};
      // std::queue<Point> points;
      // points.push(p);
    }
    // std::queue<Point> copyQueue = points;
    EndTime = millis();
    delayTime = EndTime - StartTime;
 }
void PID(double Input){
//PID设置
    Error1[2]=Error1[1];
    Error1[1]=Error1[0];
    Error1[0]=Input;
    delay(1000*dt);
    if (Input>60||Input<=180)//需要右转
    {
        Output=60;
    }else if (Input<=60&&Input>30)
    {
        pout = M2Kd * Input;
        iout += M2Ki * Input;
        Output = pout + iout;
   }else if (Input<=30&&Input>5)
    {
        pout = M2Kd * Input;
        iout += M2Ki * Input;
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        Dbuf[0] = Error1[0]-Error1[1];
        dout = M2Kd * Dbuf[0];
        Output = pout + iout + dout;
   }else if(Input<=5&&Input>=-5)
   {
    Output=0;
   }else if(Input<-60||Input>=180)//需要左转
    {
        Output=60;
    }else if (Input>=-60&&Input<-30)
    {
        pout = M2Kd * Input;
        iout += M2Ki * Input;
        Output = pout + iout;
   }else if (Input>=-30&&Input<-5)
    {
        pout = M2Kd * Input;
        iout += M2Ki * Input;
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        Dbuf[0] = Error1[0]-Error1[1];
        dout = M2Kd * Dbuf[0];
        Output = pout + iout + dout;
   } else{Output=0;}
}    
void StartNav(){//计算两个航向角以及距离，同时如果距离在五米之内，删除第一个点
  double m = latti.front();
  double n = longgi.front();
  double FilteredDeg = degFilter.updateEstimate(gps.course.deg());
  display.drawString(0, 54, String(FilteredDeg));
  double distance_m = gps.distanceBetween(gps.location.lat(), gps.location.lng(),m,n);
  double courseTo =gps.courseTo(gps.location.lat(), gps.location.lng(),m,n);//是不是拿平均值的圆心来计算更合理？
  Input = FilteredDeg-courseTo;
  display.drawString(90, 54, String(distance_m));
  display.drawString(50, 54, String(courseTo));
  PID(Input);
  while (distance_m<5)
  {
    latti.pop();
    longgi.pop();
    Serial.println("Get to the first point.");
    break;  
  }
  }
//析接收到的指令并执行相应操作
std::regex commandPattern("([^;:]+):([^:;]+)");
void parseAndExecuteCommand(String command) {
  wait = true;
  std::smatch match;
  // 将命令字符串转换为 std::string 类型
  std::string commandStr = command.c_str();
  // 尝试在命令中匹配模式
  if (std::regex_match(commandStr, match, commandPattern)) {
    if (match.size() == 3) {
      String key = match[1].str().c_str();
      String value = match[2].str().c_str();
      display.clear();
      // 根据键调用相应函数，并传递值作为参数
      if (key == "sport") {
        sport(value);
      } else if (key == "speed") {
        speed(value);
      } else if (key == "safelock") {
        safelock(value);
      } else if (key == "GPS") {
        GPS(value);
      }else if (key == "Nav") {//输入需要导航的点ABCDEF
         Navigation(value);
      }else if (key == "StartNav") {//开始导航，计算到第一个点的距离
        StartCount += 1;
        Serial.println("Startcount = 1");
      }else if(key=="collect"){
        if(value=="30"){
          // Serial.print("Startcount = ");
          // Serial.println(StartCount);
    // 收集接下来100个经纬度数据
          display.drawString(0, 36, "Wait 30s");
          display.display();
          double latitude1=0; // 获取经度
          double longitude1=0; // 获取纬度
         for (int i = 0; i < 30; i++) {
          latitude1 += gps.location.lat();
          longitude1 += gps.location.lng();
          delay(1000);
          while (i==29)
          {
            display.drawString(0, 18, String((latitude1 / 100),6));
            display.drawString(0, 36, String((longitude1 / 100),6));
            display.display();
            break;
          }  
          }
        }
        if(value=="10"){
          // Serial.print("Startcount = ");
          // Serial.println(StartCount);
    // 收集接下来12个经纬度数据
          double latitude2 = 0; // 获取经度
          double longitude2 =0; // 获取纬度 
         for (int i = 0; i < 6; i++){
      
          latitude2 += gps.location.lat();
          longitude2 += gps.location.lng();
          delay(1000);
          while (i==5)
          {
            display.drawString(60, 18, String((latitude2 / 12),6));
            display.drawString(60, 36, String((longitude2 / 12),6));
            display.display();
            break;
          }  
          }  
        }
      }else if (value=="clear")
        {
          StartCount = 0;
          Serial.println("Startcount has been cleared");
          display.clear();
        }
      }else {
        display.drawString(0, 54, "Undefined");
      }
    }
  } 


// 

//BLE初始化
void setupBLE(){
BLEDevice::init("RG-Pulse");
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
//Serial.println("blurtooth device active,waiting for connections...");

}

//  void setupOTA(){
//       ArduinoOTA
//       .onStart([]()
//                {
//          String type;
//          if (ArduinoOTA.getCommand()==U_FLASH)
//          type="sketch";
//          else type="filesystem";
//          Serial.println("Start updating " + type); })
//       .onEnd([]()
//              { Serial.println("\nEnd"); })
//       .onProgress([](unsigned int progress, unsigned int total)
//                   { Serial.println("Progress:%u%%\r,(progress/(total/100))"); })
//       .onError([](ota_error_t error)
//                { Serial.printf("Error[%u]",error);
//                if(error==OTA_AUTH_ERROR)Serial.println("Auth Failed");
//                else if(error==OTA_BEGIN_ERROR)Serial.println("Begin Failed");
//                else if(error==OTA_CONNECT_ERROR)Serial.println("Connect Failed");
//                else if(error==OTA_RECEIVE_ERROR)Serial.println("Receive Failed");
//                else if(error==OTA_END_ERROR)Serial.println("End Failed"); });
//   // 初始化OTA
//   ArduinoOTA.begin();
//   Serial.println("Ready");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
// }

void setup() {

  Serial.begin(921600);
  ss.begin(GPSBaud);
   pinMode(gpioPin13, OUTPUT);//13号引脚设置为输出
   digitalWrite(gpioPin13,HIGH);
 

//WiFi.begin(ssid, password);//初始化至STA模式，需要预先输入wifi名称密码
// while (WiFi.status()!=WL_CONNECTED)
//    {
//     delay(1000);
//     Serial.println("Connecting to wifi...");
//   }
  //setupOTA();
   setupBLE();
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear(); // 显示屏初始化
  //pid初始化误差数组
    Dbuf[0] = 0;
    Error1[0] = 0;}
//GPS信息传输
// 

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
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
  display.clear();
  StartTime = millis();
  display.drawString(0, 0, "version: " + String(version)); // 版本号

  if (deviceConnected)
  {
    display.drawString(70, 0, "on"+String(connectionCount));
   }else{
     display.drawString(70, 0, "off"+String(connectionCount));
   }
   if (safeLock)
   {
     display.drawString(90, 0, "safe");
   }else{
     display.drawString(90, 0, "no");
   }
//  if (!deviceConnected && oldDeviceConnected) {//在断开连接时会触发开启广播，如果出现其他需要检测广播状态的情况，再取消注释
//     // 设备之前连接过，但现在连接断开，尝试重新连接
//     BLEAdvertising *pAdvertising = pServer->getAdvertising();
//     pAdvertising->start();
//     oldDeviceConnected = deviceConnected;
//   }
   //This sketch displays information every time a new sentence is correctly encoded.
   while (ss.available() > 0)
     {if (gps.encode(ss.read()))
       {displayInfo();}
      //delay(1000);
      break;}
   // GPS喂数据检错
   if (millis() > 5000 && gps.charsProcessed() < 10)
   {
     Serial.println(F("No GPS detected: check wiring."));
     while (true);
  }
  if (wait)
   {
     delay(1000);
     wait = false;
   }
   int updatecount=0;
   if(gps.location.isValid()){
     delay(1000);
     deeg = true;
     updatecount++;
     display.drawString(0, 54, String(updatecount));
   }else{
     display.drawString(0, 54, "false");
   }
   if (!commandQueue.empty()&&!wait) {
     String command = commandQueue.front();
     commandQueue.pop();
     parseAndExecuteCommand(command); // 解析并执行蓝牙指令
   }

  
//  if (StartCount==1)
//  {
//   if(!latti.empty()&&!longgi.empty()){
//     StartNav();
//     }else{Serial.println("Navigation data is empty!");}
//  }
// //display.drawString(0, 18, String(delayTime)+"ms");
  // double m = latti.front();
  // double n = longgi.front();
 double a = 22.688111;
 double b = 114.064111;
 float c = gps.course.deg();
 double FilteredDeg = degFilter.updateEstimate(c);
 display.drawString(45, 36, String(FilteredDeg));
 display.drawString(0, 36, String(c));
 double distance_m = gps.distanceBetween(gps.location.lat(), gps.location.lng(), a, b);
 double courseTo = gps.courseTo(gps.location.lat(), gps.location.lng(), a, b); // 是不是拿平均值的圆心来计算更合理？
 Input = FilteredDeg - courseTo;
 display.drawString(90, 54, String(distance_m));
 display.drawString(45, 54, String(courseTo));
   if (deeg)
  {
    NowDeg.push(gps.course.deg());
    FilteredNowDeg.push(FilteredDeg);
    deeg = false;
  }
 display.display();
}

  