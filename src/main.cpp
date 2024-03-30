#include<main.h>
#include <regex>
#include <string>
#include <vector>
#include<map>
#include <iostream>
#include <iomanip> 
const float version = 3.6;//请及时更新版本号
/*目前实现的功能：

1.使用WIFI进行OTA升级（但加入蓝牙后很容易冲突导致OTA失败，得再试试蓝牙OTA升级）
2.蓝牙名称为RG_Pulse，可以接收"([^;:]+):([^:;]+)"形式的字符串，后对应不同函数
3.急停锁。当检测到锁打开以后对应引脚拉高,检测不到锁的时候拉低引脚。
4.GPS实时读取卫星信息：经纬度，时间日期，航向角等。GPS开关：发送：GPS：true来拉高GPS引脚使它工作。
5.打印连接设备数以及连接状态，还有完成指令的时间
6.连接成功，断开以后重新广播，方便重连
7.实时检测两个航向角，使用PID调节后输出一个角度信号。目前Pid被注释了。后续再用
8.锚定模式：发送archor指令给ble，接下来30s内计算经纬度平均值，作为原点。完成后开始每十秒计算一次平均值距离圆心的距离，如果在3m外需要调整，否则不动
9.目前对航向角做了卡尔曼滤波，三个滤波参数是2，100，1，暂定，2指GPS精度误差，100一直在随系统改变，1指系统噪音，需要根据实际来调节。
10.目前每10s更新一次GPS经纬度的平均值，用NowCircle_x,NowCircle_y代替平均经纬度。
11.经纬度之间计算方式使用Tinygps中的.betweenDistance(a,b,c,d)函数，a，c是纬度，b,d是经度
12.增加了print函数，每隔两秒收集一次GPS数据，收集150个。（刚好5min）收集正常信号和处理后的信号。
本来考虑使用串口打印的时候用python读取，直接绘图，但是因为串口占用，好像不太行.....

*/
unsigned long delayTime;
unsigned long StartTime;
unsigned long EndTime;//这仨是为了计算延迟用
BLEServer *pServer;//创建蓝牙服务器和特征对象
BLECharacteristic *pCharacteristicRX;
BLECharacteristic *pCharacteristicTX;
std::queue<String> commandQueue;//该队列防止堵塞，波特率9600的话可以删掉，再往高可能比较有用
std::queue<double> latti;//需要自主导航的点的纬度
std::queue<double> longgi;//需要自主导航的点的经度
std::queue<float> NowDeg;//存储原始航向角（测试哪个更准确用。）
std::queue<float> FilteredNowDeg;//存储卡尔曼滤波后的航向角
std::queue<double> CircleLat;//持续保留十个纬度数据的队列
std::queue<double> CircleLng;//持续保留十个经度数据的队列
std::queue<double> dataLatti;//记录一组GPS数据回来
std::queue<double> dataLonggi;//记录一组GPS数据回来
std::queue<double> dataCirLatti;//记录一组平滑后的GPS数据
std::queue<double> dataCirLonggi;//记录一组平滑后的GPS数据
bool deviceConnected = false;//蓝牙连接状态
bool oldDeviceConnected = false;
bool safeLock = false;//安全锁状态。ble发送"safelock:on"字符串可以打开。
bool wait = false;
bool deeg = false;
bool anchorkey = false;//锚点定位成功后置1
//bool anchorDis = false;
bool StartKey = false;


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
 void onWrite(BLECharacteristic*pCharacteristic){//同步写入，实时处理，占用内存会多一些
  std::string rxValue = pCharacteristic->getValue();
  if (!rxValue.empty()) {
  String receivedString = String(rxValue.c_str());
  display.clear();
  display.drawString(0, 18, receivedString);
  display.display();
  //commandQueue.push(receivedString);
  }
  }
// void write(BLECharacteristic*pCharacterisitic){

// }




//   virtual void onWrite_nr(BLECharacteristic*pCharacteristic){//异步写入，后台处理，会有延迟，一般影响不大，但不排除有情况导致高延迟
//   std::string rxValue = pCharacteristic->getValue();
//    if (!rxValue.empty()) {
//   String receivedString = String(rxValue.c_str());
//   display.clear();
//   display.drawString(0, 36, "!!"+receivedString);
//   display.display();
//   //commandQueue.push(receivedString);
//   }
// }
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
    pCharacteristicTX->setValue("speed:");
    pCharacteristicTX->notify();
    wait = false;
    EndTime = millis();
    delayTime = EndTime - StartTime;
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
  wait = false;
}
void GPS(String value){
  if (value=="off")
  {
   digitalWrite(gpioPin13, LOW);//拉高13号引脚，GPS可以正常工作
  }else{digitalWrite(gpioPin13, HIGH);}//拉高13号引脚，GPS可以正常工作
  wait = false;
  EndTime = millis();
  wait = false;
  delayTime = EndTime - StartTime;
}
void Navigation(String value){
  float Nav1 = value.toFloat();
    std::regex pattern(R"(([\d.]+),([\d.]+))");//手机经纬度数据，中间用英文版逗号隔开
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
  //display.drawString(0, 54, String(FilteredDeg));
  double distance_m = gps.distanceBetween(gps.location.lat(), gps.location.lng(),m,n);
  double courseTo =gps.courseTo(NowCircle_x, NowCircle_y,m,n);//是拿平均值的圆心来计算的
  Input = FilteredDeg-courseTo;
  display.drawString(90, 54, String(distance_m));
  display.drawString(50, 54, String(courseTo));
  PID(Input);
  while (distance_m<5)//如果队列为空，那快接近目的地的时候需要减速，这里需要引入电机的控制。
  {
    latti.pop();
    longgi.pop();
    Serial.println("Get to the first point.");
    break;  
  }
  }

void Circle(){//暂时用六个值的平均值来当作真实值，根据需求再调整
    CircleLat.push(gps.location.lat());
    CircleLng.push(gps.location.lng());
    latitudeCircle += gps.location.lat();
    longitudeCircle += gps.location.lng();
    if (CircleLat.size()>6)
    {
      latitudeCircle -= CircleLat.front();
      CircleLat.pop();
      NowCircle_x = latitudeCircle / 6;
    }
    if (CircleLng.size()>6)
    {
      longitudeCircle -= CircleLng.front();
      CircleLng.pop();
      NowCircle_y = longitudeCircle / 6;
    }
  // display.drawString(60, 18, String((latitudeCircle / 10),6));
  // display.drawString(60, 36, String((longitudeCircle / 10),6));
  // display.display();
}  
void anchor(String value){
  if(value=="on"){
  double latitude1=0; // 获取经度
  double longitude1=0; // 获取纬度
  for (int i = 0; i < 30; i++) {
    anchorkey = false;
    //anchorDis = true;
    display.clear();
    display.drawString(0, 36, "Please wait "+String(30-i)+" s.");
    display.display();
    latitude1 += gps.location.lat();
    longitude1 += gps.location.lng();
    Serial.println(gps.location.lat(), 6);
    Serial.println(gps.location.lng(), 6);
    delay(1000);
  while (i==29)
    {
      display.drawString(0, 18, String((latitude1 / 30),6));
      display.drawString(0, 36, String((longitude1 / 30),6));
      display.display();
      origin_x = latitude1 / 30;
      origin_y = longitude1 / 30;
      longitude1 = latitude1 = 0;
      anchorkey = true;
      //anchorDis = true;
      break;}  }
     }else{//关闭在5m内调整
    anchorkey = false;
     }
}

  

void AnchorDis(){
AnchorDistance=gps.distanceBetween(origin_x, origin_y, NowCircle_x, NowCircle_y);//到导航目的地
  if(AnchorDistance>=5)
  {
    //调节船外机的代码
  }else{
    Serial.println("Archoring.");
  }
}
// 控制舵机转到指定角度的函数
//void setServoAngle(int angle) {
  
    //int pulseWidth = minPulseWidth + (angle - minAngle) * (maxPulseWidth - minPulseWidth) / (maxAngle - minAngle);

  // 设置 PWM 占空比
  //
  // delayMicroseconds(pulseWidth);
  // digitalWrite(9, LOW);
  // delay(20);
  //analogWrite(9, 90);  // 输出 PWM 信号，控制舵机转到指定角度
//}
// void steering(String value){
//   int num = value.toInt();
//   //这里写入pwm控制，暂时用引脚2代替
//   int pulseWidth = map(num, 0, 180, 500, 2500);  // 将角度映射到脉冲宽度范围
//   display.drawString(0, 18, String(num));
//   display.display();

//   // delay(20);  // 等待20毫秒，以保证舵机稳定
// }
void print(String value){//打印收集到的GPS数据，但是只能遍历一次队列....
if (value=="LAT")
{
  while (!dataLatti.empty())
  {
    Serial.println(dataLatti.front());
    dataLatti.pop();
  }
}else if (value=="LNG")
{
  while (!dataLonggi.empty())
  {
    Serial.println(dataLonggi.front());
    dataLonggi.pop();
  }
}else if (value=="CIRLAT")
{
  while (!dataCirLatti.empty())
  {
    Serial.println(dataCirLatti.front());
    dataCirLatti.pop();
  }
}else if (value=="CIRLNG")
{
  while (!dataCirLonggi.empty())
  {
    Serial.println(dataCirLonggi.front());
    dataCirLonggi.pop();
  }
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
      }else if(key=="anchor"){
        anchor(value);
      }else if(key=="print"){
        print(value);
      }else if(key=="steering"){
        //steering(value);
      }else if (key == "Nav") {//输入需要导航的点ABCDEF
         Navigation(value);
      }else if (key == "StartNav") {//开始导航，计算到第一个点的距离
        StartKey = true;
      //Serial.println("Startcount = 1");
      }else {
        display.drawString(0, 54, "Undefined");
      }
    }
  } 
}
//BLE初始化
void setupBLE(){
BLEDevice::init("RG-Pulse");
pServer = BLEDevice::createServer();
pServer->setCallbacks(new MyServerCallbacks());
BLEService *pService = pServer->createService(SERVICE_UUID);
pCharacteristicRX = pService->createCharacteristic(CHARACTERISTIC_RX_UUID,  BLECharacteristic::PROPERTY_WRITE_NR|BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_NOTIFY|BLECharacteristic::PROPERTY_INDICATE);

pCharacteristicRX->setCallbacks((new MyCallbacks()));
pCharacteristicTX = pService->createCharacteristic(CHARACTERISTIC_TX_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
pCharacteristicTX->addDescriptor(new BLE2902());
pService->start();
BLEAdvertising *pAdvertising = pServer->getAdvertising();
pAdvertising->addServiceUUID(SERVICE_UUID);
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
   pinMode(gpioservo, OUTPUT);//舵机引脚设置为输出
   digitalWrite(gpioPin13,HIGH);
  //digitalWrite(gpioservo, HIGH);
    // 初始化 PWM
  ledcSetup(pwmChannel, 50, 8); // 50Hz 频率，8 位分辨率
  ledcAttachPin(gpioservo, pwmChannel);
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
  //display.clear();
  StartTime = millis();
  display.drawString(0, 0, "version: " + String(version)); // 版本号
  
//检查蓝牙连接状态
  if (deviceConnected)
  {
    display.drawString(70, 0, "on"+String(connectionCount));
   }else{
     display.drawString(70, 0, "off"+String(connectionCount));
   }
//检查蓝牙指令
  if (!commandQueue.empty()) {
      //检测蓝牙是否占用
    if (wait)
   {
     delay(1000);
     wait = false;
   }
     String command = commandQueue.front();
     commandQueue.pop();
     parseAndExecuteCommand(command); // 解析并执行蓝牙指令
   }
//检查安全锁状态（锁定运动指令）
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
   //开始GPS的检测
   while (ss.available() > 0)
     {if (gps.encode(ss.read()))
     //  {displayInfo();}
      //delay(1000);
      break;}
   // GPS喂数据检错
   if (millis() > 5000 && gps.charsProcessed() < 10)
   {
     Serial.println(F("No GPS detected: check wiring."));
     while (true);
  }
   int updatecount=0;
   if(gps.location.isValid()){
     delay(1000);
     deeg = true;
     updatecount++;
     display.drawString(0, 54, String(updatecount));
   }else{
     //display.drawString(0, 54, "false");
   }
  ledcWrite(pwmChannel, 1000);
// //display.drawString(0, 18, String(delayTime)+"ms");
  // double m = latti.front();
  // double n = longgi.front();
 double a = 22.688111;
 double b = 114.064111;
 float c = gps.course.deg();
 double FilteredDeg = degFilter.updateEstimate(c);
 //display.drawString(45, 36, String(FilteredDeg));
 //display.drawString(0, 36, String(c));
 double distance_m = gps.distanceBetween(gps.location.lat(), gps.location.lng(), a, b);
 double courseTo = gps.courseTo(gps.location.lat(), gps.location.lng(), a, b); // 是不是拿平均值的圆心来计算更合理？
 Input = FilteredDeg - courseTo;
 //display.drawString(90, 54, String(distance_m));
 //display.drawString(45, 54, String(courseTo));
//这里存储100个航向角和卡尔曼滤波后的航向角，可以一定程度观测GPS的平滑程度。
//但是并没有写记录一段数据回来的代码。
 unsigned long currentTime = millis();
//每间隔一秒处理一次的任务
   if (currentTime - lastMethod1Time >= 1000) {
    Circle();//计算十个点的平均值
    lastMethod1Time = currentTime;
  }
//每间隔两秒计算一次的函数：更新航向角
  if (currentTime - lastMethod2Time >= 2000) {
    NowDeg.push(gps.course.deg());
    if (NowDeg.size()>500)
    {
     NowDeg.pop();
    }
    FilteredNowDeg.push(FilteredDeg);
    if(FilteredNowDeg.size()>100)
    {
     FilteredNowDeg.pop();
    }
//取一次GPS数据，队列dataLatti和dataLonggi
   //需要一个固定的口令完成
   if (dataLatti.size()<150)
   {
    dataLatti.push(gps.location.lat());
    dataLonggi.push(gps.location.lng());
    dataCirLatti.push(NowCircle_x);
    dataCirLonggi.push(NowCircle_y);
   }
    lastMethod2Time = currentTime;
  }
  
 if (StartKey)
 {
  if(!latti.empty()&&!longgi.empty()){
    StartNav();
    }else{Serial.println("Navigation data is empty!");}
 }
  // 每隔十秒执行一次的函数：如果锚点模式开启，十秒计算一次
  if (currentTime - lastMethod2Time >= 10000) {
    if (anchorkey){AnchorDis();}
    lastMethod3Time = currentTime;
  }
 display.display();
}

  