/*
  物联网种植智能感知与控制下位机（主控板源程序）
*/
//========================= 中间件通讯信息 =========================
String client_id = "19158"; // 用户ID
String DEVICEID = "28950"; // 智能设备ID
String APIKEY = "e91ca5476"; // 智能设备密码
String INPUTID1 = "25500"; // 数据接口1
String INPUTID2 = "25501"; // 数据接口2
String INPUTID3 = "25502"; // 数据接口3
String INPUTID4 = "25503"; // 数据接口4
String INPUTID5 = "25811"; // 数据接口5
String INPUTID6 = "25906"; // 数据接口6

unsigned long lastCheckStatusTime = 0; // 记录上次报到时间
unsigned long lastCheckInTime = 0; // 记录上次报到时间
unsigned long lastUpdateTime = 0; // 记录上次上传数据时间
const unsigned long postingInterval = 40000; // 每隔40秒向服务器报到一次
const unsigned long updateInterval = 5000; // 数据上传间隔时间5秒
unsigned long checkoutTime = 0; // 登出时间

//======================= ADS1115 模拟拓展 =======================
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;

//======================= 74HC595 数字拓展 =======================
const int pin_HC595_big = 12; // ST引脚
const int pin_HC595_push = 13; // SH引脚
const int pin_HC595_datain = 11; // DS引脚
int HC595_data[8] = {0, 0, 0, 0, 0, 1, 1, 1};
const int Q7 = 0;
const int Q6 = 1;
const int Q5 = 2;
const int Q4 = 3;
const int Q3 = 4;
const int Q2 = 5;
const int Q1 = 6;
const int Q0 = 7;

//======================= 棚内外温湿度 通风 =======================
#include <dht.h>  
dht DHT_OUT; // 创建棚外温湿度对象
#define DHT11_PIN_OUT 6 // 棚外温湿度传感数据线引脚

#include <Servo.h>
Servo myservo;

const int pin_ventilate = 7; // 通风引脚（直流电机继电器）
const int pin_window = 10; // 开窗通风引脚（舵机）

int auto_ventilate = 0; // 自动通风模式标志位（初始值为0）
int set_hum = 40; // 空气湿度上限值（初始值为40）

//====================== 土壤温湿度 升温 灌溉 ======================
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS_1 4 // 一层土壤温度数据输入引脚定义
#define ONE_WIRE_BUS_2 5 // 二层土壤温度数据输入引脚定义

OneWire oneWire_1(ONE_WIRE_BUS_1);
DallasTemperature sensors_1(&oneWire_1);
OneWire oneWire_2(ONE_WIRE_BUS_2);
DallasTemperature sensors_2(&oneWire_2);

const int pin_heat_1 = Q5; // 一层土壤升温引脚（一层陶瓷片继电器）
const int pin_heat_2 = Q6; // 二层土壤升温引脚（二层陶瓷片继电器）
int auto_heat_1 = 0; // 一层土壤自动升温标志位（初始为0）
int auto_heat_2 = 0; // 二层土壤自动升温标志位（初始为0）
int set_soilTem_1 = 18; // 一层土壤温度下限值（初始为18）
int set_soilTem_2 = 18; // 二层土壤温度下限值（初始为18）

float soilHum_1 = 0; // 一层土壤湿度（湿度越大 数值越小 MAX = 100）
float soilHum_2 = 0; // 二层土壤湿度（湿度越大 数值越小 MAX = 100）
const int pin_soilHum_1 = 2; // 一层土壤湿度模拟数据输入引脚（拓展接口A2）
const int pin_soilHum_2 = 3; // 二层土壤湿度模拟数据输入引脚（拓展接口A3）
const int pin_water = Q7; // 灌溉引脚（水泵继电器）
int auto_water = 0; // 自动灌溉模式标志位（初始为0）
int set_soilHum = 10; // 土壤湿度下限值（初始为10）

//====================== 太阳光强度 紫外线强度 ======================
const int pin_light = 1; // 太阳光强度值引脚（拓展接口A1）
const int pin_addLight = Q3; // 太阳光强度值引脚（拓展接口Q3）
int auto_light = 0; // 自动补光标志位
int set_light = 40; // 光强下限（初始为40）

int uRays = 0; // 紫外线强度

//========================== 当前降雨量 ==========================
const int pin_rain_cur = 0; // 当前降雨量引脚（拓展接口A0）

//==================== 可吸入颗粒物含量 PM2.5 ====================
#define pin_ppm A3 // PM2.5含量采集引脚
const int pin_ppm_power = 2; // PM2.5采集状态指示引脚

//========================== 土壤PH值 ==========================
#define pin_pH A2 // 土壤PH值采集引脚
#define Offset 25.677
#define pin_pH_power 3 // 土壤PH采集状态指示引脚
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40
int pHArray[ArrayLenth];
int pHArrayIndex = 0;

//========================== 可燃气体浓度 ==========================
#define pin_co A1 // CO等可燃气体浓度采集引脚
int policeFire = 1; // 火灾报警模式标志位
const int pin_policeFire = Q1; // 火灾报警引脚（拓展接口Q1）

//============================= 风速 =============================
#define pin_windSpd A0 // 风速探测引脚
int policeWind = 1; // 大风报警模式标志位
const int pin_policeWind = Q2; // 大风报警引脚（拓展接口Q2）

/*
  引脚说明：
  A0：风速探测
  A1：可燃气体浓度采集
  A2：土壤PH值采集
  A3：PM2.5浓度采集
  A4：拓展模拟输入SDA信号
  A5：拓展模拟输入SCL信号
  A0（拓展）：降雨量采集
  A1（拓展）：太阳光强度采集
  A2（拓展）：一层土壤湿度采集
  A3（拓展）：二层土壤湿度采集
  0/RX：ESP8266接收信号线
  1/TX：ESP8266发送信号线
  2：PM2.5采集状态指示引脚
  3：土壤PH值采集状态指示引脚
  4：一层土壤温度采集
  5：二层土壤温度采集
  6：空气温湿度采集
  7：通风继电器控制
  8：
  9：
  10：舵机控制
  11：74HC595 DS引脚
  12：74HC595 ST引脚
  13：74HC595 SH引脚
  Q0：
  Q1：CO 声光报警器 控制
  Q2：风速 声光报警器 控制
  Q3：白炽补光灯控制
  Q4：
  Q5：一层土壤升温继电器控制
  Q6：二层土壤升温继电器控制
  Q7：水泵继电器控制
*/
void setup() {
  Serial.begin(115200); // 串口通信频率设定为115200Hz

  ads.begin(); // ADS1115 模拟拓展 初始化

  pinMode(pin_HC595_big, OUTPUT); // 74HC595 ST引脚 定义为信号输出
  pinMode(pin_HC595_push, OUTPUT); // 74HC595 SH引脚 定义为信号输出
  pinMode(pin_HC595_datain, OUTPUT); // 74HC595 DS引脚 定义为信号输出

  HC595_submit(); // 74HC595 初始状态更新

  myservo.attach(pin_window); // 初始化棚窗引脚
  closeVentilate();

  Serial.println("Wifi连接中...");
  delay(5000); // 延时5s 等待ESP8266连接
  Serial.println("WiFi连接完成");

  pinMode(pin_ventilate, OUTPUT); // 通风引脚 定义为信号输出
  digitalWrite(pin_ventilate, LOW); // 通风引脚电平置低（高电平触发）

  sensors_1.begin(); // 一层土壤温度采集初始化函数
  sensors_2.begin(); // 二层土壤温度采集初始化函数

  pinMode(pin_ppm_power,OUTPUT); // PM2.5采集状态指示引脚 定义为输出
  pinMode(pin_ppm, INPUT); // PM2.5含量采集引脚 定义为输入

  pinMode(pin_pH_power,OUTPUT);  // 土壤pH值采集状态指示引脚 定义为输出

  pinMode(pin_co,INPUT);  // CO等可燃气体浓度采集引脚 定义为输入

  checkIn(); // 设备登录
}

void loop() {
  // 每一定时间查询一次设备在线状态 同时替代心跳
  if(millis() - lastCheckStatusTime > postingInterval) {
    checkStatus();
  }
  // checkout 50ms后 checkin
  if(checkoutTime != 0 && millis() - checkoutTime > 50) {
    checkIn();
    checkoutTime = 0;
  }

  readPH(); // 读取土壤PH值

  // 每隔一定时间上传一次接口数据
  if(millis() - lastUpdateTime > updateInterval) {
    int val1; // 接口1 通风自动模式标志位
    int val2; // 接口2 一层土壤升温自动模式标志位
    int val3; // 接口3 二层土壤升温自动模式标志位
    int val4; // 接口4 补光自动模式标志位
    int val5; // 接口5 火灾报警模式标志位
    int val6; // 接口6 大风报警模式标志位

    val1 = auto_ventilate; // 通风自动模式标志位
    val2 = auto_heat_1; // 一层土壤升温自动模式标志位
    val3 = auto_heat_2; // 二层土壤升温自动模式标志位
    val4 = auto_light; // 补光自动模式标志位
    val5 = policeFire; // 火灾报警模式标志位
    val6 = policeWind; // 大风报警模式标志位

    if(auto_water == 1) {
      if(readSoilHum_1() > set_soilHum) {
        openWater();
      } else {
        closeWater();
      }
    }

    if(auto_ventilate == 1) {
      if(readHum() > set_hum) {
        openVentilate();
      } else {
        closeVentilate();
      }
    }

    if(auto_heat_1 == 1) {
      if(readSoilTem_1() < set_soilTem_1) {
        openHeat_1();
      } else {
        closeHeat_1();
      }
    }

    if(auto_heat_2 == 1) {
      if(readSoilTem_2() < set_soilTem_2) {
        openHeat_2();
      } else {
        closeHeat_2();
      }
    }

    if(auto_light == 1) {
      if(readLight() < set_light) {
        openLight();
      } else {
        closeLight();
      }
    }

    if(policeFire == 1) {
      if(readCO() > 1.5) {
        openFire();
      } else {
        closeFire();
      }
    }

    if(policeWind == 1) {
      if(readWindSpd() > 10) {
        openWind();
      } else {
        closeWind();
      }
    }

    DHT_OUT.read11(DHT11_PIN_OUT); // 空气温湿度读取
    allDataFun();

    HC595_submit();

    update(DEVICEID, INPUTID1, val1, INPUTID2, val2, INPUTID3, val3, INPUTID4, val4, INPUTID5, val5, INPUTID6, val6); // 上传各标志位
    lastUpdateTime = millis();
  }
  // 读取串口字符信息
  while(Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    // 打印收到的JSON字符串
    // Serial.println("test_inputString:");
    // Serial.println(inputString);
    // 接收下位机与中间件连接信息
    if(inputString.indexOf("WELCOME") != -1)
    {
      checkOut();
      checkoutTime = millis();
    }
    else if(inputString.indexOf("connected") != -1)
    {
      checkIn();
    }
    if(inputString.indexOf("C\"") != -1 && inputString.indexOf("\",\"T") != -1) {
      String order = inputString.substring(inputString.indexOf("C\"") + 4, inputString.indexOf("\",\"T"));
      Serial.println(order);
          // 根据web端指令操作执行器
      if(order.indexOf("opt") != -1) {
        if(order.indexOf("Tem1") != -1) {
          if(order == "optTem1_18") // 一层土壤温度下限设定
          {
            set_soilTem_1 = 18;
          }
          else if(order == "optTem1_22")
          {
            set_soilTem_1 = 22;
          }
          else if(order == "optTem1_26")
          {
            set_soilTem_1 = 26;
          }
        }
        else if(order.indexOf("Tem2") != -1) {
          if(order == "optTem2_18") // 二层土壤温度下限设定
          {
            set_soilTem_2 = 18;
          }
          else if(order == "optTem2_22")
          {
            set_soilTem_2 = 22;
          }
          else if(order == "optTem2_26")
          {
            set_soilTem_2 = 26;
          }
        }
        else  if(order.indexOf("Hum") != -1) {
          if(order == "optHum_40") // 空气湿度上限设定
          {
            set_hum = 40;
          }
          else if(order == "optHum_60")
          {
            set_hum = 60;
          }
          else if(order == "optHum_80")
          {
            set_hum = 80;
          }
        }
        else  if(order.indexOf("Li") != -1) {
          if(order == "optLi_40") // 光强下限设定
          {
            set_light = 40;
          }
          else if(order == "optLi_60")
          {
            set_light = 60;
          }
          else if(order == "optLi_80")
          {
            set_light = 80;
          }
        }
        else  if(order.indexOf("So") != -1) {
          if(order == "optSo_10") // 加权土壤湿度下限设定
          {
            set_soilHum = 10;
          }
          else if(order == "optSo_30")
          {
            set_soilHum = 30;
          }
          else if(order == "optSo_50")
          {
            set_soilHum = 50;
          }
        }
      } 
      else if(order.indexOf("ex") != -1)
      {
        if(order == "exOpHe_1") // 一层土壤加热 开启
        {
          openHeat_1();
        }
        else if(order == "exCloHe_1") // 一层土壤加热 关闭
        {
          closeHeat_1();
        }
        else if(order == "exOpHe_2") // 二层土壤加热 开启
        {
          openHeat_2();
        }
        else if(order == "exCloHe_2") // 二层土壤加热 关闭
        {
          closeHeat_2();
        }
        else if(order == "exOpWa") // 灌溉 开启
        {
          openWater();
        }
        else if(order == "exCloWa") // 灌溉 关闭
        {
          closeWater();
        }
        else if(order == "exOpVen") // 通风 开启
        {
          openVentilate();
        }
        else if(order == "exCloVen") // 通风 关闭
        {
          closeVentilate();
        }
        else if(order == "exOpLi") // 补光 开启
        {
          openLight();
        }
        else if(order == "exCloLi") // 补光 关闭
        {
          closeLight();
        }
      }
      else if(order.indexOf("alt") != -1) 
      { 
        if(order == "altAuVen") // 通风 自动模式开启
        {
          auto_ventilate = 1;
          closeVentilate();
        }
        else if(order == "altHaVen") // 通风 自动模式关闭
        {
          auto_ventilate = 0;
          closeVentilate();
        }
        else if(order == "altAuHe_1") // 一层土壤升温 自动模式开启
        {
          auto_heat_1 = 1;
          closeHeat_1();
        }
        else if(order == "altHaHe_1") // 一层土壤升温 自动模式关闭
        {
          auto_heat_1 = 0;
          closeHeat_1();
        }
        else if(order == "altAuHe_2") // 二层土壤升温 自动模式开启
        {
          auto_heat_2 = 1;
          closeHeat_2();
        }
        else if(order == "altHaHe_2") // 二层土壤升温 自动模式关闭
        {
          auto_heat_2 = 0;
          closeHeat_2();
        }
        else if(order == "altAuLi") // 补光 自动模式开启
        {
          auto_light = 1;
          closeLight();
        }
        else if(order == "altHaLi") // 补光 自动模式关闭
        {
          auto_light = 0;
          closeLight();
        }
        else if(order == "altAuWa") // 灌溉 自动模式开启
        {
          auto_water = 1;
          closeWater();
        }
        else if(order == "altHaWa") // 灌溉 自动模式关闭
        {
          auto_water = 0;
          closeWater();
        }
        else if(order == "altOpAlaF") // 火灾报警模式开启
        {
          policeFire = 1;
        }
        else if(order == "altCloAlaF") // 火灾报警模式关闭
        {
          policeFire = 0;
          closeFire();
        }
        else if(order == "altOpAlaW") // 大风报警模式开启
        {
          policeWind = 1;
        }
        else if(order == "altCloAlaW") // 大风报警模式关闭
        {
          policeWind = 0;
          closeWind();
        }
      }
      delay(50);
      HC595_submit();
    }
  }
}

/*
  设备登录
  数据格式 {"M":"checkin","ID":"xx1","K":"xx2"}\n
*/
void checkIn() {
  Serial.print("{\"M\":\"checkin\",\"ID\":\"");
  Serial.print(DEVICEID);
  Serial.print("\",\"K\":\"");
  Serial.print(APIKEY);
  Serial.print("\"}\n");
}

/*  
  强制设备下线 用于消除设备掉线延时
  数据格式 {"M":"checkout","ID":"xx1","K":"xx2"}\n  
*/
void checkOut() {
  Serial.print("{\"M\":\"checkout\",\"ID\":\"");
  Serial.print(DEVICEID);
  Serial.print("\",\"K\":\"");
  Serial.print(APIKEY);
  Serial.print("\"}\n");
}

/*
  查询设备在线状态
  数据格式 {"M":"status"}\n 
*/
void checkStatus() {
  Serial.print("{\"M\":\"status\"}\n");
  lastCheckStatusTime = millis();
}

/*
  与用户对话
  数据格式 {"M":"say","ID":"xx1","C":"xx2","SIGN":"xx3"}\n  
*/
void sayToClient(String client_id, String content, String sign) {
  Serial.print("{\"M\":\"say\",\"ID\":\"U");
  Serial.print(client_id);
  Serial.print("\",\"C\":\"");
  Serial.print(content);
  Serial.print("\",\"SIGN\":\"");
  Serial.print(sign);
  Serial.print("\"}\r\n");
  lastCheckInTime = millis();
}

/*
  更新接口数据
  数据格式 {"M":"update","ID":"xx1","V":{"id1":"value1",...}}\n  
*/
void update(String did, String inputid1, int value1, String inputid2, int value2, String inputid3, int value3, String inputid4, int value4, String inputid5, int value5, String inputid6, int value6) {
  Serial.print("{\"M\":\"update\",\"ID\":\"");
  Serial.print(did);
  Serial.print("\",\"V\":{\"");

  Serial.print(inputid1);
  Serial.print("\":\"");
  Serial.print(value1);
  Serial.print("\",\"");

  Serial.print(inputid2);
  Serial.print("\":\"");
  Serial.print(value2);
  Serial.print("\",\"");

  Serial.print(inputid3);
  Serial.print("\":\"");
  Serial.print(value3);
  Serial.print("\",\"");

  Serial.print(inputid4);
  Serial.print("\":\"");
  Serial.print(value4);
  Serial.print("\",\"");

  Serial.print(inputid5);
  Serial.print("\":\"");
  Serial.print(value5);
  Serial.print("\",\"");

  Serial.print(inputid6);
  Serial.print("\":\"");
  Serial.print(value6);
  Serial.println("\"}}");
}

/*
  将各采集因子对应数据集成 JSON字符串 并返回
  参数说明：  东经 北纬 空气温度（°C） 空气湿度（%RH） 一层土壤温度（°C） 一层土壤湿度（%） 二层土壤温度（°C） 二层土壤湿度（%）
            粉尘浓度值（ug/m3） pm2.5指数（index） 土壤PH值（mol·L） CO等可燃气体浓度值（ppm） 风速（m/s） 当前降雨量（mm） 光强（cd/m2） 紫外线强度（W）
*/
void allDataFun() {
  sayToClient(client_id, String(readTem()), "Tem");
  sayToClient(client_id, String(readHum()), "Hum");
  sayToClient(client_id, String(readSoilTem_1()), "soilTem_1");
  sayToClient(client_id, String(readSoilHum_1()), "soilHum_1");
  sayToClient(client_id, String(readSoilTem_2()), "soilTem_2");
  sayToClient(client_id, String(readSoilHum_2()), "soilHum_2");
  sayToClient(client_id, String(readPpm()), "ppm");
  sayToClient(client_id, String(readPH()), "pH");
  sayToClient(client_id, String(readCO()), "CO");
  sayToClient(client_id, String(readWindSpd()), "windSpd");
  sayToClient(client_id, String(readRain()), "rain_cur");
  sayToClient(client_id, String(readLight()), "light");
  sayToClient(client_id, "2", "uRays");
}

//======================= 执行器 控制函数 =======================
// 开启通风
void openVentilate() {
  digitalWrite(pin_ventilate, HIGH); // 通风引脚电平置高（高电平触发）
  myservo.write(90); // 舵机写入旋转角度90°
}

// 关闭通风
void closeVentilate() {
  digitalWrite(pin_ventilate, LOW); // 通风引脚电平置低（高电平触发）
  myservo.write(0); // 舵机写入旋转角度0°
}

// 开启一层土壤升温
void openHeat_1() {
  HC595_update(pin_heat_1, 1); // 一层土壤升温引脚电平置高（高电平触发）
}

// 关闭通一层土壤升温
void closeHeat_1() {
  HC595_update(pin_heat_1, 0); // 一层土壤升温引脚电平置低（高电平触发）
}

// 开启二层土壤升温
void openHeat_2() {
  HC595_update(pin_heat_2, 1); // 二层土壤升温引脚电平置高（高电平触发）
}

// 关闭通二层土壤升温
void closeHeat_2() {
  HC595_update(pin_heat_2, 0); // 二层土壤升温引脚电平置低（高电平触发）
}

// 开启灌溉
void openWater() {
  HC595_update(pin_water, 1); // 灌溉引脚电平置高（高电平触发）
}

// 关闭灌溉
void closeWater() {
  HC595_update(pin_water, 0); // 灌溉引脚电平置低（高电平触发）
}

// 开启白炽补光灯
void openLight() {
  HC595_update(pin_addLight, 1); // 补光引脚电平置高
}

// 关闭白炽补光灯
void closeLight() {
  HC595_update(pin_addLight, 0); // 补光引脚电平置低
}

// 开启火灾声光报警
void openFire() {
  HC595_update(pin_policeFire, 0); // 火灾报警引脚电平置低
}

// 关闭火灾声光报警
void closeFire() {
  HC595_update(pin_policeFire, 1); // 火灾报警引脚电平置高
}

// 开启大风声光报警
void openWind() {
  HC595_update(pin_policeWind, 0); // 大风报警引脚电平置低
}

// 关闭大风声光报警
void closeWind() {
  HC595_update(pin_policeWind, 1); // 大风报警引脚电平置高
}
//======================= 传感器 控制函数 =======================
// 读取模拟量(拓展接口) 当前降雨量
String readRain() {
  int rain_cur = ads.readADC_SingleEnded(pin_rain_cur);
  String rain_cur_str = "";
  if (rain_cur > 4400) {
    rain_cur_str = "大暴雨";
  } else if (rain_cur > 3800) {
    rain_cur_str = "暴雨";
  } else if (rain_cur > 3600) {
    rain_cur_str = "大雨";
  } else if (rain_cur > 3000) {
    rain_cur_str = "中雨";
  } else if (rain_cur > 2600) {
    rain_cur_str = "小雨";
  } else {
    rain_cur_str = "晴";
  }
  return rain_cur_str;
}

// 读取模拟量(拓展接口) 太阳光强
float readLight() {
  float light = ((ads.readADC_SingleEnded(pin_light) / 21) * 0.106);
  return light;
}

// 读取模拟量(拓展接口) 一层土壤湿度值
float readSoilHum_1() {
  int temp = ads.readADC_SingleEnded(pin_soilHum_1);
  if(temp < 0) {
    temp = 0;
  }
  float soilHum_1 = 100 - ((temp - 6000) / 160);
  if(soilHum_1 < 0) {
    soilHum_1 = 0;
  }
  return soilHum_1;
}

// 读取模拟量(拓展接口) 二层土壤湿度值
float readSoilHum_2() {
  int temp = ads.readADC_SingleEnded(pin_soilHum_2);
  if(temp < 0) {
    temp = 0;
  }
  float soilHum_2 = 100 - ((temp - 6000) / 160);
  if(soilHum_2 < 0) {
    soilHum_2 = 0;
  }
  return soilHum_2;
}

// 读取模拟量 一层土壤温度值
float readSoilTem_1() {
  sensors_1.requestTemperatures(); // 采集一层土壤温度数据
  return sensors_1.getTempCByIndex(0);
}

// 读取模拟量 二层土壤温度值
float readSoilTem_2() {
  sensors_2.requestTemperatures(); // 采集二层土壤温度数据
  return sensors_2.getTempCByIndex(0);
}

// 读取模拟量 空气温度
float readTem() {
  return DHT_OUT.temperature;
}

// 读取模拟量 空气湿度
float readHum() {
  return DHT_OUT.humidity;
}

// 读取模拟量 PM2.5指数（中国标准）
int readPpm() {
  float dustDensity = 0;
  int ppm;
  for(int i = 0; i < 10; i++) {
    digitalWrite(pin_ppm_power, LOW);
    delayMicroseconds(180);
    dustDensity += ((analogRead(pin_ppm) * 0.0049 * 0.1667) - 0.1) * 1000; // 读取粉尘浓度值
    delayMicroseconds(20);
    digitalWrite(pin_ppm_power, HIGH);
  }
  dustDensity /= 10;
  // 将粉尘浓度值转换成PM2.5指数（中国标准）
  if(dustDensity < 35) {
    ppm = dustDensity * 1.4286;
  }
  else if(dustDensity < 150) {
    ppm = (dustDensity - 35) * 1.25 + 50;
  }
  else if(dustDensity < 250) {
    ppm = (dustDensity - 150) * 1 + 200;
  }
  else{
    ppm = (dustDensity - 250) * 0.8 + 300;
  }
  ppm -= 50;
  if(ppm < 0) {
    ppm = 0;
  }
  return ppm;
}

// 读取模拟量 土壤pH值
float readPH() {
  float pH;
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue, voltage;
  if(millis()-samplingTime > samplingInterval) {
    pHArray[pHArrayIndex++] = analogRead(pin_pH);
    if(pHArrayIndex == ArrayLenth)
      pHArrayIndex = 0;
    voltage = avergeArray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = -5.8887 * voltage + Offset;
    samplingTime = millis();
  }
  if(pHValue <= 0.0)
    pHValue = 0.0;
  if(pHValue > 14.0)
    pHValue = 14.0;
  if(millis() - printTime > printInterval) {
    pH = pHValue;
    digitalWrite(pin_pH_power, digitalRead(pin_pH_power) ^ 1);
    printTime = millis();
  }
  return pH;
}

// 土壤pH值读取辅助函数
double avergeArray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if(number <= 0) {
    return 0;
  }
  if(number < 5) {
    for(i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if(arr[0] < arr[1]) {
      min = arr[0];
      max = arr[1];
    } else {
      min = arr[1];
      max = arr[0];
    }
    for(i = 2; i < number; i++) {
      if(arr[i] < min) {
        amount += min;
        min = arr[i];
      } else { 
        if(arr[i] > max) {
          amount += max;
          max = arr[i];
        } else {
          amount += arr[i];
        }
      }
    }
    avg = (double) amount / (number - 2);
  }
  return avg;
}

// 读取CO等可燃气体浓度值
float readCO() {
  float co = analogRead(pin_co) * 0.0049 - 0.4; // 读取CO等可燃气体浓度引脚模拟值并转换为浓度值
  if(co < 0)
    co = 0;
  return co;
}

// 读取风速值
float readWindSpd() { 
  float windSpd;
  windSpd = 100 * (analogRead(pin_windSpd) * (5.0 / 1024.0)) / 3.6 - 24; // 采集风速探测引脚模拟值并转换成风速
  if(windSpd < 0)
    windSpd = 0;
  return windSpd;
}

// 将引脚HC595_pin状态置为state 并更新
void HC595_update(int HC595_pin, int state) {
  HC595_data[HC595_pin] = state;
  HC595_submit();
}

// 更新74HC595各引脚状态
void HC595_submit() {
  digitalWrite(pin_HC595_big, LOW);
  for(int i = 0; i < 8; i++) {
    digitalWrite(pin_HC595_push, LOW);
    digitalWrite(pin_HC595_datain, HC595_data[i]);
    digitalWrite(pin_HC595_push, HIGH);
  }
  digitalWrite(pin_HC595_big, HIGH);
}