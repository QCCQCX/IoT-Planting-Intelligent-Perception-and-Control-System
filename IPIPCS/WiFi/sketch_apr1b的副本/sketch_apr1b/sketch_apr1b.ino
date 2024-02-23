#include <SoftwareSerial.h>
SoftwareSerial mySerial(2,3); //设置模拟串口针脚(RX, TX)

String WiFiSSID = "qcc"; // 路由器名称
String WiFiPASSWORD = "quchenxi"; // WiFi密码

int flag = 0;
void setup() {
  Serial.begin(9600);
  mySerial.begin(115200);
}
void loop() {
  if(flag == 0) { 
    Serial.println("waiting start");
    // ESP8266通电启动等待
    delay(10000);
    Serial.println("setting start");
    // 如果是透传模式 退出透传
    Serial.println("exit pass-through mode");
    mySerial.print("+++");
    delay(1000);
    mySerial.print("AT\r\n");
    delay(1000);
    printmssage();
    // 关闭回显命令
    mySerial.print("ATE0\r\n");
    delay(1000);
    printmssage();
    // 设置WiFi应用模式为兼容模式
    Serial.println("choose station mode");
    mySerial.print("AT+CWMODE=3\r\n");  
    delay(2000);
    printmssage();
    // 连接到无线路由器
    Serial.println("connect wireless router");
    mySerial.print("AT+CWJAP=\"");
    mySerial.print(WiFiSSID);
    mySerial.print("\",\"");
    mySerial.print(WiFiPASSWORD);
    mySerial.print("\"\r\n");
    delay(20000); // 此处需根据路由器连接速度设置delay的时间 可适当加长
    printmssage();
    // 设置为单连接
    mySerial.print("AT+CIPMUX=0\r\n");
    delay(1000);
    printmssage();
    // 设置模块传输模式为透传模式
    Serial.println("choose pass-through mode");
    mySerial.print("AT+CIPMODE=1\r\n");
    delay(1000);
    printmssage();
    // 连接贝壳物联服务器
    Serial.println("connect bigiot.net and save translink");
    mySerial.print("AT+SAVETRANSLINK=1,\"121.42.180.30\",8181,\"TCP\"\r\n");
    delay(20000); // 此处需根据网路连接速度设置delay的时间 可适当加长
    printmssage();
    // 重启
    Serial.println("restarting...");
    mySerial.print("AT+RST\r\n");
    flag=1;
  }
  printmssage();
}
void printmssage() {
  if (mySerial.available()) {
    Serial.println(mySerial.readStringUntil('\n'));
  }
}
