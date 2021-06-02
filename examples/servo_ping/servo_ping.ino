/*
 * 舵机通讯检测
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/04/23
 **/
#if defined(ARDUINO_ARCH_AVR)
#include <SoftwareSerial.h>
#endif

#include "FashionStar_UartServoProtocol.h" // 串口总线舵机通信协议
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// 配置
#define SERVO_ID 0 //舵机ID号
#define BAUDRATE 115200 // 波特率

#if defined(ARDUINO_ARCH_AVR)
// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800
SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
#elif defined(ARDUINO_ARCH_ESP32)
#define DEBUG_SERIAL_BAUDRATE 115200
#endif 

FSUS_Protocol protocol(BAUDRATE); //协议
FSUS_Servo uservo(SERVO_ID, &protocol); // 创建舵机

void setup(){
    protocol.init(); // 舵机通信协议初始化
    uservo.init(); // 串口总线舵机初始化

    // 调试串口初始化
#if defined(ARDUINO_ARCH_AVR)
    softSerial.begin(SOFT_SERIAL_BAUDRATE);
    softSerial.println("Start To Ping Servo\n"); // 打印日志
#elif defined(ARDUINO_ARCH_ESP32)
    Serial.begin(DEBUG_SERIAL_BAUDRATE);
    Serial.println(("Start To Ping Servo\n"));
#endif

    
}

void loop(){
    bool isOnline = uservo.ping(); // 舵机通讯检测
    String message = "servo #"+String(uservo.servoId,DEC); // 日志输出
    if(isOnline){
        // 调试串口初始化
    #if defined(ARDUINO_ARCH_AVR)
        softSerial.println(message+" is online.");
    #elif defined(ARDUINO_ARCH_ESP32)
        Serial.println(message+" is online.");
    #endif
    }else{
        // 调试串口初始化
    #if defined(ARDUINO_ARCH_AVR)
        softSerial.println(message+" is offline.");
    #elif defined(ARDUINO_ARCH_ESP32)
        Serial.println(message+" is offline.");
    #endif
    }
    // 等待1s
    delay(1000);
}
