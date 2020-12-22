/*
 * 舵机角度回读实验
 * 用手掰动舵机, 角度回读并将角度读数通过SPI发送
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/04/23
 **/
#include <SoftwareSerial.h>
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800

// 配置参数
#define BAUDRATE 115200 // 波特率

#define SERVO_ID 0 //舵机ID号
#define DAMPING_POWER 800 // 阻尼模式下的功率(单位mW) 500,800,1000

SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
FSUS_Protocol protocol(BAUDRATE); //协议
FSUS_Servo uservo(SERVO_ID, &protocol); // 创建舵机

void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE);
    protocol.init(); // 通信协议初始化
    
    uservo.init(); //舵机角度初始化
    uservo.setDamping(DAMPING_POWER);

    softSerial.println("Query Servo Angle");
}

void loop(){
    // 舵机角度查询 (更新角度)
    uservo.queryAngle(); 
    // 日志输出
    String message = "Status Code: " + String(uservo.protocol->responsePack.recv_status, DEC) + " servo #"+String(uservo.servoId, DEC) + " , Current Angle = "+String(uservo.curAngle, 1)+" deg";
    softSerial.println(message);
}