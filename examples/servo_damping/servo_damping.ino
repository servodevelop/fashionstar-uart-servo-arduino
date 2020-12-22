/*
 * 设置舵机为阻尼模式
 * 调整参数`DAMPING_POWER`感受不同的阻尼力
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
    uservo.init(); // 舵机初始化

    softSerial.println("Set Servo Mode To Dammping");
    uservo.setDamping(DAMPING_POWER);
}

void loop(){
    // TODO;
}
