/* 
 * 设置舵机的角度(多圈模式)
 * 提示: 拓展板上电之后, 记得按下Arduino的RESET按键
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2021/02/22
 */
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

SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
FSUS_Protocol protocol(BAUDRATE); //协议
FSUS_Servo uservo(SERVO_ID, &protocol); // 创建舵机

unsigned long interval;  // 运行周期 单位ms 
unsigned int t_acc;     // 加速时间 单位ms
unsigned int t_dec;     // 减速时间 单位ms
float velocity;         // 目标转速 单位°/s

/* 等待并报告当前的角度*/
void waitAndReport(){
    uservo.wait();          // 等待舵机旋转到目标角度
    softSerial.println("Real Angle = " + String(uservo.curRawAngle, 1) + " Target Angle = "+String(uservo.targetRawAngle, 1));
    delay(2000); // 暂停2s
}

void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE); // 初始化软串口的波特率
    protocol.init(); // 通信协议初始化
    uservo.init(); //舵机角度初始化
    softSerial.println("Set Servo Angle");
}

void loop(){
    softSerial.println("Set Angle = 900°");
    uservo.setRawAngleMTurn(900.0);  // 设置舵机的角度
    waitAndReport();

    softSerial.println("Set Angle = -900.0°");
    uservo.setRawAngleMTurn(-900.0);
    waitAndReport();

    softSerial.println("Set Angle = 900° - Set Interval = 10s");
    interval = 10000;
    t_acc = 100;
    t_dec = 100;
    uservo.setRawAngleMTurnByInterval(900, interval, t_acc, t_dec, 0);
    waitAndReport();

    softSerial.println("Set Angle = -900° - Set Velocity = 200°/s");
    velocity = 200.0;
    t_acc = 100;
    t_dec = 100;
    uservo.setRawAngleMTurnByVelocity(-900, velocity, t_acc, t_dec, 0);
    waitAndReport();
}