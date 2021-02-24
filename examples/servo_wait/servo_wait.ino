/* 
 * 测试wait()函数,轮询角度直到舵机旋转到目标位置
 * 提示: 拓展板上电之后, 记得按下Arduino的RESET按键
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/05/13
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

void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE); // 初始化软串口的波特率
    protocol.init(); // 通信协议初始化
    uservo.init(); //舵机初始化

    softSerial.println("Test Wait");
}

void loop(){
    softSerial.println("Set Angle = 90.0");
    uservo.setAngle(90.0); // 设置舵机的角度
    uservo.wait();
    softSerial.println("Real Angle = "+String(uservo.curAngle, 2));
     
    softSerial.println("Set Angle = -90.0");
    uservo.setAngle(-90);
    uservo.wait();
    softSerial.println("Real Angle = "+String(uservo.curAngle, 2));
}