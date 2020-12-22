/*
 * 舵机数据读取实验
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/11/14
 **/
#include <SoftwareSerial.h>
#include "FashionStar_UartServoProtocol.h" // 串口总线舵机通信协议
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800

// 配置
#define SERVO_ID 4 //舵机ID号
#define BAUDRATE 115200 // 波特率

SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
FSUS_Protocol protocol(BAUDRATE); //协议
FSUS_Servo uservo(SERVO_ID, &protocol); // 创建舵机

// 读取数据
uint16_t voltage;       // 电压 mV
uint16_t current;       // 电流 mA
uint16_t power;         // 功率 mW
uint16_t temperature;   // 温度 ℃

void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE);
    protocol.init(); // 舵机通信协议初始化
    uservo.init(); // 串口总线舵机初始化

    softSerial.println("Start To Test Servo Data Read \n"); // 打印日志
    uservo.setAngle(-25.0,  1000, 200); // 设置舵机角度(限制功率)
}

void loop(){
    voltage = uservo.queryVoltage();
    current = uservo.queryCurrent();
    power = uservo.queryPower();
    temperature = uservo.queryTemperature();
    softSerial.println("voltage: "+String((float)voltage, 1)+" mV\n");
    delay(100);
    softSerial.println("current: "+String((float)current, 1)+" mA\n");
    delay(100);
    softSerial.println("power: "+String((float)power, 1)+" mW\n");
    delay(100);
    softSerial.println("temperature: "+String((float)temperature, 1)+" Celsius\n");
    delay(1000);
}