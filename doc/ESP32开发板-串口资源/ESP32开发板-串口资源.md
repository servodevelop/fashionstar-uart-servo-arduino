# ESP32开发板-串口资源

## ESP32板载资源 - 串口

ESP32一共有三组UART资源

| 功能     | GPIO    |
| -------- | ------- |
| UART0 Tx | GPIO 1  |
| UART0 Rx | GPIO 3  |
| UART1 Tx | GPIO 10 |
| UART1 Rx | GPIO 9  |
| UART2 Tx | GPIO 17 |
| UART2 Rx | GPIO 16 |







## NodeMCU32s



### 硬件资源

[NodeMCU-32S 引脚说明书](http://www.1zlab.com/wiki/micropython-esp32/pins-and-gpio/#nodemcu-32s)

![](image/NodeMCU32S管脚图.jpg)

| 功能     | GPIO    | 板载标记 |
| -------- | ------- | -------- |
| UART0 Tx | GPIO 1  | TX       |
| UART0 Rx | GPIO 3  | RX       |
| UART1 Tx | GPIO 10 | D3       |
| UART1 Rx | GPIO 9  | D2       |
| UART2 Tx | GPIO 17 | 17       |
| UART2 Rx | GPIO 16 | 16       |



## ESP32 Arduino 串口资源



在 ESP32 芯片默认配置情况下 6-12 GPIO 配置 FLASH 的接口，不能被其他程序使用，所以默认 Serial1 不能直接使用`Serial1`， 需要调用库

```

```



