# rmt_serial
esp32-c3 serial base on rmt

ESP32-C3硬件串口资源较少, 有的项目需要较多的串口, 有一个arduino库实现了用GPIO模拟的串口, SoftwareSerial. 这里给出一个基于RMT外设的实现, 接收时避免频繁进入中断, 发送时不用禁止中断以保证发送时序.RMT（Remote Control）外设不仅可以用于红外遥控、LED灯条控制等场景，还可以通过灵活配置实现UART串行通信功能.

In scenarios where the ESP32-C3 has limited hardware UART resources and certain projects require multiple serial ports, an Arduino library named SoftwareSerial exists that enables the simulation of UART communication using GPIO pins. Here, we present an implementation based on the RMT (Remote Control) peripheral which, in addition to its applications in infrared remote control and LED strip control, can be flexibly configured to emulate UART serial communication functionality. With this RMT-based solution, during reception, it avoids frequent entry into interrupts, and for transmission, it does not necessitate disabling interrupts to maintain accurate timing sequences, thus ensuring reliable serial communication without compromising system responsiveness.
