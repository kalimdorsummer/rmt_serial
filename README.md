# rmt_serial
esp32-c3 serial base on rmt

ESP32-C3硬件串口资源较少, 有的项目需要较多的串口, 有一个arduino库实现了用GPIO模拟的串口, SoftwareSerial. 这里给出一个基于RMT外设的实现, 接收时避免频繁进入中断, 发送时不用禁止中断以保证发送时序.RMT（Remote Control）外设不仅可以用于红外遥控、LED灯条控制等场景，还可以通过灵活配置实现UART串行通信功能.目前仅支持9600波特率,1位起始位, 1位停止位.

In scenarios where the ESP32-C3 has limited hardware UART resources and certain projects require multiple serial ports, an Arduino library named SoftwareSerial exists that enables the simulation of UART communication using GPIO pins. Here, we present an implementation based on the RMT (Remote Control) peripheral which, in addition to its applications in infrared remote control and LED strip control, can be flexibly configured to emulate UART serial communication functionality. With this RMT-based solution, during reception, it avoids frequent entry into interrupts, and for transmission, it does not necessitate disabling interrupts to maintain accurate timing sequences, thus ensuring reliable serial communication without compromising system responsiveness.Currently, support is limited to a baud rate of 9600 with 1 start bit and 1 stop bit.
```cpp
#include "ks_software_serial.h"
#include "soc/usb_serial_jtag_reg.h"

#define TEST_UART_RX				2
#define TEST_UART_TX				3

class ks_software_serial_c test_uart;
static void test_uart_callback(void *data, uint32_t data_len)
{
	Serial.println("test_uart_callback");
	uint8_t *data_buf = (uint8_t *)data;
	for (int i = 0; i < data_len; i++) {
		Serial.printf("%02x ", data_buf[i]);
	}
	Serial.println();

	test_uart.write(data_buf, data_len);	// send back

	test_uart.flush();
}
void setup() {
	// initialize serial communication at 115200 bits per second:
	static const int SEND_BUF_SIZE = 1024;
	Serial.setTxBufferSize(SEND_BUF_SIZE); 	// If set to zero, driver will not use TX buffer, TX function will block task until all data have been sent out.
	
	Serial.begin ( 115200 );
	Serial.println("start");


	test_uart.begin(9600, -1, TEST_UART_RX, TEST_UART_TX, 64, KS_SWS_BASED_RMT);
	//test_uart.begin(9600, -1, TEST_UART_RX, TEST_UART_TX, 64, KS_SWS_BASED_GPIO);	// you can use gpio if rmt channel is not enough, not recommended
	//CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_DP_PULLUP);	// for io19

	test_uart.set_timeout_callback(test_uart_callback);

	test_uart.start_rx();
} 



void loop() {
	test_uart.perform_work();
}
```
