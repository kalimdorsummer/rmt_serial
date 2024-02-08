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