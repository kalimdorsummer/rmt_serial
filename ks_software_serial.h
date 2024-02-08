#ifndef KS_SOFTWARE_SERIAL_H__
#define KS_SOFTWARE_SERIAL_H__
#include <Arduino.h>
#include <stdint.h>
#include <driver/gpio.h>
#include "driver/rmt.h"
#include <vector>
#include <array>

#define BAUD_9600_US	104


#define IDLE_TIMEOUT_US	25

enum{
	KS_SWS_BASED_RMT,
	KS_SWS_BASED_GPIO,
};

class ks_software_serial_c {
public:
	ks_software_serial_c();
	~ks_software_serial_c();
	void begin(int baud = 9600, int cfg = -1, int rx_pin = -1, int tx_pin = -1, size_t buf_len = 64, uint8_t sws_base = -1);
	void end();
	size_t write(const uint8_t* buffer, size_t size);

    void perform_work();	// call in a loop

    typedef void (*timeout_cb_t)(void *data, uint32_t data_len);
    timeout_cb_t timeout_callback;
    void set_timeout_callback(timeout_cb_t cb_func);		// register user callback, after the last bit 25ms, will callback
	void start_rx();
	void stop_rx();
	void flush();
	uint32_t * get_cap_micro_buf();
	uint16_t * get_cap_delta_buf();
	uint8_t * get_cap_level_buf();
	size_t get_cap_idx();

private:
	static void IRAM_ATTR gpio_isr_handler(void* arg);
	void rmt_tx_init(void);
	void rmt_rx_init(void);
	int rmt_us_to_pulse_cnt(int us);
	int rmt_send_raw(std::vector<uint16_t> &buf, std::vector<uint8_t> &level);

	volatile uint32_t* m_rxReg;
	uint32_t m_rxBitMask;
	uint32_t last_milli;
	bool m_has_int;


	size_t parse_out_max;
	size_t cap_edge_max;
	int cap_idx;
	uint32_t *cap_micro_buf;	
	uint16_t *cap_delta_buf;	
	uint8_t *cap_level_buf;

	uint8_t *parse_out_buf;


	RingbufHandle_t rb;
	int m_rx_pin;
	int m_tx_pin;

	int rmt_rx_channel;
	int rmt_tx_channel;

	uint8_t rx_based_on;	// 0 is rmt, 1 is gpio
	uint8_t tx_based_on;	

	bool rx_onoff;
	
};

#endif
