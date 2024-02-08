#include "ks_software_serial.h"


#define LOG_INFO Serial.printf
#define LOG_PURE Serial.printf

using namespace std;
static const int RMT_SEND_MAX_LEN = 512;
static const int RMT_CLK_DIV = 80;			// 80: every pulse is 1 us, 160: every pulse is 2 us

#define RMT_RX_CHANNEL_ENCODING_START (SOC_RMT_CHANNELS_PER_GROUP-SOC_RMT_TX_CANDIDATES_PER_GROUP)
#define RMT_TX_CHANNEL_ENCODING_END   (SOC_RMT_TX_CANDIDATES_PER_GROUP-1)

// static const int RX_CHANNEL = RMT_RX_CHANNEL_ENCODING_START + 1;
static int rmt_tx_channel_valid = RMT_TX_CHANNEL_ENCODING_END;
static int rmt_rx_channel_valid = RMT_RX_CHANNEL_ENCODING_START;
ks_software_serial_c::ks_software_serial_c()
{
	m_rxReg = NULL;
	m_rxBitMask = 0;
	last_milli = 0;
	m_has_int = false;	

	parse_out_max = 0;
	cap_edge_max = 0;
	cap_idx = 0;
	cap_micro_buf = NULL;
	cap_delta_buf = NULL;
	cap_level_buf = NULL;
	parse_out_buf = NULL;

	timeout_callback = NULL;

	rb = NULL;
	rmt_rx_channel = -1;
	rmt_tx_channel = -1;
	m_rx_pin = -1;
	m_tx_pin = -1;

	rx_based_on = -1;
	tx_based_on = -1;

	rx_onoff = false;
}

ks_software_serial_c::~ks_software_serial_c()
{
	if ( rmt_rx_channel >= 0 ) {
		rmt_rx_stop((rmt_channel_t)rmt_rx_channel);
	}
	
	
	delete []cap_micro_buf;
	delete []cap_delta_buf;
	delete []cap_level_buf;
	delete []parse_out_buf;
}

void IRAM_ATTR ks_software_serial_c::gpio_isr_handler(void* arg) 
{
	ks_software_serial_c *self = (ks_software_serial_c *)arg;
	bool level = *self->m_rxReg & self->m_rxBitMask;
	int64_t isr_micro = esp_timer_get_time();
	if ( self->cap_idx < self->cap_edge_max) {
		
		self->cap_micro_buf[self->cap_idx] = isr_micro;
		self->cap_level_buf[self->cap_idx] = level;
		self->cap_idx++;
	}
	self->last_milli = isr_micro / 1000;
	self->m_has_int = true;

}
static bool install_isr_service_flag;
void ks_software_serial_c::begin(int baud, int cfg, int rx_pin, int tx_pin, size_t buf_len, uint8_t sws_base)
{
	parse_out_max = buf_len;
	cap_edge_max = parse_out_max * 10;
	cap_micro_buf = new uint32_t[cap_edge_max](); // one frame need 10 bit, so there is 10 edges
	cap_delta_buf= new uint16_t[cap_edge_max]();
	cap_level_buf = new uint8_t[cap_edge_max]();
	parse_out_buf = new uint8_t[parse_out_max](); 

	if ( rx_pin >= 0 ) {
		m_rx_pin = rx_pin;

		// esp32-c3 have 4 rmt_channel, 2 3 is rx, 0 1 is tx
		if ( sws_base == KS_SWS_BASED_RMT ) {
			LOG_INFO("rmt_rx_channel_valid: %d\n", rmt_rx_channel_valid);
			rmt_rx_channel = rmt_rx_channel_valid;
			rmt_rx_channel_valid++;
			rmt_rx_init();	
			rx_based_on = sws_base;		
		}else if(sws_base == KS_SWS_BASED_GPIO){	// rmt no resources, use gpio
			m_rxReg = portInputRegister(digitalPinToPort(m_rx_pin));
			m_rxBitMask = digitalPinToBitMask(m_rx_pin);	
			gpio_config_t rx_conf	 = {
				.pin_bit_mask = 1ull << m_rx_pin,
				.mode = GPIO_MODE_INPUT,
				.pull_up_en = GPIO_PULLUP_ENABLE,
				.pull_down_en = GPIO_PULLDOWN_DISABLE,
				.intr_type = GPIO_INTR_ANYEDGE,
			};
			esp_err_t res = gpio_config(&rx_conf);
			LOG_INFO("gpio_config res: %d\n", res);

			if ( !install_isr_service_flag ) {
				install_isr_service_flag = true;
				// res = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);	// priority too high will affect mqtt
				res = gpio_install_isr_service(0);
				LOG_INFO("gpio_install_isr_service res: %d\n", res);				
			}

			
			res = gpio_isr_handler_add((gpio_num_t)m_rx_pin, gpio_isr_handler, this);
			LOG_INFO("gpio_isr_handler_add res: %d\n", res);	
			rx_based_on = sws_base;		

			gpio_intr_disable((gpio_num_t)m_rx_pin);
		}


	}

	if ( tx_pin >= 0 ) {
		if ( sws_base == KS_SWS_BASED_RMT ) {
			LOG_INFO("rmt_tx_channel_valid: %d\n", rmt_tx_channel_valid);
			rmt_tx_channel = rmt_tx_channel_valid;
			rmt_tx_channel_valid--;
			m_tx_pin = tx_pin;
			rmt_tx_init();	
			tx_based_on = sws_base;		
		}else if(sws_base == KS_SWS_BASED_GPIO){
			LOG_INFO("init gpio tx pin: %d\n", tx_pin);
			m_tx_pin = tx_pin;
			pinMode(m_tx_pin, OUTPUT);
			tx_based_on = sws_base;
		}

	}

}

void ks_software_serial_c::end()
{
	// LOG_INFO("before end() getFreeHeap: %d\n", ESP.getFreeHeap());
	if ( rx_based_on == KS_SWS_BASED_RMT ) {
		if ( rmt_rx_channel >= 0 ) {
			rmt_driver_uninstall((rmt_channel_t)rmt_rx_channel);	
			rmt_rx_channel = -1;
		}
		if ( rmt_tx_channel > 0 ) {
			rmt_driver_uninstall((rmt_channel_t)rmt_tx_channel);	
			rmt_tx_channel = -1;
		}
		
	}else if ( rx_based_on == KS_SWS_BASED_GPIO ) {
		if ( m_rx_pin >= 0 ) {
			gpio_isr_handler_remove((gpio_num_t)m_rx_pin);	
			m_rx_pin = -1;
		}
		if ( m_tx_pin >= 0 ) {
			gpio_isr_handler_remove((gpio_num_t)m_tx_pin);
			m_tx_pin = -1;	
		}

		
	}
	// LOG_INFO("after end() getFreeHeap: %d\n", ESP.getFreeHeap());
}


static size_t parse_data(uint32_t *micro, uint8_t *lvl, size_t len, uint8_t *out_byte, size_t out_max)
{
	int out_idx = 0;
	int lvl_idx = 0;
	int micro_idx = 0;
	int start_bit_idx = 0;

    // at the end of whole pack, level is high, the left bit will be lost, need manual add it back
    uint32_t tail = (micro[len - 1] - micro[0]) % (BAUD_9600_US * 10);
    // LOG_INFO("tail: %d\n", tail);
    uint16_t high_lvl_tail = BAUD_9600_US * 10 - tail;
    // LOG_INFO("high_lvl_tail: %d\n", high_lvl_tail);
    micro[len] = high_lvl_tail + micro[len - 1];
    lvl[len] = 0;


    while(micro_idx < (int)len){
        uint32_t sample_stamp = micro[micro_idx];
        sample_stamp += BAUD_9600_US / 2;	// for 9600, mid of the start bit(a clock is 104 us)
        micro_idx++;

		
        uint8_t parse_byte = 0;
        bool one_frame_lvl[10] = {0};
        int one_frame_idx = 0;
        while(sample_stamp < micro[start_bit_idx] + (BAUD_9600_US / 2) + 9 * BAUD_9600_US){	// while < stop bit mid
            bool cur_lvl = lvl[lvl_idx];
            one_frame_lvl[one_frame_idx] = cur_lvl;
            one_frame_idx++;
            sample_stamp += BAUD_9600_US;
			
            if ( sample_stamp > micro[micro_idx] ) {
                micro_idx++;
                lvl_idx++;
            }
        }


        for (int i = 1; i < 9; i++) {
            parse_byte |= one_frame_lvl[i] << (i - 1);
        }
        start_bit_idx = micro_idx;
        lvl_idx++;

		if ( out_idx < out_max ) {
			out_byte[out_idx] = parse_byte;
			out_idx++;
		}

    }

    return out_idx;

}


void ks_software_serial_c::set_timeout_callback(timeout_cb_t cb_func)
{
    timeout_callback = cb_func;
}

size_t ks_software_serial_c::write(const uint8_t* buffer, size_t size)
{
	if ( m_tx_pin >= 0 ) {
		if ( tx_based_on == KS_SWS_BASED_RMT ) {
			std::vector<uint16_t> send_buf;
			std::vector<uint8_t> send_level;
			for (size_t i = 0; i < size; i++) {
				send_buf.push_back(BAUD_9600_US);	// start bit
				send_level.push_back(false);

				uint8_t data_byte = buffer[i];
				
				for (int j = 0; j < 8; j++) {
					send_buf.push_back(BAUD_9600_US);
					if ( data_byte & 0x01 ) {
						send_level.push_back(true);
					}else{
						send_level.push_back(false);
					}
					data_byte >>= 1;
				}
				send_buf.push_back(BAUD_9600_US);
				send_level.push_back(true);			// stop bit
			}

			rmt_send_raw(send_buf, send_level);		
			return size;
		}else if ( tx_based_on == KS_SWS_BASED_GPIO ) {	// by iFLYTEK Spark AI
            for (size_t i = 0; i < size; i++) {
                uint8_t data_byte = buffer[i];

                // start bit
                digitalWrite(m_tx_pin, LOW);
                delayMicroseconds(BAUD_9600_US);

                // data bits
                for (int j = 0; j < 8; j++) {
                    if (data_byte & 0x01) {
                        digitalWrite(m_tx_pin, HIGH);
                    } else {
                        digitalWrite(m_tx_pin, LOW);
                    }
                    data_byte >>= 1;
                    delayMicroseconds(BAUD_9600_US);
                }

                // stop bit
                digitalWrite(m_tx_pin, HIGH);
                delayMicroseconds(BAUD_9600_US);
            }

            return size;			
		}
	}

	return 0;

}


static inline bool is_odd(uint32_t x)
{
    return (x & 0x01);
}

static inline bool is_even(uint32_t x)
{
    return !is_odd(x);
}
// us convert to pulse cnt
int ks_software_serial_c::rmt_us_to_pulse_cnt(int us)
{
	// rmt_source_clk_t	src_clk;
	// rmt_get_source_clk(rmt_tx_channel, &src_clk);
	// // LOG_INFO("src_clk: %d\n", src_clk);
	// int clk_mhz;
	// if ( src_clk == RMT_BASECLK_XTAL ) {
	// 	clk_mhz = 40;
	// }else{
	// 	clk_mhz = 80;
	// }


	// int one_is_us_scale =  RMT_CLK_DIV / clk_mhz;
	// int pulse_cnt = us / one_is_us_scale;
	// //  LOG_INFO("us: %d, one_is_us_scale: %d, pulse_cnt: %d\n", us, one_is_us_scale, pulse_cnt);


	uint32_t clock_hz = 500000;
	rmt_get_counter_clock((rmt_channel_t)rmt_tx_channel, &clock_hz);
	int pulse_cnt = us / (1000000 / clock_hz);
	return pulse_cnt;
}


void ks_software_serial_c::rmt_tx_init(void)
{
	int init_cnt = 0;

	//01 Common parts of the RMT driver
	LOG_INFO("m_tx_pin: %d, rmt_tx_channel: %d\n", m_tx_pin, rmt_tx_channel);
	rmt_config_t rmt = RMT_DEFAULT_CONFIG_TX((gpio_num_t)m_tx_pin, (rmt_channel_t)rmt_tx_channel);
	// rmt.channel = (rmt_channel_t)rmt_tx_channel; //RMT有0-7一共8个通道
	rmt.clk_div = RMT_CLK_DIV;  //The default clock is 80MHZ, and the divider is 8 bits. This clock is related to the length of the signal level when sending and the count when receiving. n clock cycles represent the length of the signal level. 
	// rmt.gpio_num = (gpio_num_t)m_tx_pin;
	rmt.mem_block_num = 4; //By default, each channel uses 1 block. The total block size is 64x32bit, which can store 128 data
	rmt.rmt_mode = RMT_MODE_TX; //Configure to send mode

	//02 Configure the unique parts of TX
	rmt.tx_config.carrier_en = false;
	// rmt.tx_config.carrier_freq_hz = 38000; 
	// rmt.tx_config.carrier_duty_percent = 50; 
	// rmt.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH; 
	rmt.tx_config.idle_output_en = true; 
	rmt.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH; 
	rmt.tx_config.loop_en = false; 

	//03 

	esp_err_t err = rmt_config(&rmt);
	LOG_INFO("rmt_config err: %d\n", err);
	err = rmt_set_source_clk((rmt_channel_t)rmt_tx_channel, RMT_BASECLK_APB);
	LOG_INFO("rmt_set_source_clk err: %d\n", err);

	rmt_tx_memory_reset((rmt_channel_t)rmt_tx_channel);

	// //----------------- get info ---------------------
	// rmt_source_clk_t	src_clk = (rmt_source_clk_t)0;
	// rmt_get_source_clk(rmt_tx_channel, &src_clk);

	// uint8_t div_cnt = 0;
	// rmt_get_clk_div(rmt_tx_channel, &div_cnt);

	// bool idle_out_en;
	// rmt_idle_level_t level;
	// rmt_get_idle_level(rmt_tx_channel, &idle_out_en, &level);
	

	// uint32_t clock_hz = 0;
	// rmt_get_counter_clock(rmt_tx_channel, &clock_hz);
	// LOG_INFO("src_clk: %d, div_cnt: %d, idle_out_en: %d, level: %d, clock_hz: %d\n", src_clk, div_cnt, idle_out_en, level, clock_hz);
	// if ( clock_hz != 500000 ) {
	// 	init_cnt++;
	// 	LOG_INFO("clock_hz not correct, init_cnt: %d\n", init_cnt);
	// 	delay(1000);
	// 	if ( init_cnt < 3 ) {
	// 		goto start;
	// 	}
		
	// }
	// //----------------- get info end ---------------------	

	//04 
	ESP_ERROR_CHECK(rmt_driver_install((rmt_channel_t)rmt_tx_channel, 0, 0)); 

}

// buf unit is us
int ks_software_serial_c::rmt_send_raw(std::vector<uint16_t> &buf, std::vector<uint8_t> &level)
{
	esp_err_t err;
    // LOG_INFO("len: %d\n", buf.size());
    uint16_t send_len;
    send_len = (buf.size() + 1) & 0xFFFE;   // keep len even
	if ( is_odd(buf.size()) ) {
		buf.resize(send_len);
	}

    if(send_len > RMT_SEND_MAX_LEN){
        send_len = RMT_SEND_MAX_LEN;  
    }
	
	std::vector<rmt_item32_t> rmt_data_send;
	rmt_item32_t tmp;
    for(int i = 0; i < send_len / 2; i++){
		tmp.duration0 = rmt_us_to_pulse_cnt(buf.at(i * 2) );
		tmp.level0 = level.at(i * 2);
		tmp.duration1 = rmt_us_to_pulse_cnt(buf.at(i * 2 + 1) );
		tmp.level1 = level.at(i * 2 + 1);
		rmt_data_send.push_back(tmp);
		// LOG_PURE("%d %d ", tmp.duration0, tmp.duration1);
		// LOG_PURE("%d %d ", tmp.level0, tmp.level1);
    }
	// LOG_PURE("\n");
	tmp.duration0 = 0;
	tmp.level0 = 1;
	tmp.duration1 = 0;
	tmp.level1 = 0;
	rmt_data_send.push_back(tmp);
	
	// err = rmt_set_gpio(rmt_tx_channel, RMT_MODE_TX, (gpio_num_t)kIrLed, false);
	// LOG_INFO("err: %d\n", err);
	// rmt_source_clk_t	src_clk;
	// rmt_get_source_clk(rmt_tx_channel, &src_clk);
	// LOG_INFO("src_clk: %d\n", src_clk);

    rmt_write_items((rmt_channel_t)rmt_tx_channel, rmt_data_send.data(), rmt_data_send.size(), false);
	// err = rmt_write_items((rmt_channel_t)rmt_tx_channel, rmt_data_send.data(), rmt_data_send.size(), true);
	// LOG_INFO("rmt_write_items err: %d\n", err);
	// esp_err_t rmt_write_sample(rmt_channel_t channel, const uint8_t *src, size_t src_size, bool wait_tx_done);
    
    err = rmt_wait_tx_done((rmt_channel_t)rmt_tx_channel, pdMS_TO_TICKS(1000));
	// LOG_INFO("rmt_wait_tx_done err: %d\n", err);
	err = rmt_tx_stop((rmt_channel_t)rmt_tx_channel);
	// LOG_INFO("err: %d\n", err);

    return 0;   
}


void ks_software_serial_c::rmt_rx_init(void)
{
  #define RMT_DEFAULT_CONFIG_RX(gpio, channel_id) \
    {                                           \
      .rmt_mode = RMT_MODE_RX,                \
      .channel = channel_id,                  \
      .gpio_num = gpio,                       \
      .clk_div = 80,                          \
      .mem_block_num = 1,                     \
      .flags = 0,                             \
      .rx_config = {                          \
        .idle_threshold = 12000,            \
        .filter_ticks_thresh = 100,         \
        .filter_en = true,                  \
      }                                       \
    }
    rmt_config_t rmt_rx = RMT_DEFAULT_CONFIG_RX((gpio_num_t)m_rx_pin, (rmt_channel_t)rmt_rx_channel);
	LOG_INFO("rmt_rx_channel: %d\n", rmt_rx_channel);
	LOG_INFO("m_rx_pin: %d\n", m_rx_pin);
    // rmt_rx.channel = (rmt_channel_t)rmt_rx_channel;
    // rmt_rx.gpio_num = (gpio_num_t)m_rx_pin;
    rmt_rx.clk_div = RMT_CLK_DIV;  
    rmt_rx.mem_block_num = 4;   
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = false;  
   	rmt_rx.rx_config.filter_ticks_thresh = 20; 
    rmt_rx.rx_config.idle_threshold = 50000;  
    esp_err_t err = rmt_config(&rmt_rx);
    LOG_INFO("rmt rx config err: %d\n", err);

    ESP_ERROR_CHECK(rmt_driver_install(rmt_rx.channel, 1024, 0));
	// ESP_ERROR_CHECK(rmt_driver_install(rmt_rx.channel, 1024, ESP_INTR_FLAG_LEVEL2));
	
    LOG_INFO("rx driver initialization ok\n");


    err = rmt_get_ringbuf_handle((rmt_channel_t)rmt_rx_channel, &rb);
    LOG_INFO("err: %d, rb: %p\n", err, rb);

	// rmt_rx_memory_reset((rmt_channel_t)rmt_rx_channel);
	// delay(1500);
	// rmt_rx_start((rmt_channel_t)rmt_rx_channel, true);

	
	// if ( rb != NULL ) {
	// 	rmt_item32_t * item = NULL;
	// 	size_t rx_size = 0;
	// 	item = ( rmt_item32_t * ) xRingbufferReceive ( rb, &rx_size, pdMS_TO_TICKS ( 0 ) );		
	// 	LOG_INFO ( "item: %p, rx_size: %d\n", item, rx_size );
	// 	if ( item != NULL ) {
	// 		LOG_INFO("vRingbufferReturnItem\n");
	// 		rmt_item32_t * item_back = item;
	// 		vRingbufferReturnItem ( rb, ( void * ) item_back );
	// 	}			
	// }
	
}

void ks_software_serial_c::start_rx()
{
	if ( !rx_onoff ) {
		if ( rx_based_on == KS_SWS_BASED_RMT ) {
			rmt_rx_start((rmt_channel_t)rmt_rx_channel, true);	
		}else if ( rx_based_on == KS_SWS_BASED_GPIO ) {
			gpio_intr_enable((gpio_num_t)m_rx_pin);
		}		
		rx_onoff = true;
	}

	
}
void ks_software_serial_c::stop_rx()
{
	if ( rx_onoff ) {
		if ( rx_based_on == KS_SWS_BASED_RMT ) {
			rmt_rx_stop((rmt_channel_t)rmt_rx_channel);	
		}else if ( rx_based_on == KS_SWS_BASED_GPIO ) {
			gpio_intr_disable((gpio_num_t)m_rx_pin);
		}		
		rx_onoff = false;	
	}

}
void ks_software_serial_c::perform_work()
{
	if ( rx_based_on == KS_SWS_BASED_RMT ) {
		if ( rmt_rx_channel >= 0 ) {
			size_t rx_size = 0;
			rmt_item32_t * item = NULL;
			if ( rb != NULL ) {
				item = ( rmt_item32_t * ) xRingbufferReceive ( rb, &rx_size, pdMS_TO_TICKS ( 0 ) );			
			}

			
			if ( item != NULL ) {
				rmt_item32_t * item_back = item;
				LOG_INFO ( "perform_work: item: %p, rx_size: %d\n", item, rx_size );
				size_t buf_idx = 0;
				memset ( cap_micro_buf, 0, cap_edge_max );
				memset ( cap_level_buf, 0, cap_edge_max );

				uint32_t micro_abs = 0;

				for ( int i = 0; i < rx_size / 4; i++ ) {

					cap_micro_buf[buf_idx] = micro_abs;
					cap_level_buf[buf_idx] = false;
					buf_idx++;
					//          LOG_PURE("%d %d ", item->duration0, item->duration1);
					cap_delta_buf[buf_idx] = item->duration0;
					micro_abs += item->duration0;

					cap_micro_buf[buf_idx] = micro_abs;
					cap_level_buf[buf_idx] = true;
					buf_idx++;
					micro_abs += item->duration1;
					cap_delta_buf[buf_idx] = item->duration1;
					item++;
				}

				cap_idx = buf_idx;
				//        LOG_PURE("\n");

				// for ( int i = 0; i < buf_idx; i++ ) {
				// 	LOG_PURE ( "%u ", cap_micro_buf[i] );
				// }

				// LOG_PURE ( "\n" );

				size_t parse_len  = 0;

				parse_len = parse_data ( cap_micro_buf, cap_level_buf, buf_idx, parse_out_buf, parse_out_max );

				// for ( int i = 0; i < parse_len; i++ ) {
				// 	LOG_INFO ( "%02x ", parse_out_buf[i] );
				// }

				 LOG_INFO ( "\n" );
				//Return Item
				vRingbufferReturnItem ( rb, ( void * ) item_back );

	            // user call back
				if ( timeout_callback ) {
					timeout_callback(parse_out_buf, parse_len);
				}				

			} else {
				//Failed to receive item
				//        LOG_PURE("Failed to receive item\n");
			}		
		}		
	}else if(rx_based_on == KS_SWS_BASED_GPIO){
	#define tick_after(future, history) ((long)(history) - (long)(future) < 0)
	    if (m_has_int){

	        uint32_t cur_milli = esp_timer_get_time() / 1000;		// millis();
	        // need consider overflow
	        
	        if (tick_after(cur_milli, last_milli + IDLE_TIMEOUT_US)){

	            m_has_int = false;
				// LOG_INFO("rcv timeout\n");

				size_t parse_len = parse_data(cap_micro_buf, cap_level_buf, cap_idx, parse_out_buf, parse_out_max);
				
	            // user call back
				if ( timeout_callback ) {
					timeout_callback(parse_out_buf, parse_len);
				}
				cap_idx = 0;

	        }
	    }
	}


}

void ks_software_serial_c::flush()
{
	if ( rx_based_on == KS_SWS_BASED_RMT ) {
		if ( rmt_rx_channel >= 0 ) {
			rmt_rx_memory_reset((rmt_channel_t)rmt_rx_channel);	
		}
		
	}else if(rx_based_on == KS_SWS_BASED_GPIO){

	}
	
	memset(cap_micro_buf, 0, cap_edge_max);
	memset(cap_level_buf, 0, cap_edge_max);
}

uint32_t * ks_software_serial_c::get_cap_micro_buf()
{
	return cap_micro_buf;
}

uint8_t * ks_software_serial_c::get_cap_level_buf()
{
	return cap_level_buf;
}

uint16_t * ks_software_serial_c::get_cap_delta_buf()
{
	return cap_delta_buf;
}
size_t ks_software_serial_c::get_cap_idx()
{
	return cap_idx;
}
