//frame settings
#define frame_start_byte 0x04
#define frame_stop_byte 0x05
#define frame_max_data_size 16
#define frame_buffer_size 21 //max_data_size + rest of frame (5)
#define frame_position_address 0
#define frame_position_function 1
#define frame_position_data_size 3
#define frame_position_data 4

//port settings
#define port_speed 38400
#define port_rx_pin 2
#define port_tx_pin 3