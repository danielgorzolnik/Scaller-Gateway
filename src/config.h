//OPTIONAL FUNCTIONS
#define ENABLE_LED_ACK
// #define ENABLE_SCALLER_RS485

//GENERAL
#define MAX_SLAVE_COUNT 16
#define SCALLERCOM_TIMEOUT 10 //multiple by 10ms

//PINS
#if defined(ENABLE_LED_ACK)
    #define pin_led_ack 13
#endif
#if defined(ENABLE_SCALLER_RS485)
    #define pin_rs_dir 12
#endif