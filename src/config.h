//optional functions
#define ENABLE_LED_ACK
// #define ENABLE_SCALLER_RS485

//pins
#if defined(ENABLE_LED_ACK)
    #define pin_led_ack 13
#endif
#if defined(ENABLE_SCALLER_RS485)
    #define pin_rs_dir 12
#endif