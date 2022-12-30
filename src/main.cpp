#include <Arduino.h>
#include <ScallerCom.h>
#include "config.h"

enum work_modes{
  mode_boot,
  mode_normal,
};

ScallerCom scallercom;

//module main
bool actual_work_mode = mode_boot;

//module boot variables
byte boot_actual_slave = 0;

//comminication
bool flag_wait_for_response = false;
bool flag_response_timeout = false;
byte counter_response_timer = 0;
byte waiting_address = 0;
uint16_t waiting_function = 0;

//100ms tick
bool flag_100ms = false;
byte counter_100ms = 0;

void sendFrame(scaller_frame *Scaller_Frame){
  scallercom.send(Scaller_Frame);
  if (Scaller_Frame->function != FUNCTION_UC_RESET){
    flag_wait_for_response = true;
    flag_response_timeout = false;
    waiting_address = Scaller_Frame->address;
    waiting_function = Scaller_Frame->function;
    counter_response_timer = 0;
  }
}

void scallercomCallback(scaller_frame *Scaller_Frame){
  //received function is equal to expected
  if (Scaller_Frame->address == waiting_address && Scaller_Frame->function == waiting_function){
    if (Scaller_Frame->function == FUNCTION_ACK){
      
    }
  }
  flag_wait_for_response = false; 
  flag_response_timeout = false;
}

void work_cycle(){
  //setup work mode 
  if (actual_work_mode == mode_boot){
    //exit boot mode
    if (boot_actual_slave >= MAX_SLAVE_COUNT){
      actual_work_mode = mode_normal;
    }

    else if (!flag_wait_for_response){
      scaller_frame ack_frame;
      ack_frame.address = boot_actual_slave + 1;
      ack_frame.function = FUNCTION_ACK;
      ack_frame.data_size = 0;
      sendFrame(&ack_frame);
      boot_actual_slave++;
    }
  }

  // Normal work mode
  if (actual_work_mode == mode_normal){

  }
}

void setup() {
  //SETUP TIMERS
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;
  TIMSK0 = 0;
  OCR0A = 156; //156 = 10ms (16MHz)  78 = 10ms (8Mhz)
  TCCR0A |= ((1 << WGM00) | (1 << WGM01));
  TCCR0B |= ((1 << CS00) | (1 << CS02));
  TIMSK0 |= (1 << OCIE0A);

  #if defined(ENABLE_LED_ACK)
    pinMode(pin_led_ack, OUTPUT);
  #endif

  scallercom.init();
  scallercom.setMode(MODE_MASTER);
  scallercom.add_callback(&scallercomCallback);
  #if defined(ENABLE_SCALLER_RS485)
    scallercom.set485pin(pin_rs_dir);
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  work_cycle();
  if (flag_100ms){
    #if defined(ENABLE_LED_ACK)
      pinMode(pin_led_ack, !digitalRead(pin_led_ack));
    #endif
    flag_100ms = false;
  }
  if (flag_wait_for_response){
    scallercom.scallercom_read();
  }
}

//10ms tick
ISR(TIMER0_COMPA_vect){
  //forocom timeout
  if (flag_wait_for_response){
    counter_response_timer++;
    if (counter_response_timer > SCALLERCOM_TIMEOUT){
      flag_response_timeout = true;
      flag_wait_for_response = false;
      counter_response_timer = 0;
    }
  }

  //100ms flag
  if (counter_100ms >= 10){
    flag_100ms = true;
    counter_100ms = 0;
  }
  counter_100ms++;
}