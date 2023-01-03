#include "config.h"
#include <Arduino.h>
#include <ScallerCom.h>

enum work_modes{
  mode_boot,
  mode_normal,
};

struct Slave {
  bool active = false;
  ACK_DATA ack_data;
};

//ScallerCom
ScallerCom scallercom;

//SlaveList
Slave slave_list[MAX_SLAVE_COUNT];

//module main
bool actual_work_mode = mode_boot;

//module boot variables
byte boot_actual_slave = 0;

//ack
bool send_ack = false;
byte ack_actual_slave = 0;

//comminication
bool flag_wait_for_response = false;
bool flag_response_timeout = false;
byte counter_response_timer = 0;
byte waiting_address = 0;
uint16_t waiting_function = 0;

//100ms tick
bool flag_100ms = false;
bool flag_1000ms = false;
byte counter_100ms = 0;
byte counter_1000ms = 0;

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

void structToFrame(scaller_frame *Scaller_Frame, uint8_t* struct_ptr, uint8_t struct_size, uint8_t data_offset = 0){
    for (byte i= 0; i < struct_size; i++){
        Scaller_Frame->data[i + data_offset] = *struct_ptr++;
    }
    Scaller_Frame->data_size = Scaller_Frame->data_size + struct_size;
}

void frameToStruct(scaller_frame *Scaller_Frame, uint8_t* struct_ptr, uint8_t struct_size){
    for (byte i= 0; i < struct_size; i++){
        *struct_ptr = Scaller_Frame->data[i];
        struct_ptr++;
    }
}

void scallercomCallback(scaller_frame *Scaller_Frame){
  //received function is equal to expected
  if (Scaller_Frame->address == waiting_address && Scaller_Frame->function == waiting_function){
    if (Scaller_Frame->function == FUNCTION_ACK){
      slave_list[Scaller_Frame->address - 1].active = true;
      frameToStruct(Scaller_Frame, (uint8_t*) &slave_list[Scaller_Frame->address - 1].ack_data, sizeof(ACK_DATA));
    }
  }
  flag_wait_for_response = false; 
  flag_response_timeout = false;
}

void scallercomTimout(){
  if (waiting_function == FUNCTION_ACK){
    // slave_list[waiting_address - 1].active = true;
  }
}

void work_cycle(){
  //setup work mode 
  if (actual_work_mode == mode_boot){
    //exit boot mode
    if (boot_actual_slave >= MAX_SLAVE_COUNT){
      actual_work_mode = mode_normal;
      
      //DEBUG
      for (byte i=0; i<MAX_SLAVE_COUNT; i++){
        Serial.print("Slave: ");
        Serial.print(i+1);
        Serial.print(" active=");
        Serial.println(slave_list[i].active);
      }
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

    //ack
    if (send_ack){
      if (ack_actual_slave >= MAX_SLAVE_COUNT){
        ack_actual_slave = 0;
        send_ack = false;
      }
      else if (!flag_wait_for_response ){
        if (slave_list[ack_actual_slave].active){
          scaller_frame ack_frame;
          ack_frame.address = boot_actual_slave + 1;
          ack_frame.function = FUNCTION_ACK;
          ack_frame.data_size = 0;
          sendFrame(&ack_frame);
        }
        ack_actual_slave++;
      } 
    }
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
  sei();

  #if defined(ENABLE_LED_ACK)
    pinMode(pin_led_ack, OUTPUT);
  #endif

  scallercom.init();
  scallercom.setMode(MODE_MASTER);
  scallercom.add_callback(&scallercomCallback);
  #if defined(ENABLE_SCALLER_RS485)
    scallercom.set485pin(pin_rs_dir);
  #endif

  Serial.begin(9600);
}

void loop() {
  work_cycle();
  if (flag_100ms){
    #if defined(ENABLE_LED_ACK)
      digitalWrite(pin_led_ack, !digitalRead(pin_led_ack));
    #endif
    flag_100ms = false;
  }
  if (flag_1000ms){
      send_ack = true;
      flag_1000ms = false;
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
      scallercomTimout();
    }
  }
  //100ms flag
  if (counter_100ms >= 10){
    flag_100ms = true;
    counter_100ms = 0;
  }
  counter_100ms++;
  //1000ms flag
  if (counter_1000ms >= 100){
    flag_1000ms = true;
    counter_1000ms = 0;
  }
  counter_1000ms++;
}