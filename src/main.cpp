#include <Arduino.h>
#include <ScallerCom.h>

ScallerCom scallercom;

void forocom_module_setup(scaller_frame *Scaller_Frame){
  Scaller_Frame->data[0] = 127;
  Scaller_Frame->function = 0x0b0c;
  Scaller_Frame->address = 0x0a;
  digitalWrite(13, !digitalRead(13));
}

void setup() {
  scallercom.init();
  scallercom.add_callback(&forocom_module_setup);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  scallercom.scallercom_read();
}