//define for scallercom
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "ScallerCom.h"
#include "config.h"

//Soft seria;
SoftwareSerial softSerial (port_rx_pin, port_tx_pin);

byte frame_buffer[frame_buffer_size];
byte actual_frame_position = 0;
bool transmission_started = false;

//functions
bool calculate_crc(byte *buffer);
scaller_frame generate_struct(byte *buffer);
void send_frame(scaller_frame *Scaller_Frame);
void generate_checksum(scaller_frame *Scaller_Frame);

void ScallerCom::init(){
    softSerial.begin(port_speed);
}

void ScallerCom::scallercom_read(){ //function is called every 1ms
    if (softSerial.available() > 0){
        byte incoming_byte = softSerial.read();
        if (incoming_byte == frame_start_byte && actual_frame_position == 0 && !transmission_started){ //start transmission
            for (byte i = 0; i < frame_buffer_size; i++) frame_buffer[i] = 0; //clean frame
            transmission_started = true;
        }
        else if (incoming_byte == frame_stop_byte && 
        ((actual_frame_position >= frame_buffer_size) || 
        ((actual_frame_position >= frame_position_data_size) && (actual_frame_position >= (frame_buffer[frame_position_data_size] + 5)))) && 
        transmission_started){ //stop transmission
            actual_frame_position = 0;
            transmission_started = false;
            if (calculate_crc(frame_buffer)){
                scaller_frame Scaller_Frame = generate_struct(frame_buffer);
                _callback(&Scaller_Frame);
                generate_checksum(&Scaller_Frame);
                send_frame(&Scaller_Frame); //send data
            }
        }
        else if (transmission_started){
            if (actual_frame_position >= frame_buffer_size ||
            ((actual_frame_position >= frame_position_data_size) && (actual_frame_position >= (frame_buffer[frame_position_data_size] + 5)))){
                actual_frame_position = 0;
                transmission_started = false;
            }
            else{
                frame_buffer[actual_frame_position] = incoming_byte;
                actual_frame_position++;
            }
        }
    }
}

bool calculate_crc(byte *buffer){
    uint16_t sum = 0;
    byte data_size = buffer[frame_position_data_size];
    byte checksum_pos = data_size + 4;
    for (byte i=0; i < (frame_buffer_size - 1); i++){
        if (!(i > (data_size + 3))){
            sum = sum + buffer[i];
        }
    }
    if (sum % 255 == buffer[checksum_pos]){
        return true;
    }
    else {
        return false;
    }
}

scaller_frame generate_struct(byte *buffer){
    byte data_size = buffer[frame_position_data_size];
    scaller_frame Scaller_Frame;
    Scaller_Frame.address = buffer[frame_position_address];
    Scaller_Frame.function = buffer[frame_position_function + 1] | buffer[frame_position_function] << 8;
    Scaller_Frame.data_size = data_size;
    for (byte i=0; i<data_size; i++){
        Scaller_Frame.data[i] = buffer[frame_position_data + i];
    }
    Scaller_Frame.checksum = buffer[data_size + 5]; 
    return Scaller_Frame;
}

void generate_checksum(scaller_frame *Scaller_Frame){
    uint16_t sum = 0;
    sum = Scaller_Frame->address;
    sum = sum + ((Scaller_Frame->function >> 8) & 0xff);
    sum = sum + (Scaller_Frame->function & 0xff);
    sum = sum + Scaller_Frame->data_size;
    for (byte i=0; i<Scaller_Frame->data_size; i++){
        sum = sum + Scaller_Frame->data[i];
    }
    Scaller_Frame->checksum = sum % 0xff;
}

void send_frame(scaller_frame *Scaller_Frame){
    softSerial.write(frame_start_byte);
    softSerial.write(Scaller_Frame->address);
    softSerial.write((Scaller_Frame->function >> 8) & 0xff);
    softSerial.write(Scaller_Frame->function);
    softSerial.write(Scaller_Frame->data_size);
    for (byte i=0; i<Scaller_Frame->data_size; i++){
        softSerial.write(Scaller_Frame->data[i]);
    }
    softSerial.write(Scaller_Frame->checksum);
    softSerial.write(frame_stop_byte);
    softSerial.flush();
}