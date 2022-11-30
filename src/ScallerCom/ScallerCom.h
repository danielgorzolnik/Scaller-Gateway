#ifndef ForoCom_h
#define ForoCom_h
#include <Arduino.h>
#include "frame.h"

class ScallerCom{
    using my_callback = void (*)(scaller_frame *Foro_Frame);
    private:
        byte module_address;
        my_callback _callback;
        void execute_command(scaller_frame *Scaller_Frame);
        void generate_frame(scaller_frame *Scaller_Frame);
    public:
        void init();
        void write(scaller_frame);
        void setType(byte);
        void setAddress(byte);
        void scallercom_read();
        void add_callback(my_callback get_callback)
        {
            _callback = get_callback;
        }
};


#endif