#include <Arduino.h>
#include <MsTimer2.h>
#include <interrupt.h>

void setupTimerInt(int ms){
    MsTimer2::set(ms, timerInt);
}
void startTimerInt(){
    MsTimer2::start();
}
void timerInt(){
    timer();
}
void serialEvent1(){
    if(Serial1.available() > 0){
        receive();
    }
}
void serialEvent2(){
    if(Serial2.available() > 0){
        
    }
}
