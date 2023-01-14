//Define characteristic of vehicle
#define HIGH 0
#define LOW 1

float rollingCircumference = 1.959;
int pulsesPerRotation = 20;

//Revese seeting of signal as we switch to ground on high level
int crankValue[120] = {HIGH,HIGH,HIGH,HIGH,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,
                       HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW};


