//Define characteristic of vehicle (as the signal will pulled to ground the logic is invers)
#define LOW 1
#define HIGH 0

float rollingCircumference = 1.959;
int pulsesPerRotation = 20;

//Revese seeting of signal as we switch to ground on LOW level
int crankValue[120] = {LOW,LOW,LOW,LOW,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,
                       LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH};


