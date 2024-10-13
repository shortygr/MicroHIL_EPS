//Define characteristic of vehicle (as the signal will pulled to ground the logic is invers)
#define LOW 1
#define HIGH 0

float rollingCircumferenceFF = 1.959;
float rollingCircumferenceRR = 1.959;
int pulsesPerRotation = 48;

//Revese seeting of signal as we switch to ground on LOW level
int crankValue[121] = {LOW,LOW,LOW,LOW,LOW,LOW,HIGH,LOW,HIGH,LOW,HIGH,
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


