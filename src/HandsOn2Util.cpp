#include <Arduino.h>
#include "HandsOn2Util.hpp"
#include "C610Bus.h"

// Global access to CAN bus
extern C610Bus<CAN2> bus; 

void purgeSerial(){
    while (Serial.available()) {
        Serial.read();
    }
}

void waitForStart(){
    long last_print = millis();
    while (true)
    {
        char c = Serial.read();
        if (c == 's')
        {
        Serial.println("Starting code.");
        break;
        }
        if (millis() - last_print > 2000) {
        Serial.println("Press s to start.");
        last_print = millis();
        }
    }
}

void checkForStop(){
    if (Serial.available())
    {
        if (Serial.read() == 's')
        {
            bus.CommandTorques(0, 0, 0, 0, C610Subbus::kIDZeroToThree);
            Serial.println("Stopping.");
            while (true)
                ;
        }
    }
}