#include "alarm_clock.h"  /* Include the header*/
#include <Arduino.h>
#include <time.h>
#include <NTPClient.h>
#include "LiquidCrystal_I2C.h"

#define WAKE_UP_TIMEOUT 300000 // 300000 = 5 minutes, 1800000 = 30 minutes
#define FSR_THRESHOLD 400

// create an LCD object (Hex address, # characters, # rows)
// my LCD display in on Hex address 27 and is a 20x2 version
LiquidCrystal_I2C lcd_two(0x27, 20, 2); 

time_t now;
struct tm wakeUpWindow;
double seconds;
int wakeUpHour;
int wakeUpMin;
int Movements[10];
int value;

// Set the time of the wake up window (specified time is the end of the window)
void setWakeUp(int Hour, int Minute) {
    wakeUpWindow.tm_hour = Hour;
    wakeUpWindow.tm_min = Minute;
    return;
}

// // Checks if we are currently in the wake up window
int checkIfWakeUpWindow() {
    time(&now);  // get current time
    seconds = difftime(now, mktime(&wakeUpWindow)); // Calculates the difference in seconds between now and wake up window start
    
    if((seconds / 60) < 30) {
        return 1;
    }
    else return 0;
}

// int identifyMovement(int fsr1, int fsr2, int fsr3, int fsr4, int fsr5) {

// }

void saveValues() {
    for (int i = 9; i > 0; i--) {
        Movements[i] = Movements[i-1];
    }
    Movements[0] = value;
}

int predictWakeTime() {
    float P = 0.25; // P constant
    
    saveValues();
    
    int D = P * (1 * Movements[0] + 0.9 * Movements[1] + 0.8 * Movements[2]
             + 0.7 * Movements[3] + 0.6 * Movements[4] + 0.5 * Movements[5]
             + 0.4 * Movements[6] + 0.3 * Movements[7] + 0.2 * Movements[8]
             + 0.1 * Movements[9]);


    if (D >= 1) {
        return 1;
    }
    else return 0;
}

// Alarm function 
// void startAlarm(boolean persistentWakeUp) {
//     unsigned long startWakeUpTime = millis();
    
//     if (persistentWakeUp == true) {
//         while(true) {
//             startActuators(); 
//             }
//     }
// }

// All actuator functionality happens here - voice, light, vibration
// void startActuators() {
//     Serial.println("Wake Up!!!!");
//     lcd_two.clear();
//     lcd_two.print("Wake Up!!!");
// }