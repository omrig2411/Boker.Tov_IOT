#include "alarm_clock.h"  /* Include the header*/
#include <Arduino.h>
#include <time.h>
#include <NTPClient.h>
#include "LiquidCrystal_I2C.h"

#define WAKE_UP_TIMEOUT 300000       // 300000 = 5 minutes, 1800000 = 30 minutes
#define FSR_THRESHOLD 100            // FSR sensitivity to pressure threshold
#define ALARM_WINDOW 30              // The constant window for predicting wake up time
#define EMPTY_BED_THRESHOLD 200      // Threshold for FSr sensors on bed, if input < threshold then bed is empty
#define snoozeBasicFuncPERIOD 5               //set duration for snoozeBasicFunc

// create an LCD object (Hex address, # characters, # rows)
// my LCD display in on Hex address 27 and is a 20x2 version
LiquidCrystal_I2C lcd_two(0x27, 20, 2); 

// relay port
const int RELAY =  26;

time_t now;
double seconds;
struct tm wakeUpWindow;
boolean regularWakeUpSetting;
boolean snoozeBasicFuncSetting;
boolean sleepCycleSetting;
boolean wakeUpConfirmationSetting;
boolean soundSetting;
boolean lightSetting;
boolean vibrationSetting;
boolean stopAlarmFromMobileSetting;
int Movements[10];
int value = 0;
int sumFSRSensors = 0;
int snoozeBasicFuncButton = 0;
int stopAlarmButton = 0;
float DToServer;
boolean alarmOn = 0;
boolean snoozeBasicFuncOn = 0;
boolean firstTimeSignalOn = 1;
boolean alarmSetOn;
unsigned long startMillisSnooze;

//integers to hold raw data from FSR
int FSRData1;
int FSRData2;
int FSRData3;
int FSRData4;
int FSRData5;

//integers to hold previous value of FSR sensors
int prevFSR1 = 0;
int prevFSR2 = 0;
int prevFSR3 = 0;
int prevFSR4 = 0;
int prevFSR5 = 0;

// Buzzer module variables
int freq = 2000;
int channel = 0;
int resolution = 8;

/* Set the time of the wake up window (specified time is the end of the window)*/
void setWakeUp(int Hour, int Minute) {
    wakeUpWindow.tm_hour = Hour;
    wakeUpWindow.tm_min = Minute;
    return;
}

/* Set all other configurations of alarm clock*/
void setAlarmConfig(boolean regularWakeUp, boolean snoozeBasicFunc, boolean sleepCycle, boolean wakeUpConfirmation, boolean sound, boolean light, boolean vibration) {
    regularWakeUpSetting = regularWakeUp;
    snoozeBasicFuncSetting = snoozeBasicFunc;
    sleepCycleSetting = sleepCycle;
    wakeUpConfirmationSetting = wakeUpConfirmation;
    soundSetting = sound;
    lightSetting = light;
    vibrationSetting = vibration;
}

/* Set the flag to stop alarm via mobile*/
void setStopAlarmFromMobile(boolean stopAlarmfromMobile) {
    stopAlarmFromMobileSetting = stopAlarmfromMobile;
}

/* Checks if we are currently in the wake up window.
    if we are currently in a wake up window function retrns 1*/
int checkIfWakeUpWindow() {
    time(&now);  // get current time
    seconds = difftime(now, mktime(&wakeUpWindow)); // Calculates the difference in seconds between now and wake up window start
    
    if((seconds / 60) < ALARM_WINDOW) {
        return 1;
    }
    else return 0;
}

/* Function that is called only when inside a wake up window.
    decides which wake up method to use according to user's preferences */
void triggerWakeUp(int currH, int currM, float Dpredict) { 
    if(alarmSetOn) {
        if(regularWakeUpSetting == 1) {
        if((currH == wakeUpWindow.tm_hour) && (currM == wakeUpWindow.tm_min) && firstTimeSignalOn) {
            startAlarm();
        }
    }
    else if(regularWakeUpSetting == 0) {
        if(Dpredict < 1) {
            if((currH == wakeUpWindow.tm_hour) && (currM == wakeUpWindow.tm_min) && firstTimeSignalOn){
            startAlarm(); 
        } /* Stages 3-5 of sleep cycle, don't wake up unless the wake up time is due*/
        }
        else if(Dpredict >= 1) {
            startAlarm(); /* stages 1-2 of sleep cycle, do wake up*/
        }
        
    }
    }
}
/*finish writing*/
void identifyMovementPIR() {
    value++;
}

void identifyMovementFSR(int F1, int F2, int F3, int F4, int F5) {
    int M = 0;
    if (abs(prevFSR1 - F1) >= FSR_THRESHOLD) {
        M++;
    }
    if (abs(prevFSR2 - F2) >= FSR_THRESHOLD) {
        M++;
    }
    if (abs(prevFSR3 - F3) >= FSR_THRESHOLD) {
        M++;
    }
    if (abs(prevFSR4 - F4) >= FSR_THRESHOLD) {
        M++;
    }
    if (abs(prevFSR5 - F5) >= FSR_THRESHOLD) {
        M++;
    }
    if(M >= 2) {
        value++;
    }

    prevFSR1 = F1;
    prevFSR2 = F2;
    prevFSR3 = F3;
    prevFSR4 = F4;
    prevFSR5 = F5;
}

void cleanValue(int loopNum) {
    if((loopNum % 15) > 1) {
        value = 0;
    }
    else return;
}

void saveValues() {
    for (int i = 9; i > 0; i--) {
    Movements[i] = Movements[i-1];
    }
    Movements[0] = value;
}

float Dpredict() {
    float P = 0.10; // P constant
    
    saveValues();
    
    float D = P * (1 * Movements[0] + 0.9 * Movements[1] + 0.8 * Movements[2]
             + 0.7 * Movements[3] + 0.6 * Movements[4] + 0.5 * Movements[5]
             + 0.4 * Movements[6] + 0.3 * Movements[7] + 0.2 * Movements[8]
             + 0.1 * Movements[9]);

    DToServer = D;
    
    /*// Test predict
    Serial.print("0: ");
    Serial.println(Movements[0]);
    Serial.print("1: ");
    Serial.println(Movements[1]);
    Serial.print("2: ");
    Serial.println(Movements[2]);
    Serial.print("3: ");
    Serial.println(Movements[3]);
    Serial.print("4: ");
    Serial.println(Movements[4]);
    Serial.print("5: ");
    Serial.println(Movements[5]);
    Serial.print("6: ");
    Serial.println(Movements[6]);
    Serial.print("7: ");
    Serial.println(Movements[7]);
    Serial.print("8: ");
    Serial.println(Movements[8]);
    Serial.print("9: ");
    Serial.println(Movements[9]);*/
    return D;
}

/* Check the sum of all current readings to determine if user is on bed or not
    if 1 returned, bed is not empty*/
int checkSumFSR(int F1, int F2, int F3, int F4, int F5) {
    sumFSRSensors = F1 + F2 + F3 + F4 + F5;
    if(sumFSRSensors >= EMPTY_BED_THRESHOLD) {
        return 1;
    }
    else return 0;
}

// Alarm function, including snoozeBasicFunc option and wake up confirmation 
void startAlarm() {
    alarmOn = 1;
    firstTimeSignalOn = 0;
    
    if (wakeUpConfirmationSetting == 1) {
        if(checkSumFSR(FSRData1, FSRData2, FSRData3, FSRData4, FSRData5) < EMPTY_BED_THRESHOLD && ((stopAlarmButton == 1) || (stopAlarmFromMobileSetting = 1))) {
            alarmOn = 0;
            firstTimeSignalOn = 1;
            return;
        }
        else if(snoozeBasicFuncSetting == 1 && snoozeBasicFuncButton == 1){
            snoozeBasicFunc();                        
        }
        else if(alarmOn == 1) {
           startActuators();        
        }
    }
    else if(wakeUpConfirmationSetting == 0) {
        if((stopAlarmButton == 1) || (stopAlarmFromMobileSetting == 1)) {
            alarmOn = 0;
            firstTimeSignalOn = 1;
            return;
        }
        else if(snoozeBasicFuncSetting == 1 && snoozeBasicFuncButton == 1) {
            snoozeBasicFunc();
        }
        else if(alarmOn == 1) {
           startActuators();        
        }
    }
}

void snoozeBasicFunc() {
    alarmOn = 0;
    snoozeBasicFuncOn = 1;
    unsigned long currentTime = millis();
    if(currentTime - startMillisSnooze >= (snoozeBasicFuncPERIOD * 1000)) {
        alarmOn = 1;
        snoozeBasicFuncOn = 0;
        startMillisSnooze = currentTime;
    }
}

void IRAM_ATTR snoozeButtonPush() {
    snoozeBasicFuncButton = 1;
    Serial.println("snoozeBasicFunc");
}

void IRAM_ATTR stopAlarmButtonPush() {
    stopAlarmButton = 1;
    Serial.println("Stop Alarm");
}

/* function to set buzzer parameters*/
void buzzerSetup() {
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(25, channel);
}

void buzzerAction() {
    ledcWriteTone(channel, 2000);
  
    for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle=dutyCycle+10) {
        ledcWrite(channel, dutyCycle);
        delay(500);
    }
    
    ledcWrite(channel, 125);
    
    for (int freq = 255; freq < 10000; freq = freq + 250) {
        ledcWriteTone(channel, freq);
        delay(500);
    }
}

void lightAction() {
    int l;
    for(l = 0; l < 5; l++) {
    digitalWrite(RELAY, HIGH);
    delay(500);
    digitalWrite(RELAY, LOW);
    delay(500);
    }
    delay(2000);      
}

void vibrationAction() {
    //
}

// All actuator functionality happens here - voice, light, vibration
void startActuators() {
    while(1) {
        Serial.println("Wake Up!!!!");
        lcd_two.clear();
        lcd_two.print("Wake Up!!!");
        
        if(soundSetting == 1) {
            buzzerAction();
        } 
        if(lightSetting == 1) {
            lightAction();
        }
        if(vibrationSetting == 1) {
            vibrationAction();
        }        
    }
}