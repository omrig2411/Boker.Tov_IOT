#include "alarm_clock.h"  /* Include the header*/
#include <Arduino.h>
#include <time.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "LiquidCrystal_I2C.h"
#include <Tone32.h>
#include "main.cpp"
#include "esp_sntp.h"

#define WAKE_UP_TIMEOUT 300000                  // 300000 = 5 minutes, 1800000 = 30 minutes
#define FSR_THRESHOLD 100                       // FSR sensitivity to pressure threshold
#define ALARM_WINDOW 30                         // The constant window for predicting wake up time in minutes
#define EMPTY_BED_THRESHOLD 200                 // Threshold for FSr sensors on bed, if input < threshold then bed is empty
#define snoozeBasicFuncPERIOD 5                 // Set duration for snoozeBasicFunc
#define BUZZER 25                           // Buzzer pin
#define BUZZER_CHANNEL 0                        // Buzzer channel

// ntp client variables
WiFiUDP ntpUDP;

// ntp server details
NTPClient timeClient(ntpUDP, "il.pool.ntp.org", 3*3600, 60000);

// Relay port
const int RELAY =  26;

time_t now;
double epochTimeSec;
struct tm wakeUpWindow;
double timeRemaining;
boolean regularWakeUpSetting;
boolean snoozeBasicFuncSetting;
boolean sleepCycleSetting;
boolean wakeUpConfirmationSetting;
boolean soundSetting;
boolean lightSetting;
boolean vibrationSetting;
boolean stopAlarmFromMobileSetting = 0;
boolean alarmSetOnSetting;
int Movements[10];
int value = 0;
int sumFSRSensors = 0;
int snoozeBasicFuncButton = 0;
int stopAlarmButton = 0;
float DToServer;
boolean inWindow = 0;
boolean alarmOn = 0;
boolean snoozeBasicFuncOn = 0;
boolean firstTimeSignalOn = 1;
int actuatorsSwitch = 0;
boolean vibrate = 0;

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
    // struct tm* ltm;                         // **Delete after recieving date by json
    // time(&now);                             // **Delete after recieving date by json
    // ltm = localtime(&now);                  // **Delete after recieving date by json

    
    wakeUpWindow.tm_year = 121/*(number - 1900)*/;       // **Change to input from json
    wakeUpWindow.tm_mon = 5/*(number - 1)*/;             // **Change to input from json
    wakeUpWindow.tm_mday = 4;                            // **Change to input from json
    wakeUpWindow.tm_hour = Hour;
    wakeUpWindow.tm_min = Minute;
    return;
}

/* Set all other configurations of alarm clock*/
void setAlarmConfig(boolean alarmSetOn, boolean regularWakeUp, boolean snoozeBasicFunc, boolean sleepCycle, boolean wakeUpConfirmation, boolean sound, boolean light, boolean vibration) {
    alarmSetOnSetting = alarmSetOn;
    regularWakeUpSetting = regularWakeUp;
    snoozeBasicFuncSetting = snoozeBasicFunc;
    sleepCycleSetting = sleepCycle;
    wakeUpConfirmationSetting = wakeUpConfirmation;
    soundSetting = sound;
    lightSetting = light;
    vibrationSetting = vibration;
    actuatorsSwitch = defineActuators(soundSetting, lightSetting, vibrationSetting);
}

/* Set the flag to stop alarm via mobile*/
void setStopAlarmFromMobile(boolean stopAlarmfromMobile) {
    stopAlarmFromMobileSetting = stopAlarmfromMobile;
}

/* Checks if we are currently in the wake up window.
    if we are currently in a wake up window function retrns 1*/
void checkIfWakeUpWindow() {
    struct tm currentTime;
    Serial.println("currentTime");
    time_t epochTimeSec = timeClient.getEpochTime();
    Serial.println("epochTimeSec");
    memcpy(&currentTime, localtime(&epochTimeSec), sizeof(struct tm));
    Serial.println("memcpy");
    timeRemaining = difftime(mktime(&wakeUpWindow), epochTimeSec);
    Serial.println("time remaining");
    
    Serial.print("now seconds: ");
    Serial.println(epochTimeSec);
    Serial.print("wake up seconds: ");
    Serial.println(mktime(&wakeUpWindow));
    Serial.print("-----currentTime - Year: ");
    Serial.print(currentTime.tm_year);
    Serial.print(" Month: ");
    Serial.print(currentTime.tm_mon);
    Serial.print(" Day: ");
    Serial.print(currentTime.tm_mday);
    Serial.print(" Hour: ");
    Serial.print(currentTime.tm_hour);
    Serial.print(" Minute: ");
    Serial.println(currentTime.tm_min);

    Serial.print("-----wakeUpWindow - Year: ");
    Serial.print(wakeUpWindow.tm_year);
    Serial.print(" Month: ");
    Serial.print(wakeUpWindow.tm_mon);
    Serial.print(" Day: ");
    Serial.print(wakeUpWindow.tm_mday);
    Serial.print(" Hour: ");
    Serial.print(wakeUpWindow.tm_hour);
    Serial.print(" Minute: ");
    Serial.println(wakeUpWindow.tm_min);

    Serial.print("timeRemaining: ");
    Serial.println(timeRemaining / 60);
    
    if ((timeRemaining > 0 && ((timeRemaining / 60) < 30)) || alarmOn) {
        inWindow = 1;
    }
    else inWindow = 0;
    Serial.print("inWindow: ");
    Serial.println(inWindow);
}


/* Function that is called only when inside a wake up window.
    decides which wake up method to use according to user's preferences */
void triggerWakeUp(int currH, int currM, float Dpredict) { 
    Serial.println("@@@@@triggerWakeUp(), 0");
    if(alarmSetOnSetting) {
        Serial.println("@@@@@triggerWakeUp(), 1.0");
        if(regularWakeUpSetting == 1 && alarmOn == 0) {
            Serial.println("@@@@@triggerWakeUp(), 1.1");
            if((currH == wakeUpWindow.tm_hour) && (currM == wakeUpWindow.tm_min) && firstTimeSignalOn) {
                Serial.println("@@@@@triggerWakeUp(), 1.1.1");
                alarmOn = 1;
            }
        }   
        else if(regularWakeUpSetting == 0 && alarmOn == 0) {
            Serial.println("@@@@@triggerWakeUp(), 1.2");
            if(Dpredict < 1) {
                Serial.println("@@@@@triggerWakeUp(), 1.2.1");
                if((currH == wakeUpWindow.tm_hour) && (currM == wakeUpWindow.tm_min) && firstTimeSignalOn){
                    Serial.println("@@@@@triggerWakeUp(), 1.2.1.1");
                    alarmOn = 1; 
                } /* Stages 3-5 of sleep cycle, don't wake up unless the wake up time is due*/
                return;
            }
            else if(Dpredict >= 1) {
                Serial.println("@@@@@triggerWakeUp(), 1.2.2");
                alarmOn = 1; /* stages 1-2 of sleep cycle, do wake up*/
            }       
        }
        else if(alarmOn == 1) {
            return;
        }
    }
    return;
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
    
    return D;
}

/* Check the sum of all current readings to determine if user is on bed or not
    if 1 returned, bed is not empty*/
int checkSumFSR(int F1, int F2, int F3, int F4, int F5) {
    sumFSRSensors = F1 + F2 + F3 + F4 + F5;
    if(sumFSRSensors >= EMPTY_BED_THRESHOLD) {
        return 1;
    }
    else {
        Serial.println("/////bed empty");
        return 0;
    }
}

// Alarm function, including snoozeBasicFunc option and wake up confirmation 
void startAlarmState() {
    firstTimeSignalOn = 0;
    Serial.println("$$$$$startAlarmState(), 0");

    if (wakeUpConfirmationSetting == 1) {
        Serial.println("$$$$$startAlarmState(), 1.0");
        if((checkSumFSR(FSRData1, FSRData2, FSRData3, FSRData4, FSRData5) < EMPTY_BED_THRESHOLD) && stopAlarmButton == 1) {
            Serial.println("$$$$$startAlarmState(), 1.1");
            alarmOn = 0;
            inWindow = 0;
            firstTimeSignalOn = 1;
            stopAlarmButton = 0;
            vibrate = 0;
            ESPSend(2);
            return;
        }
        else if(snoozeBasicFuncSetting == 1 && snoozeBasicFuncButton == 1){
            Serial.println("$$$$$startAlarmState(), 1.2");
            snoozeBasicFunc();   
            snoozeBasicFuncButton = 0;                     
        }
        else if(alarmOn == 1) {
            Serial.println("$$$$$startAlarmState(), 1.3");
            startActuators(actuatorsSwitch);        
        }
    }
    else if(wakeUpConfirmationSetting == 0) {
        Serial.println("$$$$$startAlarmState(), 2.0");
        if(stopAlarmButton == 1) {
            Serial.println("$$$$$startAlarmState(), 2.1");
            alarmOn = 0;
            inWindow = 0;
            firstTimeSignalOn = 1;
            stopAlarmButton = 0;
            vibrate = 0;
            ESPSend(2);
            return;
        }
        else if(snoozeBasicFuncSetting == 1 && snoozeBasicFuncButton == 1) {
            Serial.println("$$$$$startAlarmState(), 2.2");
            snoozeBasicFunc();
        }
        else if(alarmOn == 1) {
            Serial.println("$$$$$startAlarmState(), 2.3");
            startActuators(actuatorsSwitch);        
        }
    }
}

void snoozeBasicFunc() {
    alarmOn = 0;
    snoozeBasicFuncOn = 1;
    Serial.println("*****snoozeBasicFunc(), 0");
    unsigned long currentTime = millis();
    if(currentTime - startMillisSnooze >= (snoozeBasicFuncPERIOD * 1000)) {
        Serial.println("*****snoozeBasicFunc(), 1.0");
        alarmOn = 1;
        snoozeBasicFuncOn = 0;
        startMillisSnooze = currentTime;
    }
}

void IRAM_ATTR snoozeButtonPush() {
    Serial.println("*****snooze Button press");
    snoozeBasicFuncButton = 1;    
}

void IRAM_ATTR stopAlarmButtonPush() {
    Serial.println("*****Stop Alarm button press");
    stopAlarmButton = 1;    
}

void buzzerAction() {
    tone(BUZZER, NOTE_C4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_D4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_E4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_F4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_G4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_A4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_B4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_A4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_G4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_F4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_E4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_D4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_C4, 250, BUZZER_CHANNEL);
    noTone(BUZZER, BUZZER_CHANNEL);
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
    vibrate = 1;
    ESPSend(2);
}

int defineActuators(boolean sound, boolean light, boolean vibration) {
    if(sound && light && vibration) {
        Serial.println("^^^^^defineActuators(), case 0");
        return 0;
    }
    else if(sound && light && !vibration) {
        Serial.println("^^^^^defineActuators(), case 1");
        return 1;
    }
    else if(sound && !light && vibration) {
        Serial.println("^^^^^defineActuators(), case 2");
        return 2;
    }
    else if(!sound && light && vibration) {
        Serial.println("^^^^^defineActuators(), case 3");
        return 3;
    }
    else if(sound && !light && !vibration) {
        Serial.println("^^^^^defineActuators(), case 4");
        return 4;
    }
    else if(!sound && light && !vibration) {
        Serial.println("^^^^^defineActuators(), case 5");
        return 5;
    }
    else if(!sound && !light && vibration) {
        Serial.println("^^^^^defineActuators(), case 6");
        return 6;
    }
    else if(!sound && !light && !vibration) {
        Serial.println("^^^^^defineActuators(), case 7");
        return 7;
    }
    Serial.println("^^^^^defineActuators(), case 8");
    return 8;
}

// All actuator functionality happens here - voice, light, vibration
void startActuators(int actuMode) {   
    Serial.println("Wake Up!!!!");
    if(stopAlarmButton == 0) {
        switch (actuMode) {
        case 0:
            Serial.println("!!!!!startActuators(), case 0");
            vibrationAction();
            buzzerAction();
            lightAction();
            
            break;
        
        case 1:
            Serial.println("!!!!!startActuators(), case 1");
            buzzerAction();
            lightAction();
            break;

        case 2:
            Serial.println("!!!!!startActuators(), case 2");
            vibrationAction();
            buzzerAction();
            
            break;

        case 3:
            Serial.println("!!!!!startActuators(), case 3");
            lightAction();
            vibrationAction();
            break;
        
        case 4:
            Serial.println("!!!!!startActuators(), case 4");
            buzzerAction();
            break;
        
        case 5:
            Serial.println("!!!!!startActuators(), case 5");
            lightAction();
            break;
        
        case 6:
            Serial.println("!!!!!startActuators(), case 6");
            vibrationAction();
            break;
        
        case 7:
            Serial.println("!!!!!startActuators(), case 7");
            break;
        default:
            Serial.println("Error activating actuators");
            break;
        }
    }
    else return;
}



        
        
// if(soundSetting == 1) {
//     buzzerAction();
// } 
// if(lightSetting == 1) {
//     lightAction();
// }
// if(vibrationSetting == 1) {
//     vibrationAction();

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
    