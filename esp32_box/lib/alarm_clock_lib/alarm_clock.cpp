#include "alarm_clock.h"  /* Include the header*/
#include <Arduino.h>
#include <time.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "LiquidCrystal_I2C.h"
#include <Tone32.h>
#include "main.cpp"
#include "esp_sntp.h"

#define uS_TO_M_FACTOR 60000000                 // Conversion factor for micro seconds to minutes
#define ms_TO_M_FACTOR 60000                    // Conversion factor for milli seconds to minutes
#define uS_TO_S_FACTOR 1000000                  // Conversion factor for micro seconds to seconds
#define ms_TO_S_FACTOR 1000                     // Conversion factor for milli seconds to seconds
#define WAKE_UP_TIMEOUT 300000                  // 300000 = 5 minutes, 1800000 = 30 minutes
#define FSR_THRESHOLD 200                       // FSR sensitivity to pressure threshold
#define ALARM_WINDOW 30                         // The constant window for predicting wake up time in minutes
#define EMPTY_BED_THRESHOLD 200                 // Threshold for FSr sensors on bed, if input < threshold then bed is empty
#define snoozeBasicFuncPERIOD 1                 // Set duration for snoozeBasicFunc
#define BUZZER 25                               // Buzzer pin
#define BUZZER_CHANNEL 0                        // Buzzer channel
#define YEAR_GAP_CONSTANT 1900                  // The year constant to subtract to calculate epoch
#define MONTH_GAP_CONSTANT 1                     // The month constant to subtract to calculate epoch

// ntp client variables
WiFiUDP ntpUDP;

// ntp server details
NTPClient timeClient(ntpUDP, "il.pool.ntp.org", 3*3600, 60000);

// Relay port
const int RELAY =  26;

time_t now;
time_t epochTimeSec;
struct tm wakeUpWindow;
struct tm currentTime;
int wakeUpDay = 0;
double timeRemaining;
unsigned long deepSleepMillis;
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

// create an LCD object (Hex address, # characters, # rows)
// my LCD display in on Hex address 27 and is a 20x2 version
LiquidCrystal_I2C lcd(0x27, 20, 2); 

/* Set the time of the wake up window (specified time is the end of the window)*/
void setWakeUp(int Hour, int Minute, int Day) {
    stopAlarmButton = 0;
    epochTimeSec = timeClient.getEpochTime();
    memcpy(&currentTime, localtime(&epochTimeSec), sizeof(struct tm));

    /* calculate date of upcoming alarm */
    int wYear = 1900;
    int wMonth = 0;
    int wDayMon = 1;
    int currDay = (timeClient.getDay());
    int dayDiff = 0;
    Serial.print("Today is: ");
    Serial.println(currDay);
    
    if(Day >= currDay) {
        dayDiff = Day - currDay;
    }
    else {
        dayDiff = 7 - (abs(Day - currDay));
    }
    Serial.print("Difference in days from now to wakeup time: ");
    Serial.println(dayDiff);
    
    if(wDayMon > 28) {
        Serial.println("[][][][][]setWakeUp(), 1.0");
        if(currentTime.tm_mon == 1) {
            Serial.println("[][][][][]setWakeUp(), 1.1");
            wMonth = currentTime.tm_mon + 1;
            wDayMon = (currentTime.tm_mday + dayDiff) % 28;
        }
        else if((wDayMon > 30) && (currentTime.tm_mon == 3 || currentTime.tm_mon == 5 || currentTime.tm_mon == 8 || currentTime.tm_mon == 10)) {
            Serial.println("[][][][][]setWakeUp(), 1.2");
            wMonth = currentTime.tm_mon + 1;
            wDayMon = (currentTime.tm_mday + dayDiff) % 30;
        }
        else if((wDayMon > 31) && (currentTime.tm_mon == 0 || currentTime.tm_mon == 2 || currentTime.tm_mon == 4 || currentTime.tm_mon == 6 || currentTime.tm_mon == 7 || currentTime.tm_mon == 9)) {
            Serial.println("[][][][][]setWakeUp(), 1.3");
            wMonth = currentTime.tm_mon + 1;
            wDayMon = (currentTime.tm_mday + dayDiff) % 31;
        }
        else if((wDayMon > 31) && (currentTime.tm_mon == 11)) {
            Serial.println("[][][][][]setWakeUp(), 1.4");
            wYear = currentTime.tm_year + 1;
            wMonth = currentTime.tm_mon + 1;
            wDayMon = (currentTime.tm_mday + dayDiff) % 31;
        }
    }
    else {
        Serial.println("[][][][][]setWakeUp(), 2.0");
        wYear = currentTime.tm_year;
        wMonth = currentTime.tm_mon;
        wDayMon = currentTime.tm_mday + dayDiff;
    }
    
    wakeUpWindow.tm_year = wYear;           // **Change to input from json (number - 1900)
    wakeUpWindow.tm_mon = wMonth;           // **Change to input from json (number - 1)
    wakeUpWindow.tm_mday = wDayMon;         // **Change to input from json
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
    actuatorsSwitch = defineActuators(soundSetting, lightSetting, vibrationSetting);
}

/* Set the flag to stop alarm via mobile*/
void setStopAlarmFromMobile(boolean stopAlarmfromMobile) {
    stopAlarmFromMobileSetting = stopAlarmfromMobile;
}

/* Checks if we are currently in the wake up window.
    if we are currently in a wake up window function retrns 1*/
void checkIfWakeUpWindow() {   
    epochTimeSec = timeClient.getEpochTime();
    memcpy(&currentTime, localtime(&epochTimeSec), sizeof(struct tm));
    timeRemaining = difftime(mktime(&wakeUpWindow), epochTimeSec);

    // Serial.println("time remaining in seconds:");
    // Serial.print("now seconds- ");
    // Serial.println(epochTimeSec);
    // Serial.print("wake up seconds- ");
    // Serial.println(mktime(&wakeUpWindow));
    
    Serial.print("-----currentTime: ");
    Serial.print(currentTime.tm_mday);
    Serial.print("/");
    Serial.print(currentTime.tm_mon + MONTH_GAP_CONSTANT);
    Serial.print("/");
    Serial.print(currentTime.tm_year + YEAR_GAP_CONSTANT);
    Serial.print(", ");
    if(currentTime.tm_hour <= 9) {
        Serial.print(0);
    }
    Serial.print(currentTime.tm_hour);
    Serial.print(":");
    if(currentTime.tm_min <= 9) {
        Serial.print(0);
    }
    Serial.println(currentTime.tm_min);

    Serial.print("-----wakeUpWindow: ");
    Serial.print(wakeUpWindow.tm_mday);
    Serial.print("/");
    Serial.print((wakeUpWindow.tm_mon + MONTH_GAP_CONSTANT));
    Serial.print("/");
    Serial.print((wakeUpWindow.tm_year + YEAR_GAP_CONSTANT));
    Serial.print(", ");
    if(wakeUpWindow.tm_hour <= 9) {
        Serial.print(0);
    }
    Serial.print(wakeUpWindow.tm_hour);
    Serial.print(":");
    if(wakeUpWindow.tm_min <= 9) {
        Serial.print(0);
    }
    Serial.println(wakeUpWindow.tm_min);

    Serial.print("timeRemaining in minutes: ");
    Serial.println(timeRemaining / 60);
    
    if ((timeRemaining > 0 && ((timeRemaining / 60) <= 30)) || alarmOn) {
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
        if(stopAlarmButton == 1) {
                Serial.println("@@@@@triggerWakeUp(), 1.1");
                return;
        }
        if(regularWakeUpSetting == 1 && alarmOn == 0 && firstTimeSignalOn == 1) {
            Serial.println("@@@@@triggerWakeUp(), 1.2");
            // Serial.print("first time signal:");
            // Serial.println(firstTimeSignalOn);
            // Serial.print("Hour condition:");
            // Serial.println((currH == wakeUpWindow.tm_hour));
            // Serial.print("Minute condition:");
            // Serial.println((currM == wakeUpWindow.tm_min));
            if((currH == wakeUpWindow.tm_hour) && (currM == wakeUpWindow.tm_min)) {
                Serial.println("@@@@@triggerWakeUp(), 1.2.1");
                alarmOn = 1;
                deepSleepMillis = millis();
                return;
            }
            return;
        }   
        else if(regularWakeUpSetting == 0 && alarmOn == 0 && stopAlarmButton == 0 && firstTimeSignalOn == 1) {
            Serial.println("@@@@@triggerWakeUp(), 1.3");
            if(Dpredict < 1) {/* Stages 3-5 of sleep cycle, don't wake up unless the wake up time is due*/
                Serial.println("@@@@@triggerWakeUp(), 1.3.1");
                // Serial.print("currH: ");
                // Serial.println(currH);
                // Serial.print("wakeUpWindow.tm_hour: ");
                // Serial.println(wakeUpWindow.tm_hour);
                // Serial.print("currM: ");
                // Serial.println(currM);
                // Serial.print("wakeUpWindow.tm_min: ");
                // Serial.println(wakeUpWindow.tm_min);
                // Serial.print("firstTimeSignalOn: ");
                // Serial.println(firstTimeSignalOn);

                if((currH == wakeUpWindow.tm_hour) && ((currM + 1) == wakeUpWindow.tm_min) && firstTimeSignalOn == 1){
                    Serial.println("@@@@@triggerWakeUp(), 1.3.1.1");
                    alarmOn = 1; 
                    deepSleepMillis = millis();
                    return;
                }
            }
            else if(Dpredict >= 1) {/* stages 1-2 of sleep cycle, do wake up*/
                Serial.println("@@@@@triggerWakeUp(), 1.3.2");
                alarmOn = 1; 
                deepSleepMillis = millis();
                return;
            }
            return;    
        }
        else if(alarmOn == 1) {
            Serial.println("@@@@@triggerWakeUp(), 1.4");
            return;
        }
    }
    return;
}

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
    float P = 0.2; // P constant
    
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
        Serial.println("/////bed NOT empty");
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
        Serial.print("/////Stop button: ");
        Serial.println(stopAlarmButton);
        Serial.print("/////Snooze Setting: ");
        Serial.println(snoozeBasicFuncSetting);
        Serial.print("/////Snooze button: ");
        Serial.println(snoozeBasicFuncButton);

        if(!(checkSumFSR(FSRData1, FSRData2, FSRData3, FSRData4, FSRData5)) && stopAlarmButton == 1) {
            Serial.println("$$$$$startAlarmState(), 1.1");
            alarmOn = 0;
            alarmSetOnSetting = 0;
            inWindow = 0;
            firstTimeSignalOn = 1;
            vibrate = 0;
            ESPSend(2);
            return;
        }
        else if(checkSumFSR(FSRData1, FSRData2, FSRData3, FSRData4, FSRData5) && stopAlarmButton == 1) {
            Serial.println("$$$$$startAlarmState(), 1.2");
            startActuators(actuatorsSwitch);
            return; 
        }
        else if(snoozeBasicFuncSetting == 1 && snoozeBasicFuncButton == 1){
            Serial.println("$$$$$startAlarmState(), 1.3");
            snoozeBasicFunc();   
            snoozeBasicFuncButton = 0;
            return;                     
        }
        else if(alarmOn == 1) {
            Serial.println("$$$$$startAlarmState(), 1.4");
            startActuators(actuatorsSwitch);
            return;        
        }
    }
    else if(wakeUpConfirmationSetting == 0) {
        Serial.println("$$$$$startAlarmState(), 2.0");
        Serial.print("Stop button: ");
        Serial.println(stopAlarmButton);
        if(stopAlarmButton == 1) {
            Serial.println("$$$$$startAlarmState(), 2.1");
            alarmOn = 0;
            alarmSetOnSetting = 0;
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
            return;
        }
        else if(alarmOn == 1) {
            Serial.println("$$$$$startAlarmState(), 2.3");
            startActuators(actuatorsSwitch);
            return;        
        }
        return;
    }
    return;
}

void snoozeBasicFunc() {
    unsigned long currentSnoozeTime = millis();
    alarmOn = 0;
    snoozeBasicFuncOn = 1;
    Serial.println("*****snoozeBasicFunc(), 0");
    
    while(currentSnoozeTime <= (snoozeBasicFuncPERIOD * ms_TO_M_FACTOR)) {
        Serial.println("*****snoozeBasicFunc(), 1.0");
        timeClient.update();
        lcd.clear();
        lcd.print(timeClient.getFormattedTime());
        lcd.setCursor(0, 1);
        lcd.print("Snooze: ");
        lcd.print(((snoozeBasicFuncPERIOD * 60) - (currentSnoozeTime / ms_TO_S_FACTOR)));
        lcd.print(" sec.");

        delay(1000);
    }

    alarmOn = 1;
    snoozeBasicFuncOn = 0;
}

void IRAM_ATTR snoozeButtonPush() {
    if(snoozeBasicFuncButton == 0) {
        Serial.println("*****snooze Button press");
        snoozeBasicFuncButton = 1;
    }
    delay(50);
    // making sure there are no phantom button clicks
    if(snoozeBasicFuncButton == 0) {
        Serial.println("*****snooze Button press");
        snoozeBasicFuncButton = 1;
    }    
}

void IRAM_ATTR stopAlarmButtonPush() {
    if(stopAlarmButton == 0) {
        Serial.println("*****Stop Alarm button press");
        stopAlarmButton = 1;
    }
    delay(50);
    // making sure there are no phantom button clicks
    if(stopAlarmButton == 0) {
        Serial.println("*****Stop Alarm button press");
        stopAlarmButton = 1;
    }    
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
    return;
}



        

    //  for(i = (snoozeBasicFuncPERIOD * 60); i == 0;) {
    //     currentSnoozeTime = millis();
    //     i = (snoozeBasicFuncPERIOD * 60) - (currentSnoozeTime / ms_TO_S_FACTOR);

    //     Serial.println("*****snoozeBasicFunc(), 1.0");
    //     timeClient.update();
    //     lcd.clear();
    //     lcd.print(timeClient.getFormattedTime());
    //     lcd.setCursor(0, 1);
    //     lcd.print("Snooze: ");
    //     lcd.print(((snoozeBasicFuncPERIOD * 60) - currentSnoozeTime));
    //     lcd.print(" sec.");

    //     delay(1000);
    // }



    // while(currentSnoozeTime <= (snoozeBasicFuncPERIOD * ms_TO_M_FACTOR)) {
    //     Serial.println("*****snoozeBasicFunc(), 1.0");
    //     timeClient.update();
    //     lcd.clear();
    //     lcd.print(timeClient.getFormattedTime());
    //     lcd.setCursor(0, 1);
    //     lcd.print("Snooze: ");
    //     lcd.print(((snoozeBasicFuncPERIOD * 60) - currentSnoozeTime));
    //     lcd.print(" sec.");

        
    //     currentSnoozeTime = millis();
    //     delay(1000);
    // }

// epochTimeSec = timeClient.getEpochTime();
    // memcpy(&currentTime, localtime(&epochTimeSec), sizeof(struct tm));

    // int snoozeHour = currentTime.tm_hour;
    // int snoozeMin = currentTime.tm_min;
    
    // if((snoozeMin + snoozeBasicFuncPERIOD) > 60) {
    //     snoozeHour += 1;
    //     snoozeMin = (snoozeMin + snoozeBasicFuncPERIOD) % 60;
    // }

    // setWakeUp(snoozeHour, snoozeMin, wakeUpDay);
    // alarmSetOnSetting = 1;
        
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
    