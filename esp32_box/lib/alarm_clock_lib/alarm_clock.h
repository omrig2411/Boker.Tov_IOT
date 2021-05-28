#ifndef ALARM_CLOCK_H_   /* Include guard */
#define ALARM_CLOCK_H_

#include <Arduino.h>

extern const int RELAY;
extern boolean alarmOn;
extern float DToServer;
extern unsigned long startMillisSnooze;

extern int FSRData1;  
extern int FSRData2;
extern int FSRData3;
extern int FSRData4;
extern int FSRData5;

extern boolean alarmSetOn;

void setWakeUp(int Hour, int Minute);  

void setAlarmConfig(boolean classicWakeUp, boolean snooze, boolean sleepCycle, boolean wakeUpConfirmation, boolean sound, boolean light, boolean vibration);

void setStopAlarmFromMobile(boolean stopAlarmfromMobile);

int checkIfWakeUpWindow();

void triggerWakeUp(int currH, int currM, float predictWakeTime);

void collectvalues();

void identifyMovementPIR();

void identifyMovementFSR(int F1, int F2, int F3, int F4, int F5);

void cleanValue(int loopNum);

void saveValues();

float Dpredict();

int checkSumFSR(int F1, int F2, int F3, int F4, int F5);

void snoozeButtonPush();

void stopAlarmButtonPush();

void startAlarm();

void snoozeBasicFunc(); 

void buzzerSetup();

void buzzerAction();

void lightAction();

void startActuators();

#endif // ALARM_CLOCK_H_