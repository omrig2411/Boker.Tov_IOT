#ifndef ALARM_CLOCK_H_   /* Include guard */
#define ALARM_CLOCK_H_

#include <Arduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

extern WiFiUDP ntpUDP;

extern NTPClient timeClient;

extern const int RELAY;
extern boolean alarmOn;
extern float DToServer;
extern unsigned long startMillisSnooze;
extern struct tm wakeUpWindow;
extern boolean inWindow;
extern int snoozeBasicFuncButton;
extern int stopAlarmButton;
extern boolean vibrate;

extern int FSRData1;  
extern int FSRData2;
extern int FSRData3;
extern int FSRData4;
extern int FSRData5;

extern boolean alarmSetOn;

void ESPSend(int state);

void setWakeUp(int Hour, int Minute);  

void setAlarmConfig(boolean alarmSetOn, boolean classicWakeUp, boolean snooze, boolean sleepCycle, boolean wakeUpConfirmation, boolean sound, boolean light, boolean vibration);

void setStopAlarmFromMobile(boolean stopAlarmfromMobile);

void checkIfWakeUpWindow();

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

void startAlarmState();

void snoozeBasicFunc(); 

void buzzerAction();

void lightAction();

int defineActuators(boolean sound, boolean light, boolean vibrate);

void startActuators(int actuMode);

#endif // ALARM_CLOCK_H_