#ifndef ALARM_CLOCK_H_   /* Include guard */
#define ALARM_CLOCK_H_

void setWakeUp(int Hour, int Minute);  /* An example function declaration */

int checkIfWakeUpWindow();

void collectvalues();

int identifyMovement(int fsr1, int fsr2, int fsr3, int fsr4, int fsr5);

void saveValues();

int predictWakeTime();

// void startAlarm(boolean persistentWakeUp);

void startActuators();

#endif // ALARM_CLOCK_H_