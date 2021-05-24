#ifndef ALARM_CLOCK_H_   /* Include guard */
#define ALARM_CLOCK_H_

void setWakeUp(int Hour, int Minute);  /* An example function declaration */

int checkIfWakeUpWindow();

void collectvalues();

void saveValues();

void predict();

// void startAlarm(boolean persistentWakeUp);

void startActuators();

#endif // ALARM_CLOCK_H_