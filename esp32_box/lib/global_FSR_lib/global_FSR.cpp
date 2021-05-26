#include "global_FSR.h"  /* Include the header*/
#include <Arduino.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>



typedef struct struct_message_in {
  int FSR1;
  int FSR2;
  int FSR3;
  int FSR4;
  int FSR5;
  boolean reSend = 0;
} struct_message_in;

typedef struct struct_message_out {
  boolean startSampling;
  boolean goToSleep;
} struct_message_out;

// Create a struct_message_in called outgoingCommand to hold 
struct_message_out outgoingCommand;

// Create a struct_message_in to hold incoming sensor readings
struct_message_in incomingReadings;