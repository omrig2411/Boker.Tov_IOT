// Import required libraries
#include <Arduino.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <analogWrite.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "credentials.h"

//wifi credentials
#define WIFI_TIMEOUT_MS 5000            // Time to try connecting to wifi until timeout
#define uS_TO_S_FACTOR 1000000          // Conversion factor for micro seconds to seconds 
#define uS_TO_M_FACTOR 60000000         // Conversion factor for micro seconds to minutes
#define ms_TO_M_FACTOR 60000            // Conversion factor for milli seconds to minutes
#define TIME_TO_SLEEP  1                // Duration of time ESP32 will go to sleep (in seconds) 

//deep sleep variables
int wakeUpStart = 0;
boolean goToSleep = 0;

// esp_now MAC adress of box board
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; /*{0xAC, 0x67, 0xB2, 0x35, 0x32, 0x9C}*/

const int WIFIpin = 21;

RTC_DATA_ATTR int bootCount = 0; // this attribute will keep its value during a deep sleep / wake cycle

//fsr variables
const int fsrAnalogPin1 = 39; // First FSR is connected to analog 39
const int fsrAnalogPin2 = 34; // Second FSR is connected to analog 34
const int fsrAnalogPin3 = 35; // Third FSR is connected to analog 35
const int fsrAnalogPin4 = 32; // Fourth FSR is connected to analog 32
const int fsrAnalogPin5 = 33; // Fifth FSR is connected to analog 33
int fsrReading1;              // the analog reading from the FSR resistor divider
int fsrReading2;
int fsrReading3;
int fsrReading4;
int fsrReading5;
int LEDbrightness1;
int LEDbrightness2;


// data to to be sent to esp32 box board
int outgoingFSR1;      // the analog reading from the FSR resistor divider
int outgoingFSR2;
int outgoingFSR3;
int outgoingFSR4;
int outgoingFSR5;

// Vibration variable
const int vibrationMotorsPin = 16;
boolean vibrate = 0;

// incoming flag variable from esp32 box - start sending FSR sensors data to esp32 box 
boolean startDataCollection;

typedef struct struct_message_out {
  int FSR1;
  int FSR2;
  int FSR3;
  int FSR4;
  int FSR5;
  boolean reSend = 0;
  boolean reSendvib = 0;
} struct_message_out;

typedef struct struct_message_in {
  boolean startSensors = 0;
  boolean goToSleep;
  boolean startVibrationMotors;
} struct_message_in;

// Create a struct_message_out called incomingMessage to hold 
struct_message_in incomingMessage;

// Create a struct_message_out to hold outgoing sensor readings
struct_message_out outGoingReadings;

// Variable to store if sending data was successful
String success; 

// Register peer variable
esp_now_peer_info_t peerInfo;

//wifi connection
void connectToWiFi() {
  Serial.print("Connecting to wifi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS)
  {
    Serial.print(".");
    delay(1000);
  };

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Failed!");
    Serial.println("trying again...");
    // WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
    connectToWiFi();
  }
  else
  {
    Serial.print("Connected! IP:");
    Serial.println(WiFi.localIP());
    digitalWrite(WIFIpin, HIGH);
  }
}

void connectESPNow() {
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; 
  if (peerInfo.channel != WiFi.channel()) {
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(WiFi.channel(), WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
  } 
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

//callback upon sending data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\n@@@@@ Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//callback upon recieving data
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  Serial.print("@@@@@ Bytes recieved: ");
  Serial.println(len);
  startDataCollection = incomingMessage.startSensors;
  Serial.print("@@@@@ before data recv fuction: ");
  Serial.println(goToSleep);
  goToSleep = incomingMessage.goToSleep;
  Serial.print("@@@@@ after data recv fuction: ");
  Serial.println(goToSleep);
  vibrate = incomingMessage.startVibrationMotors;
  Serial.print("@@@@@ Vibration value: ");
  Serial.println(vibrate);
}

void ESPSend() {
  // Set values to send
  outGoingReadings.FSR1 = fsrReading1;
  outGoingReadings.FSR2 = fsrReading2;
  outGoingReadings.FSR3 = fsrReading3;
  outGoingReadings.FSR4 = fsrReading4;
  outGoingReadings.FSR5 = fsrReading5;
  if(incomingMessage.startSensors == 0) {
    outGoingReadings.reSend = 1;
  } else outGoingReadings.reSend = 0;
  if(incomingMessage.startVibrationMotors == 1) {
    outGoingReadings.reSendvib = 1;
  } else outGoingReadings.reSendvib = 0;
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outGoingReadings, sizeof(outGoingReadings));
  if(result != ESP_OK) {
    Serial.println("Error sending the data");
  }
}

//sensor reading to be sent
void getReadings() {
  if (startDataCollection == 1){
    fsrReading1 = analogRead(fsrAnalogPin1);
    fsrReading2 = analogRead(fsrAnalogPin2);
    fsrReading3 = analogRead(fsrAnalogPin3);
    fsrReading4 = analogRead(fsrAnalogPin4);
    fsrReading5 = analogRead(fsrAnalogPin5);
  }
  else if(startDataCollection == 0) {
    ESPSend();
  }
}

void updateDisplay() {
  //Serial monitor display
  Serial.print("sensors start: ");
  Serial.println(startDataCollection);
  Serial.println("--------------------");
}

//Method to print the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

//deep sleep starting function, determines duration of ESP32 staying awake.
// 60000 = 1 minute, 300000 = 5 minutes, 600000 = 10 minutes, 7200000 = 2 hours
void ESPGoToSleep() {
    if (goToSleep == 1) { 
    Serial.print("deep sleep function: ");
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
    
  //Print the wakeup reason for ESP32 and touchpad too
  print_wakeup_reason();

  delay(1000);

  connectToWiFi();
  connectESPNow();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(WIFIpin, OUTPUT);
  pinMode(vibrationMotorsPin, OUTPUT);

  // Configure timer as wakeup source
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_M_FACTOR);
  Serial.println("Setup ESP32 to sleep " + String(TIME_TO_SLEEP) +
  " Minutes intervals");
}

void loop() {
  getReadings();

  ESPSend();

  Serial.print("FSR 1 = ");
  Serial.println(fsrReading1);
  Serial.print("FSR 2 = ");
  Serial.println(fsrReading2);
  Serial.print("FSR 3 = ");
  Serial.println(fsrReading3);
  Serial.print("FSR 4 = ");
  Serial.println(fsrReading4);
  Serial.print("FSR 5 = ");
  Serial.println(fsrReading5);

  // we'll need to change the range from the analog reading (0-1023) down to the range
  // used by analogWrite (0-255) with map!
  LEDbrightness1 = map(fsrReading1, 0, 1023, 0, 255);
  LEDbrightness2 = map(fsrReading2, 0, 1023, 0, 255);

  Serial.println("--------------------");

  if (WiFi.status() != WL_CONNECTED){
    Serial.println("wifi Disconnected! ");
    digitalWrite(WIFIpin, LOW);
    connectToWiFi();
  }

  if(vibrate == 1) {
    int i;
    for(i = 0; i <= 3; i++) {
      digitalWrite(vibrationMotorsPin, HIGH);
      delay(200);
      digitalWrite(vibrationMotorsPin, LOW);
    } 
  }

  updateDisplay();
  
  ESPGoToSleep();

  delay(1000);
}