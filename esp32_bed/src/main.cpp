// Import required libraries
#include <Arduino.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <analogWrite.h>
#include <esp_now.h>
#include <esp_wifi.h>

//wifi credentials
#define WIFI_NETWORK "AndroidO"
#define WIFI_PASSWORD "123456789"
#define WIFI_TIMEOUT_MS 10000

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2, 0x35, 0x32, 0x9C};

const int WIFIpin = 25;

//fsr variables
const int fsrAnalogPin1 = 34; // first FSR is connected to analog 34
const int fsrAnalogPin2 = 35; // second FSR is connected to analog 35
int fsrReading1;              // the analog reading from the FSR resistor divider
int fsrReading2;
int LEDbrightness1;
int LEDbrightness2;
const int FSRLEDpin1 = 32;
const int FSRLEDpin2 = 33;

// data to to be sent to esp32 box board
int outgoingFSR1;      // the analog reading from the FSR resistor divider
int outgoingFSR2;

// incoming data from esp32 box board
boolean startDataCollection;

typedef struct struct_message {
  int FSR1;
  int FSR2;
  boolean startSensors = 0;
} struct_message;

// Create a struct_message called incomingStartCommand to hold 
struct_message incomingStartCommand;

// Create a struct_message to hold outgoing sensor readings
struct_message outgoingReadings;

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

//callback upon sending data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Fail" : "Delivery Success");
  if (status == 0) {
    Serial.println("Delivery Success :)");
  }else {
    Serial.println("Delivery Fail :(");
  }
  Serial.println("--------------------");
}

//callback upon recieving data
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingStartCommand, incomingData, sizeof(incomingStartCommand));
  Serial.print("Bytes recieved: ");
  Serial.println(len);
  startDataCollection = incomingStartCommand.startSensors;
}

//sensor reading to be sent
void getReadings() {
  if (startDataCollection == 1){
    fsrReading1 = analogRead(fsrAnalogPin1);
    fsrReading2 = analogRead(fsrAnalogPin2);
  } 
}

void updateDisplay() {
  //Serial monitor display
  Serial.print("sensors start: ");
  Serial.println(startDataCollection);
  Serial.println("--------------------");
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);

  connectToWiFi();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  // esp_now_peer_info_t peerInfo;
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

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(FSRLEDpin1, OUTPUT);
  pinMode(FSRLEDpin2, OUTPUT);
  pinMode(WIFIpin, OUTPUT);
}

void loop() {
  getReadings();

  // Set values to send
  outgoingReadings.FSR1 = fsrReading1;
  outgoingReadings.FSR2 = fsrReading2;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));

  if(result == ESP_OK) {
    Serial.println("Sent successfully");
  } 
  else {
    Serial.println("Error sending the data");
  }

  Serial.print("FSR 1 = ");
  Serial.println(fsrReading1);
  Serial.print("FSR 2 = ");
  Serial.println(fsrReading2);

  // we'll need to change the range from the analog reading (0-1023) down to the range
  // used by analogWrite (0-255) with map!
  LEDbrightness1 = map(fsrReading1, 0, 1023, 0, 255);
  LEDbrightness2 = map(fsrReading2, 0, 1023, 0, 255);

  // LED gets brighter the harder you press
  analogWrite(FSRLEDpin1, LEDbrightness1);
  analogWrite(FSRLEDpin2, LEDbrightness2);

  Serial.println("--------------------");

  if (WiFi.status() != WL_CONNECTED){
    Serial.println("wifi Disconnected! ");
    digitalWrite(WIFIpin, LOW);
    connectToWiFi();
  }

  updateDisplay();
  
  delay(1000);
}