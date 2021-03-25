// Import required libraries
#include <Arduino.h>
#include <analogWrite.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "WiFi.h"
#include "Wire.h"
#include "LiquidCrystal_I2C.h"
#include "credentials.h"


#define TIMESECONDS 3 // Delay between PIR sensor readings
#define WIFI_TIMEOUT_MS 10000
#define Threshold 40 /* Greater the value, more the sensitivity */

// esp_now MAC adress of bed board
uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2, 0x36, 0x58, 0xAC};

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// ntp client variables
WiFiUDP ntpUDP;
// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "il.pool.ntp.org", 2*3600, 6000000);
const int WIFIpin =  25;

// PIR module and LED variables
const int PIRpin =  26;      // the number of the LED pin
const int PIR = 27;
int ledState = LOW;             // ledState used to set the LED
        
// timer variables
unsigned long now_M = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

// FSR variables
int LEDbrightness1;
int LEDbrightness2;
const int FSRbuttonLEDpin1 = 32;
const int FSRbuttonLEDpin2 = 33;

// incoming data from esp32 bed board
int incomingFSR1;      
int incomingFSR2;

// // button variables
// const int buttonPin = 17;     // the number of the pushbutton pin
// const int buttonLEDpin =  18;      // the number of the LED pin
// int buttonState = 0;         // variable for reading the pushbutton status

// data to to be sent to esp32 bed board
boolean startDataCollection;

typedef struct struct_message {
  int FSR1;
  int FSR2;
  boolean startSensors;
} struct_message;

// Create a struct_message called outgoingStartCommand to hold 
struct_message outgoingStartCommand;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// Variable to store if sending data was successful
String success; 

// Register peer variable
esp_now_peer_info_t peerInfo;

// create an LCD object (Hex address, # characters, # rows)
// my LCD display in on Hex address 27 and is a 20x4 version
LiquidCrystal_I2C lcd(0x27, 20, 4); 

//wifi connection
void connectToWiFi() {
  Serial.print("Connecting to wifi");
  lcd.clear();
  lcd.print("Connect wifi");

  WiFi.mode(WIFI_AP_STA);  
  WiFi.softAP("esp_daddy", "123456789");
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();
  
  lcd.setCursor(0,1);
  while(WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
    Serial.print(".");
    lcd.print(".");
    delay(1000);
  };

  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Failed!");
    lcd.clear();
    lcd.print("Failed!");
    Serial.println("Trying again...");
    lcd.setCursor(1,1);
    lcd.print("Trying again...");
    // WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
    connectToWiFi();

  }else{
    Serial.print("Connected! IP:");
    lcd.clear();
    lcd.print("Connected! IP:");
    lcd.setCursor(0,1);
    Serial.println(WiFi.localIP());
    lcd.print(WiFi.localIP());
    digitalWrite(WIFIpin, HIGH);
    delay(3000);
  }
}

void currentTime() {
    lcd.clear();
    lcd.print(timeClient.getFormattedTime());
}

//callback upon sending data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status:");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    Serial.println("Delivery Success :)");
  }else {
    Serial.println("Delivery Fail :(");
  }
  Serial.println("--------------------");
}

//callback upon recieving data
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes recieved: ");
  Serial.println(len);
  incomingFSR1 = incomingReadings.FSR1;
  incomingFSR2 = incomingReadings.FSR2;
}

//used to trigger data collection in the bed board
void getReadings() {
  startDataCollection = 1;

  // // read the state of the pushbutton value:
  // buttonState = digitalRead(buttonPin);
  // Serial.print("Button state: ");
  // Serial.print(buttonState);


  // // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  // if (buttonState == HIGH) {
  //   // turn LED on:
  //   digitalWrite(buttonLEDpin, HIGH);
  //   startDataCollection = buttonState;
  // } else {
  //   // turn LED off:
  //   digitalWrite(buttonLEDpin, LOW);
  //   startDataCollection = buttonState;
  // }
}

void updateDisplay() {
  lcd.setCursor(0, 1);
  lcd.print("1: ");
  lcd.print(incomingFSR1);
  lcd.print(" 2: ");
  lcd.println(incomingFSR2);

  //Serial monitor display
  Serial.println("sensor readings:");
  Serial.print("FSR1: ");
  Serial.println(incomingFSR1);
  Serial.print("FSR2: ");
  Serial.println(incomingFSR2);
  Serial.println("--------------------");
}

// Checks if motion was detected, interupt sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED AT:");

  digitalWrite(PIRpin, HIGH);
  startTimer = true;
  lastTrigger = millis();
  
  Serial.println(daysOfTheWeek[timeClient.getDay()]);
  Serial.println(timeClient.getFormattedTime());
  Serial.println("--------------------");
}

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.clear();

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

  //begin tnpclient
  timeClient.begin();
  // set the digital pin as output:
  pinMode(PIRpin, OUTPUT);
  pinMode(WIFIpin, OUTPUT);
  digitalWrite(PIRpin, LOW);
  // pinMode(buttonLEDpin, OUTPUT);
  // // initialize the pushbutton pin as an input:
  // pinMode(buttonPin, INPUT);

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(PIR, INPUT_PULLUP);

  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(PIR), detectsMovement, HIGH);
}

void loop() {
  timeClient.update();
  now_M = millis();

  currentTime();

  getReadings();

  // Set values to send
  outgoingStartCommand.startSensors = startDataCollection;  
;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingStartCommand, sizeof(outgoingStartCommand));

  if(result == ESP_OK) {
    Serial.println("Sent successfully");
  } 
  else {
    Serial.println("Error sending the data");
  }

  if (startTimer && (now_M - lastTrigger > (TIMESECONDS * 1000))) {
    // Serial.println("Motion stopped");
    // Serial.println("--------------------");
    
    digitalWrite(PIRpin, LOW);
    startTimer = false;
  }

  if(WiFi.status() != WL_CONNECTED){
    Serial.println("wifi Disconnected! ");
    digitalWrite(WIFIpin, LOW);
    connectToWiFi();
  }

  updateDisplay();

  delay(1000);
}