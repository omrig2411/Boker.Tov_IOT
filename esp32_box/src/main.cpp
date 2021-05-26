#include <Arduino.h>
#include <analogWrite.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <time.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Wire.h"
#include "LiquidCrystal_I2C.h"
#include "credentials.h"
#include "alarm_clock.h"

#define TIMESECONDS 3 // Delay between PIR sensor readings
#define WIFI_TIMEOUT_MS 10000 // Time to try connecting to wifi until timeout (in miliseconds)
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Duration of time ESP32 will go to sleep (in seconds) */
// #define sensor_pin_bitmask 0x8000000 // bitmask to determine rtc GPIO pins to monitor for deep sleep wakeup

//loop function counter
int loopCounter = 0;

// esp_now MAC adress of bed board
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};/*{0xAC, 0x67, 0xB2, 0x36, 0x58, 0xAC}*/

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// ntp client variables
WiFiUDP ntpUDP;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "il.pool.ntp.org", 3*3600, 6000000);
const int WIFIpin =  25;

RTC_DATA_ATTR int bootCount = 0; // this attribute will keep its value during a deep sleep / wake cycle

// Store the MQTT server, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

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

// incoming data from esp32 bed board
int incomingFSR1;      
int incomingFSR2;
int incomingFSR3;
int incomingFSR4;
int incomingFSR5;
boolean reSend = 0;

// alarm clock variables
int alarmHour;
int alarmMin;
int alarmSetOn;
int prevAlarmHour;
int prevAlarmMin;
int inWakeUpWin = 0;
boolean alarmStarted;
int sumFSRSensors = 0;

// MQTT user wake-up preferences variables
int wakeUpHour;
int wakeUpMinute;
boolean classicWakeUp;
boolean sleepCycle;
boolean wakeUpConfirmation;
boolean sound;
boolean light;
boolean vibration;

/* flag variable to to be sent to esp32 bed-
start sampling FSR sensors in esp32 bed
ESP32 bed go to sleep */
boolean startDataSampling = 1;
boolean goToSleep = 1;

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

// Variable to store if sending data was successful
String success; 

// Register peer variable
esp_now_peer_info_t peerInfo;

// Create an ES32 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

// Create a JSON document
StaticJsonDocument<256> doc;

// create an LCD object (Hex address, # characters, # rows)
// my LCD display in on Hex address 27 and is a 20x2 version
LiquidCrystal_I2C lcd(0x27, 20, 2); 

//struct to hold raw data from FSR
typedef struct FSRData {
  int FSRData1 [70];
  int FSRData2 [70];
  int FSRData3 [70];
  int FSRData4 [70];
  int FSRData5 [70];
} FSRData;

//instance of FSR raw data struct
FSRData FSRD;

/****************************** Feeds ***************************************/

// Setup a feed called 'server' for subscribing to changes.
const char server_FEED[] PROGMEM = AIO_USERNAME "/feeds/server";
Adafruit_MQTT_Subscribe server = Adafruit_MQTT_Subscribe(&mqtt, server_FEED);

// Setup a feed called 'mobile' for publishing changes.
Adafruit_MQTT_Publish server2 = Adafruit_MQTT_Publish(&mqtt, server_FEED);

// Setup a feed called 'mobile' for subscribing to changes.
const char mobile_FEED[] PROGMEM = AIO_USERNAME "/feeds/mobile";
Adafruit_MQTT_Subscribe mobile = Adafruit_MQTT_Subscribe(&mqtt, mobile_FEED);

// Setup a feed called 'mobile' for publishing changes.
Adafruit_MQTT_Publish mobile2 = Adafruit_MQTT_Publish(&mqtt, mobile_FEED);

// Wifi connection function
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

// ESP_NOW connection function
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

// MQTT connection function
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.println("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 2 seconds...");
       mqtt.disconnect();
       delay(2000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

// Current time display on LCD screen function
void displayCurrentTime() {
    lcd.clear();
    lcd.print(timeClient.getFormattedTime());
}

//callback upon sending data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status:");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println("--------------------");
}

//callback upon recieving data
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  // Serial.print("Bytes recieved: ");
  // Serial.println(len);
  FSRD.FSRData1 [loopCounter]= incomingReadings.FSR1;
  FSRD.FSRData2 [loopCounter]= incomingReadings.FSR2;
  FSRD.FSRData3 [loopCounter]= incomingReadings.FSR3;
  FSRD.FSRData4 [loopCounter]= incomingReadings.FSR4;
  FSRD.FSRData5 [loopCounter]= incomingReadings.FSR5;
  reSend = incomingReadings.reSend;
}

// Update LCD screen display
void updateDisplay() {
  // Serial monitor display
  Serial.println("sensor readings:");
  Serial.print("FSR 3: ");
  Serial.println(FSRD.FSRData1 [loopCounter]);
  Serial.print("FSR 3: ");
  Serial.println(FSRD.FSRData2 [loopCounter]);
  Serial.print("FSR 3: ");
  Serial.println(FSRD.FSRData3 [loopCounter]);
  Serial.print("FSR 4: ");
  Serial.println(FSRD.FSRData4 [loopCounter]);
  Serial.print("FSR 5: ");
  Serial.println(FSRD.FSRData5 [loopCounter]);
  Serial.println("--------------------");

  // LCD monitor display
  // lcd.setCursor(0, 1);
  // lcd.print("1: ");
  // lcd.print(incomingFSR1);
  // lcd.print(" 2: ");
  // lcd.println(incomingFSR2);
}

// Method to print the reason by which ESP32 has been awaken from sleep
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

// Checks if motion was detected, interupt sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED AT:");

  digitalWrite(PIRpin, HIGH);
  startTimer = true;
  lastTrigger = millis();
  
  Serial.println(daysOfTheWeek[timeClient.getDay()]);
  Serial.println(timeClient.getFormattedTime());
  Serial.println("--------------------");
    
  // Motion detection led pin timer
  if (startTimer && (now_M - lastTrigger > (TIMESECONDS * 1000))) {
    digitalWrite(PIRpin, LOW);
    startTimer = false;
  }
}

// Function to set the values to be sent to ESP32 bed
void ESPSend(int state) {
  if(state == 0) {
    // Set values to start data sampling and send to ESP32 bed
    outgoingCommand.startSampling = startDataSampling; 
  }
  else if (state == 1) {
    // Set values to go to sleep and send to ESP32 bed
    outgoingCommand.goToSleep = goToSleep;
  }
   
  // Send message via ESP-NOW to ESP32 bed
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingCommand, sizeof(outgoingCommand));
  if(result != ESP_OK) {
    Serial.println("Error sending the data");
  }
}

/* deep sleep starting function, determines duration of ESP32 staying awake.
60000 = 1 minute, 300000 = 5 minutes, 600000 = 10 minutes, 7200000 = 2 hours */
void ESPGoToSleep() {
  //Go to sleep if
  if (inWakeUpWin == 0 && millis() > 60000) {
    ESPSend(1);
    Serial.println("sleeping");
    lcd.clear();
    lcd.print("sleeping");
    lcd.off();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }
}

void fetchJson(byte* payload) {
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, payload);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Fetch values
   wakeUpHour = doc["wuh"];
   wakeUpMinute = doc["wum"];
   if(doc["cup"] == "false") {
     classicWakeUp = 0;
   }else classicWakeUp = 1;
   if(doc["sc"] == "false") {
     sleepCycle = 0;
   }else sleepCycle = 1;
   if(doc["wuc"] == "false") {
     wakeUpConfirmation = 0;
   }else wakeUpConfirmation = 1;
   if(doc["s"] == "false") {
     sound = 0;
   }else sound = 1;
   if(doc["l"] == "false") {
     light = 0;
   }else light = 1;
   if(doc["v"] == "false") {
     vibration = 0;
   }else vibration = 1;

  // Print values.
  Serial.println(wakeUpHour);
  Serial.println(wakeUpMinute);
  Serial.println(classicWakeUp);
  Serial.println(sleepCycle);
  Serial.println(wakeUpConfirmation);
  Serial.println(sound);
  Serial.println(light);
  Serial.println(vibration);
}

// Check for new MQTT feed subscription messages
void checkSubscription() {
  Adafruit_MQTT_Subscribe *subscription;
  if ((subscription = mqtt.readSubscription(200))) {
    if (subscription == &server) {
      Serial.print(F("Got: "));
      Serial.println((char *)server.lastread);
      fetchJson(server.lastread);
    }
    if (subscription == &mobile) {
      Serial.print(F("Got: "));
      Serial.println((char *)mobile.lastread);
      // doc["wakeUpHour"] = 
      // Serial.println(server.)
    }
  }
}

// Check the sum of all current readings to determine if user is on bed or not
void checkSumFSR() {
  sumFSRSensors = incomingReadings.FSR1 + incomingReadings.FSR2 + incomingReadings.FSR3 + incomingReadings.FSR4 + incomingReadings.FSR5;
}

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  lcd.println("Boot number: " + String(bootCount));
  
  //Print the wakeup reason for ESP32 and touchpad too
  print_wakeup_reason();

  connectToWiFi();
  connectESPNow();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  //begin ntpclient
  timeClient.begin();

  // set the digital pin as output:
  pinMode(PIRpin, OUTPUT);
  pinMode(WIFIpin, OUTPUT);
  digitalWrite(PIRpin, LOW);

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(PIR, INPUT_PULLUP);

  alarmSetOn = 0;
  alarmStarted = false;

  // Generate the prettified JSON and send it to the Serial port.
  serializeJsonPretty(doc, Serial);
  Serial.println();


  //set callback to fetch MQTT messages
  void setCallback(SubscribeCallbackUInt32Type deserializeJson);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(PIR), detectsMovement, HIGH);

  // Configure timer as wakeup source
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep " + String(TIME_TO_SLEEP / 60) +
  " Minutes intervals");

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&server);
  mqtt.subscribe(&mobile);

  /* Send message to ESP32 bed to start collecting sensors data 
  one time only on ESP32 box setup() */
  ESPSend(0);
}

void loop() {
  timeClient.update();
  now_M = millis();
  displayCurrentTime();
  MQTT_connect();

  // Wifi connection status
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("wifi Disconnected! ");
    digitalWrite(WIFIpin, LOW);
    connectToWiFi();
  }

  // ESP_NOW check message recieved by ESP32 bed
  if(reSend == 1) {
    ESPSend(0);
  }
  
  checkSubscription();

  /***************wake-up sequence of events****************/
  
  // // Check wake up window
  // inWakeUpWin = checkIfWakeUpWindow();
  // if (inWakeUpWin == 1) {
  //   if(predict() == true) { // Start prediction function if inside window of upcoming wake up
  //     // startAlarm(doc["wakeUpConfirmation"]);
  //   }  
  // }

  updateDisplay();
  ++loopCounter;
  Serial.println("loop number: " + String(loopCounter));
  ESPGoToSleep();
  delay(1000);
}