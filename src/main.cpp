/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"
#include <ArduinoOTA.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

#define ADS1115_SERIAL_DEBUG
#include "ADS1115.h"


#define greenLedPin 13
#define yellowLedPin 12
#define redLedPin 14
#define beeper 16
#define blueChipLedPin 2
#define button 2

#define logToSerial Serial

// =============================================
// global vars

int analog = 0;
float corrected = 0.0;
float coPpm = 0.0;
float temp = 0.0;
float pressure = 0.0;
float humidity = 0.0;

unsigned long lastEventCheckTime = 0;
int eventDelayCountdown = 0;

// ============================================

const char WiFiApName[] = "sensor-co";
const char WiFiApSecret[] = "Sensor123";

String logStorage = String();

ESP8266WebServer httpServer(8090);
ESP8266HTTPUpdateServer httpUpdater;

Adafruit_BME280 bme; 
ADS1115 ads0(0x48); 

// ============================================
// logging 

void cutLog() {

  if (logStorage.length() > 2000) {

    logStorage = logStorage.substring(100);
  }
}

template <typename T>
void LOG(T t) {
  
  logStorage += String(t);
  logStorage += String("\n");
  cutLog();

  #ifdef logToSerial
    logToSerial.println(t);
  #endif
}

// recursive variadic function
template<typename T, typename... Args>
void LOG(T t, Args... args) {

    logStorage += String(t);
    cutLog();

    LOG(args...);
}

// ====================================
// event handling

void applyEventDelay(int delay) {

  eventDelayCountdown = delay;
}

bool couldProcessNextEvent() {
  
  if (eventDelayCountdown <= 1) {

    eventDelayCountdown = 0;
    return true;
  }

  unsigned long currentTime = millis();
  unsigned long timeElapsed = currentTime - lastEventCheckTime;
  lastEventCheckTime = currentTime;

  // String uptimeMessage = String((F((F("event time elapsed: ")))));
  // uptimeMessage += String(timeElapsed);
  // uptimeMessage += String((F((F("ms")))));
  // LOG(uptimeMessage);

  eventDelayCountdown -= timeElapsed;
  
  return false;
}

// ===========================
// State

String stateString() {
  
  String result = String();

  result += ((F("\ndevice config\n")));

  result += ((F("\nSketch size: ")));
  result += String(ESP.getSketchSize());

  result += ((F("\nFree size: ")));
  result += String(ESP.getFreeSketchSpace());

  result += ((F("\nFree Heap: ")));
  result += String(ESP.getFreeHeap());

  return result;
}

// ============================

float readADSValues() {

  LOG((F("Sensor 1 ************************")));
  // Set the gain (PGA) +/- 1.024v
  // ads0.setGain(ADS1115_PGA_0P256);

  // Get the number of counts of the accumulator
  LOG((F("Counts for sensor 1 is:")));
  
  // The below method sets the mux and gets a reading.
  int sensorOneCounts=ads0.getConversionP2N3();  // counts up to 16-bits  
  LOG(sensorOneCounts);

  // To turn the counts into a voltage, we can use
  LOG((F("Voltage for sensor 1 is:")));
  //  float value = sensorOneCounts * ads0.getMvPerCount();
  float value = ads0.getMilliVolts();
  //  float value = sensorOneCounts * 0.007813;
  
  LOG(String(value, 8));   
   
  // // 2nd sensor is on P2/N3 (pins 6/7)
  // LOG((F("Sensor 2 ************************")));
  // // Set the gain (PGA) +/- 0.256v
  // adc0.setGain(ADS1115_PGA_0P256);

  // // Manually set the MUX  // could have used the getConversionP* above
  // adc0.setMultiplexer(ADS1115_MUX_P2_N3); 
  // LOG((F("Counts for sensor 2 is:")));
  // LOG(adc0.getConversion());  

  // LOG((F("mVoltage sensor 2 is:")));
  // LOG(adc0.getMilliVolts());  // Convenience method to calculate voltage

  // LOG();
  
  return value;
}

// =============================
// responds

void respondWithState() {

  String response = stateString();

  httpServer.send(200, (F("text/plain")), response.c_str() );
}

void respondWithLog() {

  String response = String(F("<html><body><p>"));
  response += String(F("<br/>in Access Point mode: "));
  response += String(WiFi.getMode() == WIFI_AP ? (F("YES")):(F("NO")));
  // response += String(F("<br/>apnStartTime: "));
  // response += String(apnStartTime);

  // response += String(F("<br/>Guard Pin Raised: "));
  // response += String(digitalRead(guardPin) == 1 ? (F("YES")):(F("NO")));
  // response += String(F("<br/>Alarm Pin Rased: "));
  // response += String(digitalRead(alarmPin) == 1 ? (F("YES")):(F("NO")));
  // response += String(F("<br/>Next Event Countdown: "));
  // response += String(eventDelayCountdown);
  // response += String(F("<br/>Modem Network Connection Sequental Failures Count: "));
  // response += String(modemConnectionSequentalFailuresCount);
  response += String((F("</p><pre>")));
  response += logStorage;
  response += String((F("</pre></body></html>\n\r")));
  
  httpServer.send(200, (F("text/html")), response.c_str());
}

void respondWithMeasured() {
  // Check if a client has connected

  digitalWrite(blueChipLedPin, HIGH);

  // if (req.indexOf((F("/led/0"))) != -1)

  // Prepare the response. Start with the common header:
  // readCoValues();
  // readBmeValues();

  float a = readADSValues();

  // String s = "HTTP/1.1 200 OK\r\n";
  // s += "Content-Type: application/json\r\n\r\n";
  String response = String((F("{")));

  response += String((F("\"analog\":")));
  response += String(a, 8);
  response += String((F(",")));

  response += String((F("\"coPpm\":")));
  response += String(coPpm, 8);
  response += String((F(",")));

  response += String((F("\"temp\":")));
  response += String(temp, 8);
  response += String((F(",")));

  response += String((F("\"pressure\":")));
  response += String(pressure, 8);
  response += String((F(",")));

  response += String((F("\"humidity\":")));
  response += String(humidity, 8);

  response += String((F("}")));

  // Send the response to the client
  httpServer.send(200, (F("application/json")), response.c_str());
  LOG((F("Client disonnected")));
  digitalWrite(blueChipLedPin, LOW);
  // The client will actually be disconnected
  // when the function returns and 'client' object is detroyed
}

// =============================
// WiFi

String wifiApName() {

  byte mac[6];
   WiFi.macAddress(mac);

   LOG((F("MAC: ")));
   LOG(mac[5],HEX);
   LOG((F(":")));
   LOG(mac[4],HEX);
   LOG((F(":")));
   LOG(mac[3],HEX);
   LOG((F(":")));
   LOG(mac[2],HEX);
   LOG((F(":")));
   LOG(mac[1],HEX);
   LOG((F(":")));
   LOG(mac[0],HEX);

   String result = String(WiFiApName);
   result += String((F(" - ")));
   result += String(mac[5], HEX);
   result += String(mac[4], HEX);

   return result;
}

void printButton() {

  LOG(((F("button pressed"))));
}

/**************************
 *   S E T U P
 **************************/

 void setupWiFi() {
  
   WiFi.softAP(wifiApName().c_str(), WiFiApSecret);
   LOG((F("Finished with wifi setup")));
 }

 void setupServer() {

  MDNS.begin("espco");

  httpUpdater.setup(&httpServer, "/update");

  // httpServer.on("/",[](){httpServer.send(200,"text/plain","Hello World!");});

  httpServer.on("/", respondWithState);
  httpServer.on("/log", respondWithLog);
  httpServer.on("/measure", respondWithMeasured);

  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
 }

 void setupBme() {

   if (!bme.begin(0x76)) {
     LOG((F("Could not find a valid bme280 sensor, check wiring!")));
     //ESP.deepSleep( 30 * 1000000, WAKE_RF_DISABLED);
   } else {
     LOG((F("BME280 found!")));
   }
   LOG((F("Finished with BME setup")));
 }

 void setupADS() {

  LOG((F("start ads")));
  ads0.initialize();
  LOG((F("Getting single-ended readings from AIN0..3")));
  
  
  if (!ads0.testConnection()) {
    LOG((F("Could not find a valid ADS1115, check wiring!")));
  } else {
    LOG((F("ADS1115 found!")));
  }

  // adc0.setMode(ADS1115_MODE_CONTINUOUS);
  ads0.setMode(ADS1115_MODE_SINGLESHOT);
  ads0.setRate(ADS1115_RATE_8);
  // ads0.setGain(ADS1115_PGA_0P512);
  ads0.setGain(ADS1115_PGA_0P256);

  #ifdef ADS1115_SERIAL_DEBUG
  ads0.showConfigRegister();
  LOG((F("HighThreshold="))); LOG(ads0.getHighThreshold(),BIN);
  LOG((F("LowThreshold="))); LOG(ads0.getLowThreshold(),BIN);
  #endif

  // pinMode(1,INPUT_PULLUP);
  // ads0.setConversionReadyPinMode();

  LOG((F("Finished with ADS1115 setup")));
}

void setupSOC() {

  pinMode(blueChipLedPin, OUTPUT);
  pinMode(beeper, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(button, INPUT);

  Wire.begin(5,4);

  attachInterrupt(button, printButton, FALLING);

  LOG((F("SOC setup finished")));
}

void prepareTestPinsState() {

  digitalWrite(beeper, LOW);
  digitalWrite(redLedPin, HIGH);
  digitalWrite(yellowLedPin, HIGH);
  digitalWrite(greenLedPin, HIGH);
  digitalWrite(blueChipLedPin, LOW);
}

void setup() {

  

  logToSerial.begin(74880);
  delay(10);

  LOG((F("Starting setup of the device")));

  setupSOC();

  //  setupBme();
  setupADS();

  setupServer();

  setupWiFi();

  //prepareTestPinsState();
  
  LOG((F("Finished setup of the device")));
}

  /**************************
 *  L O O P
 **************************/

int correctedAnalogRead(int value) {
  int zeroVoltageValue = 10;
  int correctedValue = value - zeroVoltageValue;
  int input = correctedValue > 0 ? correctedValue : 0;
  // measured 3.12 v source stabilazed voltage
  //          Usource(mV) / stepsTotal * currentSteps
  // float result = (3120.0 / float(1024.0-zeroVoltageValue) * float(input));
  // LOG((F("read :")));
  // LOG(value);
  // LOG(result);
  return input;
}

float ppmUsingMiliVolts(float mV) {
  //          sensor U * (10e6(mA to nA) / current resistor(50kOm))
  float nA = mV * (1000000 / 50000);
  //                  sensor current  * sensor nA->ppm coefficient / amplifier coefficient * correction;
  float result = nA * 1.4 / 108 * 6.11;
  // LOG((F("ppm :")));
  // LOG(nA);
  // LOG(result);
  return result;
}




void readBmeValues() {
  temp = bme.readTemperature();
  pressure = bme.readPressure();
  humidity = bme.readHumidity();
}

void readSocAnalog() {
  analog = analogRead(0);
  corrected = correctedAnalogRead(analog);
  coPpm = corrected * 3.5;
}

void loopMeasure() {
  delay(1e3);

  digitalWrite(blueChipLedPin, HIGH);
  float a = readADSValues();
  LOG((F("analog = ")));
  LOG(a, 8);
  LOG((F(" mV")));
  digitalWrite(blueChipLedPin, LOW);
}

void loop_check() {
  delay(1e3);
  readSocAnalog();
  LOG((F("analog = ")));
  LOG(analog);
  LOG((F("corrected = ")));
  LOG(corrected);
  LOG((F("CO = ")));
  LOG(coPpm);
  LOG((F("==")));
}

void loop() {
  //loop_check();
  // loopMeasure();

  httpServer.handleClient();
  yield();
}
