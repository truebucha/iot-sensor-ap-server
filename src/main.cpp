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


#define sdaPin 5
#define slcPin 4

#define greenLedPin 13
#define yellowLedPin 12
#define redLedPin 14
#define buzzerPin 16
#define blueChipLedPin 2
#define buttonPin 2

#define logToSerial Serial

// ===========================================
// CO alarm values

#define greenCOTierValue 0.0000001
#define yellowCOTierValue 0.00001
#define redCOTierValue 0.001

// =============================================
// global vars

const int stationTimeoutSec = 20;
const int apnTimeoutSec = 120; 
unsigned long apnStartTime = 0;
unsigned long lastEventCheckTime = 0;

// mDNS

const char socNetworkName[] = "cosensor";

//AP
const char apnName[] = "CoSensor";
const char apnPass[] = "Sensor123";

const IPAddress localIP(192,168,4,1);
const IPAddress gateway(192,168,4,1);
const IPAddress subnet(255,255,255,0);

// Station
const char satationAp[]  = "***";
const char satationPass[] = "***";

const int socLoopCycleDelay = 1e2;
const int eventsLoopDelay = 2e3;
const int repeatingEventsDelay = 5e3;

unsigned long systemTime = 0;
unsigned long repeatingEventsNextTimeRun = 0;
unsigned long eventsLoopNextTimeRun = 0;

typedef enum EventType {

    NULL_EVENT = 0,
    SINGLE_BEEP_EVENT = 1,
    SINGLE_CO_MEASURE_EVENT = 2,
    SINGLE_ENVIRONMENT_MEASURE_EVENT = 3,

    ALARM_STATE_DID_CHANGE_EVENT = 4,

    PERMANENT_BEEP_INITIATION_EVENT = 50, PERMANENT_BEEP_DISSMISAL_EVENT = 51,
    GREEN_LEED_INITIATION_EVENT = 70, GREEN_LEED_DISSMISAL_EVENT = 71,
    YELLOW_LEED_INITIATION_EVENT = 80, YELLOW_LEED_DISSMISAL_EVENT = 81,
    RED_LEED_INITIATION_EVENT = 90, RED_LEED_DISSMISAL_EVENT = 91,

} EventType_t;

typedef enum BeepType {

  NO_BEEP = 0, SINGLE_BEEP = 1, PERMANENT_BEEP = 2
} BeepType_t;

typedef enum AlarmType {

    NO_ALARM = 0, GREEN_ALARM = 1, YELLOW_ALARM = 2, RED_ALARM = 3
} AlarmType_t;

const int maxEventCount = 20;
int eventsCount = 0;
EventType_t eventsQueue[maxEventCount];

BeepType_t beepTypeValue;
AlarmType_t alarmTypeValue;

int analog = 0;
float analogCO = 0;
float corrected = 0.0;
float coPpm = 0.0;
float temp = 0.0;
float pressure = 0.0;
float humidity = 0.0;

// ============================================

String logStorage = String();

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

Adafruit_BME280 bme; 
ADS1115 ads0(0x48); 

// ===========================================
// alarm type determination logic

AlarmType_t checkForAlarmTypeusingCoValue(float coLevel) {

   if (coLevel >= redCOTierValue) {
    
    return RED_ALARM;
  }

   if (coLevel >= yellowCOTierValue) {
    
    return YELLOW_ALARM;
  }

  if (coLevel >= greenCOTierValue) {

    return GREEN_ALARM;
  }

  if (coLevel < 0.000001) {

    return NO_ALARM;
  }
}

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

void doActionForEvent(EventType_t event);

void scheduleEvent(EventType_t event) {

  if (eventsCount >= maxEventCount) {
    return;
  }

  eventsQueue[eventsCount] = event;
  eventsCount += 1;

  LOG(String((F("Sheduled:"))) + String(event, DEC));
}

EventType_t retriveEvent() {

  int index = eventsCount - 1;

  if (index < 0) {
    return NULL_EVENT;
  }

  eventsCount -= 1;
  EventType_t result = eventsQueue[index];

  LOG(String((F("Retrived:"))) + String(result, DEC));

  return result;
}

bool processEvents() {

  bool result = false;

  EventType_t event = retriveEvent();

  while (event != NULL_EVENT) {

    doActionForEvent(event);
    yield();
    event = retriveEvent();

    result = true;
  }

  return result;
}

// bool applyTimedDelayFor(int delay, int * inOutCountdown) {
  
//   *inOutCountdown = delay;
//   // LOG(String((F("delay: "))) + (String(*countdown, DEC)));
// }

bool couldDoTimedFor(unsigned long * inOutCountdown, int nextTimeAfterDelay) {

  // LOG(String((F("timed: "))) + (String(*countdown, DEC)));

  unsigned long currentTime = millis();

  if (systemTime > currentTime) {

    // soc goes througth overflow of unsigned long 
    // and begin from 0 again
    // we reset events timers

    repeatingEventsNextTimeRun = 0;
    eventsLoopNextTimeRun = 0;
  }

  systemTime = currentTime;

  if (*inOutCountdown < systemTime) {

    *inOutCountdown += nextTimeAfterDelay;
    return true;
  }

  // String uptimeMessage = String((F((F("event time elapsed: ")))));
  // uptimeMessage += String(timeElapsed);
  // uptimeMessage += String((F((F("ms")))));
  // LOG(uptimeMessage);

  return false;
}


// =========================================
// soc event timers

bool couldSheduleRepeatingEvents() {

  bool result = couldDoTimedFor(&repeatingEventsNextTimeRun, repeatingEventsDelay);

  // LOG(String((F("schedule countdown: ")))+ (String(repeatingEventScheduleCountdown, DEC)));

  return result;
}

bool couldProcessEventsLoop() {

  bool result = couldDoTimedFor(&eventsLoopNextTimeRun, eventsLoopDelay);

  // LOG(String((F("events loop countdown: ")))+ (String(eventsLoopCountDown, DEC)));

  return result;
}

// ===========================
// wifi

String wifiApName() {

   byte mac[6];
   WiFi.macAddress(mac);
   String macString = String((F("MAC: ")));
   macString += String(mac[5],HEX);
   macString += String((F(":")));
   macString += String(mac[4],HEX);
   macString += String((F(":")));
   macString += String(mac[3],HEX);
   macString += String((F(":")));
   macString += String(mac[2],HEX);
   macString += String((F(":")));
   macString += String(mac[1],HEX);
   macString += String((F(":")));
   macString += String(mac[0],HEX);
   LOG(macString);

   String result = String(apnName);
   result += String((F("-")));
   result += String(mac[5], HEX);
   result += String(mac[4], HEX);

   return result;
}

void checkWifiConnection() {

  unsigned long apnUptime = 0;

  if (WiFi.getMode() == WIFI_AP) {

    apnUptime = millis() - apnStartTime;

    String uptimeMessage = String((F("Apn uptime: ")));
    uptimeMessage += String(apnUptime / 1000);
    uptimeMessage += String((F("s")));
    LOG(uptimeMessage);
  }

  if (WiFi.getMode() == WIFI_AP
     && WiFi.softAPgetStationNum() == 0 
     && apnUptime > (apnTimeoutSec * 1000)) {

    LOG((F("Disconnect AP during no clients")));
    WiFi.softAPdisconnect(true);
  }

  if (WiFi.getMode() == WIFI_AP
     || WiFi.status() == WL_CONNECTED) {

    LOG((F("WiFi Ok")));
    return;
  } 

  WiFi.mode(WIFI_STA);
  WiFi.begin(satationAp, satationPass);

  LOG((F("Connecting station")));

  int ms = 0;
  int timeout = stationTimeoutSec * 1000;

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);
    ms += 500;
    LOG((F(".")));
    if (ms > timeout) {

      LOG((F("Aborted during timeout")));
      WiFi.disconnect();
      break;
    }
  }

  if  (WiFi.status() == WL_CONNECTED) {

    LOG((F("Station connected after ")));
    LOG(ms);
    LOG((F("ms, IP address: ")));
    LOG(WiFi.localIP().toString());
    return;
  }

  LOG((F("Starting Access Point...")));
  WiFi.mode(WIFI_AP);



  WiFi.softAPConfig(localIP, gateway, subnet);
  if (WiFi.softAP(wifiApName().c_str(), apnPass) == false) {

    return;
  }

  apnStartTime = millis();

  LOG((F("Acceess Point is up, IP address: ")));
  LOG(WiFi.softAPIP().toString());

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
  // LOG((F("Counts for sensor 1 is:")));
  
  // // The below method sets the mux and gets a reading.
  // int sensorOneCounts=ads0.getConversionP2N3();  // counts up to 16-bits  
  // LOG(sensorOneCounts);

  // To turn the counts into a voltage, we can use
  //  float value = sensorOneCounts * 0.007813;
  float value = ads0.getMilliVolts();
  LOG(String((F("Voltage for sensor 1 is:"))) + String(value, 8));
  //  float value = sensorOneCounts * ads0.getMvPerCount();

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

// ============================
// actions

void doSingleBeep() {

  if (beepTypeValue != NO_BEEP) {
    return;
  }

  beepTypeValue = SINGLE_BEEP;

  digitalWrite(buzzerPin, HIGH);
  delay(5e2);
  digitalWrite(buzzerPin, LOW);

  beepTypeValue = NO_BEEP;

  LOG(((F("made single beep sound"))));
}

void doDeviceSetupForAlarm(AlarmType_t alarm) {

  switch (alarm) {

      case GREEN_ALARM:

        scheduleEvent(PERMANENT_BEEP_INITIATION_EVENT);
        scheduleEvent(GREEN_LEED_INITIATION_EVENT);

      break;

      case YELLOW_ALARM:

        scheduleEvent(PERMANENT_BEEP_INITIATION_EVENT);
        scheduleEvent(YELLOW_LEED_INITIATION_EVENT);

      break;

      case RED_ALARM:

        scheduleEvent(PERMANENT_BEEP_INITIATION_EVENT);
        scheduleEvent(RED_LEED_INITIATION_EVENT);

      break;

      case NO_ALARM:
      default:

        scheduleEvent(PERMANENT_BEEP_DISSMISAL_EVENT);
        scheduleEvent(GREEN_LEED_DISSMISAL_EVENT);
        scheduleEvent(YELLOW_LEED_DISSMISAL_EVENT);
        scheduleEvent(RED_LEED_DISSMISAL_EVENT);

      break;
  }
}

// =============================
// events

void doActionForEvent(EventType_t event) {
  
  switch (event) {

    case SINGLE_BEEP_EVENT:

      LOG(((F("{ start SINGLE_BEEP_EVENT"))));
      doSingleBeep();
      LOG(((F("made SINGLE_BEEP_EVENT }"))));

    break;

    case SINGLE_CO_MEASURE_EVENT: 

      LOG(((F("{ start SINGLE_CO_MEASURE_EVENT"))));
      analogCO = readADSValues();
      LOG(((F("made SINGLE_CO_MEASURE_EVENT }"))));
    break;

    case ALARM_STATE_DID_CHANGE_EVENT: 

      LOG(((F("{ start ALARM_STATE_DID_CHANGE_EVENT"))));
      doDeviceSetupForAlarm(alarmTypeValue);
      LOG(((F("made ALARM_STATE_DID_CHANGE_EVENT }"))));
    break;

    case PERMANENT_BEEP_INITIATION_EVENT: 

      LOG(((F("{ start PERMANENT_BEEP_INITIATION_EVENT"))));
      beepTypeValue = PERMANENT_BEEP;
      digitalWrite(buzzerPin, HIGH);
      LOG(((F("made PERMANENT_BEEP_INITIATION_EVENT }"))));
    break;

    case PERMANENT_BEEP_DISSMISAL_EVENT: 

      LOG(((F("{ start PERMANENT_BEEP_DISSMISAL_EVENT"))));
      beepTypeValue = NO_BEEP;
      digitalWrite(buzzerPin, LOW);
      LOG(((F("made PERMANENT_BEEP_DISSMISAL_EVENT }"))));
    break;

    case GREEN_LEED_INITIATION_EVENT: 

      LOG(((F("{ start made GREEN_LEED_INITIATION_EVENT"))));
      digitalWrite(greenLedPin, HIGH);
      LOG(((F("made GREEN_LEED_INITIATION_EVENT }"))));
    break;

    case GREEN_LEED_DISSMISAL_EVENT: 

      LOG(((F("{ start GREEN_LEED_DISSMISAL_EVENT"))));
      digitalWrite(greenLedPin, LOW);
      LOG(((F("made GREEN_LEED_DISSMISAL_EVENT }"))));
    break;

    case YELLOW_LEED_INITIATION_EVENT: 

      LOG(((F("{ start YELLOW_LEED_INITIATION_EVENT"))));
      digitalWrite(yellowLedPin, HIGH);
      LOG(((F("made YELLOW_LEED_INITIATION_EVENT }"))));
    break;

    case YELLOW_LEED_DISSMISAL_EVENT: 

      LOG(((F("{ start YELLOW_LEED_DISSMISAL_EVENT"))));
      digitalWrite(yellowLedPin, LOW);
      LOG(((F("made YELLOW_LEED_DISSMISAL_EVENT }"))));
    break;

    case RED_LEED_INITIATION_EVENT: 

      LOG(((F("{ start RED_LEED_INITIATION_EVENT"))));
      digitalWrite(redLedPin, HIGH);
      LOG(((F("made RED_LEED_INITIATION_EVENT }"))));
    break;

    case RED_LEED_DISSMISAL_EVENT: 

      LOG(((F("{ start RED_LEED_DISSMISAL_EVENT"))));
      digitalWrite(redLedPin, LOW);
      LOG(((F("made RED_LEED_DISSMISAL_EVENT }"))));
    break;

    default:

    break;
  }
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
  response += String(F("<br/>System time: "));
  response += String(systemTime, DEC);
  response += String(F("<br/>Next Event Loop Countdown: "));
  response += String(long(eventsLoopNextTimeRun - systemTime), DEC);
  response += String(F("<br/>Next Repeating Events Countdown: "));
  response += String(long(repeatingEventsNextTimeRun - systemTime), DEC);
  response += String(F("<br/>============================= "));
  response += String(F("<br/>Co voltage: "));
  response += String(analogCO, DEC);
  response += String(F("<br/>============================= "));
  response += String(F("<br/>Buzzer Pin Rased: "));
  response += String(digitalRead(buzzerPin) == 1 ? (F("YES")):(F("NO")));
  response += String(F("<br/>Green Pin Rased: "));
  response += String(digitalRead(greenLedPin) == 1 ? (F("YES")):(F("NO")));
  response += String(F("<br/>Yellow Pin Rased: "));
  response += String(digitalRead(yellowLedPin) == 1 ? (F("YES")):(F("NO")));
  response += String(F("<br/>Red Pin Rased: "));
  response += String(digitalRead(redLedPin) == 1 ? (F("YES")):(F("NO")));
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

  // String s = "HTTP/1.1 200 OK\r\n";
  // s += "Content-Type: application/json\r\n\r\n";
  String response = String((F("{")));

  response += String((F("\"analog\":")));
  response += String(analogCO, 8);
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
// interrupts

void handleButtonInterrupt() {

  LOG(((F("button pressed"))));

  if (beepTypeValue == NO_BEEP) {

    scheduleEvent(SINGLE_BEEP_EVENT);
  } else {

    scheduleEvent(PERMANENT_BEEP_DISSMISAL_EVENT);
  }
}

/**************************
 *   S E T U P
 **************************/

 void setupServer() {

  MDNS.begin(socNetworkName);

  httpUpdater.setup(&httpServer, "/update");

  //httpServer.on("/",[](){httpServer.send(200,"text/plain","Hello World!");});

  httpServer.on("/", respondWithState);
  httpServer.on("/log", respondWithLog);
  httpServer.on("/measure", respondWithMeasured);

  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  LOG((F("Finished server setup")));
 }

 void setupBme() {

   if (!bme.begin(0x76)) {
     LOG((F("Could not find a valid bme280 sensor, check wiring!")));
     //ESP.deepSleep( 30 * 1000000, WAKE_RF_DISABLED);
     return;

   } else {
     LOG((F("BME280 found!")));
   }
   LOG((F("Finished with BME setup")));
 }

 void setupADS() {

  LOG((F("Starting ADS1115 Setup")));
  ads0.initialize();  
  
  if (!ads0.testConnection()) {
    LOG((F("Could not find a valid ADS1115, check wiring!")));
    return;

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
    LOG((F("HighThreshold="))); LOG(String(ads0.getHighThreshold(),BIN));
    LOG((F("LowThreshold="))); LOG(String(ads0.getLowThreshold(),BIN));
  #endif

  // pinMode(1,INPUT_PULLUP);
  // ads0.setConversionReadyPinMode();

  LOG((F("Finished with ADS1115 setup")));
}

void setupSOC() {

  pinMode(blueChipLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(buzzerPin, INPUT);

  Wire.begin(sdaPin, slcPin);

  attachInterrupt(buttonPin, handleButtonInterrupt, FALLING);

  LOG((F("SOC setup finished")));
}

void prepareTestPinsState() {

  digitalWrite(buzzerPin, LOW);
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
  setupBme();

  setupADS();

  setupServer();
  WiFi.mode(WIFI_STA);

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

void eventsLoop() {

  LOG((F("[ Running Events Loop")));

  checkWifiConnection();

  if (couldSheduleRepeatingEvents()) {

    LOG((F("< Start Scheduling Repeting Events To EventsLoop")));

    scheduleEvent(SINGLE_CO_MEASURE_EVENT);
    scheduleEvent(SINGLE_ENVIRONMENT_MEASURE_EVENT);

    LOG((F("Scheduled Repeting Events To EventsLoop >")));
  }

  if (processEvents()) {

    AlarmType_t incomingAlarmType = checkForAlarmTypeusingCoValue(analogCO);

    if (incomingAlarmType != alarmTypeValue) {

      scheduleEvent(ALARM_STATE_DID_CHANGE_EVENT);
    }
  }
  LOG((F("Finished Events Loop ]")));
  LOG((F("[---   ---   ---  ---  ---]")));
}

void loop() {
  // loop_check();
  // loopMeasure();

  if (couldProcessEventsLoop()) {

    eventsLoop();
  }

  httpServer.handleClient();  

  delay(socLoopCycleDelay);
}
