/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>

#define ADS1115_SERIAL_DEBUG
#include "ADS1115.h"


#define led_green 13
#define led_yellow 12
#define led_red 14
#define beeper 16
#define led_chip_blue 2
#define button 2


// replace with your channel’s thingspeak API key,
// tutorial https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server

const char WiFiApName[] = "sensor-co-";
const char WiFiApSecret[] = "Sensor123";
WiFiServer server(8090);

Adafruit_BME280 bme; // I2C
ADS1115 ads0(0x48); 
// Adafruit_ADS1115 ads; 


void printButton() {

  Serial.print((F("button pressed")));
}
/**************************
 *   S E T U P
 **************************/

 void setupWiFi() {
   byte mac[6];
   WiFi.macAddress(mac);

   Serial.print(F("MAC: "));
   Serial.print(mac[5],HEX);
   Serial.print(F(":"));
   Serial.print(mac[4],HEX);
   Serial.print(F(":"));
   Serial.print(mac[3],HEX);
   Serial.print(F(":"));
   Serial.print(mac[2],HEX);
   Serial.print(F(":"));
   Serial.print(mac[1],HEX);
   Serial.print(F(":"));
   Serial.println(mac[0],HEX);

   String apName = WiFiApName;
   apName += String(mac[5], HEX);
   apName += String(mac[4], HEX);

   Serial.print(F("ApName: "));
   Serial.println(apName);

   WiFi.mode(WIFI_AP);

   char ApName[15];
   apName.toCharArray(ApName, 15);

   WiFi.softAP(ApName, WiFiApSecret);
   Serial.println(F("Finished with wifi setup"));
 }

 void initBme() {
   if (!bme.begin(0x76)) {
     Serial.println("Could not find a valid bme280 sensor, check wiring!");
     //ESP.deepSleep( 30 * 1000000, WAKE_RF_DISABLED);
   } else {
     Serial.println(F("BME280 found!"));
   }
   Serial.println("Finished with BME setup");
 }

 void initADS() {
  Serial.println(F("start ads"));
  ads0.initialize();
  Serial.println("Getting single-ended readings from AIN0..3");
  
  
  if (!ads0.testConnection()) {
    Serial.println(F("Could not find a valid ADS1115, check wiring!"));
  } else {
    Serial.println(F("ADS1115 found!"));
  }

  // adc0.setMode(ADS1115_MODE_CONTINUOUS);
  ads0.setMode(ADS1115_MODE_SINGLESHOT);
  ads0.setRate(ADS1115_RATE_8);
  // ads0.setGain(ADS1115_PGA_0P512);
  ads0.setGain(ADS1115_PGA_0P256);

  #ifdef ADS1115_SERIAL_DEBUG
  ads0.showConfigRegister();
  Serial.print("HighThreshold="); Serial.println(ads0.getHighThreshold(),BIN);
  Serial.print("LowThreshold="); Serial.println(ads0.getLowThreshold(),BIN);
  #endif

  // pinMode(1,INPUT_PULLUP);
  // ads0.setConversionReadyPinMode();

  Serial.println(F("Finished with ADS1115 setup"));
}

void initHardware() {

  pinMode(led_chip_blue, OUTPUT);
  pinMode(beeper, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_yellow, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(button, INPUT);

  attachInterrupt(button, printButton, FALLING);
   Wire.begin(5,4);

  //  initBme();
   initADS();
   Serial.println("Finished with hardware setup");
   // Don't need to set ANALOG_PIN as input,
   // that's all it can be.
}

void setup() {
  Serial.begin(9600);
  delay(10);
  Serial.println("Start board setup");
  initHardware();
  digitalWrite(led_chip_blue, HIGH);
  setupWiFi();
  server.begin();
  digitalWrite(led_chip_blue, LOW);
  Serial.println("Finished with setup");

  digitalWrite(beeper, LOW);
  digitalWrite(led_red, HIGH);
  digitalWrite(led_yellow, HIGH);
  digitalWrite(led_green, HIGH);
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
  // Serial.print("read :");
  // Serial.println(value);
  // Serial.println(result);
  return input;
}

float ppmUsingMiliVolts(float mV) {
  //          sensor U * (10e6(mA to nA) / current resistor(50kOm))
  float nA = mV * (1000000 / 50000);
  //                  sensor current  * sensor nA->ppm coefficient / amplifier coefficient * correction;
  float result = nA * 1.4 / 108 * 6.11;
  // Serial.print("ppm :");
  // Serial.println(nA);
  // Serial.println(result);
  return result;
}


int analog = 0;
float corrected = 0.0;
float coPpm = 0.0;
float temp = 0.0;
float pressure = 0.0;
float humidity = 0.0;

void readBmeValues() {
  temp = bme.readTemperature();
  pressure = bme.readPressure();
  humidity = bme.readHumidity();
}

float readADSValues() {
  Serial.println("Sensor 1 ************************");
  // Set the gain (PGA) +/- 1.024v
  // ads0.setGain(ADS1115_PGA_0P256);

  // Get the number of counts of the accumulator
  Serial.print("Counts for sensor 1 is:");
  
  // The below method sets the mux and gets a reading.
  int sensorOneCounts=ads0.getConversionP2N3();  // counts up to 16-bits  
  Serial.println(sensorOneCounts);

  // To turn the counts into a voltage, we can use
  Serial.print("Voltage for sensor 1 is:");
  //  float value = sensorOneCounts * ads0.getMvPerCount();
  float value = ads0.getMilliVolts();
  //  float value = sensorOneCounts * 0.007813;
  Serial.println(value, 8);
  
  Serial.println();
   
   
  // // 2nd sensor is on P2/N3 (pins 6/7)
  // Serial.println("Sensor 2 ************************");
  // // Set the gain (PGA) +/- 0.256v
  // adc0.setGain(ADS1115_PGA_0P256);

  // // Manually set the MUX  // could have used the getConversionP* above
  // adc0.setMultiplexer(ADS1115_MUX_P2_N3); 
  // Serial.print("Counts for sensor 2 is:");
  // Serial.println(adc0.getConversion());  

  // Serial.print("mVoltage sensor 2 is:");
  // Serial.println(adc0.getMilliVolts());  // Convenience method to calculate voltage

  // Serial.println();
  
  return value;
}

void readChipAnalog() {
  analog = analogRead(0);
  corrected = correctedAnalogRead(analog);
  coPpm = corrected * 3.5;
}

void loopMeasure() {
  delay(1e3);

  digitalWrite(led_chip_blue, HIGH);
  float a = readADSValues();
  Serial.print(F("analog = "));
  Serial.print(a, 8);
  Serial.println(F(" mV"));
  digitalWrite(led_chip_blue, LOW);
}

void loop_check() {
  delay(1e3);
  readChipAnalog();
  Serial.print(F("analog = "));
  Serial.println(analog);
  Serial.print(F("corrected = "));
  Serial.println(corrected);
  Serial.println(F("CO = "));
  Serial.println(coPpm);
  Serial.println(F("=="));
}

void loop_work() {
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  digitalWrite(led_chip_blue, HIGH);

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);

  client.flush();

  // if (req.indexOf("/led/0") != -1)

  // Prepare the response. Start with the common header:
  // readCoValues();
  // readBmeValues();
  float a = readADSValues();

  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: application/json\r\n\r\n";
  s += "{";

  s += "\"analog\":";
  s += String(a, 8);
  s += ",";

  s += "\"coPpm\":";
  s += String(coPpm, 8);
  s += ",";

  s += "\"temp\":";
  s += String(temp, 8);
  s += ",";

  s += "\"pressure\":";
  s += String(pressure, 8);
  s += ",";

  s += "\"humidity\":";
  s += String(humidity, 8);

  s += "}";

  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println("Client disonnected");
  digitalWrite(led_chip_blue, LOW);
  // The client will actually be disconnected
  // when the function returns and 'client' object is detroyed
}

void loop() {
  //loop_check();
  loop_work();
  // loopMeasure();
}
