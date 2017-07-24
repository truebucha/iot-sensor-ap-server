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


// replace with your channel’s thingspeak API key,
// tutorial https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server

const char WiFiApName[] = "sensor-co-";
const char WiFiApSecret[] = "Sensor123";
WiFiServer server(8090);
Adafruit_BME280 bme; // I2C



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
   Wire.begin(5,4);
   if (!bme.begin(0x76)) {
     Serial.println("Could not find a valid bme280 sensor, check wiring!");
     //ESP.deepSleep( 30 * 1000000, WAKE_RF_DISABLED);
   } else {
     Serial.println(F("BME280 found!"));
   }
   Serial.println("Finished with BME setup");
 }

void initHardware() {
   pinMode(2, OUTPUT);
   initBme();
   Serial.println("Finished with hardware setup");
   // Don't need to set ANALOG_PIN as input,
   // that's all it can be.
}

void setup() {
  Serial.begin(74880);
  delay(10);
  Serial.println("Start board setup");
  initHardware();
  digitalWrite(2, HIGH);
  setupWiFi();
  server.begin();
  digitalWrite(2, LOW);
  Serial.println("Finished with setup");
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

void readCoValues() {
  analog = analogRead(0);
  corrected = correctedAnalogRead(analog);
  coPpm = corrected * 3.5;
}

void loop_check() {
  delay(1e3);
  readCoValues();
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

  digitalWrite(2, HIGH);

  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);

  client.flush();

  // if (req.indexOf("/led/0") != -1)

  // Prepare the response. Start with the common header:
  readCoValues();
  readBmeValues();

  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: application/json\r\n\r\n";
  s += "{";

  s += "\"analog\":";
  s += String(analog);
  s += ",";

  s += "\"coPpm\":";
  s += String(coPpm);
  s += ",";

  s += "\"temp\":";
  s += String(temp);
  s += ",";

  s += "\"pressure\":";
  s += String(pressure);
  s += ",";

  s += "\"humidity\":";
  s += String(humidity);

  s += "}";

  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println("Client disonnected");
  digitalWrite(2, LOW);
  // The client will actually be disconnected
  // when the function returns and 'client' object is detroyed
}

void loop() {
  //loop_check();
  loop_work();
}
