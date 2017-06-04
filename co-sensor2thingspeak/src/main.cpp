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

Adafruit_BME280 bme; // I2C
// replace with your channel’s thingspeak API key,

String apiKey = "F8GQ63NFVB7917HP";
const char* ssid = "sensor";
const char* password = "123123123";
const char* server = "api.thingspeak.com";
WiFiClient client;


/**************************
 *   S E T U P
 **************************/

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);

  Wire.begin(5,4);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    ESP.deepSleep( 30 * 1000000, WAKE_RF_DISABLED);
  } else {
    Serial.println(F("BME280 found!"));
  }

  setup_wifi();
}

  /**************************
 *  L O O P
 **************************/

void checkCommand() {
if (Serial.available() > 0) {
    String str = Serial.readString();

    if (str.startsWith("wifi")) {

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("disconnect wifi");
        WiFi.disconnect();
      } else {
        Serial.println("connect wifi");
        setup_wifi();
      }
    }
  }
}

float miliVoltsUsingAnalogRead(int value) {
  int zeroVoltageValue = 5;
  int correctedValue = value - zeroVoltageValue;
  int input = correctedValue > 0 ? correctedValue : 0;
  // measured 3.12 v source stabilazed voltage
  //          Usource(mV) / stepsTotal * currentSteps
  float result = (3120.0 / float(1024.0-zeroVoltageValue) * float(input));
  // Serial.print("read :");
  // Serial.println(value);
  // Serial.println(result);
  return result;
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

void sendValues() {
  if (client.connect(server,80) == false) {
    client.stop();
    return;
  }
  int analog = analogRead(0);
  float mV = miliVoltsUsingAnalogRead(analog);
  float coppm = ppmUsingMiliVolts(mV);

  String postStr = apiKey;
  postStr +="&field1=";
  postStr += String(bme.readTemperature());
  postStr +="&field2=";
  postStr += String(bme.readPressure());
  postStr +="&field3=";
  postStr += String(bme.readHumidity());
  postStr +="&field4=";
  postStr += String(bme.readAltitude(1013.25));
  postStr +="&field5=";
  postStr += String(coppm);
  postStr +="&field6=";
  postStr += String(mV);
  postStr +="&field7=";
  postStr += String(analog);
  postStr += "\r\n\r\n";

  client.print("POST /update HTTP/1.1\n");
  client.print("Host: api.thingspeak.com\n");
  client.print("Connection: close\n");
  client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
  client.print("Content-Type: application/x-www-form-urlencoded\n");
  client.print("Content-Length: ");
  client.print(postStr.length());
  client.print("\n\n");
  client.print(postStr);

  client.stop();
}

int count = 0;
void loop() {
    checkCommand();
    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(2, LOW);
    } else {
      digitalWrite(2, HIGH);
    }

    //float mV = readMiliVoltsFromSensor();
    //float coppm = ppmUsingMiliVolts(mV);
    // Serial.print("Value :");
    // Serial.println(coppm);
    // sendValues();
    sendValues();
}
