// -*- mode: c++;  c-basic-offset: 2 -*-
/**
 * Simple server compliant with Mozilla's proposed WoT API
 * Originally based on the HelloServer example
 * Tested on ESP8266, ESP32, Arduino boards with WINC1500 modules (shields or
 * MKR1000)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#define LARGE_JSON_BUFFERS 1

#include <Arduino.h>
#include <Thing.h>
#include <WebThingAdapter.h>
#include <SPI.h>
#include "LedControl.h"

#ifdef ESP32
#include <analogWrite.h>
#endif

using namespace std;
// TODO: Hardcode your wifi credentials here (and keep it private)

#include "***REMOVED***.h"
/// Only used for monitoring, can be removed it's not part of our "thing"

#if defined(LED_BUILTIN)
const int ledPin = LED_BUILTIN;
#else
const int ledPin = 2; // manually configure LED pin was 13 2 is blue led built in
#endif
// pin definitions
const int pinCS = 15;  // Chip Select
const int pinCLK = 18; // Clock pin
const int pinDIN = 23; // Data
const int anzMAX = 1;  //Anzahl der kaskadierten  Module = Number of Cascaded modules

// for optional properties
// const char * valEnum[5] = {"RED", "GREEN", "BLACK", "white", nullptr};
// const char * valEnum[5] = {"#db4a4a", "#4adb58", "000000", "ffffff",
// nullptr};

WebThingAdapter *adapter;

const char *deviceTypes[] = {"Light", "OnOffSwitch", "ColorControl", nullptr};
ThingDevice device("ABC", "ABC", deviceTypes);
ThingDevice redLed1("RED LED 1", "LED", deviceTypes);

ThingProperty deviceOn("on", "Whether the led is turned on", BOOLEAN,
                       "OnOffProperty");
ThingProperty redLed1On("on", "Where Red Led1 turned on", BOOLEAN, "LedOnOffProperty");
ThingProperty deviceLevel("level", "The level of light from 0-100", NUMBER,
                          "BrightnessProperty");
ThingProperty deviceColor("color", "The color of light in RGB", STRING,
                          "ColorProperty");
// think property for device2
bool lastOn = false;
String lastColor = "#ffffff";

const unsigned char redPin = 12;
const unsigned char greenPin = 13;
const unsigned char bluePin = 14;

void setupLamp(void)
{
  pinMode(redPin, OUTPUT);
  digitalWrite(redPin, HIGH);
  pinMode(greenPin, OUTPUT);
  digitalWrite(greenPin, HIGH);
  pinMode(bluePin, OUTPUT);
  digitalWrite(bluePin, HIGH);
}
//-------------------------------------------------------------------
LedControl lc = LedControl(pinDIN, pinCLK, pinCS, 1);
//------------------------------------------------------------------

//------------------------------------------------------------------
void setup(void)
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  setupLamp();

  Serial.begin(115200);
  Serial.println("");
  Serial.print("Connecting to \"");
  Serial.print(ssid);
  Serial.println("\"");

  WiFi.mode(WIFI_STA);

  //------------------Initialize MAX7219-------------
  printf("Initializing MAX77219..\n");
  lc.shutdown(0, false);
  lc.setIntensity(0, 10);
  // -- turn off display
  for (int i = 0; i <= 9999; i++)
  {
    lc.printF(i, (char *)"%.2f");
    if (i == 99999999)
    {
      i = 0;
      lc.clearDisplay(0);
      continue;
    }
  }

  WiFi.begin(ssid, ***REMOVED***);
  Serial.println("");

  // Wait for connection
  bool blink = true;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    digitalWrite(ledPin, blink ? LOW : HIGH); // active low led
    blink = !blink;
  }
  digitalWrite(ledPin, HIGH); // active low led

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  adapter = new WebThingAdapter("rgb-lamp", WiFi.localIP());

  device.addProperty(&deviceOn);
  ThingPropertyValue levelValue;
  levelValue.number = 100; // default brightness TODO
  deviceLevel.setValue(levelValue);
  device.addProperty(&deviceLevel);

  redLed1.addProperty(&redLed1On);

  // optional properties
  // deviceColor.propertyEnum = valEnum;
  // deviceColor.readOnly = true;
  // deviceColor.unit = "HEX";

  ThingPropertyValue colorValue;
  colorValue.string = &lastColor; // default color is white
  deviceColor.setValue(colorValue);
  device.addProperty(&deviceColor);

  adapter->addDevice(&device);
  adapter->addDevice(&redLed1);
  Serial.println("Starting HTTP server");
  adapter->begin();
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.print("/things/");
  Serial.println(device.id);
#ifdef analogWriteRange
  analogWriteRange(255);
#endif

  /*
  lc.setChar(0, 0, '7', false);
  lc.setChar(0, 1, '6', false);
  lc.setChar(0, 2, '5', false);
  lc.setChar(0, 3, '4', false);
  lc.setChar(0, 3, '3', false);
  

  lc.setChar(0, 5, '2', false);
  lc.setChar(0, 6, '1', false);
  lc.setChar(0, 7, '0', false);
  */
  delay(1000);
}

void update(String *color, int const level)
{
  if (!color)
    return;
  float dim = level / 100.;
  int red, green, blue;
  if (color && (color->length() == 7) && color->charAt(0) == '#')
  {
    const char *hex = 1 + (color->c_str()); // skip leading '#'
    sscanf(0 + hex, "%2x", &red);
    sscanf(2 + hex, "%2x", &green);
    sscanf(4 + hex, "%2x", &blue);
  }
  analogWrite(redPin, red * dim);
  analogWrite(greenPin, green * dim);
  analogWrite(bluePin, blue * dim);
}

void loop(void)
{
  //digitalWrite(ledPin, micros() % 0xFFFF ? HIGH : LOW); // Heartbeat

  digitalWrite(23, HIGH);
  digitalWrite(ledPin, HIGH);

  delay(100);

  digitalWrite(23, LOW);
  digitalWrite(ledPin, LOW);
  delay(100);

  if (micros() % 0xFFFF == LOW)
    printf(" ledPin**=%i  ---> %i\n ", ledPin, HIGH);
  adapter->update();

  bool on = deviceOn.getValue().boolean;
  int level = deviceLevel.getValue().number;
  update(&lastColor, on ? level : 0);

  if (on != lastOn)
  {
    Serial.print(device.id);
    Serial.print(": on: ");
    Serial.print(on);
    Serial.print(", level: ");
    Serial.print(level);
    Serial.print(", color: ");
    Serial.println(lastColor);
  }
  lastOn = on;
}
