// test one 
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

#include "password.h"
/// Only used for monitoring, can be removed it's not part of our "thing"

#if defined(LED_BUILTIN)
const int ledPin = LED_BUILTIN;
#else
const int ledPin = 2; // manually configure LED pin was 13 2 is blue led built in
const int lampPin = 2;
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
long mmap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
WebThingAdapter *adapter; // adapter could then be used as as device
ThingActionObject *action_generator(DynamicJsonDocument *);
const char *lampTypes[] = {"OnOffSwitch", "Light", nullptr};
ThingDevice lamp("ABC", "ABC", lampTypes);
ThingProperty lampOn("on", "Whether the lamp is turned on", BOOLEAN, "OnOffProperty");
ThingProperty lampLevel("brightness", "The level of light from 0-100", INTEGER, "BrightnessProperty");
StaticJsonDocument<256> fadeInput;
JsonObject fadeInputObj = fadeInput.to<JsonObject>();
ThingAction fade("fade", "Fade", "Fade the lamp to a given level", "FadeAction", &fadeInputObj, action_generator);
ThingEvent overheated("overheated", "The lamp has exceeded its safe operating temperature", NUMBER, "OverheatedEvent");

// think property for device2

String lastColor = "#ffffff";

const unsigned char redPin = 12;
const unsigned char greenPin = 13;
const unsigned char bluePin = 14;

//-------------------------------------------------------------------
LedControl lc = LedControl(pinDIN, pinCLK, pinCS, 1);
//------------------------------------------------------------------

//------------------------------------------------------------------
void setup(void)
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

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
  lc.clearDisplay(0);
  WiFi.begin(ssid, password);
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
  digitalWrite(ledPin, HIGH); // turn off active low led

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // THINGS BEGIN HERE

  adapter = new WebThingAdapter("led-lamp", WiFi.localIP()); // instantiate the adapter

  lamp.description = "A web conneced lamp";
  lamp.title = "On/Off";
  lamp.addProperty(&lampOn);
  //---
  lampLevel.title = "Brightness";
  lampLevel.minimum = 0;
  lampLevel.maximum = 100;
  lampLevel.unit = "%";
  lamp.addProperty(&lampLevel);

  fadeInputObj["type"] = "object";
  JsonObject fadeInputProperties = fadeInputObj.createNestedObject("properties");

  JsonObject brightnessInput = fadeInputProperties.createNestedObject("brightness");
  brightnessInput["type"] = "integer";
  brightnessInput["minimum"] = 0;
  brightnessInput["maximum"] = 100;
  brightnessInput["unit"] = "percent";

  JsonObject durationInput = fadeInputProperties.createNestedObject("duration");
  durationInput["type"] = "integer";
  durationInput["minimum"] = 1;
  durationInput["unit"] = "milliseconds";

  lamp.addAction(&fade);

  overheated.unit = "degree C";
  lamp.addEvent(&overheated);

  adapter->addDevice(&lamp);
  adapter->begin();
}

void loop(void)
{
  //digitalWrite(ledPin, micros() % 0xFFFF ? HIGH : LOW); // Heartbeat

  digitalWrite(23, HIGH);

  adapter->update();
}
void do_fade(const JsonVariant &input)
{
  JsonObject inputObj = input.as<JsonObject>();
  long long int duration = inputObj["duration"];
  long long int brightness = inputObj["brightness"];

  delay(duration);

  ThingDataValue value = {.integer = brightness};
  lampLevel.setValue(value);

     

  int level = Arduino_h::map(brightness, 0, 100, 255, 0);
  printf("value =%i , level(mapped)=%i\n", value.integer, level);
  analogWrite(lampPin, level, 255);
  lc.clearDisplay(0);
  lc.printF((float)brightness, (char *)"%.2f");
  ThingDataValue val;
  val.number = 102;

  ThingEventObject *ev = new ThingEventObject("overheated", NUMBER, val);
  lamp.queueEventObject(ev);
}
ThingActionObject *action_generator(DynamicJsonDocument *input)
{
  String output;
  serializeJson(*input, output);
  printf("Going to print \n");
  printf("printing ->input %s\n", output.c_str());
  return new ThingActionObject("fade", input, do_fade, nullptr);
}