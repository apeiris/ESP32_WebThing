// current device mac 2462ABCEBDBC
// * Tested on ESP8266, ESP32, Arduino boards with WINC1500 modules (shields or
// t11

// -*- mode: c++;  c-basic-offset: 2 -*-
/**
 * Simple server compliant with Mozilla's proposed WoT API
 * Originally based on the HelloServer example
 * MKR1000)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * this code is based on
 * https://github.com/WebThingsIO/webthing-arduino/blob/master/examples/LEDLamp/LEDLamp.ino
 */
#define LARGE_JSON_BUFFERS 1
#include <Adafruit_AHTX0.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <StreamUtils.h>
#include <analogWrite.h>
#include <map>
#include <PubSubClient.h>
#include "LedControl.h"
#include "Thing.h"
#include "WebthingAdapter.h"
#include "esp_log.h"
#include "esp_system.h"
#include "password.h"

using namespace std;
static const char *TAG = "main.cpp";

IPAddress mySqlIP(192, 168, 0, 10);
/// Only used for monitoring, can be removed it's not part of our "thing"

#define pushInterval (60000 / 1)
#if defined(LED_BUILTIN)
const int ledPin = LED_BUILTIN;
#else
const int ledPin =
    2;  // manually configure LED pin was 13 2 is blue led built in
const int lampPin = 2;
#endif
// AHT10 pins
const int pinSDA = 5;   // white wire
const int pinSCL = 17;  // the purple
// pin definitions for MAX7129
const int pinCS = 15;   // Chip Select
const int pinCLK = 18;  // Clock pin
const int pinDIN = 23;  // Data
const int anzMAX = 1;  // Anzahl der kaskadierten  Module = Number of Cascaded modules
long mmap(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
String sDevicesJson="" ;
WebThingAdapter *adapter;  // adapter could then be used as as device
ThingActionObject *action_generator(DynamicJsonDocument *);

const char *lampTypes[] = {"OnOffSwitch", "Light", nullptr};
const char *sensorTypes[] = {"Sensor", "Sensor", nullptr};
const char *asyncProperties[] = {"asyncProperty", "I/O", nullptr};
StaticJsonDocument<256> fadeInput;
JsonObject fadeInputObj = fadeInput.to<JsonObject>();
ThingAction fade("fade", "Fade", "Fade the lamp to a given level", "FadeAction", &fadeInputObj, action_generator);
ThingEvent overheated("overheated", "The lamp has exceeded its safe operating temperature", NUMBER, "OverheatedEvent");

ThingDevice lamp("LED", "Dimmable LED", lampTypes);
ThingDevice AHT10Device("AHT10", "AHT10", sensorTypes);
ThingDevice textDisplay("Text", "text Async Property Test", asyncProperties);
ThingDevice testDevice("AHT10", "AHT10", sensorTypes);

// Forward declaration
void textDisplayTextChanged(ThingPropertyValue newVal);
ThingProperty lampOn("on", "Whether the lamp is turned on", BOOLEAN, "OnOffProperty");
ThingProperty lampLevel("brightness", "The level of light from 0-100", INTEGER, "BrightnessProperty");
ThingProperty AHT10TemperatureProperty("Temperature", "Temperature in C", NUMBER, "Centigrades");
ThingProperty AHT10HumidityProperty("Humidity", "Humidity (RH) %", NUMBER, "%");
//ThingProperty textDisplayText("text","text description",STRING,nullptr,textDisplayTextChanged);
ThingProperty textDisplayText("text", "description", STRING, "yo", textDisplayTextChanged);
ThingProperty testDeviceProperty("test", "testing", NUMBER, "%");
// ThingProperty
// textDisplayToggle("toggle","",STRING,nullptr,textDisplayToggled);
// ThingProperty
// textDisplayNumber("number","",STRING,nullptr,textDisplayNumbenewrChanged);
String message = "message";
String lastMessage = message;
void textDisplayTextChanged(ThingPropertyValue newVal) {
    String x = *newVal.string;
    ESP_LOGI(TAG, "text=>%s\n", x.c_str());
}

String lastColor = "#ffffff";

const unsigned char redPin = 12;
const unsigned char greenPin = 13;
const unsigned char bluePin = 14;
//---------------GPIO33 pulled low and connected to button to pull high to 3.3v
#define BUTTON_WAKEUP_BITMASK 0x200000000  // 2^33 in hex / GPIO33
//------------------------------------------------------------------

//------------Adafruit Lib -----------------------------------------
// the AHT must be connected to Bords SDA->GPIO21 and SCL ->GPIO22
//  to detect the sensor board
Adafruit_AHTX0 aht;
WiFiClient client;
PubSubClient mqtt(client);
//------------------------------------------------------------------
LedControl lc = LedControl(pinDIN, pinCLK, pinCS, 1);
//------------MQTT callback-----------------------------------------
void mqtt_callback(char* topic,byte* payload,unsigned int length){
    ESP_LOGI(TAG,"Message arrived[%s]\n",topic);
  printf("HI here is the topic %s\n",topic);
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

}
 
void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    printf("Attempting MQTT connection...\n");
   
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    
    if (mqtt.connect(clientId.c_str())) {
      printf("\tconnected & now publishing \"outTopic\"\n" );
      // Once connected, publish an announcement...
      mqtt.publish("outTopic", "hello world");
      mqtt.subscribe("HI");
      // ... and resubscribe
      mqtt.subscribe("greenBottles/#");
    } else {
     printf("Failed to connect ..\n");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//----------------Init MAX7219--------------------------------------
void initMAX7219() {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    Serial.begin(115200);
    ESP_LOGI(TAG, "Connecting to %s \n", ssid);
    //--- RTC wakeup on gpio 33 (wired to button)
    esp_sleep_enable_ext1_wakeup(BUTTON_WAKEUP_BITMASK,
                                 ESP_EXT1_WAKEUP_ANY_HIGH);
    //------------------Initialize MAX7219-------------
    ESP_LOGI(TAG, "Initializing MAX7219..\n");
    lc.shutdown(0, false);
    lc.setIntensity(0, 10);
    lc.clearDisplay(0);
    lc.printF(0, (char *)"%0.2f");
}
//--------------------initWifi()------------------------------------
void initWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    bool blink = false;
    while (WiFi.status() != WL_CONNECTED) {
        printf(".");
        digitalWrite(ledPin, blink ? LOW : HIGH);  // active low led
        digitalWrite(ledPin, blink);
        delay(80);
    }
}
//----------------initAHT()-----------------------------------------
void initAHT() {
    if (!aht.begin()) {
        ESP_LOGI(TAG, "Could not find AHT ! \n");
    } else
        ESP_LOGI(TAG, "AHT10 or AHT20 found \n");
}
//----------------initAdapterAndAddDevices()------------------------
void initAdapterAndAddDevices() {
    String x = WiFi.macAddress();
    x.replace(":", "");
    adapter = new WebThingAdapter(x, WiFi.localIP());
    // devices and the properties
    {
        lamp.description = "A web conneced lamp";
        lamp.title = "Dimmable LED";
        lampLevel.title = "Brightness";
        lampLevel.minimum = 0;
        lampLevel.maximum = 100;
        lampLevel.unit = "%";
        lamp.addProperty(&lampLevel);
        // create fade Input object and json
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

        lamp.addProperty(&lampOn);
        AHT10TemperatureProperty.readOnly = true;
        AHT10Device.addProperty(&AHT10TemperatureProperty);
        AHT10HumidityProperty.readOnly = true;
        AHT10Device.addProperty(&AHT10HumidityProperty);

        textDisplay.addProperty(&textDisplayText);
        testDeviceProperty.readOnly = true;
        testDevice.addProperty(&testDeviceProperty);
    }

    adapter->addDevice(&lamp);
    adapter->addDevice(&textDisplay);
    adapter->addDevice(&testDevice);
    adapter->addDevice(&AHT10Device);
    adapter->begin();
    ESP_LOGI(TAG, "Webthing Adapter initialized with %i devices\n");
}
//------------------------------------------------------------------
void registerDevice(ThingDevice *d, ThingProperty *p) {
}
//------------------------------------------------------------------
void serielizeProperty(ThingProperty t) {
    DynamicJsonDocument doc(500);
    String out;
    doc[t.title] = t.title;
    doc[t.description] = t.description;
    ThingDataValue x = t.getValue();
    doc["value"] = x.number;
    serializeJson(doc, out);
    ESP_LOGI(TAG, "SerializeProperty=%s  \n\t len=%i\n", out.c_str(), out.length());
}
void serializeDevice(ThingDevice d) {
    return;
    DynamicJsonDocument doc(500);
    String out;
    doc["id"] = d.id;

    serializeJson(doc, out);
    ESP_LOGI(TAG, "serializeDevice:", out.c_str(), out.length());
}
String MACasID() {
    String x = WiFi.macAddress();
    x.replace(":", "");
    return x;
}
//----registerDevices() in the SQL Serve from the adapter-----------
void registerDevices() {
    DynamicJsonDocument doc(2000);
    doc["macId"]=MACasID();
    JsonArray  devices = doc.createNestedArray(&"devices");
    JsonObject device;
    JsonArray props;
    JsonObject prop;
    ThingDevice *d = adapter->getFirstDevice();
    int di = 0;
    while (d) {
        printf("\tIn D loop Id =%s\n", d->id.c_str());
       device = devices.createNestedObject();
        device["seed"] =di;
        device["id"] =d->id;   
        device["title"]=d->title;
        device["name"]=d->id;    
        props = device.createNestedArray("props"); 
        ThingProperty *p=d->firstProperty;
        int pi = 0;
        while (p) {
             printf("\t\tproperty for the device=%s:prop=%s di=%i pi=%i\n", d->id.c_str(), p->id.c_str(), di, pi);
             prop=props.createNestedObject();
          
            prop["seed"]=pi;
            prop["Id"]= p->id;
            prop["dbId"]=p-> propertyDbId;

        
            prop["description"]=p->description;

            pi++;
            p = (ThingProperty *)p->next;
            
        }
        
        di++;
        d=d->next;
        
    }
 
    serializeJsonPretty(doc, Serial);
    printf("\n\n---------------------\n\n");
  // serializeJson(doc, Serial);

    serializeJson(doc,sDevicesJson);
    printf("json=%s\n\n",sDevicesJson.c_str());
    printf("\n\n");
    //ESP_LOGI(TAG, "\nout=\n%s\n\t\t len=%i\n", output.c_str(), output.length());
}
//----initMqtt( int MaxMsglen=256)----------------------------------  
 void initMqtt(int maxMsgLen=256){
  mqtt.setServer(MQTT_IP,MQTT_Port);
  mqtt.setCallback(mqtt_callback);
  mqtt.setBufferSize(maxMsgLen);
  ESP_LOGI(TAG,"MQTT initialized with IP= %s:Port=%i  Max Messsage Length =%i\n",MQTT_IP.toString().c_str(),MQTT_Port,mqtt.getBufferSize());
 }
//------------------------------------------------------------------
void mqttReConnect()
{
    while(!mqtt.connected())
    {
        printf("Attempting MQTT connection");
        String clientId="ESP32Client-";
        clientId+=String(random(0xffff),HEX);
        if(mqtt.connect(clientId.c_str())){
            ESP_LOGI(TAG,"Client connected, now publishing \"HI\"\n\t %s\n",sDevicesJson.c_str());
            mqtt.publish("HI",sDevicesJson.c_str());
            mqtt.subscribe("HO");
        };
    }
}
//------------------------------------------------------------------

void setup(void) {
    esp_log_level_set(TAG, ESP_LOG_ERROR);
    initMAX7219();               // (1) --- init MAX7219-------------------------------
    initAHT();                   // (2)----- init AHT----------------------------------
    initWifi();                  // (3)------init WiFi---------------------------------
    initAdapterAndAddDevices();  // (4) -- intiAdapterAndAddDevices()------------------
    initMqtt(1024);              // (5) -- initMqtt(1024) override default 256 --------
    registerDevices();           // (6) --All devices in the adapter, Json form--------
                                 // (7) mqtt_reconnect() will be executed in the LOOP--

}
static int i = 0;
ThingPropertyValue toPvalueNumber(double n) {
    ThingPropertyValue pv;
    pv.number = n;
    return pv;
}

// ReadLoggingStream rs(Serial, Serial);
void readAHT10() {
    sensors_event_t humidity, temperature;
    aht.getEvent(&humidity, &temperature);  //   gcvt(humidity.relative_humidity,5,fs);
    AHT10HumidityProperty.setValue(toPvalueNumber(humidity.relative_humidity));
    AHT10TemperatureProperty.setValue(toPvalueNumber(temperature.temperature)); 
    mqtt_reconnect();
    ESP_LOGI(TAG, "%05d Humidity=%.2lf%% dbid=%i : Tempurature=%.2lf dbid=%i \n", ++i, humidity.relative_humidity, AHT10HumidityProperty.propertyDbId, temperature.temperature, AHT10TemperatureProperty.propertyDbId);
}
void loop(void) {
    digitalWrite(23, HIGH);
   
    mqttReConnect();
    
    readAHT10();
    adapter->update();  // pushit to the iot gateway
    delay(pushInterval);
}
void do_fade(const JsonVariant &input) {
    JsonObject inputObj = input.as<JsonObject>();
    ESP_LOGD(TAG, "inputObj=%s\n", inputObj);
    long long int duration = inputObj["duration"];
    long long int brightness = inputObj["brightness"];

    delay(duration);

    ThingDataValue value = {.integer = brightness};
    lampLevel.setValue(value);

    int level = (int)Arduino_h::map(brightness, 0, 100, 255, 0);
    ESP_LOGD(TAG, "value =%i , level(mapped)=%d \n", (int)value.integer, level);
    analogWrite(lampPin, level, 255);
    lc.clearDisplay(0);
    lc.printF((float)brightness, (char *)"%.2f");
    ThingDataValue val;

    ThingEventObject *ev = new ThingEventObject("overheated", NUMBER, val);
    ESP_LOGI(TAG, " Queu event(overheated) %2.2f\n", val.number);
    lamp.queueEventObject(ev);
}
ThingActionObject *action_generator(DynamicJsonDocument *input) {
    String output;
    serializeJson(*input, output);
    ESP_LOGI(TAG, "Serialising  DynamicJsonDocument:\n\toutput:%s\n", output.c_str());
    return new ThingActionObject("fade", input, do_fade, nullptr);
}