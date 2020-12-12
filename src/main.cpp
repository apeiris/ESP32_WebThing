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
#define dbgx
#include <Adafruit_AHTX0.h>
#include <Arduino.h>
//#include <ArduinoOTA.h>
#include <SPI.h>
//#include <StreamUtils.h>
#include <analogWrite.h>
#include <map>
#include <PubSubClient.h>
#include "LedControl.h"
#include "Thing.h"
#include "WebthingAdapter.h"
#include "esp_log.h"
#include "esp_system.h"
#include "password.h"
#include <iostream>
using namespace std;

static int loopInterval = 5000;
#if defined(LED_BUILTIN)
const int ledPin = LED_BUILTIN;
#else
const int ledPin =
    2; // manually configure LED pin was 13 2 is blue led built in
const int lampPin = 2;
#endif
// AHT10 pins
const int pinSDA = 5;  // white wire
const int pinSCL = 17; // the purple
// pin definitions for MAX7129

const int pinCLK = 18; // Clock pin
const int pinCS = 19;  // Chip Select

const int pinDIN = 23; // Data
const int anzMAX = 1;  // Anzahl der kaskadierten  Module = Number of Cascaded modules

String output = "";
WebThingAdapter *adapter; // adapter could then be used as as device
ThingActionObject *action_generator(DynamicJsonDocument *);
static String sMacId = "";
static const char *TAG = "*";
static int i = 0;
const char *sensorTypes[] = {"Sensor", "Sensor", nullptr};
const char *asyncProperties[] = {"asyncProperty", "I/O", nullptr};

ThingDevice AHT10Device("AHT10", "AHT10", sensorTypes);
ThingDevice textDisplay("Text", "text Async Property Test", asyncProperties);
ThingDevice testDevice("AHT10", "AHT10", sensorTypes);

// Forward declaration
void textDisplayTextChanged(ThingPropertyValue newVal);
void updateDeviceDbIds(String s);

ThingProperty AHT10TemperatureProperty("Temperature", "Temperature in C", NUMBER, "Centigrades");
ThingProperty AHT10HumidityProperty("Humidity", "Humidity (RH) %", NUMBER, "%");

ThingProperty textDisplayText("text", "description", STRING, "yo", textDisplayTextChanged);
ThingProperty testDeviceProperty("test", "testing", NUMBER, "%");
//------------------------------------------------------------------
void textDisplayTextChanged(ThingPropertyValue newVal)
{
    String x = *newVal.string;
    ESP_LOGI(TAG, "text=>%s", x.c_str());
}
const unsigned char redPin = 12;
const unsigned char greenPin = 13;
const unsigned char bluePin = 14;
//---------------GPIO33 pulled low and connected to button to pull high to 3.3v
#define BUTTON_WAKEUP_BITMASK 0x200000000 // 2^33 in hex / GPIO33
//------------------------------------------------------------------
IPAddress mqttIp = IPAddress(192, 168, 0, 10); // dockerized broker on laptop
//------------Adafruit Lib -----------------------------------------
// the AHT must be connected to Bords SDA->GPIO21 and SCL ->GPIO22
//  to detect the sensor board
Adafruit_AHTX0 aht;
WiFiClient client;
PubSubClient mqtt(client);
//------------------------------------------------------------------
LedControl lc = LedControl(pinDIN, pinCLK, pinCS, 1);

//----------------Init MAX7219--------------------------------------
void initMAX7219()
{
    ESP_LOGI(TAG, "Initializing MAX7219..");
    lc.shutdown(0, false);
    lc.setIntensity(0, 10);
    lc.clearDisplay(0);
    lc.printF(0, (char *)"%0.2f");
}
//--------------------initWifi()------------------------------------
void initWifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    bool blink = false;
    ESP_LOGI(TAG, "Waiting WiFi == WL_CONNECTED \n\t");
    while (WiFi.status() != WL_CONNECTED)
    {
        printf(".");
        digitalWrite(ledPin, blink ? LOW : HIGH); // active low led
        digitalWrite(ledPin, blink);
        delay(80);
    }
}
//----------------initAHT()-----------------------------------------
void initAHT()
{
    if (!aht.begin())
    {
        ESP_LOGI(TAG, "Could not find AHT !");
    }
    else
        ESP_LOGI(TAG, "AHT10 or AHT20 found");
}
//----------------initAdapterAndAddDevices()------------------------
//StaticJsonDocument<256> fadeInput;
StaticJsonDocument<256> sJdoc;
//JsonObject fadeInputObj = fadeInput.to<JsonObject>();
JsonObject jObj = sJdoc.to<JsonObject>();
ThingAction fade("fade", "Fade", "Fade the lamp to a given level", "FadeAction", &jObj, action_generator);

//--------------------lamp (LED) stuff ---------------------------
const char *lampTypes[] = {"OnOffSwitch", "Light", nullptr};
ThingEvent overheated("overheated", "The lamp has exceeded its safe operating temperature", NUMBER, "OverheatedEvent");
ThingDevice lamp("LED", "Dimmable LED", lampTypes);
ThingProperty lampOn("on", "Whether the lamp is turned on", BOOLEAN, "OnOffProperty");
ThingProperty lampLevel("brightnesss", "The level of light from 0-100", INTEGER, "BrightnessProperty");
//------------Setup device and props for StationId------------------
JsonObject jStation = sJdoc.to<JsonObject>();
const char *stationIdTypes[] = {"StationId", "YOHOO", nullptr};
ThingDevice stationDevice("StationId", "Station", stationIdTypes);
ThingProperty stationIdProperty("StationId", "", STRING, "macIdProperty");
ThingProperty stationLoopIntervalPropery("LoopInterval", "Loop delay in milli seconds", INTEGER, "Loop Interval");
ThingAction liChanged("liChanged", "Loop Interval change", "Change the Loop Interval (Dealy)", "Interval change", &jStation, action_generator);
//------------MQTT callback-----------------------------------------
void callback(char *topic, byte *payload, unsigned int length)
{
    ESP_LOGI(TAG, "\n\n\nMessage arrived[%s]\n\n\n", topic);
    payload[length] = '\0';
    String s = String((char *)payload);
    if (strcmp("IOT/DevicesRegistered", topic) == 0)
    {
        updateDeviceDbIds(s);
    }
    if (strcmp("MQTT/ListOfSubscriptions", topic) == 0)
    {
        ESP_LOGI(TAG, "Here is the list of subs \n\t%s", s.c_str());
    }

    ESP_LOGI(TAG, "\t payload=%s", s.c_str());

    mqtt.publish("cb", "Callback..");
}
//----------------------------------------------------------------
void initAdapterAndAddDevices()
{
    String x = WiFi.macAddress();
    x.replace(":", "");
    adapter = new WebThingAdapter(x, WiFi.localIP());
    { // devices and the properties
        //---------Station Device ------------------------
        const char *pMacId;
        pMacId = sMacId.c_str();
        ThingDataValue sValue = {.string = (String *)pMacId};

        stationIdProperty.setValue(sValue);
        stationIdProperty.readOnly = true;
        ThingDataValue iValue = {.integer = loopInterval};
        ESP_LOGD(TAG, "setting loopInterval= %i", loopInterval);
        stationLoopIntervalPropery.setValue(iValue);

        stationDevice.id = "Station";
        stationDevice.title = pMacId;

        jObj["type"] = "object";
        JsonObject liChangedProps = jObj.createNestedObject("properties");
        JsonObject loopIntervalInput = liChangedProps.createNestedObject("loopInterval");
        loopIntervalInput["type"] = "integer";
        loopIntervalInput["minimum"] = 1000;   // 1 second
        loopIntervalInput["maximum"] = 360000; // 1 hour

        stationDevice.addProperty(&stationIdProperty);
        stationDevice.addProperty(&stationLoopIntervalPropery);
        stationDevice.addAction(&liChanged);
        adapter->addDevice(&stationDevice);
        //---------Lamp-----------------------------------
        lampLevel.minimum = 0;
        lampLevel.maximum = 100;
        lampLevel.unit = "%";
        lamp.addProperty(&lampLevel);
        // create fade Input object and json
        //   jObj["type"] = "object";
        JsonObject fadeInputProperties = jObj.createNestedObject("properties");            // must be properties
        JsonObject brightnessInput = fadeInputProperties.createNestedObject("brightness"); //"brightness" mustmatch property
        brightnessInput["type"] = "integer";
        brightnessInput["minimum"] = 0;
        brightnessInput["maximum"] = 100;
        brightnessInput["unit"] = "percent";
        lamp.addAction(&fade);
        overheated.unit = "degree C";
        lamp.addEvent(&overheated);
        lamp.addProperty(&lampOn);
        adapter->addDevice(&lamp);
        //-------------------------------------------------------------------
        AHT10TemperatureProperty.readOnly = true;
        AHT10Device.addProperty(&AHT10TemperatureProperty);
        AHT10HumidityProperty.readOnly = true;
        AHT10Device.addProperty(&AHT10HumidityProperty);
        adapter->addDevice(&AHT10Device);
        //--------------------------------------------------------------------
    }

    adapter->begin();
    ESP_LOGI(TAG, "Webthing Adapter initialized");
}
//------------------------------------------------------------------
void setMacId()
{
    String x = WiFi.macAddress();
    x.replace(":", "");
    sMacId = x;
}
//------------------------------------------------------------------
void mqttReConnect()
{
    mqtt.setBufferSize(2048);

    while (!mqtt.connected())
    {
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        ESP_LOGI(TAG, "Attempting MQTT connection with random ClientId=%s ", clientId.c_str());
        if (mqtt.connect(clientId.c_str(), NULL, NULL))
        {
            ESP_LOGI(TAG, "Waiting for message \"IOT/DevicesRegistered\n\t bufferSize=%i", mqtt.getBufferSize());
            mqtt.subscribe("IOT/DevicesRegistered");
            mqtt.subscribe("MQTT/ListOfSubscriptions");
        }
        else
        {
            ESP_LOGE(TAG, "***** Failed connect mqtt *****");
            delay(5000);
        };
    }
}
//----registerDevices() in the SQL Serve from the adapter-----------
String getAdapterJson()
{
    DynamicJsonDocument doc(2000);
    doc["macId"] = sMacId;
    JsonArray devices = doc.createNestedArray(&"devices");
    JsonObject device;
    JsonArray props;
    JsonObject prop;
    ThingDevice *d = adapter->getFirstDevice();
    int di = 0, pi = 0, pt = 0;
    while (d)
    {
        device = devices.createNestedObject();
        device["seed"] = di;
        device["id"] = d->id;
        device["title"] = d->title;
        device["name"] = d->id;
        props = device.createNestedArray("props");
        ThingProperty *p = d->firstProperty;
        pi = 0;
        while (p)
        {
            prop = props.createNestedObject();
            prop["seed"] = pi;
            prop["Id"] = p->id;
            prop["dbId"] = p->propertyDbId;
            prop["description"] = p->description;
            pi++;
            p = (ThingProperty *)p->next;
        }
        di++;
        pt += pi;
        d = d->next;
    }
    ESP_LOGD(TAG, "Devices in chain=%i, properties=%i", di, pt);
    String output = "";
    serializeJson(doc, output);
    return output;
}
void updateDeviceDbIds(String s)
{
    DynamicJsonDocument jd(2000);
    deserializeJson(jd, s);
    ThingDevice *d = adapter->getFirstDevice();
    int di = 0;
    ESP_LOGD(TAG, "\n");
    while (d)
    {
        ThingProperty *p = d->firstProperty;
        int pi = 0;
        while (p)
        {

            ESP_LOGD(TAG, "jd['devices'].[%i].props[%i].dbId=%i ,id=%s ", di, pi, (int)jd["devices"][di]["props"][pi]["dbId"], jd["devices"][di]["props"][pi]["Id"].as<String>().c_str());
            p->propertyDbId = (int)jd["devices"][di]["props"][pi]["dbId"];
            pi++;
            p = (ThingProperty *)p->next;
        }
        di++;
        d = d->next;
    }
    ESP_LOGD(TAG, "\n");
}
//------------------------------------------------------------------
void setup(void)
{
    Serial.begin(115200);

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);

    ESP_LOGI(TAG, "Connecting to %s \n", ssid);
    //--- RTC wakeup on gpio 33 (wired to button)
    esp_sleep_enable_ext1_wakeup(BUTTON_WAKEUP_BITMASK,
                                 ESP_EXT1_WAKEUP_ANY_HIGH);

    setMacId();
    initMAX7219();              // (1) --- init MAX7219-------------------------------
    initAHT();                  // (2)----- init AHT----------------------------------
    initWifi();                 // (3)------init WiFi---------------------------------
    initAdapterAndAddDevices(); // (4) -- intiAdapterAndAddDevices()------------------
    output = getAdapterJson();  // (6) --All devices in the adapter, Json form--------

    mqtt.setCallback(callback);
    mqtt.setServer(mqttIp, 1883);
    String topic = "IOT/RegisterDevices";
    ESP_LOGI(TAG, "Publishsing -> %s\n \t %s ", topic.c_str(), output.c_str());
    while (!mqtt.connected())
        mqttReConnect();
    mqtt.publish(topic.c_str(), output.c_str());
    ESP_LOGI(TAG, "\nout=\n%s\n\t\t len=%i", output.c_str(), output.length());
}
ThingPropertyValue toPvalueNumber(double n)
{
    ThingPropertyValue pv;
    pv.number = n;
    return pv;
}
void readAHT10()
{
    sensors_event_t humidity, temperature;

    aht.getEvent(&humidity, &temperature); //   gcvt(humidity.relative_humidity,5,fs);

    if (AHT10Device.ws->availableForWriteAll())
    {
        AHT10HumidityProperty.setValue(toPvalueNumber(humidity.relative_humidity));
        AHT10TemperatureProperty.setValue(toPvalueNumber(temperature.temperature));
        ESP_LOGI(TAG, "%05d Hum=%.2lf%% dbid=%i:Temp=%.2lf dbid=%i   Heap=%d,%d", ++i, humidity.relative_humidity, AHT10HumidityProperty.propertyDbId, temperature.temperature, AHT10TemperatureProperty.propertyDbId, ESP.getFreeHeap(),AHT10Device.ws->count());
        AHT10Device.ws->cleanupClients();
    }
    else
        ESP_LOGI(TAG, "Cann not update heap=%d", ESP.getFreeHeap());
    if (i == 10)
    {
        String topic = "MQTT/ListSubscriptions";
        mqtt.publish(topic.c_str(), "");
    }
}
void loop(void)
{
    digitalWrite(23, HIGH);
    mqttReConnect();
    mqtt.loop();
    readAHT10();
    adapter->update(); // pushit to the iot gateway
    delay(loopInterval);
}
void do_fade(const JsonVariant &input)
{
    JsonObject inputObj = input.as<JsonObject>();
    ESP_LOGD(TAG, "inputObj=%i", input.as<long>());
    long long int duration = inputObj["duration"];
    long long int brightness = inputObj["brightness"];
    ThingDataValue value = {.integer = brightness};
    lampLevel.setValue(value);
    int level = (int)Arduino_h::map(brightness, 0, 100, 255, 0);
    ESP_LOGD(TAG, "value =%i , level(mapped)=%d ", (int)value.integer, level);
    analogWrite(lampPin, level, 255);
    lc.clearDisplay(0);
    lc.printF((float)brightness, (char *)"%.2f");
    ThingDataValue val;
    ThingEventObject *ev = new ThingEventObject("overheated", NUMBER, val);
    ESP_LOGI(TAG, " Queu event(overheated) %2.2f", val.number);
    lamp.queueEventObject(ev);
}
ThingActionObject *action_generator(DynamicJsonDocument *input)
{
    String output;
    serializeJson(*input, output);
    ESP_LOGD(TAG, "Serialize = %s", output.c_str());

    return new ThingActionObject("fade", input, do_fade, nullptr);
}
