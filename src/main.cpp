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
#define DdisplayString
#undef DdisplayString
#define LARGE_JSON_BUFFERS 1
#define dbgx
#include <Adafruit_AHTX0.h>
#include <Arduino.h>
//#include <ArduinoOTA.h>
#include <SPI.h>
#include <analogWrite.h>
#include <map>
#include <PubSubClient.h>
#include "Thing.h"
#include "WebthingAdapter.h"
#include "esp_log.h"
#include "esp_system.h"
#include "password.h"
#include <iostream>
#include <string>
#include "max7219.h"
#define debugActionGen
#undef debugActionGen
// .5 minute interval
const int loopInterval = 5000;

#define TRUE (1 == 1)
#define FALSE (!TRUE)

#if defined(LED_BUILTIN)
const int ledPin = LED_BUILTIN;
#else
const int ledPin =
    2; // manually configure LED pin was 13, 2 is the blue led built in
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
String sIP = "sIPinit";
String sIPLast = sIP;

bool LEDon = false;
static const char *TAG = "*";
static int i = 0;
static bool bGotIp = false;
//const char *asyncProperties[] = {"asyncProperty", "I/O", nullptr};
//----forward declarations -----------------------------------------
void updateDeviceDbIds(String s);
//------------------------------------------------------------------
const unsigned char redPin = 25;
const unsigned char greenPin = 26;
const unsigned char bluePin = 27;
//---------------GPIO33 pulled low and connected to button to pull high to 3.3v
#define BUTTON_WAKEUP_BITMASK 0x200000000 // 2^33 in hex / GPIO33
//------------------------------------------------------------------
IPAddress mqttIp = IPAddress(10, 88, 111, 6); // dockerized broker on laptop
//------------Adafruit Lib -----------------------------------------
// the AHT must be connected to Bords SDA->GPIO21 and SCL ->GPIO22
//  to detect the sensor board
Adafruit_AHTX0 aht;
WiFiClient client;
PubSubClient mqtt(client);
MAX7219 max7219;
//----------Toggle built in LED-------------------------------------
void toggleLed()
{
    digitalWrite(ledPin, LEDon ? LOW : HIGH);
    LEDon = !LEDon;
}
//------------------------------------------------------------------
//LedControl lc = LedControl(pinDIN, pinCLK, pinCS, 1);
//------------------------------------------------------------------
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
JsonObject jObj = sJdoc.to<JsonObject>();
//--------------------lamp (LED) stuff ---------------------------

//------------Setup device and props for StationId------------------

//--------timer-79a21fe843a96acf36c6810cd5b57bf0--------------------

//------------------------------------------------------------------

//------------callback----------------------------------------------
void WifiEvent(WiFiEvent_t e, WiFiEventInfo_t i)
{
    switch (e)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        bGotIp = false;
        delay(10);
        sIP=WiFi.localIP().toString().c_str();
        delay(10);
        ESP_LOGI(TAG, "GOT_IP Local Ip=%s", sIP);
        bGotIp = true;
    
        break;
    default:
        ESP_LOGI(TAG, "Wifi Event (%d,%d)\n", e, i);
        break;
    }
}
void mqttCallback(char *topic, byte *payload, unsigned int length)
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
//------------------------------------------------------------------
#pragma region AHT10 defines
//-----------------AHT10 Ambient sensor-----------------------------
const char *AHT10Types[] = {"AmbientSensor", nullptr};
ThingDevice deviceAHT10("AHT10", "AHT10 Ambient Sensor", AHT10Types);
ThingProperty propertyAHT10Temp("Temperature", "", NUMBER, "Tempurature");
ThingProperty propertyAHT10Humidity("Humidity", "", NUMBER, "Humidity");
#pragma endregion AHT10 defines
#pragma region lamp defines
const char *lampTypes[] = {"OnOffSwitch", "Light", nullptr};
ThingDevice deviceLamp("lamp", "My Lamp", lampTypes);
ThingProperty propertyLampOn("on", "Whether the lamp is turned on", BOOLEAN, "OnOffProperty");
ThingProperty propertyLampLevel("brightness", "The level of light from 0-100", INTEGER, "BrightnessProperty");
StaticJsonDocument<256> fadeInput;
JsonObject fadeInputObj = fadeInput.to<JsonObject>();
ThingAction fade("fade", "Fade", "Fade the lamp to a given level", "FadeAction", &fadeInputObj, action_generator);
ThingEvent overheated("overheated", "The lamp has exceeded its safe operating temperature", NUMBER, "OverheatedEvent");
bool lastOn = true;
#pragma endregion lamp defines
//------------------------------------------------------------------
void initAdapterAndAddDevices()
{
    String x = WiFi.macAddress();
    x.replace(":", "");
    adapter = new WebThingAdapter(x, WiFi.localIP());

    { // devices and the properties
        ESP_LOGD(TAG, "adapter=newWebThingAdapter(\"%s\",\"%s\")\n", x.c_str(), WiFi.localIP().toString().c_str());
#pragma region AHT10
        //-------------------------------------------------------------------
        propertyAHT10Humidity.readOnly = true;
        propertyAHT10Humidity.unit = " %";
        propertyAHT10Temp.readOnly = true;
        propertyAHT10Temp.unit = " °C";
        deviceAHT10.addProperty(&propertyAHT10Temp);
        deviceAHT10.addProperty(&propertyAHT10Humidity);
        adapter->addDevice(&deviceAHT10);
#pragma endregion AHT10
#pragma region lamp
        deviceLamp.description = "A web connected lamp";
        propertyLampOn.title = "On/Off";
        deviceLamp.addProperty(&propertyLampOn);

        propertyLampLevel.title = "Brightness";
        propertyLampLevel.minimum = 0;
        propertyLampLevel.maximum = 100;
        propertyLampLevel.unit = "percent";
        deviceLamp.addProperty(&propertyLampLevel);

        fadeInputObj["type"] = "object";
        JsonObject fadeInputProperties =
            fadeInputObj.createNestedObject("properties");
        JsonObject brightnessInput =
            fadeInputProperties.createNestedObject("brightness");
        brightnessInput["type"] = "integer";
        brightnessInput["minimum"] = 0;
        brightnessInput["maximum"] = 100;
        brightnessInput["unit"] = "percent";
        JsonObject durationInput =
            fadeInputProperties.createNestedObject("duration");
        durationInput["type"] = "integer";
        durationInput["minimum"] = 1;
        durationInput["unit"] = "milliseconds";
        deviceLamp.addAction(&fade);

        overheated.unit = "degree celsius";
        deviceLamp.addEvent(&overheated);
        adapter->addDevice(&deviceLamp);
#pragma endregion lamp
    }
    adapter->begin();
    ESP_LOGI(TAG, "Webthing Adapter initialized");
}

//------------------------------------------------------------------

long lastReconnectAttempt = 0;
//------------------------------------------------------------------
void mqttReConnect()
{
    mqtt.setBufferSize(2048);

    ESP_LOGI(TAG, "mqttIp=%s", mqttIp.toString().c_str());
    while (!mqtt.connected())
    {
        long now = millis();
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        ESP_LOGI(TAG, "Attempting MQTT connection with random ClientId=%s ", clientId.c_str());
        lastReconnectAttempt = now;
        if (mqtt.connect(clientId.c_str(), NULL, NULL))
        {
            ESP_LOGI(TAG, "Waiting for message \"IOT/DevicesRegistered\n\t bufferSize=%i", mqtt.getBufferSize());
            mqtt.subscribe("IOT/DevicesRegistered");
            mqtt.subscribe("MQTT/ListOfSubscriptions");
            lastReconnectAttempt = 0;
        }
        else
        {
            // ESP_LOGE(TAG, "***** Failed connect mqtt *****\nRebooting..");
            // ESP.restart();
            ESP_LOGE(TAG, "***** Failed connect mqtt *****\nEntering deep sleep..");
            esp_deep_sleep(10000000);
        };
    }
}
//----registerDevices() in the SQL Serve from the adapter-----------
String getAdapterJson()
{
    DynamicJsonDocument doc(2000);
    String x = WiFi.macAddress();
    x.replace(":", "");
    doc["macId"] = x.c_str();
    ESP_LOGD(TAG, "macId=%s, Wifi.localip=%s", x.c_str(), WiFi.localIP().toString().c_str());
    while (!bGotIp)
    {
        ESP_LOGD(TAG, "Looping until IP is set");
        delay(100);
    }
    doc["IP"] =sIP;

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
        device["title"] = d->title.c_str();
        device["name"] = d->id;
        ESP_LOGD(TAG, "device[id]=%i\n\t Title=%s ", d->id, d->title.c_str());
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
    ESP_LOGD(TAG, "JSON=%s\n", output.c_str());
    return output;
}
void updateDeviceDbIds(String s)
{
    DynamicJsonDocument jd(2000);
    ESP_LOGI(TAG, "updateDeviceDbIds ->Gotten Json=%s\n", s.c_str());
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
//------------------------------------------------------------------
String getPropertiesJson(ThingDevice *d)
{
   // ESP_LOGD(TAG, "getPropertiesJson");
    DynamicJsonDocument doc(2500);
    JsonObject prop;
    doc["StationId"] = sMacId;
    doc["Device"] = d->id;
    JsonArray props = doc.createNestedArray(&"props");
    ThingProperty *p = d->firstProperty;
    while (p)
    {
        prop = props.createNestedObject();
        prop["Id"] = p->id;
        prop["value"] = p->getValue().number;
        prop["dbId"] = p->propertyDbId;
        p = (ThingProperty *)p->next;
    }
    String s = "";
    serializeJson(doc, s);
    ESP_LOGD(TAG, "get propertiesJson=%s\n", s.c_str());
    return s;
}
//------------------------------------------------------------------
void setup(void)
{

    Serial.begin(115200);
  
    // WiFi.onEvent(WiFiStationConnected,SYSTEM_EVENT_STA_CONNECTED);
    WiFi.onEvent(WifiEvent, SYSTEM_EVENT_STA_GOT_IP);
   
    pinMode(ledPin, OUTPUT);
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);

    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);

    digitalWrite(ledPin, HIGH);
    ESP_LOGI(TAG, "Connecting to %s \n", ssid);
    //--- RTC wakeup on gpio 33 (wired to button)
    esp_sleep_enable_ext1_wakeup(BUTTON_WAKEUP_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
    max7219.Begin();
         max7219.DisplayText(( char *)"no cue", 1);
    
    // setMacId();
    //initMAX7219(); // (1) --- init MAX7219-------------------------------

    initAHT(); // (2)----- init AHT----------------------------------
    //-------------hook the handler
    // WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);

    initWifi(); // (3)------init WiFi---------------------------------
 

    initAdapterAndAddDevices(); // (4) -- intiAdapterAndAddDevices()------------------
    ThingPropertyValue initialOn = {.boolean = true};
    propertyLampOn.setValue(initialOn);
    (void)propertyLampOn.changedValueOrNull();

    output = getAdapterJson(); // (6) --All devices in the adapter, Json form--------
    ESP_LOGD(TAG, "output=%s\n\n", output.c_str());

    mqtt.setServer((IPAddress)mqttIp, 1883);
    mqtt.setCallback(mqttCallback);
    ESP_LOGI(TAG, "mqtt state=%i\n ADF version %s\n\n", mqtt.state(), esp_get_idf_version());
    while (!mqtt.connected())
        mqttReConnect();
    if (mqtt.connected())
    {
        String topic = "IOT/RegisterDevices";
        ESP_LOGI(TAG, "Publishsing -> %s\n \t %s ", topic.c_str(), output.c_str());
        mqtt.publish(topic.c_str(), output.c_str());
        ESP_LOGI(TAG, "\ndeclare @json nvarchar(max)=\'%s\'\n\t\t len=%i", output.c_str(), output.length());
    }
    else
    {
        ESP_LOGI(TAG, "Could not connect to mqtt..\n\n");
    }
    //--------------------------------------------------------------------------
}
ThingPropertyValue toPvalueNumber(double n)
{
    ThingPropertyValue pv;
    pv.number = n;
    return pv;
}
ThingPropertyValue toPvalueString(String s)
{
    ThingPropertyValue v;
    v.string = &s;
    return v;
}
//------------------------------------------------------------------
void readAHT10()
{
    toggleLed();
    sensors_event_t humidity, temperature;
    aht.getEvent(&humidity, &temperature); //   gcvt(humidity.relative_humidity,5,fs);

    if (deviceAHT10.ws->availableForWriteAll())
    {
        propertyAHT10Humidity.setValue(toPvalueNumber(humidity.relative_humidity));

        char t[20];
        sprintf(t, "%.2lfc", temperature.temperature);

        propertyAHT10Temp.setValue(toPvalueNumber(temperature.temperature));

        ESP_LOGI(TAG, "%05d  t%s Hum=%.2lf%% dbid=%i:Temp=%.2lf dbid=%i   Heap=%d,%d", ++i, t, humidity.relative_humidity, propertyAHT10Temp.propertyDbId, temperature.temperature, propertyAHT10Temp.propertyDbId, ESP.getFreeHeap(), deviceAHT10.ws->count());
        if (propertyAHT10Temp.propertyDbId != -1)
        {
            String eData = getPropertiesJson(&deviceAHT10);
            mqtt.publish("IOT/DeviceEvent", eData.c_str());
            max7219.Clear();
            max7219.DisplayText(t, 1);
        }
    }
    else
    {
        ESP_LOGI(TAG, "Cann not update heap=%d", ESP.getFreeHeap());
        ESP_LOGI(TAG, "Rebooting .....");
        ESP.restart();
    }
    if (i == 10)
    {
        String topic = "MQTT/ListSubscriptions";
        mqtt.publish(topic.c_str(), "");
    }
}
void loop(void)
{
    digitalWrite(23, HIGH);
    mqtt.loop();
    readAHT10();
    adapter->update(); // pushit to the iot gateway
    delay(loopInterval);
}

void do_fade(const JsonVariant &input)
{
    JsonObject inputObj = input.as<JsonObject>();
    long long int duration = inputObj["duration"];
    long long int brightness = inputObj["brightness"];

    delay(duration);

    ThingDataValue value = {.integer = brightness};
    propertyLampLevel.setValue(value);
    int level = map(brightness, 0, 100, 255, 0);
    analogWrite(lampPin, level);

    ThingDataValue val;
    val.number = 102;
    ThingEventObject *ev = new ThingEventObject("overheated", NUMBER, val);
    ESP_LOGI(TAG, "Queued Event ->%s", ev->name.c_str());
    deviceLamp.queueEventObject(ev);
}

ThingActionObject *action_generator(DynamicJsonDocument *input)
{
    return new ThingActionObject("fade", input, do_fade, nullptr);
}