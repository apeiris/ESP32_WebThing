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
#include <string>
#define debugActionGen
#undef debugActionGen
static int loopInterval = 5000;
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
//const char *asyncProperties[] = {"asyncProperty", "I/O", nullptr};
//----forward declarations -------------------------------------

const char *sensorTypes[] = {"Sensor", "Sensor", nullptr};
ThingEvent eventAHT10("eventAHT10", "AHT10 raised an Event", NUMBER, "eventAHT10");
ThingDevice deviceAHT10("AHT10", "AHT10", sensorTypes);
void tc(ThingPropertyValue newValue);
void updateDeviceDbIds(String s);
//------------------------------------------------------------------
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
//----------Toggle built in LED-------------------------------------
void toggleLed()
{
    digitalWrite(ledPin, LEDon ? LOW : HIGH);
    LEDon = !LEDon;
}
//------------------------------------------------------------------
LedControl lc = LedControl(pinDIN, pinCLK, pinCS, 1);
//------------------------------------------------------------------
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
JsonObject jObj = sJdoc.to<JsonObject>();

//--------------------lamp (LED) stuff ---------------------------
const char *lampTypes[] = {"OnOffSwitch", "Light", nullptr};
ThingDevice lamp("urn:dev:ops:my-lamp-1234", "My Lamp", lampTypes);
ThingProperty lampOn("on", "Whether the lamp is turned on", BOOLEAN, "OnOffProperty");
ThingProperty lampLevel("brightness", "The level of light from 0-100", INTEGER, "BrightnessProperty");
StaticJsonDocument<256> fadeInput;
JsonObject fadeInputObj = fadeInput.to<JsonObject>();
ThingAction fade("fade", "Fade", "Fade the lamp to a given level", "FadeAction", &fadeInputObj, action_generator);
ThingEvent overheated("overheated", "The lamp has exceeded its safe operating temperature", NUMBER, "OverheatedEvent");
bool lastOn=true;

//------------------------------------------------------------------
ThingProperty AHT10TemperatureProperty("Temperature", "Temperature in C", NUMBER, "Centigrades");
ThingProperty AHT10HumidityProperty("Humidity", "Humidity (RH) %", NUMBER, "%");
//------------Setup device and props for StationId------------------
JsonObject jStation = sJdoc.to<JsonObject>();
const char *stationIdTypes[] = {"StationId", "YOHOO", nullptr};
ThingDevice stationDevice("StationId", "Station", stationIdTypes);
ThingProperty stationIdProperty("StationId", "desc", STRING, "macIdProperty", nullptr);
ThingProperty              text("text"      ,""    ,STRING ,nullptr,tc);
ThingProperty stationIPProperty("StationIP", "desc", STRING,nullptr,tc);
ThingProperty stationLoopIntervalPropery("LoopInterval", "Loop delay in milli seconds", INTEGER, "Loop Interval");
ThingAction liChanged("liChanged", "Loop Interval change", "Change the Loop Interval (Dealy)", "Interval change", &jStation, action_generator);
//------------callback----------------------------------------------

void WifiEvent(WiFiEvent_t e, WiFiEventInfo_t i)
{
    switch (e)
    {
    case SYSTEM_EVENT_WIFI_READY:
        ESP_LOGI(TAG, "Wifi  (%d,%d) WIFI_READY \n", e, i);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "Wifif GOT_IP(%d,%d) - IP:", e, i);
        ESP_LOGI(TAG,"Local Ip=%s",WiFi.localIP().toString().c_str());
         
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
//------------------------------------------------------------------
void initAdapterAndAddDevices()
{
    String x = WiFi.macAddress();
    x.replace(":", "");
    adapter = new WebThingAdapter(x, WiFi.localIP());
    { // devices and the properties
        ESP_LOGI(TAG, "adapter=newWebThingAdapter(\"%s\",\"%s\")\n", x.c_str(), WiFi.localIP().toString().c_str());
        const char *pMacId;
        stationDevice.id = "Station";
        stationDevice.title = pMacId;
        jObj["type"] = "object";
        JsonObject liChangedProps = jObj.createNestedObject("properties");
        JsonObject loopIntervalInput = liChangedProps.createNestedObject("loopInterval");
        loopIntervalInput["type"] = "integer";
        loopIntervalInput["minimum"] = 1000;   // 1 second
        loopIntervalInput["maximum"] = 360000; // 1 hour
        //----------------------Station--------------------
        //---------Station Device ------------------------
        pMacId = sMacId.c_str();
        ThingDataValue sValue = {.string = (String *)pMacId};
        stationIdProperty.setValue(sValue);
        stationIdProperty.readOnly = true;
     //   stationIPProperty.setValue(sValue); set at RegisterIP
        // stationIPProperty.readOnly=true;
        ThingDataValue iValue = {.integer = loopInterval};
        ESP_LOGD(TAG, "setting loopInterval= %i", loopInterval);
        stationLoopIntervalPropery.setValue(iValue);
        stationDevice.addProperty(&stationIdProperty);
        stationDevice.addProperty(&stationLoopIntervalPropery);
        stationDevice.addProperty(&stationIPProperty);
        //-- actions
        stationDevice.addAction(&liChanged);
        adapter->addDevice(&stationDevice);
        //---------Lamp-----------------------------------
        lamp.description="A web connected lamp";
        lamp.title=("On/Off");
        lamp.addProperty(&lampOn);

        lampLevel.title="Brightness";
        lampLevel.minimum = 0;
        lampLevel.maximum = 100;
        lampLevel.unit = "percent";
        lamp.addProperty(&lampLevel);
        // create fade Input object and json
        fadeInputObj["type"]="object";
        JsonObject fadeInputProperties = fadeInputObj.createNestedObject("properties");            // must be properties
        JsonObject brightnessInput = fadeInputProperties.createNestedObject("brightness"); //"brightness" mustmatch property
        brightnessInput["type"] = "integer";
        brightnessInput["minimum"] = 0;
        brightnessInput["maximum"] = 100;
        brightnessInput["unit"] = "percent";
        JsonObject durationInput = fadeInputProperties.createNestedObject("duration");
        durationInput["type"]="integer";
        durationInput["minimum"]=1;
        durationInput["unit"]="milliseconds";
        lamp.addAction(&fade);

        overheated.unit = "degree C";

        lamp.addEvent(&overheated);
        adapter->addDevice(&lamp);
        //-------------------------------------------------------------------
        AHT10TemperatureProperty.readOnly = true;
        deviceAHT10.addProperty(&AHT10TemperatureProperty);
        AHT10HumidityProperty.readOnly = true;
        deviceAHT10.addProperty(&AHT10HumidityProperty);
        // deviceAHT10.addEvent(&eventAHT10);
        adapter->addDevice(&deviceAHT10);
        // adding experimental event listener
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
    sIP = WiFi.localIP().toString();
    ESP_LOGI(TAG, "sIp=%s", sIP.c_str());
}
long lastReconnectAttempt = 0;
//------------------------------------------------------------------
void mqttReConnect()
{
    mqtt.setBufferSize(2048);
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
    doc["macId"] = sMacId;
  //  doc["IP"]=WiFi.localIP().toString();

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
    ESP_LOGI(TAG, "Gotten Json=%s\n", s.c_str());
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
    DynamicJsonDocument doc(2000);

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
        ESP_LOGI(TAG, "aType %d ,type=%d id=%s", p->atType, p->type, p->id.c_str());

        p = (ThingProperty *)p->next;
    }
    String s = "";
    serializeJson(doc, s);
    ///ESP_LOGI(TAG, "Seriealize =%s\n", s.c_str());
    return s;
}
//------------------------------------------------------------------
void tc(ThingPropertyValue newValue) {
 ESP_LOGI(TAG,"*************************New message : ");
  Serial.println(*newValue.string);
  ESP_LOGI(TAG,"%s",*newValue.string->c_str());
}

   
//------------------------------------------------------------------
void RegisterIP()
{
    ThingPropertyValue value;
    String msg=WiFi.localIP().toString();  
    value.string= &msg;   
    ESP_LOGI(TAG,"*********   value.string->cstr() = %s",value.string->c_str());
    text.setValue(value);
    tc(value);
    //stationIPProperty.setValue(value);
   //ESP_LOGI(TAG,"stationIPPropety.getValue()=%s",stationIdProperty.getValue().string->c_str());
}
//------------------------------------------------------------------
void setup(void)
{

    Serial.begin(115200);
    // WiFi.onEvent(WiFiStationConnected,SYSTEM_EVENT_STA_CONNECTED);
    WiFi.onEvent(WifiEvent, SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(WifiEvent,SYSTEM_EVENT_WIFI_READY);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    ESP_LOGI(TAG, "Connecting to %s \n", ssid);
    //--- RTC wakeup on gpio 33 (wired to button)
    esp_sleep_enable_ext1_wakeup(BUTTON_WAKEUP_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);

    setMacId();
    initMAX7219(); // (1) --- init MAX7219-------------------------------
    initAHT();     // (2)----- init AHT----------------------------------
    //-------------hook the handler
    // WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);

    initWifi();                 // (3)------init WiFi---------------------------------
   
    initAdapterAndAddDevices(); // (4) -- intiAdapterAndAddDevices()------------------
     //RegisterIP();
     output = getAdapterJson();  // (6) --All devices in the adapter, Json form--------

    deviceAHT10.addEvent(&eventAHT10);
    mqtt.setCallback(mqttCallback);
    mqtt.setServer(mqttIp, 1883);
    //-----------------------Setup area for WIFI event test---------------------
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    // do not use event use the method RegisterIp , it is premature to register IP yet
    //   WiFi.onEvent(WifiGotIP, SYSTEM_EVENT_STA_GOT_IP);
    //--------------------------------------------------------------------------

    String topic = "IOT/RegisterDevices";
    ESP_LOGI(TAG, "Publishsing -> %s\n \t %s ", topic.c_str(), output.c_str());
    while (!mqtt.connected())
        mqttReConnect();
    mqtt.publish(topic.c_str(), output.c_str());
    ESP_LOGI(TAG, "\nout=\n%s\n\t\t len=%i", output.c_str(), output.length());
    

    // stationIPProperty.setValue(v);
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
        AHT10HumidityProperty.setValue(toPvalueNumber(humidity.relative_humidity));
        AHT10TemperatureProperty.setValue(toPvalueNumber(temperature.temperature));
        ESP_LOGI(TAG, "%05d Hum=%.2lf%% dbid=%i:Temp=%.2lf dbid=%i   Heap=%d,%d", ++i, humidity.relative_humidity, AHT10HumidityProperty.propertyDbId, temperature.temperature, AHT10TemperatureProperty.propertyDbId, ESP.getFreeHeap(), deviceAHT10.ws->count());
        if (AHT10HumidityProperty.propertyDbId != -1)
        {
            String eData = getPropertiesJson(&deviceAHT10);
            mqtt.publish("IOT/DeviceEvent", eData.c_str());
        }
       // deviceAHT10.ws->cleanupClients();
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
    //mqttReConnect();
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
    delay(duration);

    ThingDataValue value = {.integer = brightness};
    lampLevel.setValue(value);
    int level = (int)Arduino_h::map(brightness, 0, 100, 255, 0);
    ESP_LOGD(TAG, "value =%i , level(mapped)=%d ", (int)value.integer, level);
    analogWrite(lampPin, level, 255);
    lc.clearDisplay(0);
    lc.printF((float)brightness, (char *)"%.2f");
    ThingDataValue val;
    val.number=102;
    ThingEventObject *ev = new ThingEventObject("overheated", NUMBER, val);
    ESP_LOGI(TAG, " Queu event(overheated) %2.2f", val.number);
    lamp.queueEventObject(ev);
}
ThingActionObject *action_generator(DynamicJsonDocument *input)
{

    String output;
    serializeJson(*input, output);
    ESP_LOGD(TAG, "Serialized(*input) \n\t\toutput=%s\n", output.c_str());

    return new ThingActionObject("fade", input, do_fade, nullptr);
}
