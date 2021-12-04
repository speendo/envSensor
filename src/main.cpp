// GLOBAL VARIABLES

#define DEVICE_NAME "envSensor"

#define WIFI_PASSWORD "noMoreMo7d"

#define UPDATE_FREQUENCY 5000

// short this pin with 3,3V reenable the login portal.
#define APPIN 15 // 15 is labeled with D8

#define MQTT_BUFFER_SIZE 512

// You only need to format the filesystem once
#define FORMAT_FILESYSTEM false

// END GLOBAL VARIABLES

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266Ping.h> // only for debug reasons

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

#include <LittleFS.h>
#include <ArduinoJson.h>

#include <PubSubClient.h>


StaticJsonDocument <MQTT_BUFFER_SIZE> payLoadJson;
char buffer[MQTT_BUFFER_SIZE];

#include <WEMOS_SHT3X.h>
#include <Adafruit_SGP30.h>
#include <Wire.h>

SHT3X sht30(0x45);
Adafruit_SGP30 sgp30;

#define TEMPNAME "temperature"
#define HUMNAME "relHumidity"
#define ABSHUMNAME "absHumidity"
#define DEWPOINTNAME "dewPoint"
#define TVOCNAME "tvoc"
#define ECO2NAME "eCO2"
#define RAWH2NAME "rawH2"
#define RAWETHANOLNAME "rawEthanol"
#define ECO2BASENAME "eCO2_base"
#define TVOCBASENAME "tvoc_base"

float temp;
float hum;
uint32_t absHumSGP30;
uint16_t TVOC;
uint16_t eCO2;
uint16_t rawH2;
uint16_t rawEthanol;
uint16_t eCO2_base;
uint16_t TVOC_base;

char mqtt_server[40];
char mqtt_port[6];
char mqtt_user[20];
char mqtt_password[20];

unsigned long prevTime;

uint32_t getAbsoluteHumiditySGP30(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.16
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

float getCorrectAbsoluteHumidity(float temperature, float humidity)
{
  return humidity * 6.112 * 2.1674 * exp((temperature * 17.67) / (temperature + 243.5)) / (temperature + 273.15);
}

float getDewPoint(float temperature, float humidity)
{
  const float gamma = log((humidity / 100) * exp((18.678 - (temperature / 234.5)) * (temperature / (257.14 + temperature))));
  return (257.14 * gamma) / (18.678 - gamma); 
}

String convertToHexRep(uint16_t input)
{
  String output = "0x";
  output.concat(String(input, HEX));
  return output;
}

WiFiClient wifiClient;
//WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

bool loadFileFSConfigFile(void)
{
  //clean FS
  if (FORMAT_FILESYSTEM)
  {
    Serial.println("Formatting FS");
    LittleFS.format();
  }

  //read configuration from FS json

  if (LittleFS.begin())
  {
    Serial1.println("Mounting File System");
    if (LittleFS.exists("/config.json"))
    {
      Serial.println("File " + String("/config.json") + " exists. Loading file.");

      //file exists, reading and loading
      File configFile = LittleFS.open("/config.json", "r");

      if (configFile)
      {
        Serial.println("Opening file");
        size_t configFileSize = configFile.size();

        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[configFileSize + 1]);

        configFile.readBytes(buf.get(), configFileSize);

        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get(), configFileSize);

        if (deserializeError)
        {
          Serial.println("Error deserializing config JSON.");
          return false;
        }
        else
        {
          if (json["mqtt_server"])
          {
            Serial.println("Copying mqtt_server from config file.");
            strncpy(mqtt_server, json["mqtt_server"], sizeof(mqtt_server));
          }
          if (json["mqtt_port"])
          {
            Serial.println("Copying mqtt_port from config file.");
            strncpy(mqtt_port, json["mqtt_port"], sizeof(mqtt_port));
          }
          if (json["mqtt_user"])
          {
            Serial.println("Copying mqtt_user from config file.");
            strncpy(mqtt_user, json["mqtt_user"], sizeof(mqtt_user));
          }
          if (json["mqtt_password"])
          {
            Serial.println("Copying mqtt_password from config file.");
            strncpy(mqtt_password, json["mqtt_password"], sizeof(mqtt_password));
          }
        }

        serializeJsonPretty(json, Serial);
        configFile.close();
      }
    }
    else
    {
      Serial.println("Failed to load /config.json.");
    }
  }
  else
  {
    Serial.println("No file System yet.");
    return false;
  }
  return true;
}

void saveFileFSConfigFile(void)
{
  Serial.println("saving config file");
  DynamicJsonDocument json(1024);

  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_password"] = mqtt_password;

  File configFile = LittleFS.open("/config.json", "w");

  if (!configFile)
  {
    Serial.println("Failed to open config file for writing.");
  }

  serializeJsonPretty(json, Serial);
  // Write data to file and close it
  serializeJson(json, configFile);

  configFile.close();
  //end save
}

bool saveConfigFlag = false;

void saveConfigCallback()
{
  Serial.println("Should save config");
  saveConfigFlag = true;
}

void initWiFi()
{
  loadFileFSConfigFile();

  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_password("password", "mqtt password", mqtt_password, 20);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_password);
  //  wifiManager.setConnectTimeout(60);

  wifiManager.autoConnect(DEVICE_NAME, WIFI_PASSWORD);

  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());

  if (saveConfigFlag)
  {
    saveFileFSConfigFile();
  }
}

bool connectMqtt()
{
  if (mqtt_user[0] == 0) // no user, no password
  {
//    Serial.println("Connecting to " + String(mqtt_server) + ":" + String(mqtt_port) + ", Client Name: >>" + String(DEVICE_NAME) + "<<");
    mqttClient.connect(DEVICE_NAME);
  }
  else
  {
//    Serial.println("Connecting to " + String(mqtt_server) + ":" + String(mqtt_port) + ", Client Name: >>" + String(DEVICE_NAME) + "<<, UN: >>" + String(mqtt_user) + "<<, PW: >>" + String(mqtt_password) + "<<");
    mqttClient.connect(DEVICE_NAME, mqtt_user, mqtt_password);
  }
  if (mqttClient.connected())
  {
//    Serial.println("Connection successful!, MQTT Client State: " + String(mqttClient.state()));
    return true;
  }
  else
  {
    Serial.println("Connection failed! MQTT Client State: " + String(mqttClient.state()));
    return false;
  }
}

String genSendTopic()
{
  return String("homeassistant/sensor/" + String(DEVICE_NAME));
}

String genConfigTopic(String abr)
{
  return genSendTopic() + abr + "/config";
}

String genStateTopic()
{
  return genSendTopic() + "/state";
}

char * genSetupPayload(String name, String unit, String topic)
{
  payLoadJson.clear();
  payLoadJson["expire_after"] = 60;
  payLoadJson["name"] = name;
  payLoadJson["state_topic"] = genStateTopic();
  payLoadJson["unit_of_measurement"] = unit;
  payLoadJson["value_template"] = "{{ value_json." + topic + " }}";
  
  buffer[0] = 0;
  serializeJson(payLoadJson, buffer);
  return buffer;
}

char * genSetupPayload(String devClass, String name, String unit, String topic)
{
  payLoadJson.clear();
  payLoadJson["device_class"] = devClass;
  payLoadJson["expire_after"] = 60;
  payLoadJson["name"] = name;
  payLoadJson["state_topic"] = genStateTopic();
  payLoadJson["unit_of_measurement"] = unit;
  payLoadJson["value_template"] = "{{ value_json." + topic + " }}";

  buffer[0] = 0;
  serializeJson(payLoadJson, buffer);
  return buffer;
}

bool setupMqtt()
{
  if (connectMqtt())
  {
    mqttClient.publish(genConfigTopic(TEMPNAME).c_str(), genSetupPayload("temperature", "Temperature", "°C", TEMPNAME), true);
    mqttClient.publish(genConfigTopic(HUMNAME).c_str(), genSetupPayload("humidity", "Relative Humidity", "%", HUMNAME), true);
    mqttClient.publish(genConfigTopic(ABSHUMNAME).c_str(), genSetupPayload("humidity", "Absolute Humidity", "g/m3", ABSHUMNAME), true);
    mqttClient.publish(genConfigTopic(DEWPOINTNAME).c_str(), genSetupPayload("humidity", "Dew Point", "°C", DEWPOINTNAME), true);
    mqttClient.publish(genConfigTopic(TVOCNAME).c_str(), genSetupPayload("humidity", "TVOC", "ppb", TVOCNAME), true);
    mqttClient.publish(genConfigTopic(ECO2NAME).c_str(), genSetupPayload("humidity", "eCO2", "ppm", ECO2NAME), true);
    mqttClient.publish(genConfigTopic(RAWH2NAME).c_str(), genSetupPayload("humidity", "Raw H2", "ppm", RAWH2NAME), true);
    mqttClient.publish(genConfigTopic(RAWETHANOLNAME).c_str(), genSetupPayload("humidity", "Raw Ethanol", "ppm", RAWETHANOLNAME), true);
    mqttClient.publish(genConfigTopic(ECO2BASENAME).c_str(), genSetupPayload("eCO2 Base", "", ECO2BASENAME), true);
    mqttClient.publish(genConfigTopic(TVOCBASENAME).c_str(), genSetupPayload("TVOC Base", "", TVOCBASENAME), true);

    return true;
  }
  else
  {
    Serial.println("!!! Error connecting MQTT Client. Client State " + String(mqttClient.state()) + "!!!");
    return false;
  }
}

bool sendMqtt()
{
  if (connectMqtt())
  {
    payLoadJson.clear();
    payLoadJson[TEMPNAME] = temp;
    payLoadJson[HUMNAME] = hum;
    payLoadJson[ABSHUMNAME] = getCorrectAbsoluteHumidity(temp, hum);
    payLoadJson[DEWPOINTNAME] = getDewPoint(temp, hum);
    payLoadJson[TVOCNAME] = TVOC;
    payLoadJson[ECO2NAME] = eCO2;
    payLoadJson[RAWH2NAME] = rawH2;
    payLoadJson[RAWETHANOLNAME] = rawEthanol;
    payLoadJson[ECO2BASENAME] = convertToHexRep(eCO2_base);
    payLoadJson[TVOCBASENAME] = convertToHexRep(TVOC_base);

    Serial.println();
    Serial.println(genStateTopic());
    serializeJsonPretty(payLoadJson, Serial);

    buffer[0] = 0;
    serializeJson(payLoadJson, buffer);
    mqttClient.publish(genStateTopic().c_str(), buffer, true);
    return true;
  }
  else
  {
    Serial.println("!!! Error connecting MQTT Client. Client State " + String(mqttClient.state()) + "!!!");
    return false;
  }
}

void setup()
{

  Serial.begin(115200);
  pinMode(APPIN, INPUT);
  // Enable insecure ssl (any certificate is enabled, connection still encrypted)
  // wifiClient.setInsecure();

  // SHT30
  if (sht30.get() != 0)
  {
    Serial.println("SHT30 not found :(");
  }
  else
  {
    Serial.println("SHT30 should be up and running!");
  }

  // SGP30
  if (!sgp30.begin())
  {
    Serial.println("SGP30 not found :(");
  }
  else
  {
    Serial.print("Found SGP30 serial #");
    Serial.print(sgp30.serialnumber[0], HEX);
    Serial.print(sgp30.serialnumber[1], HEX);
    Serial.println(sgp30.serialnumber[2], HEX);
  }

  // init WiFi
  initWiFi();

  // ping server to debug problems
  if (Ping.ping(mqtt_server))
  {
    Serial.println("Server >>" + String(mqtt_server) + "<< found!");
  }
  else
  {
    Serial.println("Couldn't find " + String(mqtt_server) + "!");
  }

  // init MQTT
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttClient.setServer(mqtt_server, atoi(mqtt_port));
  if (setupMqtt())
  {
    Serial.println("Configuring MQTT.");
  }
  else
  {
    Serial.println("Failed to configure MQTT.");
  }

  // finally init timer
  prevTime = millis();
}

void loop()
{
  if (digitalRead(APPIN))
  {
    Serial1.println("Erasing WiFi information and restarting");
    WiFi.disconnect(true);
    ESP.restart();
  }

  if (millis() >= prevTime + UPDATE_FREQUENCY)
  {
    prevTime = millis();

    // SHT30
    if (sht30.get() == 0)
    {
      temp = sht30.cTemp;
      hum = sht30.humidity;
      absHumSGP30 = getAbsoluteHumiditySGP30(temp, hum);
    }
    else
    {
      Serial.println("Error with SHT30!");
    }

    // SGP30
    // set absolute humidity
    sgp30.setHumidity(absHumSGP30);

    // get values
    if (sgp30.IAQmeasure())
    {
      eCO2 = sgp30.eCO2;
      TVOC = sgp30.TVOC;
    }
    else
    {
      Serial.println("Error with SGP30 IAQmeasure!");
      eCO2 = -1;
      TVOC = -1;
    }
    if (sgp30.IAQmeasureRaw())
    {
      rawH2 = sgp30.rawH2;
      rawEthanol = sgp30.rawEthanol;
    }
    else
    {
      Serial.println("Error with SGP30 IAQmeasureRaw!");
      rawH2 = -1;
      rawEthanol = -1;
    }
    if (sgp30.getIAQBaseline(&eCO2_base, &TVOC_base))
    {
      eCO2_base = eCO2_base;
      TVOC_base = TVOC_base;
    }
    else
    {
      eCO2_base = -1;
      TVOC_base = -1;
    }

    // Post it on MQTT
    if (!sendMqtt()) {
      if (setupMqtt()) {
        sendMqtt();
      }
    }

    // Debug Output
    /*
    Serial.println();
    Serial.println("==========");
    Serial.print("===Time: ");
    Serial.print(prevTime);
    Serial.println("===");
    Serial.println("==SHT30==");
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.print(" °C; Relative Humidity: ");
    Serial.print(hum);
    Serial.print(" %; Absolute Humidity: ");
    Serial.print(absHum);
    Serial.println(" g/m3");
    Serial.println();

    Serial.println("==SGP30==");
    Serial.print("TVOC: ");
    Serial.print(TVOC);
    Serial.print(" ppb; eCO2: ");
    Serial.print(eCO2);
    Serial.print(" ppm; rawH2: ");
    Serial.print(rawH2);
    Serial.print(" ppm; rawEthanol: ");
    Serial.print(rawEthanol);
    Serial.println(" ppm");
    if (!sgp30.getIAQBaseline(&eCO2_base, &TVOC_base))
    {
      Serial.println("!!! No Baseline Values !!!");
    }
    else
    {
      Serial.print("Baseline values: eCO2: 0x");
      Serial.print(convertToHexRep(eCO2_base));
      Serial.print("; TVOC: 0x");
      Serial.println(convertToHexRep(TVOC_base));
    }
    Serial.println("==========");
    Serial.println();
    */
  }
}