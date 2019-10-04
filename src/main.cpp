#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

#include <MqttUtil.h>
#include <PubSubClient.h>

#include <Wire.h>
#include <Adafruit_INA219.h>
#include "SSD1306.h"

// Generic stuff
const char* PACKAGE_NAME = "esp8266_power_meter";
const char* FW_NAME = NAME;
const int FW_VERSION = VERSION;
const char* FW_URL_BASE = FOTA_URL;

#define FIRMWARE_CHECK_INTERVAL_IN_SEC 600
#define WIFI_CONNECT_ATTEMPTS 100

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

const char *mqtt_server = MQTT_SERVER;
const int   mqtt_port = MQTT_PORT;

const char* outTopic = MQTT_OUT_TOPIC;
const char* inTopic = MQTT_IN_TOPIC;

WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
bool wifiConnected = false;
long wifiDisconnectedPeriode, wifiDisconnectedPeriodeStart;

WiFiClient mqttWifiClient;
WiFiClient firmwareWifiClient;

PubSubClient client(mqttWifiClient);

long startUpTimeEsp = millis();
long lastRun = millis();
long lastFirmwareCheck = millis();

int nodemcuChipId = ESP.getChipId(); // returns the ESP8266 chip ID as a 32-bit integer.
long lowpulseoccupancy = 0;

long startUpTimeWifi = 0;
long connectTimeWifi = 0;
long startUpTimeMqtt = 0;
long connectTimeMqtt = 0;

MqttUtil mqttUtil = MqttUtil(nodemcuChipId, PACKAGE_NAME, ssid, inTopic, outTopic, false);
// /Generic stuff


Adafruit_INA219 ina219;

#define DISPLAY_ADDRESS 0x3C
#define DISPLAY_SDA_PIN D2
#define DISPLAY_SCL_PIN D1
SSD1306 display(DISPLAY_ADDRESS, DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);

void checkForUpdates() {
    String fwURL = String(FW_URL_BASE);

    String fwVersionURL = fwURL;
    fwVersionURL.concat("version/"); fwVersionURL.concat(nodemcuChipId);
    fwVersionURL.concat("?v="); fwVersionURL.concat(FW_VERSION);
    fwVersionURL.concat("&name="); fwVersionURL.concat(FW_NAME);
    fwVersionURL.concat("&package="); fwVersionURL.concat(PACKAGE_NAME);
    fwVersionURL.concat("&pi="); fwVersionURL.concat(PUBLISH_INTERVAL);
    fwVersionURL.concat("&wifi="); fwVersionURL.concat(WIFI_SSID);
    fwVersionURL.concat("&mqs="); fwVersionURL.concat(MQTT_SERVER);
    fwVersionURL.concat("&mqp="); fwVersionURL.concat(MQTT_PORT);
    fwVersionURL.concat("&mqout="); fwVersionURL.concat(MQTT_OUT_TOPIC);
    fwVersionURL.concat("&mqin="); fwVersionURL.concat(MQTT_IN_TOPIC);
    fwVersionURL.concat("&time="); fwVersionURL.concat(millis());

    Serial.println("Checking for firmware updates.");
    Serial.print("ChipId: ");
    Serial.println(nodemcuChipId);
    Serial.print("Firmware version URL: ");
    Serial.println(fwVersionURL);

    HTTPClient httpClient;
    if (httpClient.begin(firmwareWifiClient, fwVersionURL)) {
        int httpCode = httpClient.GET();
        if (httpCode == 200 || httpCode == 304) {
            String newFWVersion = httpClient.getString();

            Serial.print("Current firmware version: "); Serial.println(FW_VERSION);
            Serial.print("Available firmware version: "); Serial.println(newFWVersion);

            int newVersion = newFWVersion.toInt();

            if (newVersion > FW_VERSION) {
                Serial.println("Preparing to update");

                String fwImageURL = fwURL;
                fwImageURL.concat("firmware/"); fwImageURL.concat(nodemcuChipId);
                Serial.print("Firmware binary URL: "); Serial.println(fwImageURL);
                t_httpUpdate_return ret = ESPhttpUpdate.update(firmwareWifiClient, fwImageURL);

                switch(ret) {
                    case HTTP_UPDATE_FAILED:
                        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                        break;
                    case HTTP_UPDATE_NO_UPDATES:
                        Serial.println("HTTP_UPDATE_NO_UPDATES");
                        break;
                    case HTTP_UPDATE_OK:
                        Serial.println("HTTP_UPDATE_OK");
                        break;
                }
            } else {
                Serial.println("Already on latest version");
            }
        } else {
            Serial.print("Firmware version check failed, got HTTP response code ");
            Serial.println(httpCode);
        }
        httpClient.end();
    } else {
        Serial.println("HTTP Error: Unable to connect");
    }
    delay(10);
}

void setupWifi() {
    // Static IP
    // IPAddress ip( 192, 168, 0, 1 );
    // IPAddress gateway( 192, 168, 0, 254 );
    // IPAddress subnet( 255, 255, 255, 0 );
    startUpTimeWifi = millis();
    
    // WiFi.forceSleepEnd();
    WiFi.forceSleepWake();
    delay(10);

    Serial.print("WiFi Connecting to "); Serial.println(ssid);

    // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
    WiFi.persistent(false);
    delay(10);

    // Bring up the WiFi connection
    WiFi.mode(WIFI_STA);
    delay(10);

    // Static IP
    // WiFi.config( ip, gateway, subnet );

    int wifi_attempts = 0;
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
        wifi_attempts++;
        if (wifi_attempts > WIFI_CONNECT_ATTEMPTS) {
            Serial.println("Can't connect to the wifi. Retrying soon.");
            delay(10000);
        }
    }
    randomSeed(micros());
    Serial.println(""); Serial.print("WiFi connected with IP: "); Serial.println(WiFi.localIP());
    connectTimeWifi = millis();
}

void shutdownWifi() {
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();

    // One of the issues at this point though, is that the WiFi can take a finite, but variable 
    // time to shut down after the final forceSleepBegin() call, so at this point I’ve just 
    // added a timeout loop, checking the status of the connection until it goes down (returning 
    // without an error) or times out (returning with an error):-
    int wifi_attempts = 0;
    while ((WiFi.status() == WL_CONNECTED) && (wifi_attempts++ < WIFI_CONNECT_ATTEMPTS)) {
        delay(100);
        Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(""); Serial.print("WiFi is now offline.");
    } else {
        Serial.println(""); Serial.print("Not able to shutdown WiFi...");
    }
    delay(100);
}

void callback(char* topic, byte* payload, int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    if ((char)payload[0] == '1') {
        // it is acive low on the ESP-01)
    } else {

    }
}

void reconnectMqtt(uint32 ipAddress, long wifiDisconnectedPeriode) {
    startUpTimeMqtt = millis();
    while (!client.connected()) {
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        Serial.print("Attempting MQTT connection: ["); Serial.print(clientId); Serial.print("] : ");
        if (client.connect(clientId.c_str())) {
            Serial.println("Connected to MQTT!");
            client.subscribe(inTopic);
            mqttUtil.sendControllerInfo(client, ipAddress, wifiDisconnectedPeriode);
        } else {
            Serial.print("failed, rc="); Serial.print(client.state()); Serial.println(" try again in 5 seconds...");
            delay(500);
        }
    }
    connectTimeMqtt = millis();
}

static void setupDisplay(void) {
    display.init();
    display.setFont(ArialMT_Plain_16);
    display.drawString(20, 20, "INA219 Power meter");
    display.display();
}

static void writeToScreen(const char * line1, const char * line2, const char * line3) {
    display.clear();
    display.drawString(0, 0, line1);
    display.drawString(0, 20, line2);
    display.drawString(0, 40, line3);
    display.display();
}

void setup() {
    shutdownWifi();

    Serial.begin(115200);
    gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) {
        wifiConnected = true;
        wifiDisconnectedPeriode = millis() - wifiDisconnectedPeriodeStart;
        Serial.print("WiFi connected, IP: "); Serial.println(WiFi.localIP());
    });
    disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event) {
        wifiConnected = false;
        wifiDisconnectedPeriodeStart = millis();
        Serial.println("WiFi disconnected...");
    });

    ina219.begin();
    ina219.setCalibration_16V_400mA();

    setupDisplay();
}

void scanI2Cdevices() {
    byte error, address;
    int nDevices;
    Serial.println("Scanning...");
    nDevices = 0;
    for(address = 1; address < 127; address++ ) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
    
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address<16) {
                Serial.print("0");
            }
            Serial.print(address,HEX);
            Serial.println("  !");
            nDevices++;
        } else if (error==4) {
            Serial.print("Unknown error at address 0x");
            if (address<16) {
                Serial.print("0");
            }
            Serial.println(address,HEX);
        }    
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.println("done\n");
    }
}

void loop() {
    delay(10); // Allow internal stuff to be executed.
    long now = millis();
    // scanI2Cdevices();

    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;

    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
  
    // Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    // Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    // Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    // Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    // Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    // Serial.println("");

    char line1[32];
    char line2[32];
    char line3[32];
    sprintf(line1, "Load: %g V", loadvoltage);
    sprintf(line2, "Current: %g mA", current_mA);
    sprintf(line3, "Power: %g mW", power_mW);

    writeToScreen(line1, line2, line3);

    // writeIdToScreen("test foo bar gomle");
    if (!wifiConnected) {
        setupWifi();
        delay(10);
    }
    if (!client.connected()) {
        client.setServer(mqtt_server, mqtt_port);
        client.setCallback(callback);
        reconnectMqtt(WiFi.localIP(), wifiDisconnectedPeriode);
    }
    client.loop();

    mqttUtil.publishKeyValueFloat(client, "busvoltage", busvoltage, 5, 3);
    mqttUtil.publishKeyValueFloat(client, "shuntvoltage", shuntvoltage, 5, 3);
    mqttUtil.publishKeyValueFloat(client, "loadvoltage", loadvoltage, 5, 3);
    mqttUtil.publishKeyValueFloat(client, "current_mA", current_mA, 5, 3);
    mqttUtil.publishKeyValueFloat(client, "power_mW", power_mW, 5, 3);


    delay(PUBLISH_INTERVAL * 1000);

    if (now - lastFirmwareCheck > (FIRMWARE_CHECK_INTERVAL_IN_SEC * 1000)) {
        mqttUtil.publishKeyValueInt(client, "FW_VERSION", FW_VERSION);
        Serial.println("No deepsleep, so we need to check for firmare updates now and then.");
        Serial.print(FIRMWARE_CHECK_INTERVAL_IN_SEC); Serial.println(" seconds between every check.");
        checkForUpdates();
        lastFirmwareCheck = now;
    }
}
