/******************************************************************************
 * Basement Studio Monior - Version 0.2.0 - Matt Rude <matt@mattrude.com>
 * 
 * This program reads the temperature & humidity from a DHT11 sensor and send
 * the data via MQTT to a central server.
 * 
 * ****************************************************************************
 * 
 * MIT License
 * 
 * Copyright (c) 2020 Matt Rude
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 *****************************************************************************/

#include <Arduino.h>            // Include the Arduino Library
#include <SPI.h>                // Include SPI Support (To talk to the DH11 Sensor)
#include <WiFiNINA.h>           // Include WiFi Support
#include <WiFiUdp.h>            // Include UDP Support
#include <PubSubClient.h>       // Include MQTT Support
#include <Adafruit_Sensor.h>    // Adafruit Common sensor library for DHT sensor
#include <DHT.h>                // Include temperature and humidity sensor support
#include <stdlib.h>             // Include the C standard library
#include <avr/dtostrf.h>        // Include dtostrf floating points
#include <WDTZero.h>            // Include the Watch Dog Timer
#include "arduino_secrets.h"    // Include Serets File

// Enable the WatchDog Process
WDTZero MyWatchDoggy;           // Load MyWatchDoggy as the WDTZero pointer

// Enable the DHT11 Sensor
#define DHTPIN 4                // Set the DHT pin
#define DHTTYPE DHT11           // Set the DHT sensor to DHT11
DHT dht(DHTPIN, DHTTYPE);       // Load the sensor pin & type

// Enable WiFi
char ssid[] = SECRET_SSID;      // your network SSID (name)
char pass[] = SECRET_PASS;      // your network password (use for WPA, or use as key for WEP)
unsigned long interval = 120;   // interval in seconds between checks
int intervalTime = 2;           // Interval in minutes between time updates.

WiFiClient wifiClient;
PubSubClient client(wifiClient);

const char broker[]         = MQTT_SERVER;
int        port             = 1883;
const char mqttPath[]       = MQTT_PATH;
const char * mqttPathF      = "home/basement/studio/temperature";
const char * mqttPathH      = "home/basement/studio/humidity";

unsigned long previousMillis = 100000;

void reconnectMQTT() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, 
                "home/control/basement/studio/boot", 1, 1, "Offline")) {
            Serial.println("connected");
            delay(250);
            client.publish("home/control/basement/studio/boot", "Online");
        } else {
            Serial.print("Failed to connect to MQTT Server, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void reconnectWiFi() {
    int status = WL_IDLE_STATUS;     // the Wifi radio's status
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to reconnect WiFi to SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        delay(10000);
    }
}

int main(void) {
    Serial.begin(115200);
    
    Serial.println();
    Serial.println("Starting Basement Monitor");
    Serial.println();
    
    /**************************************************************************************/
    
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
        Serial.print(".");
        delay(500);      // Wait 0.5 seconds
    }
    Serial.println("You're connected to the network!");
    
    /**************************************************************************************/
    
    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(broker);
    client.setServer(broker, 1883);
    client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD,
        "home/control/basement/studio/boot", 1, 1, "Offline");
    
    delay(250);
    client.publish("home/control/basement/studio/boot", "Online");
    Serial.println("You're connected to the MQTT broker!");
    Serial.println();
    
    /**************************************************************************************/
    
    Serial.println("Starting DHT Sensor");
    dht.begin();
    
    Serial.println("Starting Watchdog Timmer");
    MyWatchDoggy.setup(WDT_HARDCYCLE8S);

    /**************************************************************************************/

    while(1) {

        unsigned long currentMillis = millis();

        /**********************************************************************************/

        if (currentMillis - previousMillis >= interval * 1000) {


            if (!client.connected()) {
                reconnectMQTT();
            }

            Serial.println("Starting Temperature Query");
            digitalWrite(25, HIGH);
            const char * sensorCount = 0;

            float h = dht.readHumidity();
            float t = dht.readTemperature();
            float f = dht.readTemperature(true);

            if (isnan(h) || isnan(t) || isnan(f)) {
                Serial.println(F("Failed to read from DHT sensor!"));
                return 0;
            }

            char resultF[0];
            char resultH[0];
            int resultFL = (int) f;
            int resultHL = (int) h;
            if (f >= 0) {
                dtostrf(resultFL, 2, 0, resultF);
            } else {
                dtostrf(resultFL, 3, 0, resultF);
            }
            dtostrf(resultHL, 2, 0, resultH);

            Serial.println("Sending temperature (" + String(resultF) + ")");
            Serial.println("Sending humidity (" + String(resultH) + ")");
            client.publish(mqttPathF, resultF);
            client.publish(mqttPathH, resultH);

            sensorCount++;
            Serial.println("Will check again in: " + String(interval) + " seconds.");
            previousMillis = currentMillis;
            Serial.println();
        }

        /**********************************************************************************/

        client.loop();
        MyWatchDoggy.clear();  // refresh WDT - before finishing the loop
    }
}
