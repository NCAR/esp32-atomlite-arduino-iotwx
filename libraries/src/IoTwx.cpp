/*
    IoTwx.cpp

    This is the core implementation of the IoTwx
    class which implements communication and 
    basic functionality for the node including
    configuration settings and indiciator lights
    or messages on the physical device.
    
    Copyright (c) 2020 keith maull
    Website    : 
    Author     : kmaull
    Create Time:
    Change Log :
*/

#include "IoTwx.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "BluetoothSerial.h" // Serial Bluetooth; see tutorial on: www.circuitdigest.com 
#include "SPIFFS.h" 
#include <NTPClient.h>      /// https://github.com/arduino-libraries/NTPClient
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MQTT.h>           /// https://github.com/256dpi/arduino-mqtt
#include <ArduinoJson.h>


MQTTClient       mqttClient;
WiFiUDP          ntpUDP;
WiFiClient       networkClient;
BluetoothSerial  btSerialConnection;      
NTPClient        timeClient(ntpUDP);
unsigned long    t;
typedef uint32_t nvs_handle_t;
unsigned long    retry_count    = 0;
CRGB             leds[NUM_LEDS] = {LED_BLACK}; // Define the array of leds
bool             use_led        = false;


void init_led(){
    // set all the LEDs to black
    FastLED.clear();
    FastLED.show();
    
    FastLED.addLeds<NEOPIXEL, NEO_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
    use_led = true;
}    


void blink_led(CRGB color, int speed){
    if (use_led) {
        leds[0] = color;
        
        for (int i = 0; i < speed ; i++)
        {
            FastLED.setBrightness(1);
            leds[0] = color;
            FastLED.show();
            delay(125);

            // Now turn the LED off, then pause
            leds[0] = LED_BLACK;
            FastLED.show();
            delay(250);
        }
    }
}


/**
 * 
 * This function waits for delay_in_sec seconds for a 
 * BT serial connection. If there is a connection, it will
 * accept the input as a valid JSON file and use that
 * file for the configuration of the device.  
 * 
 * See === for the details of what needs to go into
 * the config file.
 * 
 * The code returns true if there is proper configuration 
 * file uploaded and successfully stored in NVS memory, 
 * otherwise false
 * 
 */
bool wait_for_bluetooth_config(const char* uuid, long last_millis, int delay_in_sec)
{
    File                      file;
    StaticJsonDocument<1024>  doc;
    char                      jsonConfig[1024]  = {'/0'};

    btSerialConnection.begin(uuid); 
    SPIFFS.begin(true);

    Serial.println("[] Bluetooth Device is Ready to Pair");
    Serial.println("[] BT initialization: 90s");
    blink_led(LED_BT, LED_FAST);
    
    while (millis() - last_millis < delay_in_sec*1000) {
        if (btSerialConnection.available()) //Check if we receive anything from Bluetooth
        {
            for(int i=0; btSerialConnection.available(); i++) {
                jsonConfig[i] = char(btSerialConnection.read());
                Serial.print(jsonConfig[i]);
            }
            Serial.println("[] file uploaded from BT");
            delay(750);
            
            // store JSON file for future retrieval
            Serial.println("[] trying to store file to SPIFFS");
            file = SPIFFS.open("/config.json", FILE_WRITE);

            if(!file) {
                Serial.println("[] file error");
                return false;
            }
            else {
                Serial.println("[] file open for writing");
                
                if (file.print(jsonConfig)) {          
                    // save the file
                    file.close();
                    Serial.println("[] SUCCESS JSON config file stored to SPIFFS");
                    Serial.println(jsonConfig);
                    
                    // reopen the file from SPIFFS, store essential data to NVS
                    file = SPIFFS.open("/config.json", FILE_READ);
                    deserializeJson(doc, file);
                    file.close();

                    String output;
                    serializeJson(doc, output);
                    Serial.println("[] reading serialized json");

                    if (output == "null") {
                        Serial.println("[] config.json was empty: HALTING");
                        return false;
                    } else {
                        Serial.println("[] BT config storing to NVS");
                        store_data_to_nvs("iotwx_mq_port",   (const char *)doc["iotwx_mq_port"]);
                        store_data_to_nvs("iotwx_mq_ip",     (const char *)doc["iotwx_mq_ip"]);
                        store_data_to_nvs("iotwx_wifi_ssid", (const char *)doc["iotwx_wifi_ssid"]);
                        store_data_to_nvs("iotwx_wifi_pwd",  (const char *)doc["iotwx_wifi_pwd"]);
                        store_data_to_nvs("iotwx_id",        (const char *)doc["iotwx_id"]);
                        Serial.println("[] BT config stored to NVS");
                        return true;
                    }
                }
            }
        }
        delay(20);
    }
    return false;
}

/**
 * 
 * This function reads data from NVS memory given
 * the key as input.  
 * 
 * returns NULL if there is a problem, and an empty array
 * if the key was not found, otherwise it returns the value 
 * of the key.
 * 
 */
char* read_data_from_nvs(char* key) {
  Serial.print("\n");
  Serial.print("Opening Non-Volatile Storage (NVS) handle... ");
  
  esp_err_t    err = nvs_flash_init();
  size_t       required_size;
  nvs_handle_t n_handle;

  // set up storage
  err = nvs_open("storage", NVS_READWRITE, &n_handle);
  
  if (err != ESP_OK) {
      Serial.print("Error opening NVS handle: "); Serial.print(esp_err_to_name(err));
      Serial.print("\n");

      return nullptr;
  } else {
      Serial.print("Done"); 
      Serial.print("Reading data from NVS ... ");
      
      if (nvs_get_str(n_handle, key, NULL, &required_size) == ESP_OK) {
        char* val = (char *)malloc(required_size);        
        
        switch (nvs_get_str(n_handle, key, val, &required_size)) {
            case ESP_OK:
                Serial.print("Done\n");
                Serial.print("key"); Serial.print(" = "); Serial.print(val);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                Serial.print("The value is not initialized yet!\n");
                break;
            default :
                Serial.print("Error reading: ");Serial.print(esp_err_to_name(err));
                Serial.println();
        }
        return val;        
      }
  }
}


/**
 * 
 * This function stores data into the NVS memory
 * of the ESP32.  It takes a key and value 
 * as input and returns true if successful, 
 * otherwise false.
 * 
 */
bool store_data_to_nvs(char* key, const char* v) {
  esp_err_t    err = nvs_flash_init();
  nvs_handle_t n_handle;

  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // TODO: bad things have just happened -- halt!
    // NVS partition was truncated and needs to be erased
    //      // Retry nvs_flash_init
    //      ESP_ERROR_CHECK(nvs_flash_erase());
    //      err = nvs_flash_init();
    return false;
  } else {
    ESP_ERROR_CHECK( err );

    Serial.print("\n");
    Serial.print("Opening Non-Volatile Storage (NVS) handle... ");

    err = nvs_open("storage", NVS_READWRITE, &n_handle);

    if (err != ESP_OK) {
        Serial.print("Error opening NVS handle:"); Serial.print(esp_err_to_name(err));
        return false;
    } else {
        Serial.print("Done\n");
  
        err = nvs_set_str(n_handle, key, v);
        Serial.print((err != ESP_OK) ? "Failed!\n" : "Done\n");
      
        // After setting any values, nvs_commit() must be called! 
        Serial.println("Committing updates in NVS ... ");
        err = nvs_commit(n_handle);
        Serial.print((err != ESP_OK) ? "Failed!\n" : "Done\n");
      
        // Close
        nvs_close(n_handle);
        
        return true;
    }
  }
}


IoTwx::IoTwx() {/* empty constructor */}


IoTwx::IoTwx(bool bt_config_status) {
    Serial.println("[] reading from NVS");
    Serial.println("[] setting IoTwx data");
    
    if (bt_config_status == true) {
        wifi_ssid   = (const char*)read_data_from_nvs("iotwx_wifi_ssid");
        wifi_passwd = (const char*)read_data_from_nvs("iotwx_wifi_pwd");
        mqtt_server = (const char*)read_data_from_nvs("iotwx_mq_ip");
        mqtt_port   = atoi((const char*)read_data_from_nvs("iotwx_mq_port"));
        device_id   = (const char*)read_data_from_nvs("iotwx_id");
    }
    configured = bt_config_status;          
}


IoTwx::IoTwx(const char* dev_id, const char* ssid, 
             const char* pwd, const char* mqtt_ip, 
             int mqtt_p, int tz) {
    wifi_ssid = ssid;
    wifi_passwd = pwd;
    mqtt_server = mqtt_ip;
    mqtt_port = mqtt_p;
    device_id = dev_id;
    timezone = tz;
}

/**
 * 
 * This function publishes a measurement to the given
 * IoTwx object.
 * 
 * @topic  the MQTT topic (e.g. "measurements/iotwx")
 * @sensor the sensor (e.g. "grove/i2c/bme680/pressure
 * @measurements are assumed to be floats
 * @offset is the amount of time in seconds to be subtracted
 * from the time the measurement is published to the time
 * it was measured, use an offset of 0 if the timestamp
 * of the measurement is intended to be time time it is 
 * published.
 * 
 */
void IoTwx::publishMQTTMeasurement(const char* topic, const char* sensor, float m, long offset) {
  char data[128] = {0};

  t = timeClient.getEpochTime(); // sec since 1970
    
  Serial.print("[info]: publishing = ");

  sprintf(data,
    "device: %s\nsensor: %s\nm: %f\nt: %d\n",
     device_id, sensor, m, t - offset);
  
  Serial.print(data);
  mqttClient.publish(topic, data);
  delay(750);
}


void IoTwx::establishCommunications() {
  // connect to WiFi
  retry_count = 0; 
  WiFi.begin(wifi_ssid, wifi_passwd);
  Serial.print("[info]: checking wifi..."); Serial.print(wifi_ssid); Serial.print(wifi_passwd);
       
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    blink_led(LED_WIFI, LED_MED);
    delay(7500);
    
    retry_count++;
    if (retry_count > 10) { 
      blink_led(LED_FAIL, LED_MED); blink_led(LED_WARN, LED_MED); blink_led(LED_OK, LED_MED);
      delay(30000);
      esp_restart();
    }
  }

  // connect to MQTT
  retry_count = 0;
  mqttClient.begin(mqtt_server, mqtt_port, networkClient);
  Serial.print("\n[info]: MQTT connecting..."); 
  Serial.print(mqtt_server); Serial.print(mqtt_port);
  while (!mqttClient.connect("esp32", "", "")) {
    Serial.print(".");
    blink_led(LED_MQTT, LED_MED);
    delay(5000);
    
    retry_count++;
    if (retry_count > 10) { 
      blink_led(LED_FAIL, LED_MED); blink_led(LED_WARN, LED_MED); blink_led(LED_OK, LED_MED);
      delay(30000);
      esp_restart();
    }
  }

  // start NTP client for clock and time
  timeClient.begin();
  timeClient.setTimeOffset(timezone);

  while(!timeClient.update()) {
    timeClient.forceUpdate();
    Serial.println("[info]: updating time client");
    delay(2000);
  }
  
  Serial.println("\n[info]: connection sequence done connecting!");

  blink_led(LED_OK, LED_SLOW);
}
