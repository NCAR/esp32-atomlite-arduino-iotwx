#include <NTPClient.h>      /// https://github.com/arduino-libraries/NTPClient
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MQTT.h>           /// https://github.com/256dpi/arduino-mqtt
#include <FastLED.h>        ///     
#include "IoTwx.h"


MQTTClient client;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
WiFiClient net;
unsigned long retryCount = 0;
unsigned long t;

// Define the array of leds
CRGB leds[NUM_LEDS];


void blink_led(CRGB color, int speed){
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


IoTwx::IoTwx(const char* dev_id, const char* ssid, const char* pwd, const char* mqtt_ip, int mqtt_p, int tz) {
    wifi_ssid = ssid;
    wifi_passwd = pwd;
    mqtt_server = mqtt_ip;
    mqtt_port = mqtt_p;
    device_id = dev_id;
    timezone = tz;
}


void IoTwx::mqtt_publish_measurement(const char* topic, const char* sensor, float m, long offset) {
  char data[128] = {0};

  t = timeClient.getEpochTime(); // sec since 1970
    
  Serial.print("[info]: publishing = ");

  sprintf(data,
    "device: %s\nsensor: %s/precipitation\nm: %f\nt: %d\n",
     device_id, sensor, m, t - offset);
  Serial.print(data);
  client.publish(topic, data);
}


void IoTwx::establish_communications() {
  // connect to WiFi
  retryCount = 0; 
  WiFi.begin(wifi_ssid, wifi_passwd);
  Serial.print("[info]: checking wifi..."); Serial.print(wifi_ssid);
       
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    blink_led(LED_WIFI, LED_MED);
    delay(7500);
    
    retryCount++;
    if (retryCount > 10) { 
      blink_led(LED_FAIL, LED_MED); blink_led(LED_WARN, LED_MED); blink_led(LED_OK, LED_MED);
      delay(30000);
      esp_restart();
    }
  }

  // connect to MQTT
  retryCount = 0;
  client.begin(mqtt_server, mqtt_port, net);
  Serial.print("\n[info]: MQTT connecting..."); 
  Serial.print(mqtt_server); Serial.print(mqtt_port);
  while (!client.connect("arduino", "", "")) {
    Serial.print(".");
    blink_led(LED_MQTT, LED_MED);
    delay(5000);
    
    retryCount++;
    if (retryCount > 10) { 
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
