#pragma once
#ifndef IOTWX_CONFIG_H
#define IOTWX_CONFIG_H

#define NUM_LEDS 1
#define NEO_PIN 27
#define LED_WARN CRGB::Yellow
#define LED_FAIL CRGB::Red
#define LED_OK CRGB::Green
#define LED_BLACK CRGB::Black
#define LED_WIFI CRGB::Blue
#define LED_MQTT CRGB::Purple
#define LED_SLOW 1
#define LED_MED 3
#define LED_FAST 5

class IoTwx {
    const char* wifi_ssid;
    const char* wifi_passwd;
    const char* mqtt_server;
    int mqtt_port;
    int timezone;
    const char* device_id;

    public:
        IoTwx(const char* dev_id, const char* ssid, const char* pwd, const char* mqtt_ip, 
              int mqtt_port, int timezone);
 s       void    mqtt_publish_measurement(const char* topic, const char* sensor, float m, long offset);
        void    establish_communications(); 
};
#endif
