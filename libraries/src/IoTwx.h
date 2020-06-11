/*
    IoTwx.h

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
#pragma once
#ifndef IOTWX_CONFIG_H
#define IOTWX_CONFIG_H

#include <FastLED.h>        

#define NUM_LEDS  1
#define NEO_PIN   27
#define LED_WARN  CRGB::Yellow
#define LED_FAIL  CRGB::Red
#define LED_OK    CRGB::Green
#define LED_BLACK CRGB::Black
#define LED_BT    CRGB::Blue
#define LED_WIFI  CRGB::Blue
#define LED_MQTT  CRGB::Purple
#define LED_SLOW  1
#define LED_MED   3
#define LED_FAST  5

class IoTwx {
    const char* device_id;
    const char* wifi_ssid;
    const char* wifi_passwd;
    const char* mqtt_server;
    int         mqtt_port;
    int         timezone;
    bool        configured = false;
    
    public:
        IoTwx();
        IoTwx(bool flag);
        IoTwx(const char* dev_id, const char* ssid, const char* pwd, 
              const char* mqtt_ip, int mqtt_port, int timezone);
        void
            establishCommunications(); 
        void
            publishMQTTMeasurement(const char* topic, const char* sensor, float m, long offset);
        bool 
            isConfigured() { return configured; }
};

char* read_data_from_nvs (char* key);
bool  store_data_to_nvs  (char* key, const char* v);
bool  wait_for_bluetooth_config(const char* uuid, long last_millis, int delay);
void  init_led();
void  blink_led(CRGB color, int speed);

#endif
