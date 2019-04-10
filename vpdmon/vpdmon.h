/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 * 
 * (C) C Stott 2018
 * 
 ****************************************************/
#ifndef INCL_VPDMON_H
#define INCL_VPDMON_H

#include <stdio.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>
#include <avr/wdt.h>
#include <BME280I2C.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>

#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
extern SoftwareSerial Serial1(6, 7); // RX, TX
#endif

#define setTextScale setTextSize
#if 0
#define debug_msg(x) Serial.println(x)
#define debug_msg_pre(x) Serial.print(x)
#define debug_msg_pre_h(x) Serial.print(x,HEX)
#else
#define debug_msg(x)
#define debug_msg_pre(x)
#define debug_msg_pre_h(x)
#endif
#define smsg(x) Serial.println(x)
#define smsg_pre(x) Serial.print(x)
#define smsg_pre_h(x) Serial.print(x,HEX)
#define lmsg(x) display.println(x)
#define lmsg_pre(x) display.print(x)
#define lmsg_pre_h(x) display.print(x,HEX)
#define msg(x) smsg(x);lmsg(x)
#define msg_pre(x) smsg_pre(x);lmsg_pre(x)
#define msg_pre_h(x) smsg_pre_h(x,HEX);lmsg_pre_h(x,HEX)

#define GROWMINPERIOD     50400
#define TIMEZONE          -1
#define LOCATIONS         2
#define AC_PIN          A0 // aircon
#define CT_PIN          A3 // cooltube
#define HT_PIN          A2 // heater
#define FN_PIN          A1 // fan (extractor)
#define WATCHDOG true

extern int DELAY_CC;
extern float arraySensors[2][LOCATIONS][2]; // old/new, in/out, t/h
extern float minMaxClimates[2][2][2][2]; // [G/F][Min/Max][D/N][T/H]);
extern time_t delayCC;
extern bool testMode;
extern bool climateChangePending;
extern bool isAcOn, isCtOn, isHeatOn, isFanOn;
extern bool wasFanOn, wasCtOn, wasHeatOn, wasAcOn;
extern bool isLampOn, isGrowSeason;
extern TFT_ILI9163C display;
extern char ret[6];
extern BME280I2C bme[2];                   // Default : forced mode, standby time = 1000 ms
extern bool have_time;
extern WiFiEspUDP Udp;
extern WiFiEspClient client;
extern uint64_t gtim;

// how many locations
#define LOCATIONS   2
#define   A_OLD   (uint8_t)0
#define   A_NEW   (uint8_t)1
#define   A_IN    (uint8_t)0
#define   A_OUT   (uint8_t)1
#define   A_TEMP  (uint8_t)0
#define   A_HUMID (uint8_t)1
 
#define   A_GROW    true
#define   A_FLOWER  false
#define   A_DAY     true
#define   A_NIGHT   false
#define   A_MAX     (uint8_t)1
#define   A_MIN     (uint8_t)0

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define ORANGE          0xF400


// function prototypes
void checkClimate();
void makeCooler();
void makeHotter();
void wifiStart();
void resetDevice();
void wifiRestart();
void wifiRestart(bool reset);
void wifiConnect();
void wifiGetTime();
void wifiGetTime(bool force);
void sendNTPpacket(char *ntpSrv, bool isNew);
bool processSyncMessage();
void displayReset();
void displayTime();
void digitalClockDisplay(time_t t, uint16_t color);
void digitalClockDisplay(time_t t, uint16_t color, uint8_t x, uint8_t y);
void digitalClockDisplay(uint8_t h, uint8_t m, uint16_t color);
void digitalClockDisplay(uint8_t h, uint8_t m, uint16_t color, uint8_t x, uint8_t y);
void drawText(char *text, uint8_t f, uint8_t r, uint8_t g, uint8_t b, uint8_t x, uint8_t y);
void drawText(char *text, uint16_t c, uint8_t x, uint8_t y);
void drawText(char *text, uint8_t f, uint16_t color, uint8_t x, uint8_t y);
void serialClockDisplay();
void serialClockDisplay(time_t t);
void serialClockDisplay(time_t t, bool date);
void serialClockDisplay(time_t h, time_t m, time_t s);
void serialDateDisplay();
void serialDigits(int digits);
void displaySensor(uint8_t location, uint8_t sensor);
void readAndDisplaySensors();
float vaporPressureDeficit(int s, float t, float h);
void setRelays();
char *switchOrWait(bool w, bool new_v, bool *old_v, uint8_t pin);
void graphiteMetric(char *m, float v);
void graphiteMetric(char *m, int v);
void graphiteMetric(char *ma, float v, uint8_t t, bool inOut);
//void graphiteMetric(char *ma, float v, uint8_t t, bool inOut, bool hundreds);
void graphiteMetric(char *m, char *v);
//char *stringFromFloat(float f);
//char *stringFromFloat2(float f);

#endif
