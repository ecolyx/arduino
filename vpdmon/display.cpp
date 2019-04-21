/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 * 
 * (C) C Stott 2018
 * 
 ****************************************************/
#include "vpdmon.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>

#define TIME_MSG_LEN      11 // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER       'T' // Header tag for serial time sync message
#define TIME_REQUEST      7 // ASCII bell character requests a time sync message 

struct s_DisplayDHT {
  uint16_t color;
  uint8_t column;
  uint8_t row;
} displayDHT[2][2] = {{{GREEN, 2, 0},{BLUE, 12, 0}},{{GREEN, 2, 3},{BLUE, 12, 3}}};

TFT_ILI9163C display = TFT_ILI9163C(53, 9);

String stringBuf;

// static char return buffer for strings
char ret[6];

void displayReset() {
  display.begin();
  display.setRotation(1);
  display.clearScreen();
}

void displayTime() {
  displayTime(now() % 86400, ORANGE);
}

void displayTime(time_t t, bool serialOnly) {
  displayTime(t, 0, 0, 0, serialOnly);  
}

void displayTime(time_t t, uint16_t color) {
  displayTime(t, color, 36, 42);  
}

void displayTime(time_t t, uint16_t color, uint8_t x, uint8_t y) {
  displayTime(t, color, x, y, false);  
}

void displayTime(time_t t, uint16_t color, uint8_t x, uint8_t y, bool serialOnly) {
  const time_t HR = 3600;
  uint8_t h = t / HR;
  uint8_t m = (t - h * HR) / 60;
  displayTime(h, m, color, x, y, serialOnly);  
}

void displayTime(uint8_t h, uint8_t m, uint16_t color) {
  displayTime(h, m, color, 36, 42);
}

void displayTime(uint8_t h, uint8_t m, uint16_t color, uint8_t x, uint8_t y) {
  displayTime(h, m, color, x, y, false);
}

void displayTime(uint8_t h, uint8_t m, uint16_t color, uint8_t x, uint8_t y, bool serialOnly) {
  // digital clock display of the time
  char buf[6];
  getTimeString(buf, h, m);
  Serial.println(buf);
  if (!serialOnly) {
    display.fillRect(x, y, 70, 16, BLACK);
    display.setCursor(x, y);
    display.setTextScale(2);
    display.setTextColor(color);
    display.setCursor(x, y);
    display.print(buf);
  }
}

char *getTimeString(char *buf, time_t h, time_t m, time_t s) {
  getTimeString(buf, h, m);
  strcat(buf, ":");
  catDigits(buf, s);
  return buf;
}

char *getTimeString(char *buf, time_t h, time_t m) {
  strcpy(buf, "");
  catDigits(buf, h);
  strcat(buf, ":");
  catDigits(buf, m);
  return buf;
}

void catDigits(char *buf, int d) {
  char buf2[3];
  if (d < 10) {
    strcat(buf, "0");
  }
  strcat(buf, itoa(d, buf2, 10));
}

void serialDateDisplay() {
  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year());
  Serial.print(" ");  
}

void displaySensor(uint8_t location, uint8_t sensor) {
  smsg_pre(0 == location ? "In " : "Out ");
  smsg_pre(arraySensors[A_NEW][location][sensor]);
  smsg(0 != sensor ? "%" : "°");
  if (arraySensors[A_OLD][location][sensor] != arraySensors[A_NEW][location][sensor]) {
    char buf[5];
    display.fillRect(displayDHT[location][sensor].column * 6, displayDHT[location][sensor].row * 8, 56, 16, BLACK);
    String(arraySensors[A_NEW][location][sensor], 1).toCharArray(buf, 5);
    drawText(buf, 2, displayDHT[location][sensor].color, displayDHT[location][sensor].column, displayDHT[location][sensor].row);
  }
}

void drawText(char *text, uint8_t f, uint8_t r, uint8_t g, uint8_t b, uint8_t x, uint8_t y) {
  drawText(text, f, display.Color565(r, g, b), x, y);
}

void drawText(char *text, uint16_t c, uint8_t x, uint8_t y) {
  drawText(text, (uint8_t)1, c, x, y);
}

void drawText(char *text, uint8_t f, uint16_t color, uint8_t x, uint8_t y) {
  // draws color text, size f, at x, y, relative to font size
  display.setTextScale(f);
  display.setTextColor(color);
  display.setCursor(x * 6, y * 8);
  display.print(text);
}
