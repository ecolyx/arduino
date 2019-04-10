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
} displayDHT[2][2] = {{{GREEN, 0, 0},{BLUE, 10, 0}},{{GREEN, 0, 3},{BLUE, 10, 3}}};

TFT_ILI9163C display = TFT_ILI9163C(53, 9);

// static char return buffer for strings
char ret[6];

void displayReset() {
  display.begin();
  display.setRotation(1);
  display.clearScreen();
}

void displayTime() {
  static time_t lastTime = now() % 86400;
  static float oldTmin = 0, oldTmax = 0;
  static float oldHmin = 0, oldHmax = 0;

  lastTime = now() % 86400;
  digitalClockDisplay(lastTime, ORANGE);
}

void digitalClockDisplay(time_t t, uint16_t color) {
  serialClockDisplay(t);

  digitalClockDisplay(t, color, 24, 40);  
}

void digitalClockDisplay(time_t t, uint16_t color, uint8_t x, uint8_t y) {
  const time_t HR = 3600;
  uint8_t h = t / HR;
  uint8_t m = (t - h * HR) / 60;
  digitalClockDisplay(h, m, color, x, y);  
}

void digitalClockDisplay(uint8_t h, uint8_t m, uint16_t color) {
  digitalClockDisplay(h, m, color, 24, 40);
}

void digitalClockDisplay(uint8_t h, uint8_t m, uint16_t color, uint8_t x, uint8_t y) {
  // digital clock display of the time
  char buf[6];
  display.fillRect(x, y, 70, 16, BLACK);
  display.setCursor(x, y);
  display.setTextScale(2);
  display.setTextColor(color);
  display.setCursor(x, y);
  sprintf(buf, "%02d:%02d", h, m);
  display.print(buf);
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

void serialClockDisplay() {
  serialClockDisplay(hour(), minute(), second());
}

void serialClockDisplay(time_t t, bool date) {
  if (date) {
    serialDateDisplay();
  }
  serialClockDisplay(t);
}

void serialClockDisplay(time_t t) {
  serialClockDisplay(t / 3600, (t / 60) % 60, t % 60);
}

void serialClockDisplay(time_t h, time_t m, time_t s) {
  // digital clock display of the time
  Serial.print(h);
  serialDigits(m);
  serialDigits(s);
  Serial.println();
}

void serialDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
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
  smsg(0 != sensor ? "%" : "");
  if (arraySensors[A_OLD][location][sensor] != arraySensors[A_NEW][location][sensor]) {
    char buf[5];
    String(arraySensors[A_OLD][location][sensor], 1).toCharArray(buf, 5);
    drawText(buf, 2, BLACK, displayDHT[location][sensor].column, displayDHT[location][sensor].row);
    String(arraySensors[A_NEW][location][sensor], 1).toCharArray(buf, 5);
    drawText(buf, 2, displayDHT[location][sensor].color, displayDHT[location][sensor].column, displayDHT[location][sensor].row);
    arraySensors[A_OLD][location][sensor] = arraySensors[A_NEW][location][sensor];
    //debug_msg("c=" + String(displayDHT[location + sensor * 2].column) + ", r=" + String(displayDHT[location + sensor * 2].row));
  }
}
