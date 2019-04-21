/***************************************************
   VPDMon

   climate controller for indoor grow room

   (C) C Stott 2018

 ****************************************************/

#include "vpdmon.h"
#include <Wire.h>

const time_t sunRise = 14400, sunSet = 57600; // flower
const uint16_t heartbeat = 30000; // milliseconds looping

void setup()
{
  char const *init_msg = "init..";
  // initialize serial for debugging
  Serial.begin(115200);
  // initialize serial for ESP module
  Serial1.begin(ESPBAUD);
  debug_msg(init_msg);
  displayReset();
  display.setTextColor(WHITE);
  display.println(init_msg);

  wdt_disable();

  pinMode(AC_PIN, OUTPUT);
  pinMode(CT_PIN, OUTPUT);
  pinMode(HT_PIN, OUTPUT);

  digitalWrite(AC_PIN, false);
  digitalWrite(CT_PIN, false);
  digitalWrite(HT_PIN, false);

  // set up the bme sensor
  debug_msg("Setup BME280 sensors");
  Wire.begin();
  for (int s = 0; s < 2; s++) {
    while (!bme[s].begin()) {
      Serial.println("Could not find BME280 sensor!");
      delay(1000);
    }
  }

  setTime(sunRise);
  isGrowSeason = GROWMINPERIOD < sunSet - sunRise;
  isLampOn = false;
  msg_pre("Sunset: ");
  displayTime(sunSet, true);
  msg_pre("Sunrise: ");
  displayTime(sunRise, true);
  msg_pre("grow - on: ");
  msg_pre(minMaxClimates[A_GROW][A_MIN][A_DAY][A_TEMP]);//[G/F][Min/Max][D/N][T/H]
  msg_pre("- ");
  msg_pre(minMaxClimates[A_GROW][A_MAX][A_DAY][A_TEMP]);
  msg_pre(", off: ");
  msg_pre(minMaxClimates[A_GROW][A_MIN][A_NIGHT][A_TEMP]);
  msg_pre(" - ");
  msg(minMaxClimates[A_GROW][A_MAX][A_NIGHT][A_TEMP]);
  msg_pre("flower - on: ");
  msg_pre(minMaxClimates[A_FLOWER][A_MIN][A_DAY][A_TEMP]);
  msg_pre(" - ");
  msg_pre(minMaxClimates[A_FLOWER][A_MAX][A_DAY][A_TEMP]);
  msg_pre(", off: ");
  msg_pre(minMaxClimates[A_FLOWER][A_MIN][A_NIGHT][A_TEMP]);
  msg_pre(" - ");
  msg(minMaxClimates[A_FLOWER][A_MAX][A_NIGHT][A_TEMP]);
  msg_pre("delayCC: ");
  msg(delayCC);

  if (WATCHDOG) wdt_reset();
  if (WATCHDOG) wdt_enable(WDTO_8S);     // enable the watchdog

  wifiStart();
  display.clearScreen();
  debug_msg("Setup complete");
}

void loop() {
  static uint8_t loops = 0;
  time_t tim = millis();

  if (WATCHDOG) wdt_reset();

  smsg(); // blank line on serial

  // check for input on console
  if (Serial.available()) {
    processSyncMessage();
  }

  wifiCheckTime();

  smsg_pre("millis(): ");
  smsg(millis());
  graphiteMetric("sw.restart", millis() < 60000 ? -5 : 0);

  readAndDisplaySensors();

  const uint8_t ul_x = 24, ul_y = 56, ul_scale = 1;

  debug_msg_pre("arraySensors[A_NEW][A_IN][A_TEMP] = ");
  debug_msg(arraySensors[A_NEW][A_IN][A_TEMP]);
  debug_msg_pre("minMaxClimates[isGrowSeason][A_MIN][isLampOn][A_TEMP] = ");
  debug_msg(minMaxClimates[isGrowSeason][A_MIN][isLampOn][A_TEMP]);
  debug_msg_pre("minMaxClimates[isGrowSeason][A_MAX][isLampOn][A_TEMP] = ");
  debug_msg(minMaxClimates[isGrowSeason][A_MAX][isLampOn][A_TEMP]);
  display.setTextScale(1);
  display.setTextColor(YELLOW);
  display.fillRect(0, 60, 84, 32, BLACK);
  display.setCursor(0, 60);
  display.print(minMaxClimates[isGrowSeason][A_MIN][isLampOn][A_TEMP]);
  display.print("-");
  display.println(minMaxClimates[isGrowSeason][A_MAX][isLampOn][A_TEMP]);
  display.print(isGrowSeason ? "Grow - " : "Flower - ");
  display.print(isLampOn ? "Day" : "Night");

  // check the climate
  checkClimate();

  setRelays();

  debug_msg_pre("Lamp: ");
  debug_msg(isLampOn ? "On" : "Off");
  debug_msg(isGrowSeason ? "Grow" : "Flower");
  //  displayTime();
  int d = heartbeat + tim - millis();
  graphiteMetric("heartbeat", d);
  graphiteMetric("memfree", freeRam());

  wdt_disable();
  delay(d); // regulates the loop by excluding the time to run loop code
  if (WATCHDOG) wdt_reset();
  if (WATCHDOG) wdt_enable(WDTO_8S);     // enable the watchdog
}

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
