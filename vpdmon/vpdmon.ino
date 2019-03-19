/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 * 
 * (C) C Stott 2018
 * 
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
  debug_msg(init_msg);
  displayReset();
  display.setTextColor(WHITE);
  display.println(init_msg);

  pinMode(AC_PIN, OUTPUT);
  pinMode(CT_PIN, OUTPUT);

  digitalWrite(AC_PIN, true);
  digitalWrite(CT_PIN, true);

  // set up the bme sensor
  debug_msg("Setup BME280 sensors");
  Wire.begin();
  for (int s = 0; s < 2; s++) {
    while(!bme[s].begin()){
      Serial.println("Could not find BME280 sensor!");
      delay(1000);
    }
  }

  setTime(sunRise);
  isGrowSeason = GROWMINPERIOD < sunSet - sunRise;
  isLampOn = true;
  msg_pre("Sunset: ");
  serialClockDisplay(sunSet);
  msg_pre("Sunrise: ");
  serialClockDisplay(sunRise);
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

  wifiStart();
  display.clearScreen();
  wdt_reset();
  wdt_enable(WDTO_8S);     // enable the watchdog
  debug_msg("Setup complete");
}

void loop() {
  static uint8_t loops = 0;
  time_t tim = millis();
  
  wdt_reset();

  // check for input on console
  if (Serial.available()) {
    processSyncMessage();
  }

  wifiGetTime();

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

  // might be useful for VPD
  //  hicRoomOld = hicRoom;
  //  hicRoom = roomSensor.computeHeatIndex(tRoom, hRoom, false);
  //  hicOutOld = hicOut;
  //  hicOut = outsideSensor.computeHeatIndex(tOut, hOut, false);

  setRelays();
  smsg(); // blank line on serial

  time_t timeOfDay = now() % 86400;
  isLampOn = sunRise < timeOfDay && timeOfDay < sunSet;
  debug_msg_pre("Lamp: ");
  debug_msg(isLampOn ? "On" : "Off");
  debug_msg(isGrowSeason ? "Grow" : "Flower");
//  displayTime();
  int d = heartbeat + tim - millis();
  graphiteMetric("heartbeat", d);
  wdt_disable();
  delay(d); // regulates the loop by excluding the time to run loop code
  wdt_reset();
  wdt_enable(WDTO_8S);     // enable the watchdog
}
