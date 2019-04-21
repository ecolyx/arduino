/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 * 
 * (C) C Stott 2018
 * 
 ****************************************************/
#include "vpdmon.h"
#include "climate.h"

void checkClimate() {
  if (arraySensors[A_NEW][A_IN][A_TEMP] < minMaxClimates[isGrowSeason][A_MIN][isLampOn][A_TEMP]) { // inside/new/temp < [G/F][Min/Max][D/N][T/H]
    makeHotter();
    debug_msg("makeHotter");
  } else if (arraySensors[A_NEW][A_IN][A_TEMP] > minMaxClimates[isGrowSeason][A_MAX][isLampOn][A_TEMP]) { // inside/new/temp > season/max/light/temp
    makeCooler();
    debug_msg("makeCooler");
  } else {
    // everything is fine
    delayCC = 0;
    isAcOn = false;
    isFanOn = false;
    isHeatOn = false;

    // set cooltube to find middle temp of zone, if air-con isn't needed for cooling
    //TODO - this is where we can worry about humidity
    if (arraySensors[A_NEW][A_IN][A_TEMP] > minMaxClimates[isGrowSeason][A_MIN][isLampOn][A_TEMP] + 
                                              (minMaxClimates[isGrowSeason][A_MAX][isLampOn][A_TEMP] - minMaxClimates[isGrowSeason][A_MIN][isLampOn][A_TEMP]) / 2) {
      isCtOn = true;
    } else {
      isCtOn = false;
    }
    if (!testMode) DELAY_CC = DELAY_CC_DEFAULT;
  }
}

void makeCooler() {
  if (isHeatOn) {
    isHeatOn = false;
  } else if (!isCtOn && isLampOn) {
    isCtOn = true;
  } else {
    isAcOn = true;
  }
}

void makeHotter() {
  if (isAcOn) {
    isAcOn = false;
    delayCC = 0;
  } else if (isCtOn) {
    isCtOn = false;
  } else {
    isHeatOn = true;
  }
}

float vaporPressureDeficit(int s, float t, float h) {
  // might be useful for VPD
  //  hicRoomOld = hicRoom;
  //  hicRoom = roomSensor.computeHeatIndex(tRoom, hRoom, false);
  //  hicOutOld = hicOut;
  //  hicOut = outsideSensor.computeHeatIndex(tOut, hOut, false);

  float svp = 610.7 * pow(10, (7.5 * t)/(237.3 + t));
  float vpd = (((100 - h) / 100) * svp) / 1000;
  
  smsg_pre("SVP: ");
  smsg(svp);
  graphiteMetric("svp", svp, s, false);
  smsg_pre("VPD: ");
  smsg(vpd);
  return vpd;
}
