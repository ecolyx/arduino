/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 *    
 * (C) C Stott 2018
 * 
 ****************************************************/
#include "vpdmon.h"

#define DHT_PIN1          4
#define DHT_PIN2          5

BME280I2C bme[2];                   // Default : forced mode, standby time = 1000 ms
                                    // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

float arraySensors[2][LOCATIONS][2]; // old/new, in/out, t/h

void readAndDisplaySensors() {
  static long lastUpdate = -1000;

  if (lastUpdate + 1000 < millis()) {
    for (int s = 0; s < 2; s++) {
      float pressure;
      BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
      BME280::PresUnit presUnit(BME280::PresUnit_inHg);
      arraySensors[A_OLD][s][A_TEMP] = arraySensors[A_NEW][s][A_TEMP];
      arraySensors[A_OLD][s][A_HUMID] = arraySensors[A_NEW][s][A_HUMID];
      ((BME280I2C)bme[s]).read(pressure, arraySensors[A_NEW][s][A_TEMP], arraySensors[A_NEW][s][A_HUMID], tempUnit, presUnit); 
      /* BME methods
        void read(float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
        float temp(bool celsius = false);
        float pres(uint8_t unit = 0);
        float hum();
    
        Keep in mind the temperature is used for humidity and
        pressure calculations. So it is more effcient to read
        temperature, humidity and pressure all together.
      */
      graphiteMetric("vpd", vaporPressureDeficit(s, arraySensors[A_NEW][s][A_TEMP], arraySensors[A_NEW][s][A_HUMID]), s, false);
      graphiteMetric("pressure", pressure, s, false);
      graphiteMetric("temp", arraySensors[A_NEW][s][A_TEMP], s, false);
      graphiteMetric("humidity", arraySensors[A_NEW][s][A_HUMID], s, false);
      displaySensor(s, A_TEMP);
      displaySensor(s, A_HUMID);
    }
        
    lastUpdate=millis();
  }
}

void setRelays() {
  char strDisp[16]; // = {'A', 0, 0, ' ', 'C', 0, 0, ' ', 'H', '0', '0', ' ', 'F','0', '0', 0};
  bool wait = delayCC > now();

  climateChangePending = true;

  strcpy(strDisp, "A");
  strcat(strDisp, switchOrWait(wait, isAcOn, &wasAcOn, AC_PIN));
  strcat(strDisp, " C");
  strcat(strDisp, switchOrWait(wait, isCtOn, &wasCtOn, CT_PIN));
  strcat(strDisp, " H");
  strcat(strDisp, switchOrWait(wait, isHeatOn, &wasHeatOn, HT_PIN));
  strcat(strDisp, " F00");
  smsg(strDisp);
  graphiteMetric("sw.lamp", isLampOn);
  graphiteMetric("sw.aircon", isAcOn);
  graphiteMetric("sw.ctfan", isCtOn);
  graphiteMetric("sw.heater", isHeatOn);
  
  display.fillRect(0, 80, 136, 8, BLACK);
  drawText(strDisp, (uint16_t)RED | BLUE, 0, 10);

  if (!wait) {
    if (!climateChangePending) {
      // we just changed a value, so need to wait
      delayCC = now() + DELAY_CC;
      smsg_pre("setRelays->Set wait: ");
      smsg(delayCC);
    }
  } else {
    char str[20];
    smsg_pre("Wait: ");
    strcpy(str, itoa(delayCC - now(), ret, 10));
    smsg(str);
    drawText(str, 1, 150, 75, 0, 18, 10);
  }
}

char *switchOrWait(bool w, bool new_v, bool *old_v, uint8_t pin) {
  if (*old_v != new_v) {
    if (!w) {
      digitalWrite(pin, !new_v);
      *old_v = new_v;
      climateChangePending = false;
    }
  }
  strcpy(ret, *old_v == 0 ? "0" : "1");
  strcat(ret, new_v == 0 ? "0" : "1");
  return ret;
}
