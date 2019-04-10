/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 * 
 * (C) C Stott 2018
 * 
 ****************************************************/
#include "vpdmon.h"

bool processSyncMessage() {
#if 0
  bool dateFormat = false;
  
  // if time sync available from serial port, update time and return true
  while (Serial.available() > 0) { // time message consists of header &  up to 10 ASCII digits
    char c = Serial.read();
    //Serial.print(c);
    if (c == TIME_HEADER) {
      time_t pctime = 0;
      time_t h = 0, t = 0, s = 0, d = 0, m = 0, y = 0;
      while (Serial.available() > 0) {
        c = Serial.read();
        //Serial.print(c);
        if (c >= '0' && c <= '9') {
          pctime = (10 * pctime) + (c - '0') ; // convert digits to a number
          //Serial.print("pctime = ");
          //debug_msg(pctime);
        }
        if (dateFormat || c == ':' || c == ' ') {
          //debug_msg("\" :\" or dateFormat");
          dateFormat = true;
          if (c == ':' || c == ' ' || Serial.available() == 0) {
            //debug_msg("\" :\"");
            if (h == 0) {
              h = pctime;
              pctime = 0;
              //debug_msg("set hours");
            } else if (t == 0) {
              t = pctime + 1;
              pctime = 0;
              //debug_msg("set mins");
            } else if (s == 0) {
              s = pctime + 1;
              pctime = 0;
              //debug_msg("set secs");
            } else if (d == 0) {
              d = pctime;
              pctime = 0;
              //debug_msg("set days");
            } else if (m == 0) {
              m = pctime;
              pctime = 0;
              //debug_msg("set months");
            } else {
              y = pctime;
              //debug_msg("set years");
            }
            if (Serial.available() == 0) {
              setTime(h, t == 0 ? 0 : t - 1, s == 0 ? 0 : s - 1, d == 0 ? day() : d, m == 0 ? month() : m, y == 0 ? year() : y);
              Serial.print("new time: ");
              serialClockDisplay();
              pctime = now();
            }
          }
        }
        
        if (c == 'P') {
          Serial.print("Temp = ");
          Serial.println(pctime);
          testMode = true;
          arraySensors[A_NEW][A_IN][A_TEMP] = (float)pctime;
          displaySensor(A_IN, A_TEMP);
          Serial.println("huh!");
          pctime = now();
        }
        
        if (c == 'H') {
          Serial.print("Humidity = ");
          Serial.println(pctime);
          testMode = true;
          arraySensors[A_NEW][A_IN][A_HUMID] = (float)pctime; // [inside][new][humidity]
          displaySensor(A_IN, A_HUMID);
          pctime = now();
        }

        if (c == 'D') {
          Serial.print("Delay = ");
          Serial.println(pctime);
          testMode = true;
          DELAY_CC = pctime; // adjust delay for faster testing
          delayCC = now() + DELAY_CC;
          pctime = now();
        }

        if (c == 'F') {
          Serial.println("Test mode exit");
          testMode = false;
          pctime = now();
        }

        if (c == 'R') {
          Serial.println("Reset lcd");
          displayReset();
          pctime = now();
        }
      }
      setTime(pctime); // Sync Arduino clock to the time received on the serial port
    }
  }

  return testMode;
#endif
  return false;
}
