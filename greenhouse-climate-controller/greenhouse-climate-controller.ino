/*
greenhouse-climate-controller.ino
Copyright (C) 2017  Charlie Stott

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Written: August 2017.

Hardware:
4 Relay Module
DHT22 Temp/Humidity Sensor

 */

// includes
#include <LiquidCrystal.h>
#include <Dht11.h>

// defines
#define SERIAL_BAUD       115200
#define PIN_RELAY_IN1     9
#define PIN_RELAY_IN2     10
#define PIN_RELAY_IN3     11
#define PIN_RELAY_IN4     12
#define SWITCH_AIRCON     PIN_RELAY_IN1
#define SWITCH_EXTRACTOR  PIN_RELAY_IN2
#define SWITCH_HEATER     PIN_RELAY_IN3
#define SWITCH_LAMP       PIN_RELAY_IN4
#define DHT_MAX_TIMINGS   85
#define DHT_PIN1          8
#define DHT_PIN2          13

// globals
float roomTemp = 0;
float roomHumdity = 0;
float outsideTemp = 0;
float outsideHumidity = 0;
int dht_data[5] = { 0, 0, 0, 0, 0 };
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
Dht11 roomSensor(DHT_PIN1);
Dht11 outsideSensor(DHT_PIN2);

void setup() {
  // set up serial comms for console messages
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("loading...");
  Serial.begin(SERIAL_BAUD);
  while(!Serial) {} // Wait

  Serial.println("Loading...");
  // initialise relay pins
  pinMode(PIN_RELAY_IN1, OUTPUT);
  pinMode(PIN_RELAY_IN2, OUTPUT);
  pinMode(PIN_RELAY_IN3, OUTPUT);
  pinMode(PIN_RELAY_IN4, OUTPUT);
  digitalWrite(PIN_RELAY_IN1, LOW);
  digitalWrite(PIN_RELAY_IN2, LOW);
  digitalWrite(PIN_RELAY_IN3, LOW);
  digitalWrite(PIN_RELAY_IN4, LOW);

  lcd.clear();
  Serial.println("Setup complete");
}

void loop() {
  int h = 0, t = 0;
  long timer = millis();
  char buf[17];

  Serial.println("Looping");

  roomSensor.read();
  outsideSensor.read();
  
  sprintf(buf, "In  %2d%% %2d%cC %l", roomSensor.getHumidity(), roomSensor.getTemperature(), (char)0xb2, 1000 + timer - millis());
  Serial.println(buf);
  lcd.setCursor(0, 0);
  lcd.print(buf);

  sprintf(buf, "Out %2d%% %2d%cC %l", outsideSensor.getHumidity(), outsideSensor.getTemperature(), (char)0xb2, 1000 + timer - millis());
  Serial.println(buf);
  lcd.setCursor(0, 1);
  lcd.print(buf);

  delay(2000);
//  delay(1000 + timer - millis()); /* loop every 2 seconds */
}

