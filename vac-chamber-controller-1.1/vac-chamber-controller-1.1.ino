/*
vac-chamber-controller.ino
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

Connecting the BME280 Sensor:
Sensor              ->  Board
-----------------------------
Vin (Voltage In)    ->  3.3V
Gnd (Ground)        ->  Gnd
SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro

 */

/* ==== Includes ==== */
#include <BME280I2C.h>
#include <Wire.h>             // Needed for legacy versions of Arduino.
#include <LiquidCrystal.h>
#include <EEPROM.h>
/* ====  END Includes ==== */

/* ==== Defines ==== */
#define SERIAL_BAUD 115200
#define VAC_PRESSURE_HG 15.0
#define DEFAULT_TIMER 1000L * 60L * 60L * 6L
#define NO_OF_SWITCHES 2
#define SWITCH_VAC 0
#define SWITCH_MENU 1
#define MODE_NORMAL 0
#define MODE_MENU 1
#define MENU_ITEMS 5
#define MENUTYPE_FLOAT 0
#define MENUTYPE_TIME_MMSS 1
#define MENUTYPE_INT02D 2
#define MENUTYPE_INT2D 3
#define MENUTYPE_YESNO 4
/* ==== END Defines ==== */

/* ==== Global Variables ==== */
BME280I2C bme;                   // Default : forced mode, standby time = 1000 ms
                              // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool metric = true;

// named constants for the switch and motor pins
const int switchPin[2] = {2, 10}; // the number of the switch pins
const int motorPin =  3; // the number of the motor pin
const int solenoidPin = 5; // the number of the solenoid pin
const int outputRotaryA = 18;
const int outputRotaryB = 19;

int switchState[2] = {LOW, HIGH};  // variable for reading the switch's status
int switchPreviousState[2] = {LOW, HIGH};
int toggleState[2] = {LOW, LOW};

int aRotaryState;
int aRotaryLastState; 

float pressureTarget = 0;
float pressureSetting = VAC_PRESSURE_HG;

long timerStart = 0;
long timerMillis = DEFAULT_TIMER;
long timerSetting = DEFAULT_TIMER;
bool flagSetTimerAtPressure = false;

struct s_menu {
  char description[16];
  int type;
  float value;
  float increment;
  float vmax;
  float vmin;
} menu[MENU_ITEMS] = {{"Pressure      ", MENUTYPE_FLOAT, VAC_PRESSURE_HG, 0.05, 30, 10}, 
                      {"Hours         ", MENUTYPE_INT2D, 1, .5, 12, 0}, 
                      {"Minutes       ", MENUTYPE_INT2D, 0, .5, 59, 0}, 
                      {"Seconds       ", MENUTYPE_INT2D, 0, .5, 59, 0},
                      {"Confirm       ", MENUTYPE_YESNO, 0, .5, 1, 0}};

const char signature[17] = "vccEOBJecolyx1.0";
struct s_EEPROMObject {
  float pressure;
  long timer;
  const char signature[17] = "vccEOBJecolyx1.0";
} EEPROMObject;

int menuIndex = 0;

LiquidCrystal lcd(12, 11, 9, 8, 7, 6);

int runMode = 0; // 0 = normal, 1 = menu

float temp(NAN), hum(NAN), pres(NAN);
uint8_t pressureUnit(2); // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
const char *unitString[] = {"Pa", "hPa", "Hg", "atm", "bar", "torr", "N/m^2", "psi"};
/* ==== END Global Variables ==== */


/* ==== Prototypes ==== */
/* === Print a message to stream with the temp, humidity and pressure. === */
void printBME280Data(Stream * client);
/* === Print a message to stream with the altitude, and dew point. === */
void printBME280CalculatedData(Stream* client);
/* ==== END Prototypes ==== */

/* ==== Setup ==== */
void setup() {
  int i = 0;
  
  // initialise the lcd
  lcd.begin(16, 2);
  lcd.print("loading...");

  // initialize the solenoid and motor pin as an output:
  pinMode(motorPin, OUTPUT);
  pinMode(solenoidPin, OUTPUT);
  // initialize the switch pins as an input:
  for (i = 0; i < NO_OF_SWITCHES; i++)
    pinMode(switchPin[i], INPUT);
  digitalWrite(switchPin[SWITCH_MENU], HIGH);

  pinMode(outputRotaryA, INPUT);
  pinMode(outputRotaryB, INPUT);
  aRotaryState = digitalRead(outputRotaryA);

  // set up serial comms for console messages
  Serial.begin(SERIAL_BAUD);
  while(!Serial) {} // Wait

  // set up the bme sensor
  while(!bme.begin()){
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  EEPROM.get(0, EEPROMObject);
  if (EEPROMObject.signature == signature) {
    pressureSetting = EEPROMObject.pressure;
    timerSetting = EEPROMObject.timer;
  }
  digitalWrite(13, LOW);

  lcd.setCursor(0, 0);
  lcd.print("          ");
}
/* ==== END Setup ==== */

/* ==== Loop ==== */
void loop() {
  serviceSwitches();

  processTimer();

  processControls();

  switch(runMode) {
    case MODE_NORMAL: //normal mode    
      normalMode();
      break;

    case MODE_MENU:
      menuMode();
      break;
  }
}
/* ==== End Loop ==== */

void serviceSwitches() {
  int i = 0;

  // read the state of the switches:
  for (i = 0; i < NO_OF_SWITCHES; i++) {
    switchPreviousState[i] = switchState[i];
    switchState[i] = digitalRead(switchPin[i]);
  }
}
  
void processControls() {
  static bool flagMotorOn = false;
  
  // switch on pump if toggle is on and pressure is below target (back on if pressure has slipped .2 since last off)
  if (!flagMotorOn && toggleState[SWITCH_VAC] == HIGH && pres > (pressureTarget + 0.2)) {
    digitalWrite(motorPin, HIGH);
    lcd.setCursor(15, 1);
    lcd.print("P");
    flagMotorOn = true;
  }

  if (flagMotorOn && pres < (pressureTarget)) {
    // if we reached pressure for the first time, start the timer
    if (flagSetTimerAtPressure) {
      timerStart = millis();
      flagSetTimerAtPressure = false;
    }
    
    digitalWrite(motorPin, LOW);
    lcd.setCursor(15, 1);
    lcd.print("p");
    flagMotorOn = false;
  }
}

void processTimer() {
  char buf[8], buf2[9];
  long diffS;

  // is timer running?
  if (timerStart > 0) {
    // how long for?
    diffS = (timerMillis - (millis() - timerStart)) / 1000;
    lcd.setCursor(7, 0);
    //    if (diffS > 60L * 60L) {
    sprintf(buf, "%2d:%02d:%02d", int(diffS / (60L * 60L)), int(diffS / 60L) % 60, int(diffS) % 60);
    //    } else if (diffS > 60L) {
    //      sprintf(buf, "   %2d:%02d", int(diffS / 60L), int(diffS % 60L));
    //    } else {
    //      sprintf(buf, "      %2d", int(diffS % 60L));
    //    }
    lcd.print(buf);
    dtostrf(pressureTarget, 4, 1, buf);
    sprintf(buf2, "<%s %s", buf, unitString[pressureUnit]);
    lcd.setCursor(11, 1);
    lcd.print(buf);
    
    
    // has timer elapsed?
    if (diffS <= 0) {
      // timer elapsed
      timerStart = 0;
      pressureTarget = 0;
      toggleState[SWITCH_VAC] = LOW;
      digitalWrite(motorPin, LOW);
      digitalWrite(solenoidPin, LOW);
      lcd.clear();
    }
  }
}

void normalMode() {
  // check if vac switch is pressed
  if (switchPreviousState[SWITCH_VAC] == LOW && switchState[SWITCH_VAC] == HIGH) {
    toggleState[SWITCH_VAC] = toggleState[SWITCH_VAC] == HIGH ? LOW : HIGH;
    if (toggleState[SWITCH_VAC] == HIGH) {
      pressureTarget = pressureSetting;
      digitalWrite(solenoidPin, HIGH);
      lcd.setCursor(15, 0);
      lcd.print("S");
      flagSetTimerAtPressure = true;
    } else {
      pressureTarget = 0;
      toggleState[SWITCH_VAC] = LOW;
      digitalWrite(motorPin, LOW);
      digitalWrite(solenoidPin, LOW);
      lcd.clear();
      lcd.setCursor(15, 0);
      lcd.print("s");
      lcd.setCursor(15, 1);
      lcd.print("p");
      timerStart = 0;
    }
  }

  // check if menu switch is pressed
  if (switchPreviousState[SWITCH_MENU] == HIGH && switchState[SWITCH_MENU] == LOW) {
    if (pressureTarget > 0) {
      // if under pressure, cancel program
      pressureTarget = 0;
      toggleState[SWITCH_VAC] = LOW;
      digitalWrite(motorPin, LOW);
      digitalWrite(solenoidPin, LOW);
      lcd.clear();
      lcd.setCursor(15, 0);
      lcd.print("s");
      lcd.setCursor(15, 1);
      lcd.print("p");
      timerStart = 0;
    } else {
      // enter menu mode
      runMode = MODE_MENU;
      lcd.setCursor(0, 0);
      lcd.print(menu[0].description);
    }
  } else {  
    printBME280Data(&Serial);
  }

  delay(50);
}

void menuMode() {
  int inputA = 0;
  char buf[8];

  switch(menu[menuIndex].type) {
    case MENUTYPE_FLOAT:
      dtostrf(menu[menuIndex].value, 4, 1, buf);
      strcat(buf, "    ");
      break;

    case MENUTYPE_TIME_MMSS:
      sprintf(buf, "%2d:%02d   ", int(menu[menuIndex].value / 1000 / 60), int(menu[menuIndex].value / 1000) % 60);
      break;

    case MENUTYPE_INT02D:
      sprintf(buf, "%02d   ", int(menu[menuIndex].value));
      break;

    case MENUTYPE_INT2D:
      sprintf(buf, "%2d   ", int(menu[menuIndex].value));
      break;

    case MENUTYPE_YESNO:
      sprintf(buf, "%s   ", int(menu[menuIndex].value) % 2 == 0 ? "Yes" : "No");
      break;
  }
  lcd.setCursor(0, 1);
  lcd.print(buf);

  aRotaryLastState = aRotaryState;
  aRotaryState = digitalRead(outputRotaryA);
  if (aRotaryState != aRotaryLastState) {
    if (digitalRead(outputRotaryB) != aRotaryState) {
      menu[menuIndex].value += menu[menuIndex].increment;
      if (menu[menuIndex].value > menu[menuIndex].vmax + menu[menuIndex].increment) {
        menu[menuIndex].value = menu[menuIndex].vmin;
      }
    } else {
      menu[menuIndex].value -= menu[menuIndex].increment;
      if (menu[menuIndex].value + menu[menuIndex].increment < menu[menuIndex].vmin) {
        menu[menuIndex].value = menu[menuIndex].vmax;
      }
    }
  }

  if (switchPreviousState[SWITCH_MENU] == HIGH && switchState[SWITCH_MENU] == LOW) {
    if (++menuIndex >= MENU_ITEMS) {
      // exit menu mode
      runMode = MODE_NORMAL;

      pressureSetting = menu[0].value;
      timerMillis = 60L * 60L * menu[1].value;
      timerMillis += 60L * menu[2].value;
      timerMillis += menu[3].value;
      timerMillis *= 1000;

      toggleState[SWITCH_VAC] = HIGH;
      pressureTarget = pressureSetting;
      digitalWrite(solenoidPin, HIGH);
      lcd.setCursor(15, 0);
      lcd.print("S");
      flagSetTimerAtPressure = true;
      digitalWrite(13, HIGH);
      
      EEPROMObject.pressure = pressureSetting;
      EEPROMObject.timer = timerMillis;

      EEPROM.put(0, EEPROMObject);
      digitalWrite(13, LOW);
      
      lcd.clear();
    } else {
      lcd.setCursor(0, 0);
      lcd.print(menu[menuIndex].description);
    }
  }
}

/* ==== Functions ==== */
void printBME280Data(Stream* client){
  char lcdString[34], buf[8];
  static long lastUpdate = millis();
  
  if (lastUpdate + 1000 < millis()) {
    bme.read(pres, temp, hum, metric, pressureUnit);                   // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
    /* Alternatives to ReadData():
      float temp(bool celsius = false);
      float pres(uint8_t unit = 0);
      float hum();
  
      Keep in mind the temperature is used for humidity and
      pressure calculations. So it is more effcient to read
      temperature, humidity and pressure all together.
     */

    lastUpdate=millis();
    client->print("Temp: ");
    client->print(temp);
    client->print("°"+ String(metric ? 'C' :'F'));
    client->print("\t\tHumidity: ");
    client->print(hum);
    client->print("% RH");
    client->print("\t\tPressure: ");
    client->print(pres);
    client->print(" " + String(unitString[pressureUnit]));

    printBME280CalculatedData(&Serial);
  }
  
  // update the lcd
  dtostrf(temp, 4, 1, buf);
  sprintf(lcdString, "%s%c%c", buf, 0xb2, metric ? 'C' :'F');
  lcd.setCursor(0, 0);
  lcd.print(String(lcdString));

  dtostrf(pres, 4, 1, buf);
  sprintf(lcdString, "%s %s", buf, unitString[pressureUnit]);
  lcd.setCursor(0, 1);
  lcd.print(String(lcdString));
}

void printBME280CalculatedData(Stream* client){
  float altitude = bme.alt(metric);
  float dewPoint = bme.dew(metric);
  client->print("\t\tAltitude: ");
  client->print(altitude);
  client->print((metric ? "m" : "ft"));
  client->print("\t\tDew point: ");
  client->print(dewPoint);
  client->println("°"+ String(metric ? 'C' :'F'));

}
/* ==== END Functions ==== */
