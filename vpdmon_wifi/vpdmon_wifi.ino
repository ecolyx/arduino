/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 * 
 * (C) C Stott 2018
 * 
 ****************************************************/
#define setTextScale setTextSize

// relay pins
#define AC_PIN          A0
#define CT_PIN          A1
#define HT_PIN          A2
#define FN_PIN          A3

// switch pins
#define SW1_PIN         (uint8_t)0
#define SW2_PIN         (uint8_t)1

// delay period for Climate control after change in settings to prevent over rapid switching when transitioning
// air con likes at least 5 minutes between cycles, so seems a reasonable rule to apply
#define DELAY_CC 300 // 5 mins

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

#define TIME_MSG_LEN      11 // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER       'T' // Header tag for serial time sync message
#define TIME_REQUEST      7 // ASCII bell character requests a time sync message 

#define DHT_PIN1          4
#define DHT_PIN2          5

#define GROWMINPERIOD     43201

#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <DHT.h>
#include <SoftwareSerial.h>

struct s_DisplayDHT {
  uint16_t color;
  uint8_t column;
  uint8_t row;
} displayDHT[4] = {{GREEN, 0, 0},{BLUE, 8, 0},{GREEN, 0, 3},{BLUE, 8, 3}};

TFT_ILI9163C display = TFT_ILI9163C(8, 10);

// Construct DHT objects
DHT dhtSensors[2] = {DHT(DHT_PIN1, DHT22),DHT(DHT_PIN2, DHT22)};
float arraySensors[2][2][2]; // old/new, in/out, t/h
#define   A_OLD   (uint8_t)0
#define   A_NEW   (uint8_t)1
#define   A_IN    (uint8_t)0
#define   A_OUT   (uint8_t)1
#define   A_TEMP  (uint8_t)0
#define   A_HUMID (uint8_t)1
 
float minMaxClimates[2][2][2][2] = {{{{23, 40},{18, 30}}, {{27, 70},{22, 70}}},{{{21, 30},{18, 30}}, {{25, 50},{20, 50}}}}; // grow/flower,[day/night,] min/max, t/h
#define   A_GROW    (uint8_t)0
#define   A_FLOWER  (uint8_t)1
#define   A_DAY     (uint8_t)0
#define   A_NIGHT   (uint8_t)1
#define   A_MIN     (uint8_t)0
#define   A_MAX     (uint8_t)1

#define DHTIN   dhtSensors[0]
#define DHTOUT  dhtSensors[1]

bool isAcOn = false, isCtOn = false, isHeatOn = false, isFanOn = false;
bool wasFanOn = false, wasCtOn = false, wasHeatOn = false, wasAcOn = false;
bool isLampOn = false, isGrowFlower = false;
time_t sunRise = 14400, sunSet = 57600;
static time_t delayCC = now() - DELAY_CC;

volatile uint8_t loopMode = 0;

SoftwareSerial Serial1(6, 7); // RX, TX

// A UDP instance to let us send and receive packets over UDP
const uint8_t NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
WiFiEspUDP Udp;

void setup()
{
  // initialize serial for debugging
  Serial.begin(57600);
  // initialize serial for ESP module
  Serial1.begin(9600);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("No WiFi");
    // don't continue
    while (true);
  }

  wifiConnect();

  display.begin();
  display.setRotation(2);
  display.clearScreen();

  pinMode(AC_PIN, OUTPUT);
  pinMode(CT_PIN, OUTPUT);
  pinMode(HT_PIN, OUTPUT);
  pinMode(FN_PIN, OUTPUT);

  pinMode (SW1_PIN, INPUT_PULLUP);
  pinMode (SW2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SW1_PIN), switchMode, FALLING);
  attachInterrupt(digitalPinToInterrupt(SW2_PIN), incrementTime, FALLING);

  DHTIN.begin();
  DHTOUT.begin();

  isGrowFlower = GROWMINPERIOD > sunSet - sunRise;
  isLampOn = now() % 86400 > sunRise;
  wifiGetTime();
  loopMode = 0;
}

void loop() {
  static uint8_t loops = 0;
  time_t tim = millis();

  // check for input on console
  if (Serial.available()) {
    processSyncMessage();
  }

  readAndDisplayDHT22s();

  switch(loopMode) {
    case 0:
      // check the climate
      if (arraySensors[A_NEW][A_IN][A_TEMP] > minMaxClimates[isGrowFlower][A_MAX][isLampOn][A_TEMP]) { // inside/new/temp > season/max/light/temp
        makeCooler();
      } else {
        // make sure ac is off
        if (isAcOn) {
          isAcOn = false;
        }
      }
  
      if (arraySensors[A_NEW][A_IN][A_TEMP] < minMaxClimates[isGrowFlower][A_MIN][isLampOn][A_TEMP]) { // inside/new/temp < season/min/light/temp
        makeHotter();
      }
    
      // might be useful for VPD
      //  hicRoomOld = hicRoom;
      //  hicRoom = roomSensor.computeHeatIndex(tRoom, hRoom, false);
      //  hicOutOld = hicOut;
      //  hicOut = outsideSensor.computeHeatIndex(tOut, hOut, false);
    
      setRelays();
      delay(millis() - tim + 5000);  // 2 second heartbeat
      break;

    case 1:
    case 2:
      display.setCursor(24, 56);
      display.setTextScale(1);
      display.setTextColor(BLACK);
      display.print(String("^^^^").substring(0, 4 * (loopMode - 1)));
      display.setTextColor(WHITE);
      display.print("^^^^");
      break;

    default:
      display.setCursor(24, 56);
      display.setTextScale(1);
      display.setTextColor(BLACK);
      display.print(String("^^^^^^^^"));
      loopMode = 0;
      Serial.println(loopMode);
      break;
  }

  isLampOn = sunSet > (now() % 86400) + sunRise;

  displayTime();
}

void wifiGetTime() {
  uint8_t count = 0;
  IPAddress timeServer(128,138,140,50); // time.nist.gov NTP server

  sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  while (count < 30) {
    if (Udp.parsePacket()) {
      Serial.println("pkt rcvd");
      // We've received a packet, read the data from it
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      for (int j = 0; j < NTP_PACKET_SIZE; j++) {
        Serial.print(packetBuffer[j], HEX);
        Serial.print(" ");
        if (j % 8 == 7) {
          Serial.println();
        }
      }
      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
  
      uint16_t highWord = word(packetBuffer[40], packetBuffer[41]);
      uint16_t lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      time_t t = highWord;
      t = t << 16 | lowWord;
      Serial.print("Secs since 1/1/1900 = ");
      Serial.println(t);
  
      // now convert NTP time into everyday time:
      Serial.print("Unix time = ");
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      t = t - seventyYears;
      // print Unix time:
      Serial.println(t);
  
      // print the hour, minute and second:
      Serial.print("UTC: ");       // UTC is the time at Greenwich Meridian (GMT)
      serialClockDisplay(t);
      setTime(t);
      count = 30;
    } else {
      Serial.println(count++);
      delay(1000);
    }
  }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {

  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
//  packetBuffer[0] = 0x1b;   // LI, Version, Mode
  
  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket();
  //Serial.println("6");
}

void wifiConnect() {
  char ssid[] = "HereBeDragons";            // your network SSID (name)
  char pass[] = "Sh33laZ1ggy";        // your network password
  int status = WL_IDLE_STATUS;     // the Wifi radio's status

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Connect to WPA: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println("Success!");

  Udp.begin(2390);
}

void switchMode() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200)
  {
    loopMode++;  
    last_interrupt_time = interrupt_time;
  }
}

void incrementTime() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 500 && 0 != loopMode) {
    int h = hour();
    setTime(now() + (7200 * (loopMode & 1)) + (60 * (loopMode >> 1)));
    if (h != hour()) {
      setTime(now() - 3600);
    }
    last_interrupt_time = interrupt_time;
  }
}

void displayTime() {
  static time_t lastTime = now() % 86400;
  static float oldTmin = 0, oldTmax = 0;
  static float oldHmin = 0, oldHmax = 0;

  lastTime = now() % 86400;
  serialClockDisplay(lastTime);

  display.setCursor(24, 40);

  digitalClockDisplay(lastTime, ORANGE);

/*
  if (minMaxClimates[isGrowFlower][1][isLampOn][0] != oldTmax) { // season/max/light/temp
    String str = String((int)oldTmin) + '.' + String(int(oldTmin * 10) % 10);
    drawText(str, 1, BLACK, 0, 48);
    oldTmin = minMaxClimates[isGrowFlower][0][isLampOn][0];
    str = String((int)oldTmax) + '.' + String(int(oldTmax * 10) % 10);
    drawText(str, 1, GREEN & BLUE, 0, 48);
    str = String((int)oldTmax) + '.' + String(int(oldTmax * 10) % 10);
    drawText(str, 1, BLACK, 0, 56);
    oldTmax = minMaxClimates[isGrowFlower][1][isLampOn][0];
    str = String((int)oldTmax) + '.' + String(int(oldTmax * 10) % 10);
    drawText(str, 1, GREEN & BLUE, 0, 56);

    str = String((int)oldHmin) + '.' + String(int(oldHmin * 10) % 10);
    drawText(str, 1, BLACK, 72, 48);
    oldHmin = minMaxClimates[isGrowFlower][0][isLampOn][1];
    str = String((int)oldHmin) + '.' + String(int(oldHmin * 10) % 10);
    drawText(str, 1, GREEN & BLUE, 72, 48);
    str = String((int)oldHmax) + '.' + String(int(oldHmax * 10) % 10);
    drawText(str, 1, BLACK, 72, 56);
    oldHmax = minMaxClimates[isGrowFlower][1][isLampOn][1];
    str = String((int)oldHmax) + '.' + String(int(oldHmax * 10) % 10);
    drawText(str, 1, GREEN & BLUE, 72, 56);
  }
  */
}

void digitalClockDisplay(time_t t, uint16_t color) {
  time_t h = t / 3600;
  time_t m = (t - h * 3600) / 60;
  digitalClockDisplay(h, m, color);  
}

void digitalClockDisplay(time_t h, time_t m, uint16_t color) {
  // digital clock display of the time
  display.setCursor(24, 40);
  display.setTextColor(color);
  if (h < 10)
    display.print('0');
  display.print(h);
  if (m < 10)
    display.print('0');
  display.print(m);
}

void makeCooler() {
  if (isHeatOn) {
    isHeatOn = false;
  } else if (!isCtOn && isLampOn) {
    isCtOn = true;
//  } else if (!isFanOn) {
//    isFanOn = true;  
  } else if (!isAcOn) {
    isAcOn = true;
  }
}

void makeHotter() {
  if (isAcOn) {
    isAcOn = false;
  } else if (isCtOn && isLampOn) {
    isCtOn = false;
  } else if (!isHeatOn) {
    isHeatOn = true;
  }
}

void setRelays() {
  static String strOldDisp = "";
  String strDisp;

  strDisp = String("A") + switchWhenChanging(isAcOn, &wasAcOn, AC_PIN);
  strDisp += String(" C") + switchWhenChanging(isCtOn, &wasCtOn, CT_PIN);
  strDisp += String(" H") + switchWhenChanging(isHeatOn, &wasHeatOn, HT_PIN);
  strDisp += String(" F") + switchWhenChanging(isFanOn, &wasFanOn, FN_PIN);
  
  if (!strOldDisp.equals(strDisp)) {
    drawText(strOldDisp, 1, BLACK, 0, 7);
    drawText(strDisp, 1, RED | BLUE, 0, 7);

    strOldDisp = strDisp;
    delayCC = now() + DELAY_CC;
  }
}

String switchWhenChanging(bool is, bool *was, uint8_t pin) {
  if (now() > delayCC) {
    if (*was != is) {
      digitalWrite(pin, *was);
      *was = is;
    }
  }
  return String(*was) + String(is);
}

void displaySensor(uint8_t location, uint8_t sensor) {
  if (arraySensors[location][0][sensor] != arraySensors[location][1][sensor]) {
    drawText(String((int)arraySensors[location][0][sensor]) + "." + String(int(arraySensors[location][0][sensor] * 10) % 10),
             2, BLACK, displayDHT[location * 2 + sensor].column, displayDHT[location * 2 + sensor].row);
    drawText(String((int)arraySensors[location][1][sensor]) + "." + String(int(arraySensors[location][1][sensor] * 10) % 10),
             2, displayDHT[location * 2 + sensor].color, displayDHT[location * 2 + sensor].column, displayDHT[location * 2 + sensor].row);
    arraySensors[location][0][sensor] = arraySensors[location][1][sensor];
//    Serial.println("c=" + String(displayDHT[location + sensor * 2].column) + ", r=" + String(displayDHT[location + sensor * 2].row));
  }
}

void readAndDisplayDHT22s() {
  uint8_t sensor = 0;

  for (uint8_t location = 0; location < 2; location++) { // inside 0 or outside 1 

    arraySensors[location][1][0] = dhtSensors[location].readTemperature();
    if (arraySensors[location][1][0] == 0) {
      arraySensors[location][1][0] = arraySensors[location][0][0];
    }
    arraySensors[location][1][1] = dhtSensors[location].readHumidity();
    if (arraySensors[location][1][1] == 0) {
      arraySensors[location][1][1] = arraySensors[location][0][1];
    }

    displaySensor(location, sensor);
  }
}

void drawText(String text, uint8_t f, uint8_t r, uint8_t g, uint8_t b, uint8_t x, uint8_t y) {
  drawText(text, f, display.Color565(r, g, b), x, y);
}

void drawText(String text, uint8_t f, uint16_t color, uint8_t x, uint8_t y) {
  // draws color text, size f, at x, y, relative to font size
  display.setTextScale(f);
  display.setTextColor(color);
  display.setCursor(x * 6, y * 8);
//  display.fillRect(x * f * 6, y * f * 8 , 48, 16, BLACK);
  display.print(text);
}

bool processSyncMessage() {
  bool testMode = false;
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
          //Serial.println(pctime);
        }
        if (dateFormat || c == ':' || c == ' ') {
          //Serial.println("\" :\" or dateFormat");
          dateFormat = true;
          if (c == ':' || c == ' ' || Serial.available() == 0) {
            //Serial.println("\" :\"");
            if (h == 0) {
              h = pctime;
              pctime = 0;
              //Serial.println("set hours");
            } else if (t == 0) {
              t = pctime + 1;
              pctime = 0;
              //Serial.println("set mins");
            } else if (s == 0) {
              s = pctime + 1;
              pctime = 0;
              //Serial.println("set secs");
            } else if (d == 0) {
              d = pctime;
              pctime = 0;
              //Serial.println("set days");
            } else if (m == 0) {
              m = pctime;
              pctime = 0;
              //Serial.println("set months");
            } else {
              y = pctime;
              //Serial.println("set years");
            }
            if (Serial.available() == 0) {
              setTime(h, t == 0 ? 0 : t - 1, s == 0 ? 0 : s - 1, d == 0 ? day() : d, m == 0 ? month() : m, y == 0 ? year() : y);
              Serial.print("new time: ");
              serialClockDisplay();
              pctime = now();
            }
          }
        }
        
        if (c == 'T') {
          Serial.print("Temp = ");
          Serial.println(pctime);
          testMode = true;
          arraySensors[0][1][0] = pctime; // [inside][new][temp]
          pctime = now();
        }
        
        if (c == 'H') {
          Serial.print("Humidity = ");
          Serial.println(pctime);
          testMode = true;
          arraySensors[0][1][1] = pctime; // [inside][new][humidity]
          pctime = now();
        }

        if (c == 'F') {
          Serial.println("Test mode exit");
          testMode = false;
          pctime = now();
        }
      }
      setTime(pctime); // Sync Arduino clock to the time received on the serial port
    }
  }

  return testMode;
}

void serialClockDisplay() {
  serialClockDisplay(hour(), minute(), second());
}

void serialClockDisplay(time_t t) {
  time_t h = t / 3600;
  time_t m = (t - h * 3600) / 60;
  time_t s = (t - h * 3600) % 60;
  serialClockDisplay(h, m, s);  
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
