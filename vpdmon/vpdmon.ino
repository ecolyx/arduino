/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 * 
 * (C) C Stott 2018
 * 
 ****************************************************/
#define setTextScale setTextSize
//#define debug_msg(x) Serial.println(x)
//#define debug_msg_pre(x) Serial.print(x)
//#define debug_msg_pre_h(x) Serial.print(x,HEX)
#define debug_msg(x)
#define debug_msg_pre(x)
#define debug_msg_pre_h(x)
#define smsg(x) Serial.println(x)
#define smsg_pre(x) Serial.print(x)
#define smsg_pre_h(x) Serial.print(x,HEX)

// relay pins
#define AC_PIN          A0 // aircon
#define CT_PIN          A3 // cooltube
#define HT_PIN          A2 // heater
#define FN_PIN          A1 // fan (extractor)

#if 0
// switch pins
#define SW1_PIN         (uint8_t)0
#define SW2_PIN         (uint8_t)1
#endif

// delay period for Climate control after change in settings to prevent over rapid switching when transitioning
// air con likes at least 5 minutes between cycles, so seems a reasonable rule to apply
#define DELAY_CC_DEFAULT  300 // 5 mins
#define DELAY_CC_AC       600
uint16_t DELAY_CC = DELAY_CC_DEFAULT;

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
#define TIMEZONE          -1

#define DHT_PIN1          4
#define DHT_PIN2          5

#define GROWMINPERIOD     50400

#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <DHT.h>
#include <SoftwareSerial.h>

struct s_DisplayDHT {
  uint16_t color;
  uint8_t column;
  uint8_t row;
} displayDHT[2][2] = {{{GREEN, 0, 0},{BLUE, 10, 0}},{{GREEN, 0, 3},{BLUE, 10, 3}}};

TFT_ILI9163C display = TFT_ILI9163C(10, 9);

// Construct DHT objects
DHT dhtSensors[2] = {DHT(DHT_PIN1, DHT11),DHT(DHT_PIN2, DHT22)};
float arraySensors[2][2][2]; // old/new, in/out, t/h
#define   A_OLD   (uint8_t)0
#define   A_NEW   (uint8_t)1
#define   A_IN    (uint8_t)0
#define   A_OUT   (uint8_t)1
#define   A_TEMP  (uint8_t)0
#define   A_HUMID (uint8_t)1
 
float minMaxClimates[2][2][2][2] = {{{{16, 30},{22, 30}}, {{20, 50},{26, 50}}},{{{19, 30},{25, 40}}, {{23, 70},{29, 70}}}}; // [G/F][Min/Max][D/N][T/H]);

#define   A_GROW    true
#define   A_FLOWER  false
#define   A_DAY     true
#define   A_NIGHT   false
#define   A_MAX     (uint8_t)1
#define   A_MIN     (uint8_t)0

#define DHTIN   dhtSensors[0]
#define DHTOUT  dhtSensors[1]

bool testMode = false;
bool changeClimatePending = false;
bool isAcOn = false, isCtOn = false, isHeatOn = false, isFanOn = false;
bool wasFanOn = false, wasCtOn = false, wasHeatOn = false, wasAcOn = false;
bool isLampOn = A_NIGHT, isGrowSeason = A_FLOWER;
//const time_t sunRise = 14400, sunSet = 79200; // grow
const time_t sunRise = 14400, sunSet = 57600; // flower
static time_t delayCC = 0;
volatile uint8_t loopMode = 0;
const uint16_t heartbeat = 30000; // milliseconds looping

SoftwareSerial Serial1(6, 7); // RX, TX

// A UDP instance to let us send and receive packets over UDP
const uint8_t NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
const int UDP_TIMEOUT = 2000;    // timeout in miliseconds to wait for an UDP packet to arrive
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
int status = WL_IDLE_STATUS;     // the Wifi radio's status

char timeServer[] = "time.nist.gov";  // NTP server
unsigned int localPort = 2390;        // local port to listen for UDP packets
WiFiEspUDP Udp;

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

  DHTIN.begin();
  DHTOUT.begin();

  setTime(sunRise);
  isGrowSeason = GROWMINPERIOD < sunSet - sunRise;
  isLampOn = true;
  loopMode = 0;
  smsg_pre("Sunset: ");
  serialClockDisplay(sunSet);
  smsg_pre("Sunrise: ");
  serialClockDisplay(sunRise);
  smsg_pre("grow - on: ");
  smsg_pre(minMaxClimates[A_GROW][A_MIN][A_DAY][A_TEMP]);//[G/F][Min/Max][D/N][T/H]
  smsg_pre("- ");
  smsg_pre(minMaxClimates[A_GROW][A_MAX][A_DAY][A_TEMP]);
  smsg_pre(", off: ");
  smsg_pre(minMaxClimates[A_GROW][A_MIN][A_NIGHT][A_TEMP]);
  smsg_pre(" - ");
  smsg(minMaxClimates[A_GROW][A_MAX][A_NIGHT][A_TEMP]);
  smsg_pre("flower - on: ");
  smsg_pre(minMaxClimates[A_FLOWER][A_MIN][A_DAY][A_TEMP]);
  smsg_pre(" - ");
  smsg_pre(minMaxClimates[A_FLOWER][A_MAX][A_DAY][A_TEMP]);
  smsg_pre(", off: ");
  smsg_pre(minMaxClimates[A_FLOWER][A_MIN][A_NIGHT][A_TEMP]);
  smsg_pre(" - ");
  smsg(minMaxClimates[A_FLOWER][A_MAX][A_NIGHT][A_TEMP]);
  display.clearScreen();
  wifiStart();
}

void wifiStart() {
  // initialize serial for ESP module
  Serial1.begin(9600);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("No WiFi");
    // don't continue
  } else {
    wifiConnect();
  }

  Udp.begin(localPort);
}

void wifiRestart() {
  Udp.stop();
  wifiStart(); 
}

void displayReset() {
  display.begin();
  display.setRotation(1);
  display.clearScreen();
}

void loop() {
  static uint8_t loops = 0;
  time_t loop_delay = 1000;
  time_t tim = millis();
  
  wifiGetTime();

  // check for input on console
  if (Serial.available()) {
    processSyncMessage();
  }

  readAndDisplayDHT22s();

  debug_msg_pre("loopMode = ");
  debug_msg(loopMode);
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
  if (!changeClimatePending) {
    if (arraySensors[A_NEW][A_IN][A_TEMP] < minMaxClimates[isGrowSeason][A_MIN][isLampOn][A_TEMP]) { // inside/new/temp < [G/F][Min/Max][D/N][T/H]
      makeHotter();
      debug_msg("makeHotter");
    } else if (arraySensors[A_NEW][A_IN][A_TEMP] > minMaxClimates[isGrowSeason][A_MAX][isLampOn][A_TEMP]) { // inside/new/temp > season/max/light/temp
      makeCooler();
      debug_msg("makeCooler");
    } else {
      // everything is fine
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

  // might be useful for VPD
  //  hicRoomOld = hicRoom;
  //  hicRoom = roomSensor.computeHeatIndex(tRoom, hRoom, false);
  //  hicOutOld = hicOut;
  //  hicOut = outsideSensor.computeHeatIndex(tOut, hOut, false);

  setRelays();
  loop_delay = heartbeat;
  smsg(); // blank line on serial

  time_t timeOfDay = now() % 86400;
  isLampOn = sunRise < timeOfDay && timeOfDay < sunSet;
  debug_msg_pre("Lamp: ");
  debug_msg(isLampOn ? "On" : "Off");
  debug_msg(isGrowSeason ? "Grow" : "Flower");
  displayTime();
  delay(loop_delay + tim - millis()); // regulates the loop by excluding the time to run loop code
}

void wifiGetTime() {
  static time_t t = 0;
  uint8_t count = 0;

  if (now() - t >= 10) {
    sendNTPpacket(timeServer); // send an NTP packet to a time server
  
    // wait for a reply for UDP_TIMEOUT miliseconds
    unsigned long startMs = millis();
    while (!Udp.available() && (millis() - startMs) < UDP_TIMEOUT) {}
  
    while (count < 30) {
      if (Udp.parsePacket()) {
        debug_msg("pkt rcvd");
        // We've received a packet, read the data from it
        Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  
        //the timestamp starts at byte 40 of the received packet and is four bytes,
        // or two words, long. First, esxtract the two words:
        uint16_t highWord = word(packetBuffer[40], packetBuffer[41]);
        uint16_t lowWord = word(packetBuffer[42], packetBuffer[43]);
        // combine the four bytes (two words) into a long integer
        // this is NTP time (seconds since Jan 1 1900):
        t = highWord;
        t = t << 16 | lowWord;
        debug_msg_pre("Secs since 1/1/1900 = ");
        debug_msg(t);
    
        // now convert NTP time into everyday time:
        debug_msg_pre("Unix time = ");
        // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
        const unsigned long seventyYears = 2208988800UL;
        // subtract seventy years:
        t = t - seventyYears;
        // adjust timezone
        t = t + TIMEZONE * 3600;
        // print Unix time:
        debug_msg(t);
    
        setTime(t);
        count = 30;
      } else {
        debug_msg_pre("waiting:");
        count++;
        debug_msg(count);
        if (count == 30) {
          smsg("Udp timeout: restart socket");
          //wifiRestart();
          Udp.stop();
          Udp.begin(localPort);
        }
        delay(1000);
      }
    }
  }
}

// send an NTP request to the time server at the given address
void sendNTPpacket(char *ntpSrv)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)

  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  debug_msg_pre("Udp:beginPacket to ");
  debug_msg(ntpSrv);
  Udp.beginPacket(ntpSrv, 123); //NTP requests are to port 123

//  debug_msg_pre("Udp:write pkt ");
//  debug_msg(NTP_PACKET_SIZE);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);

//  debug_msg("Udp:endPacket");
  Udp.endPacket();
  debug_msg("pkt sent");
}

void wifiConnect() {
  char ssid[] = "HereBeDragons";            // your network SSID (name)
  char pass[] = "Sh33laZ1ggy";        // your network password
  int status = WL_IDLE_STATUS;     // the Wifi radio's status

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    debug_msg_pre("Connect to WPA: ");
    debug_msg(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
}

void displayTime() {
  static time_t lastTime = now() % 86400;
  static float oldTmin = 0, oldTmax = 0;
  static float oldHmin = 0, oldHmax = 0;

  lastTime = now() % 86400;
  serialClockDisplay(lastTime);

  digitalClockDisplay(lastTime, ORANGE);

}

void digitalClockDisplay(time_t t, uint16_t color) {
  digitalClockDisplay(t, color, 24, 40);  
}

void digitalClockDisplay(time_t t, uint16_t color, uint8_t x, uint8_t y) {
  const time_t HR = 3600;
  uint8_t h = t / HR;
  uint8_t m = (t - h * HR) / 60;
  digitalClockDisplay(h, m, color, x, y, false);  
}

void digitalClockDisplay(uint8_t h, uint8_t m, uint16_t color) {
  digitalClockDisplay(h, m, color, 24, 40, true);
}

void digitalClockDisplay(uint8_t h, uint8_t m, uint16_t color, uint8_t x, uint8_t y, bool remember) {
  // digital clock display of the time
  static int old_h = 0, old_m = 0;
  if (!remember || h != old_h || m != old_m) {
    char buf[6];
    display.fillRect(x, y, 70, 16, BLACK);
    display.setCursor(x, y);
    display.setTextScale(2);
//    display.setTextColor(BLACK);
//    sprintf(buf, "%02d:%02d", old_h, old_m);
//    display.print(buf);
    display.setTextColor(color);
    display.setCursor(x, y);
    sprintf(buf, "%02d:%02d", h, m);
    display.print(buf);
    old_h = h;
    old_m = m;
  }
  
  if (remember) {
    old_h = h;
    old_m = m;
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
  } else if (isCtOn) {
    isCtOn = false;
  } else {
    isHeatOn = true;
  }
}

void setRelays() {
  static String strOldDisp = "";
  String strDisp;

  strDisp = String("A") + switchOrWait(isAcOn, &wasAcOn, AC_PIN);
  strDisp += String(" C") + switchOrWait(isCtOn, &wasCtOn, CT_PIN);
  strDisp += String(" H") + switchOrWait(isHeatOn, &wasHeatOn, HT_PIN);
  strDisp += String(" F") + switchOrWait(isFanOn, &wasFanOn, FN_PIN);
  debug_msg(strDisp);
  
  if (!strOldDisp.equals(strDisp)) {
    const uint8_t x = 0, y = 15;
    drawText(strOldDisp, (uint16_t)BLACK, x, y);
    drawText(strDisp, (uint16_t)RED | BLUE, x, y);

    strOldDisp = strDisp;
    delayCC = now() + DELAY_CC;
  }
}

String switchOrWait(bool new_v, bool *old_v, uint8_t pin) {
  if (*old_v != new_v) {
    signed int wait = delayCC - now();
    if (wait <= 0) {
      digitalWrite(pin, !new_v);
      *old_v = new_v;
      changeClimatePending = false;
    } else {
      debug_msg_pre("Wait: ");
      debug_msg(wait);
      changeClimatePending = true;
    }
  }
  return String(new_v) + String(*old_v);
}

void drawText(String text, uint8_t f, uint8_t r, uint8_t g, uint8_t b, uint8_t x, uint8_t y) {
  drawText(text, f, display.Color565(r, g, b), x, y);
}

void drawText(String text, uint16_t c, uint8_t x, uint8_t y) {
  drawText(text, (uint8_t)1, c, x, y);
}

void drawText(String text, uint8_t f, uint16_t color, uint8_t x, uint8_t y) {
  // draws color text, size f, at x, y, relative to font size
  display.setTextScale(f);
  display.setTextColor(color);
  display.setCursor(x * 6, y * 8);
  display.print(text);
}

bool processSyncMessage() {
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

void displaySensor(uint8_t location, uint8_t sensor) {
  smsg_pre(0 == location ? "In " : "Out ");
  smsg_pre(arraySensors[A_NEW][location][sensor]);
  smsg(0 != sensor ? "%" : "");
  if (arraySensors[A_OLD][location][sensor] != arraySensors[A_NEW][location][sensor]) {
    drawText(stringFromFloat(arraySensors[A_OLD][location][sensor]), 2, BLACK, displayDHT[location][sensor].column, displayDHT[location][sensor].row);
    drawText(stringFromFloat(arraySensors[A_NEW][location][sensor]), 2, displayDHT[location][sensor].color, 
                                                                        displayDHT[location][sensor].column, displayDHT[location][sensor].row);
    arraySensors[A_OLD][location][sensor] = arraySensors[A_NEW][location][sensor];
    //debug_msg("c=" + String(displayDHT[location + sensor * 2].column) + ", r=" + String(displayDHT[location + sensor * 2].row));
  }
}

String stringFromFloat(float f) {
  String ret = String(int(f)) + String(".") + String(int(f * 10) % 10);
  return ret;
}

void readAndDisplayDHT22s() {
  uint8_t sensor = 0;

  for (uint8_t location = 0; location < 2; location++) { // inside 0 or outside 1 
    float temp = 0;
    temp = dhtSensors[location].readTemperature();
    if (temp != 0) { // ignore possible false values, if temp is actually 0, room is in crisis anyway
      debug_msg_pre(0 == location ? "In" : "Out");
      debug_msg_pre(" DHT t = ");
      debug_msg(temp);
      if (!testMode) {
        if (temp != 0) {
          arraySensors[A_OLD][location][A_TEMP] = arraySensors[A_NEW][location][A_TEMP];
          arraySensors[A_NEW][location][A_TEMP] = temp;

          float humid = dhtSensors[location].readHumidity();
          if (humid != 0) {
            arraySensors[A_OLD][location][A_HUMID] = arraySensors[A_NEW][location][A_HUMID];
            arraySensors[A_NEW][location][A_HUMID] = humid;
            
            debug_msg_pre(0 == location ? "In" : "Out");
            debug_msg(" DHT  = " + stringFromFloat(humid));
          }
        }
      }
    }
    displaySensor(location, A_TEMP);
    displaySensor(location, A_HUMID);
  }
}
