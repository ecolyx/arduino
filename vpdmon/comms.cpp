/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 * 
 * (C) C Stott 2018
 * 
 ****************************************************/
#include "vpdmon.h"

bool testMode = false;
bool have_time = false;

// A UDP instance to let us send and receive packets over UDP
const uint8_t NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
const int UDP_TIMEOUT = 2000;    // timeout in miliseconds to wait for an UDP packet to arrive
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
int status = WL_IDLE_STATUS;     // the Wifi radio's status

char timeServer[] = "time.nist.gov";  // NTP server
unsigned int localPort = 2390;        // local port to listen for UDP packets
WiFiEspUDP Udp;
WiFiEspClient client;
#define ESPBAUD   115200

void wifiStart() {
  // initialize serial for ESP module
  Serial1.begin(ESPBAUD);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    msg("No WiFi");
    while(true);
    // don't continue
  } else {
    wifiConnect();
  }

  debug_msg_pre("Upd.begin: ");
  debug_msg_pre(localPort);
  debug_msg_pre(" ->");
  int u = Udp.begin(localPort);
  debug_msg(u);
  do {
    wifiGetTime(true);
  } while(!have_time);

  // init delay
  if (delayCC == 0) {
    smsg("Init climate change delay");
    delayCC = now();
  }
}

void wifiRestart() {
  WiFi.disconnect();
  msg("*** reset ESP");
  espDrv.reset();
  wifiStart();
}

void wifiConnect() {
  char ssid[] = "HereBeDragons";            // your network SSID (name)
  char pass[] = "Sh33laZ1ggy";        // your network password
  int status = WL_IDLE_STATUS;     // the Wifi radio's status

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    msg_pre("Connect to WPA: ");
    msg(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
}

void wifiGetTime() {
  wifiGetTime(false);
}
  
void wifiGetTime(bool force) {
  static time_t t = 0;
  uint8_t count = 0;

  debug_msg("WifiGetTime()");

  if (force || now() - t >= 3600) {
    debug_msg("send packet");
    sendNTPpacket(timeServer, true); // send an NTP packet to a time server
  
    // wait for a reply for UDP_TIMEOUT miliseconds
    unsigned long startMs = millis();
    debug_msg("wait for reply");
    while (!Udp.available() && (millis() - startMs) < UDP_TIMEOUT) {
      wdt_reset();
    }

    debug_msg("parse reply");
    while (count < 30) {
      short pkt_s = 0, pkt_r = 0;
      if (pkt_s = Udp.parsePacket()) {
        // We've received a packet, read the data from it
        pkt_r = Udp.read(packetBuffer, pkt_s); // read the packet into the buffer

        wdt_reset();
        graphiteMetric("udp.parsePacket", pkt_s);
        graphiteMetric("udp.read", pkt_r);

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
        have_time = true;
        break;
      } else {
        debug_msg_pre("waiting:");
        count++;
        debug_msg(count);
        if (0 == count % 10) {
          graphiteMetric("udp.error.nopkt", 1);
        }
        if (count >= 30) {
          graphiteMetric("udp.error.restart", 2);
          msg("***wifiRestart");
          wifiRestart();
        }
        delay(1000);
      }
    }
  }
  displayTime();
}

// send an NTP request to the time server at the given address
void sendNTPpacket(char *ntpSrv, bool isNew) {
  int u_b = 0, u_w = 0, u_e = 0;

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
  wdt_reset();
  debug_msg_pre("begin packet: ");
  u_b = Udp.beginPacket(ntpSrv, 123);
  debug_msg(u_b);
  if (u_b == 1) { //NTP requests are to port 123
    debug_msg("write packet");
    u_w = Udp.write(packetBuffer, NTP_PACKET_SIZE);
    debug_msg("end packet");
    u_e = Udp.endPacket();
    debug_msg("sending graphite write/end");
    graphiteMetric("udp.write", u_w);
    graphiteMetric("udp.endpacket", u_e);

    debug_msg("checking packet size");
    if (NTP_PACKET_SIZE != u_w) {
      msg("Error writing UDP packet");
      wifiRestart();
    }
    debug_msg("checking for errors");
    if (u_e == 0) {
      msg("Error nothing sent");
    }
  } else {
    debug_msg("sending graphite begin");
    graphiteMetric("udp.beginpacket", u_b);
    msg("Error opening UDP port");
    wifiRestart();
  }
}

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
