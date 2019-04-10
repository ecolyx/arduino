/***************************************************
   VPDMon

   climate controller for indoor grow room

   (C) C Stott 2018

 ****************************************************/
#include "vpdmon.h"

bool testMode = false;
bool have_time = false;
bool restartRequired = false;

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
    resetDevice();
    while (true);
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
  } while (!have_time);

  // init delay
  if (delayCC == 0) {
    smsg("Init climate change delay");
    delayCC = now();
  }
}

void resetDevice() {
  wdt_reset();
  wdt_enable(WDTO_8S);
}

void wifiRestart() {
  restartRequired = true;
}

void wifiRestart(bool reset) {
  WiFi.disconnect();
  msg("*** reset ESP");
  espDrv.reset();
  wifiStart();
  restartRequired = false;
  have_time = false;
  //  resetDevice();
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
  static uint8_t errors = 0;
  uint8_t count = 0;

  debug_msg("WifiGetTime()");

  if (force || now() - t >= 300) {
    debug_msg("send packet");
    sendNTPpacket(timeServer, true); // send an NTP packet to a time server

    // wait for a reply for UDP_TIMEOUT miliseconds
    unsigned long startMs = millis();
    debug_msg("wait for reply");
    while (!Udp.available() && (millis() - startMs) < UDP_TIMEOUT) {
      if (WATCHDOG) wdt_reset();
      //      delay(2000);
    }

    if ((millis() - startMs) < UDP_TIMEOUT) {
      debug_msg("parse reply");
      while (count < 30) {
        short pkt_s = 0, pkt_r = 0;
        if (pkt_s = Udp.parsePacket()) {
          // We've received a packet, read the data from it
          smsg("Get ntp time");
          pkt_r = Udp.read(packetBuffer, pkt_s); // read the packet into the buffer

          if (WATCHDOG) wdt_reset();
          //        graphiteMetric("udp.parsePacket", pkt_s);
          //        graphiteMetric("udp.read", pkt_r);

          //the timestamp starts at byte 40 of the received packet and is four bytes,
          // or two words, long. First, esxtract the two words:
          uint16_t highWord = word(packetBuffer[40], packetBuffer[41]);
          uint16_t lowWord = word(packetBuffer[42], packetBuffer[43]);
          debug_msg_pre("highWord:");
          debug_msg(highWord);
          debug_msg_pre("lowWord:");
          debug_msg(lowWord);

          // combine the four bytes (two words) into a long integer
          // this is NTP time (seconds since Jan 1 1900):
          t = highWord;
          t = t << 16 | lowWord;
          smsg_pre("ntp: s > 1/1/1900: ");
          smsg(t);

          if (t > 0) {
            // now convert NTP time into everyday time:
            // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
            const unsigned long seventyYears = 2208988800UL;
            // subtract seventy years:
            t = t - seventyYears;
            // adjust timezone
            t = t + TIMEZONE * 3600;
            // print Unix time:
            smsg_pre("Unix epoch time: ");
            smsg(t);

            setTime(t);
            serialClockDisplay(t);

            have_time = true;
            errors = 0;
            break;
          } else {
            smsg("time is 0, not setting");
            if (++errors > 30) {
              smsg("nist not replying");
              msg("***wifiRestart");
              errors = 0;
              wifiRestart();
            }
            break;
          }
        } else {
          debug_msg_pre("waiting:");
          count++;
          debug_msg(count);
          if (0 == count % 10) {
            Serial.print("udp.error.nopkt");
          }
          if (count >= 30) {
            Serial.print("udp.error.restart");
            msg("***wifiRestart");
            wifiRestart();
            break;
          }
          delay(1000);
        }
      }
    } else { // UDP_TIMEOUT
      smsg("UDP Timeout reached: restarting");
      wifiRestart();
      //wdt_reset();
      //wdt_enable(WDTO_8S);
    }
    Udp.flush();
    Udp.stop();
  }
  gtim = now() - TIMEZONE * 3600;
  displayTime();
  if (restartRequired) {
    wifiRestart(true);
  }
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
  if (WATCHDOG) wdt_reset();
  debug_msg_pre("begin packet: ");
  u_b = Udp.beginPacket(ntpSrv, 123);
  debug_msg(u_b);
  if (u_b == 1) { //NTP requests are to port 123
    debug_msg("write packet");
    u_w = Udp.write(packetBuffer, NTP_PACKET_SIZE);
    debug_msg("end packet");
    u_e = Udp.endPacket();
    debug_msg("sending graphite write/end");
//    graphiteMetric("udp.write", u_w);
//    graphiteMetric("udp.endpacket", u_e);

    debug_msg("checking packet size");
    if (NTP_PACKET_SIZE != u_w) {
      msg("Error writing UDP packet");
      wifiRestart(true);
    }
    debug_msg("checking for errors");
    if (u_e == 0) {
      msg("Error nothing sent");
    }
  } else {
    msg("Error opening UDP port");
    wifiRestart(true);
  }
}
