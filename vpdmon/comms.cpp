/***************************************************
   VPDMon

   climate controller for indoor grow room

   (C) C Stott 2018

 ****************************************************/
#include "vpdmon.h"

bool have_time = false;
uint8_t wifiErrors = 0;

// A UDP instance to let us send and receive packets over UDP
const uint8_t NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
const int UDP_TIMEOUT = 2000;    // timeout in miliseconds to wait for an UDP packet to arrive
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
int status = WL_IDLE_STATUS;     // the Wifi radio's status

char timeServer[] = "time.nist.gov";  // NTP server
unsigned int localPort = 2390;        // local port to listen for UDP packets
WiFiEspUDP Udp;
WiFiEspClient client;

void wifiCheckTime() {
  wifiGetTime();

  if (wifiErrors > 0) {
    if (wifiErrors > 30) {
      resetDevice();  
    } else {
      wifiRestart();
    }
  }

  gtim = now() - TIMEZONE * 3600;

  time_t timeOfDay = now() % 86400;
  isLampOn = sunRise < timeOfDay && timeOfDay < sunSet;
  displayTime();
}

void wifiStart() {
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
    if (!have_time) {
      if (WATCHDOG) wdt_disable();
      delay(30000);
      if (WATCHDOG) {
        wdt_reset();
        wdt_enable(WDTO_8S);
      }
    }
  } while (!have_time);

  // init delay
  if (delayCC == 0) {
    debug_msg("Init climate change delay");
    delayCC = now();
  }
}

void resetDevice() {
  wdt_enable(WDTO_15MS);
  while(true);
}

void wifiRestart() {
  static uint8_t count = 0;
  if (count++ > 10) {
    WiFi.disconnect();
    msg("*** reset ESP");
    Udp.stop();
    espDrv.reset();
    wifiStart();
    if (wifiErrors == 0) {
      count = 0;
    }
  } else {
    resetDevice();
  }
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
    if (WATCHDOG) wdt_reset();
    status = WiFi.begin(ssid, pass);
  }
}

void wifiGetTime() {
  do {
    wifiGetTime(false);
  } while (!have_time && (wifiErrors++ % 6) < 5);
}

#if 1
void wifiGetTime(bool force) {
  have_time = false;
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  
  // wait for a reply for UDP_TIMEOUT miliseconds
  unsigned long startMs = millis();
  while (!Udp.available() && (millis() - startMs) < UDP_TIMEOUT) {}

  debug_msg(Udp.parsePacket());
  if (Udp.parsePacket()) {
    ndebug_msg("packet received");
    // We've received a packet, read the data from it into the buffer
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    ndebug_msg_pre("Seconds since Jan 1 1900 = ");
    ndebug_msg(secsSince1900);

    // now convert NTP time into everyday time:
    ndebug_msg_pre("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    ndebug_msg(epoch);

    setTime(epoch + TIMEZONE * 3600);
    have_time = true;
    wifiErrors = 0;
  }
}
#else
void wifiGetTime(bool force) {
  static time_t t = 0;
  uint8_t count = 0;
  short pkt_s = 0, pkt_r = 0;
  unsigned long startMs = millis();

  ndebug_msg("WifiGetTime()");

  if (force || now() - t >= 300) {
    t = 0;
    have_time = false;
    sendNTPpacket(timeServer);
    // wait for a reply for UDP_TIMEOUT miliseconds

    ndebug_msg("wait for reply");
    while (!Udp.available() && (millis() - startMs) < UDP_TIMEOUT);

    ndebug_msg("parse reply");

    if (Udp.parsePacket()) {
      // We've received a packet, read the data from it
      pkt_r = Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
      smsg_pre("Packet read: ");
      smsg(pkt_r);
      if (WATCHDOG) wdt_reset();
      if (pkt_r == NTP_PACKET_SIZE) {
        //the timestamp starts at byte 40 of the received packet and is four bytes,
        // or two words, long. First, esxtract the two words:
        uint16_t highWord = word(packetBuffer[40], packetBuffer[41]);
        uint16_t lowWord = word(packetBuffer[42], packetBuffer[43]);

        // combine the four bytes (two words) into a long integer
        // this is NTP time (seconds since Jan 1 1900):
        t = highWord;
        t = t << 16 | lowWord;
        smsg_pre("ntp time: ");
        smsg(t);

        if (t > 0) {
          // now convert NTP time into everyday time:
          // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
          t -= 2208988800UL;
          // adjust timezone
          t += TIMEZONE * 3600;
          // print Unix time:
          smsg_pre("Unix epoch time: ");
          smsg(t);

          setTime(t);
          displayTime(t, true);
          have_time = true;
        }
      }
    }
  }
  if (t > 0) {
    gtim = now() - TIMEZONE * 3600;
  
    time_t timeOfDay = now() % 86400;
    isLampOn = sunRise < timeOfDay && timeOfDay < sunSet;
    
    displayTime();
  }
}
#endif

#if 1
// send an NTP request to the time server at the given address
void sendNTPpacket(char *ntpSrv)
{
  ndebug_msg(">sendNTPpacket");
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
  Udp.beginPacket(ntpSrv, 123); //NTP requests are to port 123

  Udp.write(packetBuffer, NTP_PACKET_SIZE);

  Udp.endPacket();
  ndebug_msg("<sendNTPpacket");
}
#else
// send an NTP request to the time server at the given address
bool sendNTPpacket(char *ntpSrv) {
  int u_b = 0, u_w = 0, u_e = 0;
  bool ret = true;

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
  if (u_b == 1) { 
    debug_msg_pre("write packet: ");
    u_w = Udp.write(packetBuffer, NTP_PACKET_SIZE);
    debug_msg(u_w);

    if (u_w != NTP_PACKET_SIZE) {
      msg("Error writing UDP packet");
      ret = false;
    }
  } else {
    msg("Error opening UDP port");
    ret = false;
  }
  debug_msg_pre("end packet: ");
  u_e = Udp.endPacket();
  debug_msg(u_e);
  if (u_e == 0) {
    msg("Error nothing sent");
    ret = false;
  }

  return ret;
}
#endif
