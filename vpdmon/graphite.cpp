/***************************************************
   VPDMon

   climate controller for indoor grow room

   (C) C Stott 2018

 ****************************************************/
#include "vpdmon.h"

uint64_t gtim; // buffer for graphite epoch time, keeps stats in sync each loop

void graphiteMetric(char *m, float v) {
  char buf[10];
  String(v, 2).toCharArray(buf, 10);
  graphiteMetric(m, buf);
}

void graphiteMetric(char *m, int v) {
  char buf[10];
  graphiteMetric(m, itoa(v, buf, 10));
}

void graphiteMetric(char *ma, float v, uint8_t t, bool inOut) {
  char buf[32];
  char buf2[8];

  if (inOut) {
    strcpy(buf, t == 0 ? "in." : "out.");
  } else {
    strcpy(buf, "bme");
    strcat(buf, t == 0 ? "0." : "1.");
  }
  strcat(buf, ma);

  graphiteMetric(buf, v);
}

void graphiteMetric(char *m, char *v) {
  char buf[100] = "pub.luke.roomf.";
  char buf2[12];
  strcat(buf, m);
  strcat(buf, " ");
  strcat(buf, v);
  strcat(buf, " ");
  strcat(buf, ltoa(gtim, buf2, 10));
  strcat(buf, "\n");

  if (have_time) {
    if (WATCHDOG) wdt_reset();
    signed int cc = client.connect("ec2-13-54-106-144.ap-southeast-2.compute.amazonaws.com", 2003); //Try to connect to TCP Server
    ndebug_msg_pre("client.connect: ");
    ndebug_msg(cc);

    if (cc != 0) {
      if (WATCHDOG) wdt_reset();
      int w = client.write(buf, strlen(buf));
      ndebug_msg_pre("client.write: ");
      ndebug_msg(w);

      if (strlen(buf) != w) {
        smsg("client.write error");
      } else {
        smsg_pre((char *)buf);
      }
    } else {
      smsg("client.connect error");
    }
    if (WATCHDOG) wdt_reset();
    client.flush();
    if (WATCHDOG) wdt_reset();
    client.stop();
    if (WATCHDOG) wdt_reset();
  } else {
    smsg_pre(m);
    smsg(" not sent, *time not set");
  }
}
