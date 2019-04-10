/***************************************************
   VPDMon

   climate controller for indoor grow room

   (C) C Stott 2018

 ****************************************************/
#include "vpdmon.h"

uint64_t gtim; // buffer for graphite epoch time, keeps stats in sync


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
    signed int cc = client.connect("ec2-54-206-89-240.ap-southeast-2.compute.amazonaws.com", 2003); //Try to connect to TCP Server
    debug_msg_pre("client.connect: ");
    debug_msg(cc);

    if (cc != 0 && strlen(buf) != client.write(buf, strlen(buf))) {
      if (cc != 0) {
        smsg("TCP write error");
        wifiRestart();
      }

      smsg("client.write error");
    } else {
      smsg_pre((char *)buf);
    }
    client.flush();
    client.stop();
  } else {
    if (!have_time) {
      smsg("not sent, ***no time");
    }
  }
}
