/*************************************************** 
 * VPDMon
 * 
 * climate controller for indoor grow room
 * 
 * (C) C Stott 2018
 * 
 ****************************************************/
// delay period for Climate control after change in settings to prevent over rapid switching when transitioning
// air con likes at least 5 minutes between cycles, so seems a reasonable rule to apply
#define DELAY_CC_DEFAULT  300 // 5 mins
#define DELAY_CC_AC       600
// relay pins
int DELAY_CC = DELAY_CC_DEFAULT;
float minMaxClimates[2][2][2][2] = {{{{18, 30},{22, 30}}, {{22, 60},{28, 60}}},{{{20, 30},{24, 40}}, {{24, 70},{30, 70}}}}; // [G/F][Min/Max][D/N][T/H]);
bool climateChangePending = false;
bool isAcOn = false, isCtOn = false, isHeatOn = false, isFanOn = false;
bool wasFanOn = false, wasCtOn = false, wasHeatOn = false, wasAcOn = false;
bool isLampOn = A_NIGHT, isGrowSeason = A_FLOWER;
static time_t delayCC = 0;
