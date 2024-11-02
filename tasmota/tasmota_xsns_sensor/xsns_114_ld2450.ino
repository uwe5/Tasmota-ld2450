/*
  xsns_114_ld2450.ino - Driver for Presence and Movement sensor HLK-LD2450

  Copyright (C) 2024  Uwe Gummer (inspired by Nicolas Bernaerts)

  Connexions :
    * Tx GPIO should be declared as LD2450 Tx and connected to HLK-LD2450 Rx
    * Rx GPIO should be declared as LD2450 Rx and connected to HLK-LD2450 Tx

  Settings are stored using unused parameters :
    - Settings->rf_code[2][0] : Presence detection timeout (sec.)
    - Settings->rf_code[3][0] : zone 1 x1 detection point (x10cm)
    - Settings->rf_code[3][1] : zone 1 x2 detection point (x10cm)
    - Settings->rf_code[3][2] : zone 1 y1 detection point (x10cm)
    - Settings->rf_code[3][3] : zone 1 y2 detection point (x10cm)
    - Settings->rf_code[3][4] : zone 2 x1 detection point (x10cm)
    - Settings->rf_code[3][5] : zone 2 x2 detection point (x10cm)
    - Settings->rf_code[3][6] : zone 2 y1 detection point (x10cm)
    - Settings->rf_code[3][7] : zone 2 y2 detection point (x10cm)
...
    - Settings->rf_code[6][0] : zone 7 x1 detection point (x10cm)
    - Settings->rf_code[6][1] : zone 7 x2 detection point (x10cm)
    - Settings->rf_code[6][2] : zone 7 y1 detection point (x10cm)
    - Settings->rf_code[6][3] : zone 7 y2 detection point (x10cm)
    - Settings->rf_code[6][4] : zone 8 x1 detection point (x10cm)
    - Settings->rf_code[6][5] : zone 8 x2 detection point (x10cm)
    - Settings->rf_code[6][6] : zone 8 y1 detection point (x10cm)
    - Settings->rf_code[6][7] : zone 8 y2 detection point (x10cm)

                      x1,y1 ........x2,y1
                        .  detection  .
                        .    zone     .
                      x1,y2 ........x2,y2
  Settings zones:
      as an example a smiley zone :)
        ld2450_reset
        ld2450_zone 1,-1500, -500,   0,1500
        ld2450_zone 2,  500, 1500,   0,1500
        ld2450_zone 3,-2800,-1500,2800,3500
        ld2450_zone 4, 1500, 2800,2800,3500
        ld2450_zone 5,-1800, -800,3200,4200
        ld2450_zone 6,  800, 1800,3200,4200
        ld2450_zone 7,-1300, 1300,4000,5000                      

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
*/

#ifdef USE_LD2450

#include <TasmotaSerial.h>

/*************************************************\
 *               Variables
\*************************************************/

// declare teleinfo energy driver and sensor
#define XSNS_114 114

// constant
#define LD2450_START_DELAY 10             // sensor startup delay
#define LD2450_DEFAULT_TARGET_TIMEOUT 5   // timeout to trigger inactivity [sec]
#define LD2450_DEFAULT_SAMPLE 10          // number of samples to average
#define LD2450_DIST_MAX 6000              // default minimum detection distance [mm]

#define LD2450_TRACK_TIME 5   // publication timeout [sec]
#define LD2450_TRACK_OFF 0    // track targets off
#define LD2450_TRACK_IN 1     // track targets via MQTT in of the zones
#define LD2450_TRACK_OUT 2    // track targets via MQTT out of the zones
#define LD2450_TRACK_INOUT 3  // track targets via MQTT in/out of the zones

#define LD2450_TARGET_MAX 3
#define LD2450_MSG_SIZE_MAX 32
#define LD2450_ZONE_MAX 8

#define LD2450_COLOR_ABSENT   "none"
#define LD2450_COLOR_OUTRANGE "#555555"
#define LD2450_COLOR_PRESENT  "#0f76b0"
#define LD2450_COLOR_ZONE     "#008000"

// strings
const char D_LD2450_NAME[] PROGMEM = "HLK-LD2450";

// LD2450 serial commands
const uint8_t enableConfigCommands[] = {0xFF, 0x00, 0x01, 0x00};
const uint8_t endConfigCommands[] = {0xFE, 0x00};
const uint8_t rebootModule[] = {0xA3, 0x00};
const uint8_t enableBluetooth[] = {0xA4, 0x00, 0x01, 0x00};
const uint8_t disableBluetooth[] = {0xA4, 0x00, 0x00, 0x00};
const uint8_t singleTargetTracking[] = {0x80, 0x00};
const uint8_t multiTargetTracking[] = {0x90, 0x00};

// MQTT commands
const char kHLKLD2450Commands[] PROGMEM = "ld2450_|help|timeout|track|reset|zone|bluetooth|target";
void (*const HLKLD2450Command[])(void) PROGMEM = {&CmdLD2450Help, &CmdLD2450SensorTimeout, &CmdLD2450Track, &CmdLD2450Reset, &CmdLD2450Zone, &CmdLD2450Bluetooth, &CmdLD2450Target};

/****************************************\
 *                 Data
\****************************************/

// LD2450 received message
static struct {
  uint32_t timestamp = UINT32_MAX;        // timestamp of last received character
  uint8_t idx_body = 0;                   // index of received body
  uint8_t arr_body[LD2450_MSG_SIZE_MAX];  // body of current received message
  uint8_t arr_last[4] = {0, 0, 0, 0};     // last received characters
} ld2450_received;

// LD2450 configuration
struct {
  int16_t zone_x1[LD2450_ZONE_MAX] = {-LD2450_DIST_MAX,0,0,0};  // x1 detection point (-6000 -> 6000 mm)
  int16_t zone_x2[LD2450_ZONE_MAX] = {LD2450_DIST_MAX,0,0,0};   // x2 detection point (-6000 -> 6000 mm)
  int16_t zone_y1[LD2450_ZONE_MAX] = {0,0,0,0};                 // y1 detection point (0 -> 6000 mm)
  int16_t zone_y2[LD2450_ZONE_MAX] = {LD2450_DIST_MAX,0,0,0};   // y2 detection point (O -> 6000 mm)
  uint8_t tracktime = LD2450_TRACK_TIME;                        // default publication time
  uint8_t trackmode = LD2450_TRACK_IN;
  uint8_t sensortimeout = LD2450_DEFAULT_TARGET_TIMEOUT;        // timeout to trigger inactivity
} ld2450_config;

// LD2450 status
static struct {
  TasmotaSerial *pserial = nullptr;  // pointer to serial port
  bool enabled = false;              // driver is enabled
  uint8_t counter = 0;               // detected targets counter
  uint32_t pubtimestamp = 0;         // timestamp of last publication time
  uint32_t lastsensortime = 0;       // timestamp of last detection time
} ld2450_status;

// zone status
static struct {
  bool target_in_zone[LD2450_TARGET_MAX];  
  bool target_in_zone_old[LD2450_TARGET_MAX];  
} ld2450_zone[LD2450_ZONE_MAX];

// LD2450 detected targets
static struct {
  int16_t x;        // x coordonnate
  int16_t y;        // y coordonnate
  int16_t speed;    // speed
  uint16_t dist;    // gate size
  bool in_zone;     // target within detection in zone
} ld2450_target[LD2450_TARGET_MAX];

/**************************************************\
 *                  Commands
\**************************************************/

// sensor help
void CmdLD2450Help() {
  // help on command
  AddLog(LOG_LEVEL_INFO, PSTR("HLP: ld2450_reset                  = reset detection zone"));
  AddLog(LOG_LEVEL_INFO, PSTR("HLP: ld2450_track <t>,<m>          = track targets via MQTT in/out of the zones"));
  AddLog(LOG_LEVEL_INFO, PSTR("     t     : publish time [sec]"));
  AddLog(LOG_LEVEL_INFO, PSTR("     m     : mode, 0=off, 1=targets in zones, 2=targets out zones, 3=targets in & out zones"));
  AddLog(LOG_LEVEL_INFO, PSTR("     default is track %u,%u"), LD2450_TRACK_TIME, LD2450_TRACK_IN);
  AddLog(LOG_LEVEL_INFO, PSTR("HLP: ld2450_timeout <t>            = timeout [sec] to trigger inactivity"));
  AddLog(LOG_LEVEL_INFO, PSTR("HLP: ld2450_zone <z>               = get detection zone [mm]"));
  AddLog(LOG_LEVEL_INFO, PSTR("HLP: ld2450_zone <z>,<x1,x2,y1,y2> = set detection zone [mm]"));
  AddLog(LOG_LEVEL_INFO, PSTR("     z     : from 1 to %d detection zone"), LD2450_ZONE_MAX);
  AddLog(LOG_LEVEL_INFO, PSTR("     x1,x2 : from -%d (%dm left) to +%d (%dm right)"), LD2450_DIST_MAX, LD2450_DIST_MAX / 1000, LD2450_DIST_MAX, LD2450_DIST_MAX / 1000);
  AddLog(LOG_LEVEL_INFO, PSTR("     y1,y2 : from %d (sensor) to +%d (%dm)"), 0, LD2450_DIST_MAX, LD2450_DIST_MAX / 1000);
  AddLog(LOG_LEVEL_INFO, PSTR("     default is zone 1,%d,%d,%d,%d"), -LD2450_DIST_MAX, LD2450_DIST_MAX, 0, LD2450_DIST_MAX);
  AddLog(LOG_LEVEL_INFO, PSTR("HLP: ld2450_zone <z>,0,0,0,0 : disable zone detection"));
  AddLog(LOG_LEVEL_INFO, PSTR("HLP: ld2450_bluetooth <b>          = disable/enable bluetooth"));
  AddLog(LOG_LEVEL_INFO, PSTR("     b     : 0=bluetooth disable, 1=bluetooth enable"));
  AddLog(LOG_LEVEL_INFO, PSTR("HLP: ld2450_target <sm>            = set single/multi target"));
  AddLog(LOG_LEVEL_INFO, PSTR("     sm    : s=single target set, m=multi target set"));
  ResponseCmndDone();
}

// sensor specific detection timeout
void CmdLD2450SensorTimeout() {
  if (XdrvMailbox.payload > 0) {
    ld2450_config.sensortimeout = XdrvMailbox.payload;
    LD2450SaveConfig();
  }
  ResponseCmndNumber(ld2450_config.sensortimeout);
}

// target publication time
void CmdLD2450Track() {
  char str_answer[8 +1];

  strlcpy(str_answer, XdrvMailbox.data, 8 +1);
  if (strlen(str_answer) > 0) {
    char *pstr_token = strtok(str_answer, ",");
    if (pstr_token != nullptr) {
      ld2450_config.tracktime = atoi(pstr_token);
      pstr_token = strtok(nullptr, ",");
      if (pstr_token != nullptr) ld2450_config.trackmode = atoi(pstr_token);      
      LD2450SaveConfig();
    }
  }

  LD2450LoadConfig();
  sprintf(str_answer, "%d,%d", ld2450_config.tracktime, ld2450_config.trackmode);
  ResponseCmndChar(str_answer);
}

// sensor detection zone reset
void LD2450ConfigReset() {
  uint8_t index;
  int16_t dist = LD2450_DIST_MAX;
  // set default and save config
  for (index = 0; index < LD2450_ZONE_MAX; index++) {
    ld2450_config.zone_x1[index] = -dist;
    ld2450_config.zone_x2[index] = dist;
    ld2450_config.zone_y1[index] = 0;
    ld2450_config.zone_y2[index] = dist;
    dist = 0;
  }

  ld2450_config.tracktime = LD2450_TRACK_TIME;
  ld2450_config.trackmode = LD2450_TRACK_IN;
  ld2450_config.sensortimeout = LD2450_DEFAULT_TARGET_TIMEOUT;

  LD2450SaveConfig();
}

// sensor detection zone reset
void CmdLD2450Reset() {
  char str_answer[(7*4) +2 +1];

  LD2450ConfigReset();
  LD2450InitTargets(true);
  // send result
  sprintf(str_answer, "1,%d,%d,%d,%d", ld2450_config.zone_x1[0], ld2450_config.zone_x2[0], ld2450_config.zone_y1[0], ld2450_config.zone_y2[0]);
  ResponseCmndChar(str_answer);
}

// sensor detection zone setup
void CmdLD2450Zone() {
  uint8_t zone = 0;
  uint8_t count = 0;
  char *pstr_token;
  char str_answer[(7*4) +2 +1];

  strlcpy(str_answer, XdrvMailbox.data, (7*4) +2 +1);
  if (strlen(str_answer) > 0) {
    pstr_token = strtok(str_answer, ",");
    while ((zone < LD2450_ZONE_MAX) && (pstr_token != nullptr)) {
      if (strlen(pstr_token) > 0) {
        switch (count) {
          case 0:
            zone = atoi(pstr_token) -1;
            break;
          case 1:
            ld2450_config.zone_x1[zone] = atoi(pstr_token);
            break;
          case 2:
            ld2450_config.zone_x2[zone] = atoi(pstr_token);
            break;
          case 3:
            ld2450_config.zone_y1[zone] = atoi(pstr_token);
            break;
          case 4:
            ld2450_config.zone_y2[zone] = atoi(pstr_token);
            break;
        }
      }
      pstr_token = strtok(nullptr, ",");
      count++;
    }
    if ((zone < LD2450_ZONE_MAX) && (count == 5)) {
      // save and validate zone
      LD2450SaveConfig();
      LD2450InitTargets(true);
    }
  }

  // send result
  LD2450LoadConfig();
  if (((count != 5) && (count != 1)) || (zone >= LD2450_ZONE_MAX)) {
    sprintf(str_answer, "wrong param");
  } else {
    sprintf(str_answer, "%d,%d,%d,%d,%d", zone+1, ld2450_config.zone_x1[zone], ld2450_config.zone_x2[zone], ld2450_config.zone_y1[zone], ld2450_config.zone_y2[zone]);
  }
  ResponseCmndChar(str_answer);
}

// LD2450 bluetooth enable/disable
void CmdLD2450Bluetooth() {
  char str_answer[19];

  sprintf(str_answer, "invalid parameter");

  if (XdrvMailbox.data_len == 1) {
    if (XdrvMailbox.data[0] == '0' || XdrvMailbox.data[0] == '1') {      
      // send command
      LD2450SendCommand(enableConfigCommands, sizeof(enableConfigCommands));
      if (XdrvMailbox.data[0] == '1') {
        LD2450SendCommand(enableBluetooth, sizeof(enableBluetooth));
        sprintf(str_answer, "bluetooth enabled");
      } else {
        LD2450SendCommand(disableBluetooth, sizeof(disableBluetooth));
        sprintf(str_answer, "bluetooth disabled");
      }
      LD2450SendCommand(endConfigCommands, sizeof(endConfigCommands));  

      // reboot LD2450 to take effect
      LD2450SendCommand(enableConfigCommands, sizeof(enableConfigCommands));
      LD2450SendCommand(rebootModule, sizeof(rebootModule));
      LD2450SendCommand(endConfigCommands, sizeof(endConfigCommands));
    }
  }

  ResponseCmndChar(str_answer);
}

// LD2450 single/multi target
void CmdLD2450Target() {
  char str_answer[18];

  sprintf(str_answer, "invalid parameter");

  if (XdrvMailbox.data_len == 1) {
    if (XdrvMailbox.data[0] == 's' || XdrvMailbox.data[0] == 'm') {      
      // send command
      LD2450SendCommand(enableConfigCommands, sizeof(enableConfigCommands));
      if (XdrvMailbox.data[0] == 's') {
        LD2450SendCommand(singleTargetTracking, sizeof(singleTargetTracking));
        sprintf(str_answer, "single target set");
      } else {
        LD2450SendCommand(multiTargetTracking, sizeof(multiTargetTracking));
        sprintf(str_answer, "multi target set");
      }
      LD2450SendCommand(endConfigCommands, sizeof(endConfigCommands));  
    }
  }

  ResponseCmndChar(str_answer);
}

/**************************************************\
 *                  Config
\**************************************************/

bool LD2450CheckConfig() {
  bool res = true;

  if (ld2450_config.tracktime == 0) {
    ld2450_config.tracktime = LD2450_TRACK_TIME;
    res = false;
  }
  if (ld2450_config.trackmode > LD2450_TRACK_INOUT) {
    ld2450_config.trackmode = LD2450_TRACK_IN;
    res = false;
  }
  if (ld2450_config.sensortimeout == 0) {
    ld2450_config.sensortimeout = LD2450_DEFAULT_TARGET_TIMEOUT;
    res = false;
  }

  for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) {
    if (ld2450_config.zone_x1[zone] < -LD2450_DIST_MAX) ld2450_config.zone_x1[zone] = -LD2450_DIST_MAX;
    if (ld2450_config.zone_x2[zone] > LD2450_DIST_MAX) ld2450_config.zone_x2[zone] = LD2450_DIST_MAX;
    if (ld2450_config.zone_y1[zone] < 0) ld2450_config.zone_y1[zone] = 0;
    if (ld2450_config.zone_y2[zone] > LD2450_DIST_MAX) ld2450_config.zone_y2[zone] = LD2450_DIST_MAX;

    if (ld2450_config.zone_x1[zone] > ld2450_config.zone_x2[zone]) {
      uint16_t xy = ld2450_config.zone_x1[zone];
      ld2450_config.zone_x1[zone] = ld2450_config.zone_x2[zone];
      ld2450_config.zone_x2[zone] = xy;
    }
    if (ld2450_config.zone_y1[zone] > ld2450_config.zone_y2[zone]) {
      uint16_t xy = ld2450_config.zone_y1[zone];
      ld2450_config.zone_y1[zone] = ld2450_config.zone_y2[zone];
      ld2450_config.zone_y2[zone] = xy;
    }
  }

  return res;
}

// Load configuration from flash memory
void LD2450LoadConfig() {
  uint8_t pos;
  uint8_t row;
  uint16_t checksum;
    
  checksum = Settings->rf_code[2][0];
  checksum *= 0x100;
  checksum |= Settings->rf_code[2][1];

  if (checksum != LD2450ConfigChecksum()) LD2450ConfigReset();

  // read parameters
  ld2450_config.tracktime = Settings->rf_code[2][2];
  ld2450_config.trackmode = Settings->rf_code[2][3];
  ld2450_config.sensortimeout = Settings->rf_code[2][4];
  
  pos = 0;
  row = 3;
  for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) {
    ld2450_config.zone_x1[zone] = ((int16_t)Settings->rf_code[row][pos++] - 128) * 100;
    ld2450_config.zone_x2[zone] = ((int16_t)Settings->rf_code[row][pos++] - 128) * 100;
    ld2450_config.zone_y1[zone] = ((int16_t)Settings->rf_code[row][pos++] - 128) * 100;
    ld2450_config.zone_y2[zone] = ((int16_t)Settings->rf_code[row][pos++] - 128) * 100;
    if (pos >= 7) {
      pos = 0;
      row++;
    }
  }

  LD2450CheckConfig();
}

uint16_t LD2450ConfigChecksum() {
  uint16_t checksum = 0x4711;
  uint8_t *p1 = &(Settings->rf_code[2][2]);
  uint8_t *p2 = &(Settings->rf_code[(LD2450_ZONE_MAX/2) + 2][7]); 

  while (p1 <= p2) {
    checksum += *p1;
    p1++;
  }

  return checksum;
}

// Save configuration into flash memory
void LD2450SaveConfig() {
  uint8_t pos;
  uint8_t row;
  uint16_t checksum;

  LD2450CheckConfig();

  Settings->rf_code[2][2] = ld2450_config.tracktime;
  Settings->rf_code[2][3] = ld2450_config.trackmode;
  Settings->rf_code[2][4] = ld2450_config.sensortimeout;

  pos = 0;
  row = 3;
  for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) {
    Settings->rf_code[row][pos++] = (uint8_t)((ld2450_config.zone_x1[zone] / 100) + 128);
    Settings->rf_code[row][pos++] = (uint8_t)((ld2450_config.zone_x2[zone] / 100) + 128);
    Settings->rf_code[row][pos++] = (uint8_t)((ld2450_config.zone_y1[zone] / 100) + 128);
    Settings->rf_code[row][pos++] = (uint8_t)((ld2450_config.zone_y2[zone] / 100) + 128);
    if (pos >= 7) {
      pos = 0;
      row++;
    }
  }

  checksum = LD2450ConfigChecksum();
  Settings->rf_code[2][0] = (uint8_t)(checksum / 0x100);
  Settings->rf_code[2][1] = (uint8_t)checksum;
}

// driver initialisation
void Ld2450Detect(void) {
  if (ld2450_status.pserial != nullptr) {
    return;
  }
  if (PinUsed(GPIO_LD2450_RX) && PinUsed(GPIO_LD2450_TX)) {
    ld2450_status.pserial = new TasmotaSerial(Pin(GPIO_LD2450_RX), Pin(GPIO_LD2450_TX), 2);
    ld2450_status.enabled = ld2450_status.pserial->begin(256000, SERIAL_8N1);
    if (ld2450_status.enabled && ld2450_status.pserial->hardwareSerial()) ClaimSerial();
    if (ld2450_status.enabled)
      AddLog(LOG_LEVEL_INFO, PSTR("HLK: %s sensor init at %u"), D_LD2450_NAME, 256000);
    else
      AddLog(LOG_LEVEL_INFO, PSTR("HLK: %s sensor init failed"), D_LD2450_NAME);
  }
}

void Ld2450Publish(void) {
  if (ld2450_status.enabled) {
    // Send state change to be captured by rules
    MqttPublishSensor();
  }
}

/*********************************************\
 *                   Callback
\*********************************************/

void LD2450InitTargets(bool markaschanged) {
  // loop to init targets
  for (uint8_t index = 0; index < LD2450_TARGET_MAX; index++) {
    ld2450_target[index].x = 0;
    ld2450_target[index].y = 0;
    ld2450_target[index].speed = 0;
    ld2450_target[index].dist = 0;
    ld2450_target[index].in_zone = false;
    for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) {
      ld2450_zone[zone].target_in_zone[index] = false;
      ld2450_zone[zone].target_in_zone_old[index] = markaschanged;
    }
  }
  ld2450_status.counter = 0;  
}

// driver initialisation
void LD2450Init() {
  LD2450InitTargets(false);

  // load configuration
  LD2450LoadConfig();

  Ld2450Detect();

  // log help command
  AddLog(LOG_LEVEL_INFO, PSTR("HLP: ld2450_help to get help on %s commands"), D_LD2450_NAME);
}

bool zoneEnabled(uint8_t zone) {
  if ((ld2450_config.zone_x1[zone] == 0) &&
      (ld2450_config.zone_x2[zone] == 0) &&
      (ld2450_config.zone_y1[zone] == 0) &&
      (ld2450_config.zone_y2[zone] == 0)) return false;
  return true;
}

// Handling of received data
void LD2450ReceiveData() {
  uint8_t recv_data;
  uint32_t *pheader;
  uint16_t *pfooter;
  int16_t *pint16;

  // check sensor presence
  if (ld2450_status.pserial == nullptr) return;

  // run serial receive loop
  while (ld2450_status.pserial->available()) {
    // receive character
    recv_data = (uint8_t)ld2450_status.pserial->read();

    if (TasmotaGlobal.uptime > LD2450_START_DELAY) {
      // append character to received message body
      if (ld2450_received.idx_body < LD2450_MSG_SIZE_MAX) ld2450_received.arr_body[ld2450_received.idx_body++] = recv_data;

      // update last received characters
      ld2450_received.arr_last[0] = ld2450_received.arr_last[1];
      ld2450_received.arr_last[1] = ld2450_received.arr_last[2];
      ld2450_received.arr_last[2] = ld2450_received.arr_last[3];
      ld2450_received.arr_last[3] = recv_data;

      // update reception timestamp
      ld2450_received.timestamp = millis();

      // get header and footer
      pheader = (uint32_t *)&ld2450_received.arr_last;
      pfooter = (uint16_t *)(ld2450_received.arr_last + 2);

      // look for header and footer
      if (*pheader == 0x0003ffaa) {
        memcpy(ld2450_received.arr_body, ld2450_received.arr_last, 4);
        ld2450_received.idx_body = 4;
      } else if ((ld2450_received.idx_body == 30) && (*pfooter == 0xcc55)) { // data message received
        for (uint8_t index = 0; index < LD2450_TARGET_MAX; index++) {
          // set target coordonnates
          uint32_t start = 4 + index * 8;

          // x 
          pint16 = (int16_t *)(ld2450_received.arr_body + start);
          ld2450_target[index].x = *pint16;
          if (ld2450_target[index].x < 0) ld2450_target[index].x = 0 - ld2450_target[index].x - 32768;
          ld2450_target[index].x *= -1; // x coordinate like the Android app!

          // y
          pint16 = (int16_t *)(ld2450_received.arr_body + start + 2);
          ld2450_target[index].y = *pint16;
          if (ld2450_target[index].y < 0) ld2450_target[index].y = 0 - ld2450_target[index].y - 32768;
          ld2450_target[index].y = 0 - ld2450_target[index].y;

          // speed
          pint16 = (int16_t *)(ld2450_received.arr_body + start + 4);
          ld2450_target[index].speed = *pint16;
          if (ld2450_target[index].speed < 0) ld2450_target[index].speed = 0 - ld2450_target[index].speed - 32768;

          // calculate distance
          ld2450_target[index].dist = (uint16_t)sqrt(pow(ld2450_target[index].x, 2) + pow(ld2450_target[index].y, 2));

          // calculate zone if active
          if (ld2450_target[index].dist > 0) {
            bool published;
            ld2450_status.lastsensortime = LocalTime();
            // detect zone
            for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) {
              if (!zoneEnabled(zone)) {
                ld2450_zone[zone].target_in_zone[index] = false;
                continue;
              }
              published = (ld2450_zone[zone].target_in_zone[index] == ld2450_zone[zone].target_in_zone_old[index]);
              if (((ld2450_target[index].x >= ld2450_config.zone_x1[zone]) && (ld2450_target[index].x <= ld2450_config.zone_x2[zone])) &&
                  ((ld2450_target[index].y >= ld2450_config.zone_y1[zone]) && (ld2450_target[index].y <= ld2450_config.zone_y2[zone]))) {
                if (published) ld2450_zone[zone].target_in_zone[index] = true;                
              } else {
                if (published) ld2450_zone[zone].target_in_zone[index] = false;
              }
            }
          } else {
            for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) 
              ld2450_zone[zone].target_in_zone[index] = false;
          }
        }

        // calculate target counter
        ld2450_status.counter = 0;
        for (uint8_t index = 0; index < LD2450_TARGET_MAX; index++) {
          ld2450_target[index].in_zone = false;
          for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) {
            if (ld2450_zone[zone].target_in_zone[index]) {
              ld2450_status.counter++;
              ld2450_target[index].in_zone = true;
              break;
            } 
          }
        }

        // init reception
        ld2450_received.idx_body = 0;
      }

      // give control back to system
      yield();
    }
  }
}

bool checkForZoneChanges() {
  for (uint8_t index = 0; index < LD2450_TARGET_MAX; index++) {
    for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) {
      if (ld2450_zone[zone].target_in_zone[index] != ld2450_zone[zone].target_in_zone_old[index]) {
        return true; // zone left or entered
      }
    }
  }
  return false; 
}

bool checkForTargetSensorTimeout() {
  bool publish_target = false;
  if (ld2450_status.lastsensortime + ld2450_config.sensortimeout < LocalTime()) {
    ld2450_status.lastsensortime = LocalTime() + SECS_PER_DAY;
    ld2450_status.counter = 0;
    for (uint8_t index = 0; index < LD2450_TARGET_MAX; index++) {
      ld2450_target[index].in_zone = false;
      for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) {
        ld2450_zone[zone].target_in_zone[index] = false; 
        publish_target = true;
      }
    }
  }
  return publish_target;
}

// Show JSON status (for MQTT)
// 00:00:38.384 MQT: tele/tasmota_BFBB00/SENSOR = {"Time":"2000-01-01T00:00:38","ld2450":{"detect":1,"target1":{"x":-377,"y":466,"dist":599,"speed":0},"zone1":[1,0,0]}}
// 00:00:41.005 MQT: tele/tasmota_BFBB00/SENSOR = {"Time":"2000-01-01T00:00:41","ld2450":{"detect":1,"target1":{"x":-490,"y":479,"dist":685,"speed":0}}}
void LD2450ShowJSON(bool append) {
  bool publish_zone;
  bool publish_target;

  // check sensor presence
  if (ld2450_status.pserial == nullptr) return;

  publish_target = checkForTargetSensorTimeout();

  // if at least one target detected, publish targets
  if (ld2450_config.trackmode != LD2450_TRACK_OFF) { // publish targets enabled
    if (ld2450_status.pubtimestamp + ld2450_config.tracktime < LocalTime()) {
      switch (ld2450_config.trackmode) {
      case LD2450_TRACK_IN:
        if (ld2450_status.counter > 0) publish_target = true;
        break;
      case LD2450_TRACK_OUT:
      case LD2450_TRACK_INOUT:
        publish_target = true;
        break;
      }
    }
  }

  publish_zone = checkForZoneChanges();

  if (!publish_zone && !publish_target) return;

  if (append) {
    ld2450_status.pubtimestamp = LocalTime();
    // start of ld2450 section
    ResponseAppend_P(PSTR(",\"ld2450\":{\"detect\":%u"), ld2450_status.counter);

    for (uint8_t index = 0; index < LD2450_TARGET_MAX; index++) {
      publish_target = false;
      switch (ld2450_config.trackmode) {
      case LD2450_TRACK_IN:
        if (ld2450_target[index].in_zone) publish_target = true;
        break;
      case LD2450_TRACK_OUT:
      case LD2450_TRACK_INOUT:
        publish_target = true;
        break;
      }
      if (publish_target) {
        ResponseAppend_P(PSTR(",\"target%u\":{\"x\":%d,\"y\":%d,\"dist\":%u,\"speed\":%d}"), 
          index + 1, ld2450_target[index].x, ld2450_target[index].y, ld2450_target[index].dist, ld2450_target[index].speed);
      }
    }
    if (publish_zone) {
      for (uint8_t zone = 0; zone < LD2450_ZONE_MAX; zone++) {
        bool zone_changed = false;
        for (uint8_t index = 0; index < LD2450_TARGET_MAX; index++) {
          if (ld2450_zone[zone].target_in_zone[index] != ld2450_zone[zone].target_in_zone_old[index]) { // zone left or entered
            ld2450_zone[zone].target_in_zone_old[index] = ld2450_zone[zone].target_in_zone[index];
            zone_changed = true;
          }
        }
        if (zone_changed) {
          ResponseAppend_P(PSTR(",\"zone%u\":["), zone +1);
          for (uint8_t index = 0; index < LD2450_TARGET_MAX; index++) {
            if (index > 0) ResponseAppend_P(PSTR(","));
            ResponseAppend_P(PSTR("%u"),
              (ld2450_zone[zone].target_in_zone[index] ? 1:0));
          }
          ResponseAppend_P(PSTR("]"));
        }
      }    
    }
    // end of ld2450 section
    ResponseAppend_P(PSTR("}"));
  }
}

void LD2450SendCommand(const uint8_t *data, uint16_t data_len) {
  uint8_t buffer[4 + 2 + data_len + 4];
  uint32_t idx = 0;

  // header
  buffer[idx++] = 0xFD;
  buffer[idx++] = 0xFC;
  buffer[idx++] = 0xFB;
  buffer[idx++] = 0xFA;

  // in-frame data length
  buffer[idx++] = data_len;
  buffer[idx++] = data_len >> 8;

  // in-frame data
  for (uint16_t i = 0; i < data_len; i++) {
    buffer[idx++] = data[i];
  }

  // end of frame
  buffer[idx++] = 0x04;
  buffer[idx++] = 0x03;
  buffer[idx++] = 0x02;
  buffer[idx++] = 0x01;

  for (uint32_t i = 0; i < idx; i++) {
    ld2450_status.pserial->write(buffer[i]);
  }
}

/*********************************************\
 *                   Web
\*********************************************/

#ifdef USE_WEBSERVER

// Append HLK-LD2450 sensor data to main page
void LD2450WebSensor() {
  uint8_t index, counter, position;
  uint8_t arr_pos[LD2450_TARGET_MAX];
  char str_color[8];

  // check if enabled
  if (!ld2450_status.enabled) return;

  // start of display
  WSContentSend_P(PSTR("<div style='font-size:10px;text-align:center;margin:4px 0px;padding:2px 6px;background:#333333;border-radius:8px;'>\n"));

  // scale
  WSContentSend_P(PSTR("<div style='display:flex;padding:0px;'>\n"));
  WSContentSend_P(PSTR("<div style='width:28%%;padding:0px;text-align:left;font-size:12px;font-weight:bold;'>LD2450</div>\n"));
  WSContentSend_P(PSTR("<div style='width:6%%;padding:0px;text-align:left;'>0m</div>\n"));
  for (index = 1; index < 6; index++) WSContentSend_P(PSTR("<div style='width:12%%;padding:0px;'>%um</div>\n"), index);
  WSContentSend_P(PSTR("<div style='width:6%%;padding:0px;text-align:right;'>6m</div>\n"));
  WSContentSend_P(PSTR("</div>\n"));

  // targets
  WSContentSend_P(PSTR("<div style='display:flex;padding:0px;background:none;'>\n"));
  WSContentSend_P(PSTR("<div style='width:25%%;padding:0px;text-align:left;color:white;'>&nbsp;&nbsp;Presence</div>\n"));

  // calculate target position
  for (index = 0; index < LD2450_TARGET_MAX; index++) {
    // check if target should be displayed
    if (ld2450_target[index].dist == 0)
      arr_pos[index] = UINT8_MAX;
    else
      arr_pos[index] = 28 - 3 + (uint8_t)((uint32_t)ld2450_target[index].dist * 72 / LD2450_DIST_MAX);
  }

  // adjust lower target position with 5% steps
  position = arr_pos[0];
  for (index = 1; index < LD2450_TARGET_MAX; index++) {
    if (position == UINT8_MAX)
      position = arr_pos[index];
    else if ((arr_pos[index] < 100 - 10) && (arr_pos[index] < position + 5))
      arr_pos[index] = position + 5;
    if (arr_pos[index] != UINT8_MAX) position = arr_pos[index];
  }

  // adjust higher target position with 5% steps
  position = UINT8_MAX;
  for (counter = LD2450_TARGET_MAX; counter > 0; counter--) {
    index = counter - 1;
    if (arr_pos[index] != UINT8_MAX) {
      if (arr_pos[index] > 95) arr_pos[index] = 95;
      if ((arr_pos[index] > position - 5) && (arr_pos[index] > 28 + 10)) arr_pos[index] = position - 5;
      position = arr_pos[index];
    }
  }

  // loop to display targets
  position = 25;
  for (index = 0; index < LD2450_TARGET_MAX; index++)
    if (arr_pos[index] != UINT8_MAX) {
      // if needed, set separation zone
      if (arr_pos[index] > position) WSContentSend_P(PSTR("<div style='width:%u%%;padding:0px;background:none;'>&nbsp;</div>\n"), arr_pos[index] - position);

      // check if target is in detection zone
      if (ld2450_target[index].in_zone)
        strcpy(str_color, LD2450_COLOR_PRESENT);
      else
        strcpy(str_color, LD2450_COLOR_OUTRANGE);
      WSContentSend_P(PSTR("<div style='width:5%%;padding:0px;border-radius:50%%;background:%s;'>%u</div>\n"), str_color, index + 1);

      // update minimum position
      position = arr_pos[index] + 5;
    }
  if (position < 100) WSContentSend_P(PSTR("<div style='width:%u%%;padding:0px;background:none;'>&nbsp;</div>\n"), 100 - position);

  // end of display
  WSContentSend_P(PSTR("</div>\n"));
}

#ifdef USE_LD2450_RADAR

// Radar page
void LD2450GraphRadarUpdate() {
  uint8_t index, zone;
  int32_t x1, x2, y1, y2;
  char str_class[8];

  // start of update page
  WSContentBegin(200, CT_PLAIN);

  // radar zone
  for (zone = 0; zone < LD2450_ZONE_MAX; zone++) {
    if (!zoneEnabled(zone)) continue;
    x1 = (int32_t)ld2450_config.zone_x1[zone] * 400 / LD2450_DIST_MAX + 400;
    x2 = (int32_t)ld2450_config.zone_x2[zone] * 400 / LD2450_DIST_MAX + 400;
    y1 = (int32_t)ld2450_config.zone_y1[zone] * 400 / LD2450_DIST_MAX + 50;
    y2 = (int32_t)ld2450_config.zone_y2[zone] * 400 / LD2450_DIST_MAX + 50;
    WSContentSend_P(PSTR("M %d %d L %d %d L %d %d L %d %d Z "), x1, y1, x2, y1, x2, y2, x1, y2);
  }
  WSContentSend_P(PSTR("\n"));

  // loop thru targets
  for (index = 0; index < LD2450_TARGET_MAX; index++) {
    // calculate target type
    if (ld2450_target[index].dist == 0)
      strcpy(str_class, "abs");
    else if (ld2450_target[index].in_zone)
      strcpy(str_class, "act");
    else
      strcpy(str_class, "ina");

    // calculate coordonates
    x1 = (int32_t)ld2450_target[index].x * 400 / LD2450_DIST_MAX + 400;
    y1 = (int32_t)ld2450_target[index].y * 400 / LD2450_DIST_MAX + 50;

    // display target
    WSContentSend_P(PSTR("%s;%d;%d;%d\n"), str_class, x1, y1, y1 + 5);
  }

  // end of update page
  WSContentEnd();
}

// Radar page
void LD2450GraphRadar() {
  long index;
  long x, y, r;
  long cos[7] = {-1000, -866, -500, 0, 500, 866, 1000};
  long sin[7] = {0, 500, 866, 1000, 866, 500, 0};

  // if access not allowed, close
  if (!HttpCheckPriviledgedAccess()) return;

  // set page label
  WSContentStart_P("LD2450 Radar", true);
  WSContentSend_P(PSTR("\n</script>\n"));

  // page data refresh script
  WSContentSend_P(PSTR("<script type='text/javascript'>\n\n"));

  WSContentSend_P(PSTR("function updateData(){\n"));

  WSContentSend_P(PSTR(" httpData=new XMLHttpRequest();\n"));
  WSContentSend_P(PSTR(" httpData.open('GET','/ld2450.upd',true);\n"));

  WSContentSend_P(PSTR(" httpData.onreadystatechange=function(){\n"));
  WSContentSend_P(PSTR("  if (httpData.readyState===XMLHttpRequest.DONE){\n"));
  WSContentSend_P(PSTR("   if (httpData.status===0 || (httpData.status>=200 && httpData.status<400)){\n"));
  WSContentSend_P(PSTR("    arr_param=httpData.responseText.split('\\n');\n"));
  WSContentSend_P(PSTR("    num_param=arr_param.length;\n"));
  WSContentSend_P(PSTR("    if (document.getElementById('zone')!=null) document.getElementById('zone').setAttributeNS(null,'d',arr_param[0]);\n"));
  WSContentSend_P(PSTR("    for (i=1;i<num_param;i++){\n"));
  WSContentSend_P(PSTR("     arr_value=arr_param[i].split(';');\n"));
  WSContentSend_P(PSTR("     if (document.getElementById('c'+i)!=null) document.getElementById('c'+i).classList.remove('abs','act','ina');\n"));
  WSContentSend_P(PSTR("     if (document.getElementById('c'+i)!=null) document.getElementById('c'+i).classList.add(arr_value[0]);\n"));
  WSContentSend_P(PSTR("     if (document.getElementById('c'+i)!=null) document.getElementById('t'+i).classList.remove('abs','act','ina');\n"));
  WSContentSend_P(PSTR("     if (document.getElementById('c'+i)!=null) document.getElementById('t'+i).classList.add(arr_value[0]);\n"));
  WSContentSend_P(PSTR("     if (document.getElementById('c'+i)!=null) document.getElementById('c'+i).setAttributeNS(null,'cx',arr_value[1]);\n"));
  WSContentSend_P(PSTR("     if (document.getElementById('t'+i)!=null) document.getElementById('t'+i).setAttribute('x',arr_value[1]);\n"));
  WSContentSend_P(PSTR("     if (document.getElementById('c'+i)!=null) document.getElementById('c'+i).setAttributeNS(null,'cy',arr_value[2]);\n"));
  WSContentSend_P(PSTR("     if (document.getElementById('t'+i)!=null) document.getElementById('t'+i).setAttribute('y',arr_value[3]);\n"));
  WSContentSend_P(PSTR("    }\n"));
  WSContentSend_P(PSTR("   }\n"));
  WSContentSend_P(PSTR("   setTimeout(updateData,%u);\n"), 1000);  // ask for next update in 1 sec
  WSContentSend_P(PSTR("  }\n"));
  WSContentSend_P(PSTR(" }\n"));

  WSContentSend_P(PSTR(" httpData.send();\n"));
  WSContentSend_P(PSTR("}\n"));

  WSContentSend_P(PSTR("setTimeout(updateData,%u);\n\n"), 100);  // ask for first update after 100ms

  WSContentSend_P(PSTR("</script>\n\n"));

  // set page as scalable
  WSContentSend_P(PSTR("<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=yes'/>\n"));

  // page style
  WSContentSend_P(PSTR("<style>\n"));
  WSContentSend_P(PSTR("body {color:white;background-color:#252525;font-family:Arial, Helvetica, sans-serif;}\n"));

  WSContentSend_P(PSTR("a {color:white;}\n"));
  WSContentSend_P(PSTR("a:link {text-decoration:none;}\n"));

  WSContentSend_P(PSTR("div {padding:0px;margin:0px;text-align:center;}\n"));
  WSContentSend_P(PSTR("div.title {font-size:4vh;font-weight:bold;}\n"));
  WSContentSend_P(PSTR("div.header {font-size:3vh;margin:1vh auto;}\n"));

  WSContentSend_P(PSTR("div.graph {width:100%%;margin:2vh auto;}\n"));
  WSContentSend_P(PSTR("svg.graph {width:100%%;height:80vh;}\n"));

  WSContentSend_P(PSTR("</style>\n"));

  // page body
  WSContentSend_P(PSTR("</head>\n"));
  WSContentSend_P(PSTR("<body>\n"));
  WSContentSend_P(PSTR("<div class='main'>\n"));

  // room name
  WSContentSend_P(PSTR("<div class='title'><a href='/'>%s</a></div>\n"), SettingsText(SET_DEVICENAME));

  // header
  WSContentSend_P(PSTR("<div class='header'>Live Radar</div>\n"));

  // ------- Graph --------

  // start of radar
  WSContentSend_P(PSTR("<div class='graph'>\n"));
  WSContentSend_P(PSTR("<svg class='graph' viewBox='0 0 %u %u'>\n"), 800, 400 + 50 + 4);

  // style
  WSContentSend_P(PSTR("<style type='text/css'>\n"));
  WSContentSend_P(PSTR("text {font-size:16px;fill:#aaa;text-anchor:middle;}\n"));
  WSContentSend_P(PSTR("text.abs {fill:%s;}\n"), LD2450_COLOR_ABSENT);
  WSContentSend_P(PSTR("text.zone {fill:%s;}\n"), LD2450_COLOR_ZONE);
  WSContentSend_P(PSTR("path {stroke:green;stroke-dasharray:2 2;fill:none;}\n"));
  WSContentSend_P(PSTR("path.zone {stroke-dasharray:2 4;fill:#1c1c1c;}\n"));
  WSContentSend_P(PSTR("circle {opacity:0.75;}\n"));
  WSContentSend_P(PSTR("circle.ina {fill:%s;}\n"), LD2450_COLOR_OUTRANGE);
  WSContentSend_P(PSTR("circle.act {fill:%s;}\n"), LD2450_COLOR_PRESENT);
  WSContentSend_P(PSTR("circle.abs {fill:%s;}\n"), LD2450_COLOR_ABSENT);
  WSContentSend_P(PSTR("</style>\n"));

  // radar active zone
  WSContentSend_P(PSTR("<path id='zone' class='zone' d='' />\n"));

  // radar frame lines
  for (index = 1; index < 6; index++) WSContentSend_P(PSTR("<path d='M 400 50 L %d %d' />\n"), 400 + 400 * cos[index] / 1000, 50 + 400 * sin[index] / 1000);

  // radar frame circles and distance
  WSContentSend_P(PSTR("<text x=400 y=30>LD2450</text>\n"));
  for (index = 1; index < 7; index++) {
    x = index * sin[4] * 400 / 1000 / 6;
    y = index * cos[4] * 400 / 1000 / 6;
    r = index * 400 / 6;
    WSContentSend_P(PSTR("<text x=%d y=%d>%dm</text>\n"), 400 + x, 40 + y, index);
    WSContentSend_P(PSTR("<text x=%d y=%d>%dm</text>\n"), 400 - 5 - x, 40 + y, index);
    WSContentSend_P(PSTR("<path d='M %d %d A %d %d 0 0 1 %d %d' />\n"), 400 + x, 50 + y, r, r, 400 - x, 50 + y);
  }

  // display zone num
  for (index = 0; index < LD2450_ZONE_MAX; index++) {
    if (!zoneEnabled(index)) continue;
    x = (int32_t)ld2450_config.zone_x1[index] * 400 / LD2450_DIST_MAX;
    y = (int32_t)ld2450_config.zone_y1[index] * 400 / LD2450_DIST_MAX;
    WSContentSend_P(PSTR("<text class='zone' x=%d y=%d>Z%d</text>\n"), x + 412, y + 64, index + 1);
  }

  // display targets
  for (index = 0; index < LD2450_TARGET_MAX; index++) {
    WSContentSend_P(PSTR("<circle id='c%d' class='abs' cx=400 cy=50 r=20 />\n"), index + 1);
    WSContentSend_P(PSTR("<text id='t%d' class='abs' x=400 y=55>%d</text>\n"), index + 1, index + 1);
  }

  // end of radar
  WSContentSend_P(PSTR("</svg>\n"));
  WSContentSend_P(PSTR("</div>\n"));

  // end of page
  WSContentStop();
}

#endif  // USE_LD2450_RADAR

#endif  // USE_WEBSERVER

/***************************************\
 *              Interface
\***************************************/

// LD2450 sensor
bool Xsns114(uint32_t function) {
  bool result = false;

  // swtich according to context
  switch (function) {
    case FUNC_INIT:
      LD2450Init();
      break;
    case FUNC_COMMAND:
      result = DecodeCommand(kHLKLD2450Commands, HLKLD2450Command);
      break;
    case FUNC_EVERY_100_MSECOND:
      if (TasmotaGlobal.uptime > 4) Ld2450Publish();
      break;
    case FUNC_JSON_APPEND:
      if (ld2450_status.enabled) LD2450ShowJSON(true);
      break;
    case FUNC_LOOP:
      if (ld2450_status.enabled) LD2450ReceiveData();
      break;

#ifdef USE_WEBSERVER

    case FUNC_WEB_SENSOR:
      if (ld2450_status.enabled) LD2450WebSensor();
      break;

#ifdef USE_LD2450_RADAR
    case FUNC_WEB_ADD_MAIN_BUTTON:
      if (ld2450_status.enabled) WSContentSend_P(PSTR("<p><form action='ld2450' method='get'><button>LD2450 Radar</button></form></p>\n"));
      break;
    case FUNC_WEB_ADD_HANDLER:
      if (ld2450_status.enabled) Webserver->on("/ld2450", LD2450GraphRadar);
      if (ld2450_status.enabled) Webserver->on("/ld2450.upd", LD2450GraphRadarUpdate);
      break;
#endif  // USE_LD2450_RADAR

#endif  // USE_WEBSERVER
  }

  return result;
}

#endif  // USE_LD2450
