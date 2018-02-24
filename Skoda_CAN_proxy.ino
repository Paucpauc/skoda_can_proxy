#define _SS_MAX_RX_BUFF 240 // defaul 64 bytes is too low for SIM response
#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include <mcp_can.h>
#include <EEPROM.h>

//#define DEBUG_SIM 1
#define SIM_POWER_KEY 9    // LOW for power switch
#define SIM_RX 8           // 
#define SIM_TX 7           //
#define REAR_CAM_KEY A2    // HIGH for relay on
#define CAN0_CS 10         //
#define BAT_ANALOG_PORT A3 // Any analog port with VBAT signal (Use a voltage divider!!!)

// EEPROM address for preserv some variables
#define EEPROM_VCC_CORRECT 0                                      // vcc_corect
#define EEPROM_LOCK_1 (EEPROM_VCC_CORRECT + sizeof(vcc_correct))  // lock_1
#define EEPROM_LOCK_2 (EEPROM_LOCK_1 + sizeof(pos.lock_1))        // lock_2

SoftwareSerial SIM(SIM_RX, SIM_TX);

MCP_CAN CAN0(CAN0_CS);

#define VAG_CAN_PARKTRONIK      0x54b
#define VAG_CAN_WHEEL_STEERING  0x5c1
#define VAG_CAN_ENG_RPM         0x35B
#define VAG_CAN_SPEED           0x359
#define VAG_CAN_DOOR_LOCK       0x291
#define VAG_CAN_FUEL            0x621
#define VAG_CAN_BATTERY         0x571

#define VAG_WHEEL_STEERING_KEY_NONE     0x00
#define VAG_WHEEL_STEERING_KEY_MEDIA    0x01
#define VAG_WHEEL_STEERING_KEY_NEXT     0x02
#define VAG_WHEEL_STEERING_KEY_PREV     0x03
#define VAG_WHEEL_STEERING_KEY_VOL_UP   0x06
#define VAG_WHEEL_STEERING_KEY_VOL_DOWN 0x07
#define VAG_WHEEL_STEERING_KEY_MENU     0x0a
#define VAG_WHEEL_STEERING_KEY_PHONE    0x1a
#define VAG_WHEEL_STEERING_KEY_ARR_UP   0x22
#define VAG_WHEEL_STEERING_KEY_ARR_DOWN 0x23
#define VAG_WHEEL_STEERING_KEY_OK       0x28
#define VAG_WHEEL_STEERING_KEY_BACK     0x29
#define VAG_WHEEL_STEERING_KEY_MUTE     0x2b

char shared_buf[190];

// Buffer for read from SIM
#define LINE_SIZE 190
char line[LINE_SIZE];
int line_i = 0;

unsigned long reverse_timer = 0; // rear cam timer

#define SIM_RESTART_TIMEOUT 90000
unsigned long sim_restart_timer = 5000;

#define GPRS_CHECK_INTERVAL 10000
unsigned long gprs_check_timer = GPRS_CHECK_INTERVAL;
#define GPRS_START_INTERVAL 5000
unsigned long gprs_start_timer = GPRS_START_INTERVAL;
byte gprsOk = 0;

#define GNS_GET_INTERVAL 10000
unsigned long gns_get_timer = GNS_GET_INTERVAL;
#define GNS_START_INTERVAL 5000
unsigned long gns_start_timer = GNS_START_INTERVAL;
byte gnsOk = 0;

#define GSMLOC_GET_INTERVAL 60000
unsigned long gsmloc_get_timer = GSMLOC_GET_INTERVAL;

#define QUEUE_IDLE_INTERVAL 90000
unsigned long queue_idle_timer = QUEUE_IDLE_INTERVAL;

#define SIM_FREE_TIMEOUT 10000
unsigned long SIM_free_timer = 0;

#define QUEUE_INTERVAL 3000
unsigned long queue_timer = QUEUE_INTERVAL;

#define VCC_READ_INTERVAL 10000
unsigned long vcc_read_timer = 0;

#define TEMPERATURE_INTERVAL 1000
unsigned long temperature_timer = 1000;

float vcc_correct;

byte SIMFree = 1;
void (*cb_err)(void);
void (*cb_ok)(void);

#define SRC_NONE 0
#define SRC_GNS 1
#define SRC_GSM 2
struct POS {
  char date[15];
  char lo[10];
  char la[10];
  int n;
  int src;
  int lock_1;
  int lock_2;
  int rpm;
  int fuel;
  int speed_max;
  int vcc_can;
  int vcc;
  int temp;
  unsigned long can_last;
};
POS pos;

byte http_ready = 0;

unsigned long rxId;
byte len;
byte rxBuf[8];


void setup() {
  wdt_disable();
  wdt_reset();
  Serial.begin(115200);
  SIM.begin(19200);

  EEPROM.get(EEPROM_VCC_CORRECT, vcc_correct);
  EEPROM.get(EEPROM_LOCK_1, pos.lock_1);
  EEPROM.get(EEPROM_LOCK_2, pos.lock_2);

  if (CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    Serial.println(F("Canbus init ok"));
  } else {
    Serial.println(F("Canbus init ERROR"));
  }
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  cmd((char *)"AT");
}
void loop() {
  wdt_reset();
  TaskSimRead();
  TaskCheckCAN();
  TaskRearCam();
  TaskCheckGPRS();
  TaskSimFreeTimeout();
  TaskStartGPRS();
  TaskSimRestart();
  TaskGetGNS();
  TaskStartGNS();
  TaskGetGSMLOC();
  TaskQueueHTTP();
  TaskReadHttp();
  TaskQueueIdle();
  TaskReadTemp();
  TaskReadVcc();
  //Serial.println("loop");
}
void TaskReadVcc(void) {
  if (vcc_read_timer > millis())
    return;
  int vcc = analogRead(BAT_ANALOG_PORT);
  if (vcc)
    pos.vcc = vcc * vcc_correct;
  vcc_read_timer = millis() + VCC_READ_INTERVAL;
}
  void TaskReadTemp(void) {
    if (temperature_timer > millis())
      return;
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
    ADCSRA |= _BV(ADEN);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    pos.temp = ADCW;
    analogReference(DEFAULT);
    temperature_timer = millis() + TEMPERATURE_INTERVAL;
  }
  void TaskQueueIdle(void) {
    if (queue_idle_timer > millis())
      return;
    pos.date[sizeof(pos.date) - 1] = 0x00;
    pos.lo[sizeof(pos.lo) - 1] = 0x00;
    pos.la[sizeof(pos.la) - 1] = 0x00;
    pos.src = SRC_NONE;
    pos.n = 1;
    queue_timer = 0;
  }
  void TaskReadHttp(void) {
    if (! SIMFree)
      return;
    if (! http_ready)
      return;
    http_ready = 0;
    cmd((char *)"AT+HTTPREAD");
  }
  void TaskStartGNS(void) {
    if (gnsOk)
      return;
    if (gns_start_timer > millis())
      return;
    if (! SIMFree)
      return;
    cmd((char *)"AT+CGNSPWR=1");
    gns_start_timer = millis() + GNS_START_INTERVAL;
  }
  void TaskGetGSMLOC(void) {
    if (gsmloc_get_timer > millis())
      return;
    if (! SIMFree)
      return;
    cmd((char *)"AT+CIPGSMLOC=1,1");
    gsmloc_get_timer = millis() + GSMLOC_GET_INTERVAL;
  }
  void TaskGetGNS(void) {
    if (gns_get_timer > millis())
      return;
    if (! SIMFree)
      return;
    cmd((char *)"AT+CGNSINF");
    gns_get_timer = millis() + GNS_GET_INTERVAL;
  }
  void init_http(void) {
    cmd((char *)"AT+HTTPINIT");
  }
  void send_http(void) {
    cmd((char *)"AT+HTTPACTION=0");
  }
  void TaskQueueHTTP(void) {
    if (queue_timer > millis())
      return;
    if (!SIMFree)
      return;
    if (pos.n == 0)
      return;
    sprintf_P(shared_buf, PSTR("AT+HTTPPARA=\"URL\",\"http://urbanovich.net/webhook/car.php?d=%s&la=%s&lo=%s&vcc=%d&s=%d&rpm=%d&spd=%d&f=%d&l1=%d&l2=%d&vcc_can=%d&can_last=%lu&qid=%d&t=%d\""),
              pos.date, pos.la, pos.lo, pos.vcc, pos.src, pos.rpm, pos.speed_max, pos.fuel, pos.lock_1, pos.lock_2, pos.vcc_can, pos.can_last, pos.n, pos.temp);
    cmd_cb(shared_buf, (&send_http), (&init_http));
    pos.n = 0;
    queue_timer = millis() + QUEUE_INTERVAL;
    queue_idle_timer = millis() + QUEUE_IDLE_INTERVAL;
  }
  void TaskRearCam(void) {
    if (reverse_timer > millis()) {
      pinMode(REAR_CAM_KEY, OUTPUT);
      digitalWrite(REAR_CAM_KEY, HIGH);
    } else {
      pinMode(REAR_CAM_KEY, INPUT);
    }
  }
  void TaskSimFreeTimeout(void) {
    if (SIMFree)
      return;
    if (SIM_free_timer > millis())
      return;
    SIMFree = 1;
    Serial.println(F("TIMEOUT"));
  }
  void TaskCheckCAN(void)
  {
    rxId = 0;
    if (! CAN0.checkReceive())
      return;
    if (CAN0.readMsgBuf(&rxId, &len, rxBuf) != CAN_OK)
      return;
    if (rxBuf[0] == 0x00)
      return;
    if (rxId == VAG_CAN_PARKTRONIK ) {
      reverse_timer = millis() + 9000;
    }
    if (rxId == VAG_CAN_WHEEL_STEERING) {
      if (rxBuf[0] == VAG_WHEEL_STEERING_KEY_MEDIA) {
        rxBuf[0] = VAG_WHEEL_STEERING_KEY_PHONE;
        CAN0.sendMsgBuf(rxId, len, rxBuf);
      }
    }
    if (rxId == VAG_CAN_DOOR_LOCK) {
      if (rxBuf[1] != 0) {
        if (pos.lock_1 != rxBuf[0]) {
          pos.lock_1 = rxBuf[0];
          EEPROM.put(EEPROM_LOCK_1, pos.lock_1);
        }
        if (pos.lock_2 != rxBuf[1]) {
          pos.lock_2 = rxBuf[1];
          EEPROM.put(EEPROM_LOCK_2, pos.lock_2);
          pos.n = 1;
          queue_timer = 0;
        }
      }
    }
    if (rxId == VAG_CAN_ENG_RPM) {
      pos.rpm = ((rxBuf[2] * 256) + rxBuf[1]) / 4;
    }
    if (rxId == VAG_CAN_FUEL) {
      pos.fuel = rxBuf[3] & 0x7f;
    }
    if (rxId == VAG_CAN_BATTERY) {
      pos.vcc_can = (rxBuf[0] / 2 + 50 ) * 100;
      if (vcc_read_timer < millis()) {
        int vcc = analogRead(BAT_ANALOG_PORT);
        if (vcc) {
          vcc_correct = pos.vcc_can / vcc;
          pos.vcc = vcc * vcc_correct;
          EEPROM.put(EEPROM_VCC_CORRECT, vcc_correct);
        }
        vcc_read_timer = millis() + VCC_READ_INTERVAL;
      }
      pos.can_last = millis();
    }
    if (rxId == VAG_CAN_SPEED) {
      if (rxBuf[2] != 0xff) {
        byte speed_cur = ((rxBuf[2] * 256) + (rxBuf[1] & 0xfe)) / 200;
        if (pos.speed_max < speed_cur)
          pos.speed_max = speed_cur;
      }
    }
    if (rxId == 0x3e1 && 0) {
      sprintf_P(shared_buf, PSTR("ID: %3lX "), rxId);
      Serial.print(shared_buf);
      for (byte i = 0; i < len; i++) {
        sprintf_P(shared_buf, PSTR(" %2X"), rxBuf[i]);
        Serial.print(shared_buf);
      }
      Serial.println();
    }
  }
  void cmd(char *l) {
    SIMFree = 0;
    SIM.println(l);
    SIM_free_timer = millis() + SIM_FREE_TIMEOUT;
  }
  void cmd_cb(char *l, void (*cb1)(void), void (*cb2)(void)) {
    cb_ok = cb1;
    cb_err = cb2;
    cmd(l);
  }
  void TaskCheckGPRS(void) {
    if (gprs_check_timer > millis())
      return;
    if (!SIMFree)
      return;
    cmd((char *)"AT+SAPBR=2,1");
    gprs_check_timer = millis() +  GPRS_CHECK_INTERVAL;
  }
  void TaskStartGPRS(void) {
    if (gprsOk)
      return;
    if (gprs_start_timer > millis())
      return;
    if (! SIMFree)
      return;
    cmd((char *)"AT+SAPBR=1,1");
    gprs_start_timer = millis() +  GPRS_START_INTERVAL;
  }
  void SimCommand(void) {
    void (*cb)(void);
    line[line_i] = 0x00;
#ifdef DEBUG_SIM
    Serial.print("=");
    Serial.print(line);
    Serial.println("=");
#endif
    if (strncmp(line, "+CGNSINF: 0,", 12) == 0) {
      gnsOk = 0;
      goto simcommandexit;
    }
    if (strncmp(line, "+CGNSINF: 1,", 12) == 0) {
      gnsOk = 1;
    }
    if (strncmp(line, "+CGNSINF: 1,1,2", 15) == 0) {
      substrindex(pos.date, 2, sizeof(pos.date) - 1);
      substrindex(pos.la, 3, sizeof(pos.la) - 1);
      substrindex(pos.lo, 4, sizeof(pos.lo) - 1);
      pos.src = SRC_GNS;
      gsmloc_get_timer = millis() + GSMLOC_GET_INTERVAL;
      pos.n = 1;
      queue_timer = 0;
      goto simcommandexit;
    }
    if (strncmp(line, "+CIPGSMLOC: ", 12) == 0) { // +CIPGSMLOC: 0,37.419689,55.886726,2017/12/21,21:38:20
      substrindex(shared_buf, 3, 10);
      memcpy(&pos.date[0], &shared_buf[0], 4); // YYYY
      memcpy(&pos.date[4], &shared_buf[5], 2); // MM
      memcpy(&pos.date[6], &shared_buf[8], 2); // DD
      substrindex(shared_buf, 4, 8);
      memcpy(&pos.date[8], &shared_buf[0], 2); // HH
      memcpy(&pos.date[10], &shared_buf[3], 2); // MM
      memcpy(&pos.date[12], &shared_buf[6], 2); // SS
      pos.date[14] = '\0';
      substrindex(pos.lo, 1, sizeof(pos.lo) - 1);
      substrindex(pos.la, 2, sizeof(pos.la) - 1);
      pos.src = SRC_GSM;
      pos.n = 1;
      queue_timer = 0;
      goto simcommandexit;
    }

    if (strncmp(line, "+SAPBR: 1,", 10) == 0) {
      gprsOk = 0;
    }
    if (strncmp(line, "+SAPBR: 1,1,", 12) == 0) {
      gprsOk = 1;
      goto simcommandexit;
    }
    if (strncmp(line, "+HTTPACTION:", 12) == 0) {
      http_ready = 1;
      goto simcommandexit;
    }
    if (strcmp(line, "OK") == 0) {
      SIMFree = 1;
      cb = cb_ok;
      cb_ok = NULL;
      cb_err = NULL;
      goto simcommandexit;
    }
    if (strcmp(line, "ERROR") == 0) {
      SIMFree = 1;
      cb = cb_err;
      cb_ok = NULL;
      cb_err = NULL;
      goto simcommandexit;
    }
simcommandexit:
    line_i = 0;
    if (cb)
      (*cb)();
  }
  void TaskSimRestart(void) {
    if (sim_restart_timer > millis())
      return;
    Serial.println(F("RESTART SIM"));
    pinMode(SIM_POWER_KEY, OUTPUT);
    digitalWrite(SIM_POWER_KEY, LOW);
    delay(2000);
    pinMode(SIM_POWER_KEY, INPUT);
    sim_restart_timer = millis() + SIM_RESTART_TIMEOUT;
  }
  void TaskSimRead(void) {
    for (byte i = 0; i < 30; i++) {
      if (SIM.available()) {
        sim_restart_timer = millis() + SIM_RESTART_TIMEOUT;
        char c = SIM.read();
        if ((c == '\r' ) || (c == '\n')) {
          if (line_i > 0)
            SimCommand();
          line_i = 0;
          return;
        }
        if (line_i < LINE_SIZE - 1)
          line[line_i++] = c;
      }
    }
  }
  void substrindex(char *c, int i, byte max) {
    int a = 0;
    int b = -1;
    for (int t = 0; t < line_i; t++)
      if (line[t] == ',') {
        i--;
        if (i == 0)
          a = t + 1;
        if (i == -1)
          b = t;
      }
    if ((a > 0) && (b == -1))
      b = line_i;
    c[0] = '\0';
    if (b >= a) {
      if ((b - a) > max)
        b = a + max;
      memcpy(c, &line[a], b - a);
      c[b - a] = '\0';
    }
  }
