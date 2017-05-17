#include <mcp_can.h>
#include <SPI.h>

#define UART         // Enable debug to serial console
#define WDT          // Enable watchdog. Use only if you bootloader support it!

#ifdef WDT
#include <avr/wdt.h>
#define MAX_CAN_INTERVAl_MS 5000
unsigned long lastCan0;
unsigned long lastCan1;
#endif

unsigned long rxId;
byte len;
byte rxBuf[8];
unsigned long reverse;
char msgString[128];

#define REAR_CAM 4                             // pin 4 - rear cam relay
#define REAR_CAM_DELAY_MS 10000                // 10 second delay

#define CAN0_INT 2                             // Set INT to pin 2
#define CAN1_INT 3                             // Set INT to pin 3
#define CAN0_CS 10                             // CAN0 interface usins CS on digital pin 10
#define CAN1_CS 9                              // CAN1 interface usins CS on digital pin 9
MCP_CAN CAN0(CAN0_CS);
MCP_CAN CAN1(CAN1_CS);

#define VAG_CAN_REVERSE_GEAR_SWITCH  0x54b      // example: 'Standard ID: 0x54B  DLC: 8  Data: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF'
#define VAG_CAN_WHEEL_STEERING       0x5c1      // example: 'Standard ID: 0x5C1  DLC: 4  Data: 0x01 0x00 0x00 0x53'

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

void setup(){
#ifdef WDT
  wdt_enable(WDTO_8S);
#endif
  pinMode(REAR_CAM, OUTPUT);
  pinMode(CAN0_CS, OUTPUT);
  pinMode(CAN1_CS, OUTPUT);
  pinMode(CAN0_INT, INPUT);
  pinMode(CAN1_INT, INPUT);
#ifdef UART
  Serial.begin(115200);
#endif
  if (CAN0.begin(MCP_NORMAL, CAN_100KBPS, MCP_8MHZ) == CAN_OK) {
#ifdef UART
    Serial.print("CAN0: Init OK!\r\n");
#endif
    CAN0.setMode(MCP_NORMAL);
  }
#ifdef UART
  else Serial.print("CAN0: Init Fail!!!\r\n");
#endif
  if (CAN1.begin(MCP_NORMAL, CAN_100KBPS, MCP_8MHZ) == CAN_OK) {
#ifdef UART
    Serial.print("CAN1: Init OK!\r\n");
#endif
    CAN1.setMode(MCP_NORMAL);
  }
#ifdef UART
  else Serial.print("CAN1: Init Fail!!!\r\n");
#endif
  SPI.setClockDivider(SPI_CLOCK_DIV2);         // Set SPI to run at 8MHz (16MHz / 2 = 8 MHz)
  //  reverse = millis() + 9000;
  //  rxBuf[0] = 0x11;
  //  rxId = 0x222;
  //  CAN1.sendMsgBuf(rxId, 0, 1, rxBuf);
  //  rxBuf[0] = 0x22;
  //  rxId = 0x333;
  //  CAN0.sendMsgBuf(rxId, 0, 1, rxBuf);

}
void loop() {
  rxId = 0;
  if (!digitalRead(CAN0_INT)) {                // If pin 2 is low, read CAN0 receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);       // Read data: len = data length, buf = data byte(s)
#ifdef UART
    printMsg(0);
#endif
    modifyKey();
    checkReverseId();
    CAN1.sendMsgBuf(rxId, 0, len, rxBuf);      // Immediately send message out CAN1 interface
#ifdef WDT
    lastCan0 = millis();
#endif
  }
  if (!digitalRead(CAN1_INT)) {                // If pin 3 is low, read CAN1 receive buffer
    CAN1.readMsgBuf(&rxId, &len, rxBuf);       // Read data: len = data length, buf = data byte(s)
#ifdef UART
    printMsg(1);
#endif
    modifyKey();
    checkReverseId();
    CAN0.sendMsgBuf(rxId, 0, len, rxBuf);      // Immediately send message out CAN0 interface
#ifdef WDT
    lastCan1 = millis();
#endif
  }
  checkReverseTime();
#ifdef WDT
  checkResetWdt();
#endif
}
#ifdef UART
void printMsg(byte direct) {
  if ((rxId == VAG_CAN_REVERSE_GEAR_SWITCH) ||
      (rxId == VAG_CAN_WHEEL_STEERING)) {
    sprintf(msgString, "%d 0x%.3lX  DLC: %1d  Data:", direct, rxId, len);
    Serial.print(msgString);
    for (byte i = 0; i < len; i++) {
      sprintf(msgString, " 0x%.2X", rxBuf[i]);
      Serial.print(msgString);
    }
    Serial.println();
  }
}
#endif UART
#ifdef WDT
void checkResetWdt() {
  if ((lastCan0 + MAX_CAN_INTERVAl_MS) < millis())
    return;
  if ((lastCan1 + MAX_CAN_INTERVAl_MS) < millis())
    return;
  wdt_reset();
}
#endif
void modifyKey() {
  if (rxId != VAG_CAN_WHEEL_STEERING)
    return;
  if (rxBuf[0] == VAG_WHEEL_STEERING_KEY_MEDIA)
    rxBuf[0] = VAG_WHEEL_STEERING_KEY_PHONE;
}
void checkReverseId() {
  if (rxId != VAG_CAN_REVERSE_GEAR_SWITCH)
    return;
  reverse = millis() + REAR_CAM_DELAY_MS;
}
void checkReverseTime() {
  if (reverse == 0)
    return;
  if (millis() > reverse) {               // When time is up, turn off camera
    digitalWrite(REAR_CAM, LOW);          //
    reverse = 0;                          // and reset timer
  } else
    digitalWrite(REAR_CAM, HIGH);         // else turn on camera
}

