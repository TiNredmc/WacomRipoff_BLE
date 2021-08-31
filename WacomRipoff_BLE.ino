/* Wac0m Rifoff BLE, Wac0m Ripoff project ported to Nordic nRF51.
   Coded by TinLethax 2021/08/13 +7
*/

// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEHIDPeripheral.h>
#include "BLEDigitizer.h"

// w9013 required stuffs
#include <Wire.h>
//#define TWI_FREQ 400000L // Define speed used by twi.c, 400KHz i2c

#define WACOM_ADDR      0x09 // Wacom i2c address (7-bit)

#define WACOM_CMD_QUERY0  0x04
#define WACOM_CMD_QUERY1  0x00
#define WACOM_CMD_QUERY2  0x33
#define WACOM_CMD_QUERY3  0x02
#define WACOM_CMD_THROW0  0x05
#define WACOM_CMD_THROW1  0x00
#define WACOM_QUERY_SIZE  19

#define WACOM_INT 3 // P5 (Chip pin is pin 3) as Input interrupt (but use for polling).
#define LED_stat  4 // P6 (Chip pin is pin 4) as LED status.

static uint8_t dataQ[WACOM_QUERY_SIZE];// data array, store the data received from w9013
static uint16_t Xpos;// X axis value keeper.
static uint16_t Yinvert;// Y axis value keeper (for inverting too).
static uint16_t PenPressure;// Pressure value keeper.
static uint8_t usage_report;// FOr reporting Pen tip, Eraser Tip and barrel button

// define pins (varies per shield/board)
#define BLE_REQ   6
#define BLE_RDY   7
#define BLE_RST   5

BLEHIDPeripheral bleHID = BLEHIDPeripheral(BLE_REQ, BLE_RDY, BLE_RST);
BLEDigitizer HIDd;

void w9013_send(uint8_t *buf, size_t len) {
  Wire.beginTransmission(WACOM_ADDR);// Begin transmission
  while (len--) {
    Wire.write(*buf++);
  }
  Wire.endTransmission();// End transmission
}

void w9013_read(uint8_t *buf, size_t len) {
  Wire.requestFrom(WACOM_ADDR, len);
  while (len--) {
    *buf++ = Wire.read();
  }
}

// w9013 initialization
uint8_t w9013_query_device() {
  uint8_t cmd1[4] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1,
                      WACOM_CMD_QUERY2, WACOM_CMD_QUERY3
                    };
  uint8_t cmd2[2] = { WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
  uint16_t fwVer = 0;

  // Send 2 query CMDs
  w9013_send(cmd1, 4);
  w9013_send(cmd2, 2);
  // Receive the respond
  w9013_read(dataQ, WACOM_QUERY_SIZE);

  fwVer = dataQ[13] | dataQ[14] << 8;// parsing firmware version
  if (!fwVer) { // If FWversion is 0
    return 1; // error happened.
  }
  return 0;// nomal
}

void w9013_poll() {
  do {
    w9013_read(dataQ, WACOM_QUERY_SIZE);
  } while (digitalRead(WACOM_INT) == LOW);

  //digitalWrite(LED_stat, HIGH);

  /* I rewrite the descriptor to make all the usage reports align with the actual data byte received from w9013,
     So we can send the dataQ[3] directly without switch case slowing it down
  */
  /* switch (dataQ[3]) {
     case 0x20: // Pen is in range (Windows determine as Tip is in range)
       usage_report = 0x20;
       break;

     case 0x21: // Pen tip is pressing on surface
       usage_report = 0x21;
       break;

     case 0x28: // Eraser is in range
       usage_report = 0x22;// Set in-range and invert bit
       break;

     case 0x2c: // Erase is pressing on surface
       usage_report = 0x2A;// Set in-rage bit and also invert and eraser bit.
       break;

     default:
       break;
    }*/

  //usage_report = usage_report | ((dataQ[3] & 0x02 ? 1 : 0) << 2) | ((dataQ[3] & 0x10 ? 1 : 0) << 1);

  // X position report
  Xpos = dataQ[6] | dataQ[7] << 8;// Send lower byte | higher byte << 8

  // Y position report
  Yinvert = dataQ[5] << 8 | dataQ[4];
  Yinvert = 0x344E - Yinvert;// Invert Y axis

  // Pressure report
  PenPressure = dataQ[8] | dataQ[9] << 8;// Send lower byte first | higher byte << 8

  //digitalWrite(LED_stat, LOW);

}

void setup() {
  Wire.setPins(2, 1);// set SDA, SCL pin (module pin : P4 and P3)
  Wire.begin();
  Wire.setClock(400000);

  pinMode(WACOM_INT, INPUT_PULLUP);
  pinMode(LED_stat, OUTPUT);

  while (w9013_query_device()) { // fast blink while trying to probe the w9013
    digitalWrite(LED_stat, HIGH);
    delay(100);
    digitalWrite(LED_stat, LOW);
    delay(100);
  }
  digitalWrite(LED_stat, HIGH);
  delay(100);
  digitalWrite(LED_stat, LOW);
  delay(100);

  // clears bond data on every boot
  bleHID.clearBondStoreData();
  // Set Bluetooth name
  bleHID.setDeviceName("Wac0m RipOff BLE");
  // Set local name as HID
  bleHID.setLocalName("WC0M");
  // Add HID device (BLEDigitizer)
  bleHID.addHID(HIDd);
  // Init the Bluetooth HID
  bleHID.begin();

}

void loop() {
  BLECentral central = bleHID.central();

  if (central) {
    // central connected to peripheral
    //digitalWrite(LED_stat, HIGH);
    while (central.connected()) {
      if (digitalRead(WACOM_INT) == LOW) {
        w9013_poll();
        HIDd.DigitizerReport(dataQ[3], Xpos, Yinvert, PenPressure);
        delay(1);// delay 1ms, throttle down the crazy polling rate.
      }
      // Send Bluetooth HID report
    }

    // central disconnected
    if (digitalRead(WACOM_INT) == LOW)
        w9013_poll();// poll in case of Wacom w9013 stuck after BLE disconnected, this will help when reconnect
  }
}
