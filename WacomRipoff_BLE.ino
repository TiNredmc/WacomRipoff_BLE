/* Wac0m Rifoff BLE, Wac0m Ripoff project ported to Nordic nRF51.
   Coded by TinLethax 2021/08/13 +7
*/
/* useful data
    https://lwn.net/ml/linux-kernel/20210818154935.1154-7-alistair@alistair23.me/
    https://lwn.net/ml/linux-kernel/20210326015229.141-5-alistair%40alistair23.me/
    https://github.com/reMarkable/linux/blob/zero-colors/drivers/input/touchscreen/wacom_i2c.c
*/

// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEHIDPeripheral.h>
#include "BLEDigitizer.h"

// library for MAX17043 fuel gauge https://github.com/porrey/max1704x
#include "MAX17043.h"

// w9013 required stuffs
#include <Wire.h>
//#define TWI_FREQ 400000L // Define speed used by twi.c, 400KHz i2c

#define WACOM_ADDR      0x09 // Wacom i2c address (7-bit)

/* Byte order marker */
// for commands
#define WACOM_CMD_LSB  0x04// COMMAND LSB
#define WACOM_CMD_MSB  0x00// COMMAND MSB
// for data
#define WACOM_DAT_LSB  0x05// DATA LSB
#define WACOM_DAT_MSB  0x00// DATA MSB 

/* Opcodes */
#define WACOM_OPC_RST   0x01// Opcode : reset
#define WACOM_OPC_GRP   0x02// Opcode : Get report 
#define WACOM_OPC_SRP   0x03// Opcode : Set report
#define WACOM_OPC_SPWR  0x08// Opcode : Set power modes (Sleep / Wake)

/* power modes */
#define PWR_ON    0x00
#define PWR_OFF   0x01

/* report types */
#define WACOM_CMD_RIN   0x10// report input command 
#define WACOM_CMD_ROT   0x20// report output command
#define WACOM_CMD_RPF   0x30// report feature command

/* Feature report */
#define WACOM_QUERY_REPORT  0x03// Query report command

#define WACOM_QUERY_SIZE  19

#define WACOM_INT 3 // P5 (Chip pin is pin 3) as Input interrupt (but use for polling).
#define LED_stat  4 // P6 (Chip pin is pin 4) as LED status.
#define BAT_LOW   9 // P7 (Chip pin is pin 9) as Low battery interrupt.

static uint8_t dataQ[WACOM_QUERY_SIZE];// data array, store the data received from w9013
static uint16_t Xpos;// X axis value keeper.
static uint16_t Yinvert;// Y axis value keeper (for inverting too).
static uint16_t PenPressure;// Pressure value keeper.
static uint8_t usage_report;// FOr reporting Pen tip, Eraser Tip and barrel button

// for fuel gauge status
uint8_t gauge_failed = 0;// if probing is failed, this will set to 1 and will avoid atempting to read from fuel gauge.
uint8_t bat_low = 0;// set to 1 when battery lvls is at or lower that 10%

// define pins (varies per shield/board)
#define BLE_REQ   6
#define BLE_RDY   7
#define BLE_RST   5

BLEHIDPeripheral bleHID = BLEHIDPeripheral(BLE_REQ, BLE_RDY, BLE_RST);
BLEDigitizer HIDd;

// Thanks https://github.com/kriswiner/nRF52832DevBoard/blob/master/BMP280_nRF52.ino https://developer.apple.com/forums/thread/77866
// for the battery report example.
// Battery Service
BLEService batteryService = BLEService("180F");
BLEUnsignedCharCharacteristic battlevelCharacteristic = BLEUnsignedCharCharacteristic("2A19", BLERead | BLENotify); // battery level is uint8_t
BLEDescriptor battlevelDescriptor = BLEDescriptor("2901", "Battery Level 0 - 100");

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
  uint8_t cmd1[4] = { WACOM_CMD_LSB, WACOM_CMD_MSB, // set byte order LSB first MSB last
                      WACOM_CMD_RPF | WACOM_QUERY_REPORT,// Report feature by query report from w9013
                      WACOM_OPC_GRP // run opcode "Get report" to get the report from w9013
                    };
  uint8_t cmd2[2] = { WACOM_DAT_LSB, WACOM_DAT_MSB // report data is LSB byte first MSB byte last
                    };
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

/* Experiment soon */
// Power setting, Put to sleep by set to 0, Wake by set to 1
void w9013_pwr_set(bool pwrset) {
  uint8_t cmd1[6] = { WACOM_CMD_LSB, WACOM_CMD_MSB,// CMD LSB byte frist
                      pwrset ? PWR_ON : PWR_OFF, // set sleep /wake
                      WACOM_OPC_SPWR,// by the Set power mode command
                      WACOM_DAT_LSB, WACOM_DAT_MSB// DATA LSB byte first
                    };

  w9013_send(cmd1, 6);// send the packet to w9013
  w9013_read(cmd1, 6);// dummy read from w9013, do nothing with the data.
}

// Data polling from w9013
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
  pinMode(WACOM_INT, INPUT_PULLUP);
  pinMode(LED_stat, OUTPUT);
  pinMode(BAT_LOW, INPUT);

  digitalWrite(LED_stat, HIGH);
  delay(100);
  digitalWrite(LED_stat, LOW);
  delay(100);

  Wire.setPins(2, 1);// set SDA, SCL pin (module pin : P4 and P3)
  Wire.setClock(400000);
  Wire.begin();
  

  while (w9013_query_device()) { // fast blink while trying to probe the w9013
    digitalWrite(LED_stat, HIGH);
    delay(100);
    digitalWrite(LED_stat, LOW);
    delay(100);
  }

  // fuel gauge probing
  if (FuelGauge.begin()) {
    FuelGauge.setThreshold(10);// battery Threshold is at 10%
    FuelGauge.quickstart();// quick reset the gauge to restart the calculation.
    gauge_failed = 0;
  } else { // slow blink to indicates fuel gauge probing error and then continue
    digitalWrite(LED_stat, HIGH);
    delay(500);
    digitalWrite(LED_stat, LOW);
    delay(500);
    digitalWrite(LED_stat, HIGH);
    delay(500);
    digitalWrite(LED_stat, LOW);
    delay(500);
    gauge_failed = 1;
  }


  // Battery service
  bleHID.setAdvertisedServiceUuid(batteryService.uuid());
  bleHID.addAttribute(batteryService);
  bleHID.addAttribute(battlevelCharacteristic);
  bleHID.addAttribute(battlevelDescriptor);

  // clears bond data on every boot
  bleHID.clearBondStoreData();
  // Set Bluetooth name
  bleHID.setDeviceName("Wac0m RipOff BLE");
  // Set local name as HID
  bleHID.setLocalName("Wac0m RipOff BLE");
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
        HIDd.DigitizerReport(dataQ[3], Xpos, Yinvert, PenPressure);// Send Bluetooth HID report
        delayMicroseconds(10);// delay 10us, throttle down the crazy polling rate.
      }

      // Send battery report
      if (!gauge_failed) {
        battlevelCharacteristic.setValue((uint8_t)(FuelGauge.percent()));
        if (!digitalRead(BAT_LOW)) {
          // Low battery alert
          digitalWrite(LED_stat, HIGH);
          FuelGauge.clearAlert();// clear fuel gauge alert bit.
          bat_low = 1;
        }
      }

    }

    // central disconnected
    if (digitalRead(WACOM_INT) == LOW)
      w9013_poll();// poll in case of Wacom w9013 stuck after BLE disconnected, this will help when reconnect
  }

}
