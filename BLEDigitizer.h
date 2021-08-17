/* BLE Digitizer for Wac0m Ripoff project
 * Library based on Sandeep Mistry's BLEMouse library.
 * Coded by TinLethax 2021/08/13 +7
 */
 
#ifndef BLE_DIGITIZER
#define BLE_DIGITIZER

#include "Arduino.h"

#include "BLECharacteristic.h"
#include "BLEHIDReportReferenceDescriptor.h"
#include "BLEHID.h"

// Digitizer report for Pen side, Eraser Side and barrel buttons
#define DIGIT_PEN_TIP_INRANGE 	0x20 // Pen side is hovering. 
#define DIGIT_PEN_TIP_DOWN		0x21 // Pen side touch the digitizer surface.
#define DIGIT_ERASER_INRANGE	0x22 // Eraser side is hovering.
#define DIGIT_ERASER_DOWN		0x2A // Eraser side touch the digitizer surface.

#define DIGIT_BTN1 				0x04 // First Barrel Button is pressed.
#define DIGIT_BTN2				0x10 // Second Barrel Button is pressed.

class BLEDigitizer : public BLEHID
{
  public:
    BLEDigitizer();

    void DigitizerReport(uint8_t rpt, uint16_t Xco, uint16_t Yco, uint16_t Pressure);

  protected:
    virtual void setReportId(unsigned char reportId);
    virtual unsigned char numAttributes();
    virtual BLELocalAttribute** attributes();

  private:
    BLECharacteristic                 _reportCharacteristic;
    BLEHIDReportReferenceDescriptor   _reportReferenceDescriptor;

    unsigned char                     _button;
};

#endif