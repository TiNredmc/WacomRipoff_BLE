/* BLE Digitizer for Wac0m Ripoff project
 * Coded by TinLethax 2021/08/13 +7
 */
 
 #include "BLEDigitizer.h"
 
 static const PROGMEM unsigned char HID_Digi[] = {
	 0x05, 0x0D,								// Usage page : Digitizer
		0x09, 0x02,								// Usage : Pen
		0xA1, 0x01,								// Collection : Application
			0x85, 0x02,							// Report ID for pen (report no 0x02).
			0x09, 0x20,							// Usage : Stylus
			0xA1, 0x00,							// Collection : Physical
				// Boolean 1 and 0 (1 means present, 0 means not present).
				0x09, 0x42,						// Usage : Pen Tip
				0x09, 0x3C,						// Usage : Invert (Eraser Tip)
				0x09, 0x44,						// Usage : 1st Barrel Button
				0x09, 0x45,						// Usage : Eraser Tip
				0x09, 0x5A,						// Usage : 2nd Barrel Button
				0x09, 0x32,						// Usage : pen is in-range
				0x25, 0x01,						// Logical Max is 1
				0x15, 0x00,						// Logical Min is 0
				0x75, 0x01,						// Report size is 6 usages
				0x95, 0x06,						// Report count is 1
				0x81, 0x02,						// Input (Data, Var, Abs)
				// fill all remain bit to make it byte align (8n).
				0x75, 0x01,						// 1 Null usage
				0x95, 0x02,						// fills 2 bits
				0x81, 0x83,						// Input (Const, Var, Abs)

				// Usage Page Generic Desktop will report X,Y coordinate and Pen pressure
				0x05, 0x01,						// Usage Page : Generic Desktop

				0x09, 0x30,						// Usage : X axis
				0x26, 0xB0, 0x53,				// Logical Max is 21424 (according to my w9013 feature report)
				0x15, 0x00,						// Logical Min is 0
				0x55, 0x0d,						// Unit Exponent (-3)
				0x65, 0x11,						// Unit (cm)
				0x75, 0x10,						// Report Size 16 (16bit - 2bytes)
				0x95, 0x01,						// Report count is 1
				0x81, 0x02,						// Input (Data, Var, Abs)

				0x09, 0x31,						// Usage : Y axis
				0x26, 0x4E, 0x34,				// Logical Max is 13390 (according to my w9013 feature report)
				0x15, 0x00,						// Logical Min is 0
				0x55, 0x0d,						// Unit Exponent (-3)
				0x65, 0x11,						// Unit (cm)
				0x75, 0x10,						// Report Size 16 (16bit - 2bytes)
				0x95, 0x01,						// Report count is 1
				0x81, 0x02,						// Input (Data, Var, Abs)

				// Pen tip pressure require Digitizer as a Usage page
				0x05, 0x0D,						// Usage Page : Digitizer
				0x09, 0x30,						// Usage : Tip Pressure
				0x26, 0xFF, 0x07,				// Logical Max is 2047 (according to my w9013 capability, This is Wacom EMR)
				0x15, 0x00,						// Logical Min is  0
				0x75, 0x10,						// Report Size 16 (16bit - 2bytes)
				0x95, 0x01,						// Report count is 1
				0x81, 0x02,						// Input (Data, Var, Abs)

			0xC0,								// End Collection (Phy)
		0xC0
 };
 
 
BLEDigitizer::BLEDigitizer() :
  BLEHID(HID_Digi, sizeof(HID_Digi), 11),
  _reportCharacteristic("2a4d", BLERead | BLENotify, 4),
  _reportReferenceDescriptor(BLEHIDDescriptorTypeInput),
  _button(0)
{
}

void BLEDigitizer::DigitizerReport(uint8_t rpt, uint16_t Xco, uint16_t Yco, uint16_t Pressure) {
  unsigned char HIDReport[8]= { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // copy to array before report HID event

  HIDReport[1] = rpt;// bit description : [7 - not used][6 - not used][5 - in-range][4 - 2nd Barrel Button][3 - Eraser Tip][2 - 1st Barrel Button][1 - Invert][0 - Pen Tip]
  HIDReport[2] = Xco;
  HIDReport[3] = Xco >> 8;
  HIDReport[4] = Yco;
  HIDReport[5] = Yco >> 8;
  HIDReport[6] = Pressure;
  HIDReport[7] = Pressure >> 8;

  this->sendData(this->_reportCharacteristic, HIDReport, sizeof(HIDReport));
}

void BLEDigitizer::setReportId(unsigned char reportId) {
  BLEHID::setReportId(reportId);

  this->_reportReferenceDescriptor.setReportId(reportId);
}

unsigned char BLEDigitizer::numAttributes() {
  return 2;
}

BLELocalAttribute** BLEDigitizer::attributes() {
  static BLELocalAttribute* attributes[2];

  attributes[0] = &this->_reportCharacteristic;
  attributes[1] = &this->_reportReferenceDescriptor;

  return attributes;
}