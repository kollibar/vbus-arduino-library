/** Copyright (c) 2017 - 'FatBeard' @ www.domoticz.com/forum
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge, publish, distribute,
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
* TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.*/

#ifndef vbusdecoder_h
#define vbusdecoder_h

#include <Arduino.h>

#if !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
#include <AltSoftSerial.h>
#endif

// Settings for the VBus decoding
#define FLength 6				// Framelength
#define FOffset 10				// Offset start of Frames
#define FSeptet 4				// Septet byte in Frame
#define SENSORNOTCONNECTED 8888 // Sometimes this might be 888 instead.

float floatTempFrom10e(int16_t);

class VBUSDecoder
{
  public:
	bool initialise();
	int16_t getS1Temp();
	int16_t getS2Temp();
	int16_t getS3Temp();
	int16_t getS4Temp();

	float getS1TempFloat();
	float getS2TempFloat();
	float getS3TempFloat();
	float getS4TempFloat();

	bool readSensor(long timerInterval=(2*60*1000));
	bool getP1Status();
	bool getP2Status();
	bool getAlertStatus();
	int getP1Speed();
	int getP2Speed();
	int getP1OperatingHours();
	int getP2OperatingHours();
	int getScheme();
	String getSystemTime();

	bool printFrame(uint8_t frameNum);

  protected:
  private:
#if !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
	AltSoftSerial Serial1;
#endif

	bool relayPump = false;
	bool relay3WayValve = false;
	bool SystemAlert = false;

	int16_t sensor1Temp;
	int16_t sensor2Temp;
	int16_t sensor3Temp;
	int16_t sensor4Temp;

	int16_t sensor1TempMax;
	int16_t sensor2TempMax;
	int16_t sensor3TempMax;
	int16_t sensor4TempMax;

	// Conergy DT5 specific
	char PumpSpeed1; // in  %
	char PumpSpeed2; //  in %
	char RelaisMask;
	char ErrorMask;
	char Scheme;
	char OptionPostPulse;
	char OptionThermostat;
	char OptionHQM;
	uint16_t OperatingHoursRelay1;
	uint16_t OperatingHoursRelay2;
	uint32_t HeatQuantity;
	uint16_t Version;
	uint16_t OperatingHoursRelais1Today;
	uint16_t SystemTime;

	char Relay1;	  // in  %
	char Relay2;	  //  in %
	char MixerOpen;   // in  %
	char MixerClosed; // in  %
	char SystemNotification;
	//

	// These are set neither to 'on' or 'off' at initialization so at startup the value
	// from the first valid datagram will be sent to Domoticz.
	String lastRelay1 = "1";
	String lastRelay2 = "1";
	String lastSystemAlert = "1";

	unsigned char Buffer[80];
	volatile unsigned char Bufferlength;

	unsigned int Destination_address;
	unsigned int Source_address;
	unsigned char ProtocolVersion;
	unsigned int Command;
	unsigned char Framecnt;
	unsigned char Septet;
	unsigned char Checksum;
	long lastTimeTimer;

	void clearMaxValues();
	bool vBusRead(long timerInterval=0);
	int16_t calcTemp(int Byte1, int Byte2);
	void InjectSeptet(unsigned char *Buffer, int Offset, int Length);
	void PrintHex8(unsigned char *data, uint8_t length); // prints 8-bit data in hex with leading zeroes
	unsigned char VBus_CalcCrc(unsigned char *Buffer, int Offset, int Length);
};

#endif
