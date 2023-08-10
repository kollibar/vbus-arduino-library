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

#include "VBUSDecoder.h"
#define DEBUG 1

/**
 * renvoi une température donnée sous forme de int 1/10e de degrée en float
*/
float floatTempFrom10e(int16_t temp10e){
  if( temp10e == SENSORNOTCONNECTED )
    return 0;

  return (((float)temp10e)/10.0);
}


// Clear all maximum values
void VBUSDecoder::clearMaxValues() {
  sensor1TempMax = 0;
  sensor2TempMax = 0;
  sensor3TempMax = 0;
  sensor4TempMax = 0;
}

bool VBUSDecoder::initialise(){
  Serial1.begin(9600);
  clearMaxValues();
  return true;
} // end void initialise()

int16_t VBUSDecoder::getS1Temp() const {
  return sensor1Temp;
}

int16_t VBUSDecoder::getS2Temp() const {
  return sensor2Temp;
}

int16_t VBUSDecoder::getS3Temp() const {
  return sensor3Temp;
}

int16_t VBUSDecoder::getS4Temp() const {
  return sensor4Temp;
}

float VBUSDecoder::getS1TempFloat() const {
  return floatTempFrom10e(sensor1Temp);
}

float VBUSDecoder::getS2TempFloat() const {
  return floatTempFrom10e(sensor2Temp);
}

float VBUSDecoder::getS3TempFloat() const {
  return floatTempFrom10e(sensor3Temp);
}

float VBUSDecoder::getS4TempFloat() const {
  return floatTempFrom10e(sensor4Temp);
}

bool VBUSDecoder::getP1Status() const {
  return relayPump;
}

bool VBUSDecoder::getP2Status() const {
  return relay3WayValve;
}

int VBUSDecoder::getP1Speed() const {
  return String(Relay1, DEC).toInt();
}

int VBUSDecoder::getP2Speed() const {
  return String(Relay2, DEC).toInt();
}
int VBUSDecoder::getP1OperatingHours() const {
  return String(OperatingHoursRelay1, DEC).toInt();
}

int VBUSDecoder::getP2OperatingHours() const {
  return String(OperatingHoursRelay2, DEC).toInt();
}

int VBUSDecoder::getScheme() const {
  return String(Scheme, DEC).toInt();
}

bool VBUSDecoder::getAlertStatus() const {
  return SystemAlert;
}

String VBUSDecoder::getSystemTime() const {
  int hours = SystemTime / 60;
  int minutes = SystemTime % 60;
  String toReturn = String(String(hours) + ":" + String(minutes));

  if (hours < 10)
  {
    toReturn = "0" + toReturn;
  }

  return toReturn;
}

bool VBUSDecoder::readSensor(long timerInterval){
  bool r=vBusRead(timerInterval);
  if (r) {
#if DEBUG
    Serial.println(F("------Decoded VBus data------"));
    Serial.print(F("Destination: "));
    Serial.println(Destination_address, HEX);
    Serial.print(F("Source: "));
    Serial.println(Source_address, HEX);
    Serial.print(F("Protocol Version: "));
    Serial.println(ProtocolVersion);
    Serial.print(F("Command: "));
    Serial.println(Command, HEX);
    Serial.print(F("Framecount: "));
    Serial.println(Framecnt);
    Serial.print(F("Checksum: "));
    Serial.println(Checksum);
    Serial.println(F("------Values------"));
    Serial.print(F("Sensor 1: "));
    Serial.println(sensor1Temp);
    Serial.print(F("Sensor 2: "));
    Serial.println(sensor2Temp);
    Serial.print(F("Sensor 3: "));
    Serial.println(sensor3Temp);
    Serial.print(F("Sensor 4: "));
    Serial.println(sensor4Temp);
    Serial.print(F("Relay 1: "));
    Serial.println(Relay1, DEC);
    Serial.print(F("Relay 2: "));
    Serial.println(Relay2, DEC);
    Serial.print(F("Minute of Day: "));
    Serial.println(SystemTime);
    Serial.print(F("Notifications: "));
    Serial.println(SystemNotification, DEC);
    Serial.println(F("------END------"));
#endif
  } //end VBusRead

  /*
     S1 = Sensor 1 (sensor SFB/stove)
     S2 = Sensor 2 (sensor store base)
     S3 = Sensor 3 (sensor store top)
     S4 = Sensor 4 (system-dependent)
     R1 = Pump
     R2 = 3-way valve
  */

  // Convert relay value to On or Off.
  if (Relay1 != 0x00){
    relayPump = true;
  } else {
    relayPump = false;
  }

  if (Relay2 == 0x64){
    relay3WayValve = true;
  } else if (Relay2 == 0x00) {
    relay3WayValve = false;
  } else {
    relay3WayValve = false;
  }

  if (SystemNotification != 0x00 || ErrorMask != 0x00) { // Not really sure what ErrorMask is, treating as system alert.
    SystemAlert = true;
  } else {
    SystemAlert = false;
  }

  return r;
}

// The following is needed for decoding the data
void VBUSDecoder::InjectSeptet(unsigned char *Buffer, int Offset, int Length){
  for (int i = 0; i < Length; i++)
  {
    if (Septet & (1 << i))
    {
      Buffer[Offset + i] |= 0x80;
    }
  }

  PrintHex8(&Buffer[Offset], Length);
}

// CRC calculation
// From https://danielwippermann.github.io/resol-vbus/vbus-specification.html
unsigned char VBUSDecoder::VBus_CalcCrc(unsigned char *Buffer, int Offset, int Length){
  unsigned char CRC = 0x7F;

  for (int i = 0; i < Length; i++) {
    CRC = (CRC - Buffer [Offset + i]) & 0x7F;
  }
  return CRC;
}

// The following function reads the data from the bus and converts it all
// depending on the used VBus controller.
bool VBUSDecoder::vBusRead(long timerInterval){
  int F;
  char c;
  const char sync1 = 0xAA;

  bool start = true;
  bool stop = false;
  int Bufferlength = 0;
  unsigned long lastTimeTimer = millis();

  while ((!stop)){
    if (Serial1.available()){
      c = Serial1.read();

      if (c == sync1){

#if DEBUG
// Serial.println(F("Sync found"));
#endif

        if (start){
          start = false;
          Bufferlength = 0;
          //#if DEBUG
          //#endif
        } else {
          if (Bufferlength < 20) {
            lastTimeTimer = millis();
            Bufferlength = 0;
          } else
            stop = true;
        }
      }
#if DEBUG
// Serial.println(c, HEX);
#endif
      if ((!start) and (!stop)) {
        Buffer[Bufferlength] = c;
        Bufferlength++;
      }
    }
    if ((timerInterval > 0) && (millis() - lastTimeTimer > timerInterval)) {
#if DEBUG
      Serial.print(F("Timeout: "));
      Serial.println(lastTimeTimer);
#endif
      return false;
    }
  }

  lastTimeTimer = 0;

  Destination_address = Buffer[2] << 8;
  Destination_address |= Buffer[1];
  Source_address = Buffer[4] << 8;
  Source_address |= Buffer[3];
  ProtocolVersion = (Buffer[5] >> 4) + (Buffer[5] & (1 << 15));

  Command = Buffer[7] << 8;
  Command |= Buffer[6];
  Framecnt = Buffer[8];
  Checksum = Buffer[9]; 
#if DEBUG
  Serial.println(F("---------------"));
  Serial.print(F("Destination: "));
  Serial.println(Destination_address, HEX);
  Serial.print(F("Source: "));
  Serial.println(Source_address, HEX);
  Serial.print(F("Protocol Version: "));
  Serial.println(ProtocolVersion);
  Serial.print(F("Command: "));
  Serial.println(Command, HEX);
  Serial.print(F("Framecount: "));
  Serial.println(Framecnt);
  Serial.print(F("Checksum: "));
  Serial.println(Checksum);
  Serial.println(F("---------------"));

#endif
    // Only analyse Commands 0x100 = Packet Contains data for slave
    // with correct length = 10 bytes for HEADER and 6 Bytes  for each frame

    if ((Command == 0x0100) and (Bufferlength == 10 + Framecnt * 6) and (Checksum == VBus_CalcCrc(Buffer, 1, 8) ) )
    {

      //Only decode the data from the correct source address
      //(There might be other VBus devices on the same bus).

      if (Source_address == 0x3271)
      {
#if DEBUG
        Serial.println(F("---------------"));
        Serial.println(F("Now decoding for 0x3271"));
        Serial.println(F("---------------"));

#endif

        // Frame info for the Resol ConergyDT5
        // check VBusprotocol specification for other products

        // This library is made for the ConergyDT5 (0x3271)

        //Offset  Size    Mask    Name                    Factor  Unit
        //0       2               Temperature sensor 1    0.1     &#65533;C
        //2       2               Temperature sensor 2    0.1     &#65533;C
        //4       2               Temperature sensor 3    0.1     &#65533;C
        //6       2               Temperature sensor 4    0.1     &#65533;C
        //8       1               Pump speed pump         1       1
        //9       1               Pump speed pump 2       1
        //10      1               Relay mask              1
        //11      1               Error mask              1
        //12      2               System time             1
        //14      1               Scheme                  1
        //15      1       1       Option PostPulse        1
        //15      1       2       Option thermostat       1
        //15      1       4       Option HQM              1
        //16      2               Operating hours relay 1 1
        //18      2               Operating hours relay 2 1
        //20      2               Heat quantity           1       Wh
        //22      2               Heat quantity           1000    Wh
        //24      2               Heat quantity           1000000 Wh
        //26      2               Version 0.01
        //
        // Each frame has 6 bytes
        // byte 1 to 4 are data bytes -> MSB of each bytes
        // byte 5 is a septet and contains MSB of bytes 1 to 4
        // byte 6 is a checksum
        //
        //*******************  Frame 1  *******************
        F = FOffset;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
          Septet = Buffer[F + FSeptet];
          InjectSeptet(Buffer, F, 4);
          // 'collector1' Temperatur Sensor 1, 15 bits, factor 0.1 in C
          sensor1Temp = calcTemp(Buffer[F + 1], Buffer[F]);
          // 'store1' Temperature sensor 2, 15 bits, factor 0.1 in C
          sensor2Temp = calcTemp(Buffer[F + 3], Buffer[F + 2]);
        }
        //*******************  Frame 2  *******************
        F = FOffset + FLength;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
          Septet = Buffer[F + FSeptet];
          InjectSeptet(Buffer, F, 4);
          sensor3Temp = calcTemp(Buffer[F + 1], Buffer[F]);
          sensor4Temp = calcTemp(Buffer[F + 3], Buffer[F + 2]);
        }
        //*******************  Frame 3  *******************
        F = FOffset + FLength * 2;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  PumpSpeed1 = (Buffer[F]);
	  PumpSpeed2 = (Buffer[F + 1]);
	  RelaisMask = Buffer[F + 2];
	  ErrorMask = Buffer[F + 3];
	}
        //*******************  Frame 4  *******************
        F = FOffset + FLength * 3;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  SystemTime = Buffer[F + 1] << 8 | Buffer[F];
	  Scheme = Buffer[F + 2];
	  OptionPostPulse = (Buffer[F + 3] & 0x01);
	  OptionThermostat = ((Buffer[F + 3] & 0x02) >> 1);
	  OptionHQM = ((Buffer[F + 3] & 0x04) >> 2);
	}
        //*******************  Frame 5  *******************
        F = FOffset + FLength * 4;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  OperatingHoursRelay1 = Buffer[F + 1] << 8 | Buffer[F];
	  OperatingHoursRelay2 = Buffer[F + 3] << 8 | Buffer[F + 2];
	}
        //*******************  Frame 6  *******************
        F = FOffset + FLength * 5;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  HeatQuantity = (Buffer[F + 1] << 8 | Buffer[F]) + (Buffer[F + 3] << 8 | Buffer[F + 2]) * 1000;
	}
        //*******************  Frame 7  *******************
        F = FOffset + FLength * 6;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  HeatQuantity = HeatQuantity + (Buffer[F + 1] << 8 | Buffer[F]) * 1000000;
	  Version = Buffer[F + 3] << 8 | Buffer[F + 2];
	}
        ///******************* End of frames ****************
      } // end 0x3271 Conenergy DT5

      else if (Source_address == 0x5611)
      {
#if DEBUG
        Serial.println(F("---------------"));
        Serial.println(F("Now decoding for 0x5611"));
        Serial.println(F("---------------"));
#endif
        // Frame info for the Resol Deltatherm FK and Oranier Aquacontrol III
        // check VBusprotocol specification for other products

        //

        //Offset  Size    Mask    Name                    Factor  Unit
        // Frame 1
        //0       2               Temperature sensor 1    0.1     &#65533;C
        //2       2               Temperature sensor 2    0.1     &#65533;C
        // Frame 2
        //4       2               Temperature sensor 3    0.1     &#65533;C
        //6       2               Temperature sensor 4    0.1     &#65533;C
        // Frame 3
        //8       1               Relay 1                 1       %
        //9       1               Relay 2                 1       %
        //10      1               Mixer open              1       %
        //11      1               Mixer closed            1       %
        // Frame 4
        //12      4               System date             1
        // Frame 5
        //16      2               System time             1
        //18      1               System notification     1
        //
        // Each frame has 6 bytes
        // byte 1 to 4 are data bytes -> MSB of each bytes
        // byte 5 is a septet and contains MSB of bytes 1 to 4
        // byte 6 is a checksum
        //
        //*******************  Frame 1  *******************

        F = FOffset;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  sensor1Temp = calcTemp(Buffer[F + 1], Buffer[F]);
	  sensor2Temp = calcTemp(Buffer[F + 3], Buffer[F + 2]);
	}
        //*******************  Frame 2  *******************
        F = FOffset + FLength;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  sensor3Temp = calcTemp(Buffer[F + 1], Buffer[F]);
	  sensor4Temp = calcTemp(Buffer[F + 3], Buffer[F + 2]);
	}
        //*******************  Frame 3  *******************
        F = FOffset + FLength * 2;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  // Some of the values are 7 bit instead of 8.
	  // Adding '& 0x7F' means you are only interested in the first 7 bits.
	  // 0x7F = 0b1111111.
	  // See: http://stackoverflow.com/questions/9552063/c-language-bitwise-trick
	  Relay1 = (Buffer[F] & 0X7F);
	  Relay2 = (Buffer[F + 1] & 0X7F);
	  MixerOpen = (Buffer[F + 2] & 0X7F);
	  MixerClosed = (Buffer[F + 3] & 0X7F);
	}
        //*******************  Frame 4  *******************
        F = FOffset + FLength * 3;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  // System date is not needed for Domoticz
	}
        //*******************  Frame 5  *******************
        F = FOffset + FLength * 4;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
        // System time is not needed for Domoticz

        // Status codes System Notification according to Resol:
        //0: no error / warning
        //1: S1 defect
        //2: S2 defect
        //3: S3 defect
        //4: VFD defect
        //5: Flow rate?
        //6: ΔT too high
        //7: Low water level

	  SystemNotification = Buffer[F + 2];
	}
        ///******************* End of frames ****************

      } //End 0x5611 Resol DeltaTherm FK

      else if (Source_address == 0x4212)
      {

#if DEBUG
        Serial.println(F("---------------"));
        Serial.println(F("Now decoding for DeltaSol C 0x4212"));
        Serial.println(F("---------------"));
#endif
        //*******************  Frame 1  *******************
        F = FOffset;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  sensor1Temp = calcTemp(Buffer[F + 1], Buffer[F]);
	  sensor2Temp = calcTemp(Buffer[F + 3], Buffer[F + 2]);
	}
        //*******************  Frame 2  *******************
        F = FOffset + FLength;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  sensor3Temp = calcTemp(Buffer[F + 1], Buffer[F]);
	  sensor4Temp = calcTemp(Buffer[F + 3], Buffer[F + 2]);
	}
        //*******************  Frame 3  *******************
        F = FOffset + FLength * 2;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  Relay1 = (Buffer[F]);
	  Relay2 = (Buffer[F + 1]);
	  ErrorMask = Buffer[F + 2];
	  Scheme = Buffer[F + 3];
	}
        //*******************  Frame 4  *******************
        F = FOffset + FLength * 3;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  OperatingHoursRelay1 = Buffer[F + 1] << 8 | Buffer[F];
	  OperatingHoursRelay2 = Buffer[F + 3] << 8 | Buffer[F + 2];
	}
        //*******************  Frame 5  *******************
        F = FOffset + FLength * 4;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  HeatQuantity = (Buffer[F + 1] << 8 | Buffer[F]) + (Buffer[F + 3] << 8 | Buffer[F + 2]) * 1000;
	}
        //*******************  Frame 6  *******************
        F = FOffset + FLength * 5;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  HeatQuantity = HeatQuantity + (Buffer[F + 1] << 8 | Buffer[F]) * 1000000;
	  SystemTime = Buffer[F + 3] << 8 | Buffer[F + 2];
	}
        //*******************  Frame 7  *******************
        F = FOffset + FLength * 6;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	}
        ///******************* End of frames ****************

      } // end 0x4212 DeltaSol C
      else if (Source_address == 0x7311){ // Deltasol M, alias Roth B/W Komfort
        // 6 temp frames, 12 sensors
        // Only decoding the first four due to library limitations
        unsigned int frame = 0;
        // Frame 1:
        F = FOffset + frame * FLength;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  sensor1Temp = calcTemp(Buffer[F + 1], Buffer[F]);
	  sensor2Temp = calcTemp(Buffer[F + 3], Buffer[F + 2]);
	}
        // Frame 2:
        frame++;
        F = FOffset + frame * FLength;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Septet = Buffer[F + FSeptet];
	  InjectSeptet(Buffer, F, 4);
	  sensor3Temp = calcTemp(Buffer[F + 1], Buffer[F]);
	  sensor4Temp = calcTemp(Buffer[F + 3], Buffer[F + 2]);
	}
        // Frame 7: Irradiation and unused
        /*
        frame = 7
        F = FOffset + 6 * FLength;
        irradiation = CalcTemp(Buffer[F + 1], Buffer[F]);
        */
        // Frame 8: Pulse counter 1
        // Frame 9: Pulse counter 2
        // Frame 10: Sensor errors: no sensor / shorted
        // Frame 11: Sensors
        // Frame 12: Relays 1-4
        // Frame 13: Relays 5-8
        // Frame 14: Relays 9-12
        frame = 11;
        F = FOffset + frame * FLength;
	if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
	  Relay1 = Buffer[F];
	  Relay2 = Buffer[F + 1];
	}
        // Frame 15: Not used / relays
        // Frame 16: Errors / warnings
        // Frame 17: Version, revision / time
#ifdef DEBUG
        Serial.println(F("Got values"));
        Serial.print(F("Temperature: "));
        Serial.print(sensor1Temp);
        Serial.print(F(", "));
        Serial.print(sensor2Temp);
        Serial.print(F(". Relays: "));
        Serial.print(Relay1);
        Serial.print(F(", "));
        Serial.println(Relay2);
#endif
      } else {


      #if DEBUG
        Serial.println(F("---------------"));
        Serial.print(F("Now decoding for "));
        Serial.print(getNom(Source_address));
        Serial.print(F(" 0x"));
        Serial.println(Source_address, HEX);
        Serial.println(F("---------------"));
        Serial.println();
      #endif


      #if DEBUG
        Serial.println(F("Injection des Septets"));
      #endif
      for(uint8_t i=0;i<Framecnt;i++){
        F = FOffset + FLength * i;
        if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
          Septet = Buffer[F + FSeptet];
          InjectSeptet(Buffer, F, 4);
        }
        Buffer[F+5]=VBus_CalcCrc(Buffer, F, 5);
      }
      

      // recherche dans les frame

        for(uint8_t i=0;i<Framecnt;i++){
          uint8_t bufferPos = FOffset + FLength * i;
          #if DEBUG
          printFrame(i);
          #endif

          if ( Buffer[bufferPos + 5] == VBus_CalcCrc(Buffer, bufferPos, 5) ) { // CRC ok
            switch(Source_address){
              case 0x1121:
                decodingFor0x1121(bufferPos, i);   //DelatSol CS/2
                break;
              case 0x1001:
                decodingFor0x1121(bufferPos, i);   //DelatSol SLT
                break;
              /* Add your own controller ID and code in the if statement */
              default:
                decodingForDefault(bufferPos, i);
            }
          }
        }

      #if DEBUG
        Serial.print(F("=============="));
      #endif
      }

      if (sensor1Temp > sensor1TempMax)
        sensor1TempMax = sensor1Temp;
      if (sensor2Temp > sensor2TempMax)
        sensor2TempMax = sensor2Temp;
      if (sensor3Temp > sensor3TempMax)
        sensor3TempMax = sensor3Temp;
      if (sensor4Temp > sensor4TempMax)
        sensor4TempMax = sensor4Temp;

    } // end if command 0x0100

  return true;
} // end VBusRead()

// This function converts 2 data bytes to a temperature value.
int16_t VBUSDecoder::calcTemp(int Byte1, int Byte2){
  int v;
  v = Byte1 << 8 | Byte2; //bit shift 8 to left, bitwise OR

  if (Byte1 == 0x00)
  {
    v = v & 0xFF;
  }

  if (Byte1 == 0xFF)
    v = v - 0x10000;

  return v;
}

// Prints the hex values of the char array sent to it.
void VBUSDecoder::PrintHex8(unsigned char *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
#if DEBUG
  Serial.print(F("0x"));
  for (int i = 0; i < length; i++)
  {
    if (data[i] < 0x10)
    {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }

  Serial.println();
#endif
}


bool VBUSDecoder::printFrame(uint8_t frameNum){
  uint8_t F = FOffset + FLength * frameNum;

  float sensorTemp;
  Serial.print('#');
  Serial.print(frameNum);
  Serial.print(F("    "));


  if ( Buffer[F + 5] == VBus_CalcCrc(Buffer, F, 5) ) { // CRC ok
    Serial.print(F("  ("));
    for(uint8_t i=0; i<4;i++){
      Serial.print(Buffer[F+i],DEC);
      Serial.print(F("  "));
    }
    Serial.print(')');

    Serial.print(F("    "));

    Serial.print(F("0x"));
    Serial.print(*(uint16_t*)&Buffer[F], HEX);
    Serial.print(F("  "));
    Serial.print(*(uint16_t*)&Buffer[F+2], HEX);

    Serial.print(F("  ("));
    Serial.print(*(uint16_t*)&Buffer[F], DEC);
    Serial.print(F("  "));
    Serial.print(*(uint16_t*)&Buffer[F+2], DEC);
    Serial.print(')');

    Serial.print(F("    T|"));

    sensorTemp = calcTemp(Buffer[F + 1], Buffer[F]);
    Serial.print(floatTempFrom10e(sensorTemp));
    Serial.print(F("°C"));
    Serial.print(F("  "));

    sensorTemp = calcTemp(Buffer[F + 1 + 2], Buffer[F+2]);
    Serial.print(floatTempFrom10e(sensorTemp));
    Serial.print(F("°C"));



    Serial.print(F("      "));
    Serial.print(F("0x"));

    Serial.print(*(uint32_t*)&Buffer[F], HEX);

    Serial.print(F("  ("));
    Serial.print(*(uint32_t*)&Buffer[F], DEC);
    Serial.print(')');

    Serial.println();
    
    return true;
  } else {
    Serial.println(F("erreur CRC"));
    return false;
  }
}


void VBUSDecoder::decodingFor0x1121(uint8_t bufferPos, uint8_t frameNum){
  // Frame info for the DeltaSol CS/2
  // check VBusprotocol specification for other products

  //Offset  Size    Mask    Name                    Factor  Unit
  //0       2               Temperature sensor 1    0.1     &#65533;C
  //2       2               Temperature sensor 2    0.1     &#65533;C
  //4       2               Temperature sensor 3    0.1     &#65533;C
  //6       2               Temperature sensor 4    0.1     &#65533;C
  // ??
  // ??
  // 12     1               Relay 1 (pump)

  switch(frameNum){
    case 0:
      // 'collector1' Temperatur Sensor 1, 15 bits, factor 0.1 in C
      sensor1Temp = calcTemp(Buffer[bufferPos + 1], Buffer[bufferPos]);
      // 'store1' Temperature sensor 2, 15 bits, factor 0.1 in C
      sensor2Temp = calcTemp(Buffer[bufferPos + 3], Buffer[bufferPos + 2]);
      break;
    case 1:
      sensor3Temp = calcTemp(Buffer[bufferPos + 1], Buffer[bufferPos]);
      sensor4Temp = calcTemp(Buffer[bufferPos + 3], Buffer[bufferPos + 2]);
      break;
    case 3:
      Relay1 = Buffer[bufferPos]; 
      relayPump = (Relay1 != 0);
      break;
  }
}

void VBUSDecoder::decodingFor0x1001(uint8_t bufferPos, uint8_t frameNum){
  // Frame info for the Resol DeltaSol SLT
  // check VBusprotocol specification for other products

  //Offset  Size    Mask    Name                    Factor  Unit
  //0
  //1
  //2
  //3
  //4       2               Temperature sensor 1    0.1     &#65533;C
  //6       2               Temperature sensor 2    0.1     &#65533;C
  //8       2               Temperature sensor 3    0.1     &#65533;C
  //10      2               Temperature sensor 4    0.1     &#65533;C
  // ??
  // ??
  //30      1               Relay1 speed            1       %
  //31      1               Relay2 %                2       %
  switch(frameNum){
    case 1:
      // 'collector1' Temperatur Sensor 1, 15 bits, factor 0.1 in C
      sensor1Temp = calcTemp(Buffer[bufferPos + 1], Buffer[bufferPos]);
      // 'store1' Temperature sensor 2, 15 bits, factor 0.1 in C
      sensor2Temp = calcTemp(Buffer[bufferPos + 3], Buffer[bufferPos + 2]);
      break;
    case 2:
      sensor3Temp = calcTemp(Buffer[bufferPos + 1], Buffer[bufferPos]);
      sensor4Temp = calcTemp(Buffer[bufferPos + 3], Buffer[bufferPos + 2]);
      break;
    case 7:
      Relay1 = Buffer[bufferPos+2];
      Relay2 = Buffer[bufferPos+3];
      relayPump = (Relay1 != 0);
      relay3WayValve = (Relay2 != 0);
  }
}

void VBUSDecoder::decodingForDefault(uint8_t bufferPos, uint8_t frameNum){
  // Frame info to try for other product

  //Offset  Size    Mask    Name                    Factor  Unit
  //0       2               Temperature sensor 1    0.1     &#65533;C
  //2       2               Temperature sensor 2    0.1     &#65533;C
  //4       2               Temperature sensor 3    0.1     &#65533;C
  //6       2               Temperature sensor 4    0.1     &#65533;C

  switch(frameNum){
    case 0:
      // 'collector1' Temperatur Sensor 1, 15 bits, factor 0.1 in C
      sensor1Temp = calcTemp(Buffer[bufferPos + 1], Buffer[bufferPos]);
      // 'store1' Temperature sensor 2, 15 bits, factor 0.1 in C
      sensor2Temp = calcTemp(Buffer[bufferPos + 3], Buffer[bufferPos + 2]);
      break;
    case 1:
      sensor3Temp = calcTemp(Buffer[bufferPos + 1], Buffer[bufferPos]);
      sensor4Temp = calcTemp(Buffer[bufferPos + 3], Buffer[bufferPos + 2]);
      break;
  }
}


const __FlashStringHelper * VBUSDecoder::getNom(uint16_t source_adresse){
  switch( Source_address){
    case 0x1121:
      return NOM_0x1121;
    case 0x1001:
      return NOM_0x1001:
    default:
      return NOM_UNKNOW:
  }  
}