#include "Ublox.h"
#define N_FLOATS 3

Ublox M8_Gps;
float gpsArray[N_FLOATS] = {0, 0, 0};


#include <SoftwareSerial.h>
//SoftwareSerial mySerial(RX,TX);
SoftwareSerial mySerial(10, 11);



void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600);
}

void loop()
{
   delay(10);
  if (!mySerial.available())
    return;

  while (mySerial.available())
  {
    char c = mySerial.read();
    if (M8_Gps.encode(c))
    {
      //gpsArray[0] = M8_Gps.altitude;
      gpsArray[0] = M8_Gps.latitude;
      gpsArray[1] = M8_Gps.longitude;
      gpsArray[2] = M8_Gps.sats_in_use;
      gpsArray[2] = M8_Gps.speed;
      
    }
    
  }
  for (byte i = 0; i < 4; i++)
  {
    if (i == 2)
    {
      Serial.print(gpsArray[i], 0); Serial.println("");
    }
    else
    {
      Serial.print(gpsArray[i], 6); Serial.print("_");
    }
  }
}
