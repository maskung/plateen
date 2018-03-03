/*
* Plateen Remote Control
*     loop for sending control message
*     and recieve sensor data
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SD.h>

#include <Wire.h> // must be included here so that Arduino library object file references work
#include <RtcDS1307.h>
RtcDS1307<TwoWire> Rtc(Wire);

const int chipSelect = 10;    // SD card ship select
//const int chipSelect = 8;    // SD card ship select
#define  CONNECT_STATE_PIN 6   // connect status between remote and driver
int state_connected = 0;
const long disconnect_interval = 5000;
unsigned long previousMillis = 0;

//#define led 12
//RF24 radio(9, 10); // CE, CSN
RF24 radio(9, 7); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;

const byte PIN_ANALOG_X = 0;
const byte PIN_ANALOG_Y = 1;

char walk_state[1] = "";
int x,y;

void setup() {
  pinMode(CONNECT_STATE_PIN, OUTPUT);
  digitalWrite(CONNECT_STATE_PIN,LOW);
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00001
  radio.openReadingPipe(1, addresses[0]); // 00002
  radio.setPALevel(RF24_PA_MIN);
  Serial.println("Start Reading");

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  Serial.print("localtime: ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

  Rtc.Begin();

  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  Serial.println();

  /*if (!Rtc.IsDateTimeValid()) 
    {
       
        Serial.println("RTC lost confidence in the DateTime,Please check battery!");
        Rtc.SetDateTime(compiled);
    }

    if (!Rtc.GetIsRunning())
    {
        Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) 
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        Serial.println("RTC is the same as compile time! (not expected but all is fine)");
    }
  */
    // never assume the Rtc was last configured by you, so
    // just clear them to your needed state
    Rtc.SetSquareWavePin(DS1307SquareWaveOut_Low); 
   
}
void loop() {
  //delay(5);
  radio.startListening();
  while (!radio.available()) {

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= disconnect_interval) {
      // save the last time it works
      previousMillis = currentMillis;
      
      state_connected = 0;
      digitalWrite(CONNECT_STATE_PIN,LOW);
    }
  }

  if (radio.available()) {
    
    char text[128] = "";
    char datestr[256];
    radio.read(&text, sizeof(text));
    Serial.println(text);

    //if can get message set led status to on
    if (strcmp(text,"") != 0) {
      state_connected = 1;
    } else {
      state_connected = 0;
    }

    if (state_connected == 1) {
      digitalWrite(CONNECT_STATE_PIN,HIGH);
    } else {
      digitalWrite(CONNECT_STATE_PIN,LOW);
    }
    
    
    RtcDateTime now = Rtc.GetDateTime();

    //datestr = printDateTime(now);
      
    strcpy(datestr,printDateTime(now));
    strcat(datestr,",");
    strcat(datestr,text);
    Serial.println(datestr);

    
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("envlog.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(datestr);
      dataFile.close();      
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
    
  }
  //delay(100);
  radio.stopListening();

    x = analogRead(PIN_ANALOG_X);
    y = analogRead(PIN_ANALOG_Y);
    Serial.print("x:");
    Serial.print(analogRead(PIN_ANALOG_X));
    Serial.print(" ");
  
    Serial.print("y:");
    Serial.print(analogRead(PIN_ANALOG_Y));
    Serial.print(" ");  
  
    Serial.println();

    
    if ( x >= 300 && x <= 623 && y >= 1015) {      
       //forward    
       radio.write("W", sizeof("W"));
       Serial.println("Forward");
    } else if ( x <= 5 && y >= 480 && y <= 490) {
       // turn left
       radio.write("A", sizeof("A"));
       Serial.println("Turn left");
    } else if ( x >= 1015 && y >= 480 && y <= 490) {
       radio.write("D", sizeof("D"));
       Serial.println("Turn right");
    } else radio.write("S", sizeof("S"));
  
  /*radio.read(&buttonState, sizeof(buttonState));*/
  
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

 char *printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Day(),
            dt.Month(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    //Serial.print(datestring);
    return datestring;
}

char *printDate_(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u_%02u_%04u_%02u_%02u_%02u"),
            dt.Day(),
            dt.Month(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    //Serial.print(datestring);
    return datestring;
}
