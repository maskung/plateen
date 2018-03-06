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

  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  Serial.print(F("localtime: "));
  Serial.print(__DATE__);
  Serial.print(F(" "));
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

char text[128] = "";
File dataFile;
void loop() {
  static  int state_rev =0;
  
  if (state_connected == 1) {
    digitalWrite(CONNECT_STATE_PIN,HIGH);
  } else {
    digitalWrite(CONNECT_STATE_PIN,LOW);
  }
    
  //delay(5);
  radio.startListening();
  while (!radio.available() ) {

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= disconnect_interval) {
      // save the last time it works
      previousMillis = currentMillis;
      
      state_connected = 0;
      digitalWrite(CONNECT_STATE_PIN,LOW);
    }
    
  }

  state_rev = (state_rev>=5)? 0: state_rev ;
  
  if (radio.available()) {
   
    char datestr[254];
    char tmp_str[33];
    //char head[];
    memset(tmp_str, 0, sizeof(tmp_str));
    radio.read(&tmp_str, sizeof(tmp_str));
    //Serial.println(tmp_str);
   
    switch(state_rev){
     case 0:    
               if(strcmp(tmp_str,"begin") == 0){
                 state_rev = 1;
               }
     break;
     case 1:   //memset(head,0 sizeof(head));
               //memcpy( head, tmp_str , 5 );
               //memcpy(text, &tmp_str[5] , strlen(tmp_str)+1-5 );
               if(   strstr(tmp_str,"DATA1")  !=  NULL ){
                  memset(text, 0, sizeof(text)); 
                  strcpy(text,&tmp_str[5]);
                  state_rev =2 ;
                }
     break;   
     case 2:   if(   strstr(tmp_str,"DATA2")  !=  NULL ){
                 strcat(text,&tmp_str[5]);
                 state_rev =3 ;
               }
     break;
     case 3:  if(   strstr(tmp_str,"DATA3")  !=  NULL ){
                strcat(text,&tmp_str[5]);
                state_rev =4 ;
               }
     break;
     case 4: 
             if(   strstr(tmp_str,"DATA4")  !=  NULL ){
                strcat(text,&tmp_str[5]);
                state_rev =5 ;
             }
     break;     
    }
    //Serial.println(String("Epn: ")+text);
     

    //if can get message set led status to on
    if (strcmp(text,"") != 0) {
      state_connected = 1;
    } else {
      state_connected = 0;
    }

   
 
    if(state_rev == 5 ){    
       RtcDateTime now = Rtc.GetDateTime();

      //datestr = printDateTime(now);
        
      strcpy(datestr,printDateTime(now));
      strcat(datestr,",");
      strcat(datestr,text);
      Serial.println(datestr);

      // open the file. note that only one file can be open at a time,
      // so you have to close this one before opening another.
      if (!SD.exists("envlog4.txt")) {
        Serial.println(F("envlog4.txt not exists."));
        Serial.println(F("Creating envlog4.txt..."));
        dataFile = SD.open("envlog4.txt", FILE_WRITE);
        dataFile.close();
      }else{
        dataFile = SD.open("envlog4.txt", FILE_WRITE);  
         // if the file is available, write to it:
        if (dataFile) {
          dataFile.println(datestr);
          //dataFile.println(String("Hello"));
          dataFile.close();      
        }
        // if the file isn't open, pop up an error:
        else {
          Serial.println(F("error opening datalog.txt"));
        }
        
      }
    }
      
  }

  
  //delay(100);
  radio.stopListening();

    x = analogRead(PIN_ANALOG_X);
    y = analogRead(PIN_ANALOG_Y);
    Serial.print(F("x:"));
    Serial.print(analogRead(PIN_ANALOG_X));
    Serial.print(F(" "));
  
    Serial.print(F("y:"));
    Serial.print(analogRead(PIN_ANALOG_Y));
    Serial.print(F(" "));  
  
    Serial.println();

    
    if ( x >= 300 && x <= 623 && y >= 1015) {      
       //forward    
       radio.write("W", sizeof("W"));
       Serial.println(F("Forward"));
    } else if ( x <= 5 && y >= 480 && y <= 490) {
       // turn left
       radio.write("A", sizeof("A"));
       Serial.println(F("Turn left"));
    } else if ( x >= 1015 && y >= 480 && y <= 490) {
       radio.write("D", sizeof("D"));
       Serial.println(F("Turn right"));
    }else if (  x >= 300 && x <= 623 && y <= 50 ) {
       radio.write("X", sizeof("X"));
       Serial.println(F("Turn back"));
    }else radio.write("S", sizeof("S"));
  
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
