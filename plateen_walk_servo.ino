/*
*  Plateen walk recieve from remote and drive motor
*  
*/


#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ublox.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;

#define N_FLOATS 4

Ublox M8_Gps;
float gpsArray[N_FLOATS] = {0, 0, 0,0};
#include <SoftwareSerial.h>
SoftwareSerial mySerial(A8, A9); //(RX,TX)


//#define button 4
#define DHTPIN 30
#define DHTTYPE DHT22

RF24 radio(32, 33); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
Servo myServo;
DHT dht(DHTPIN, DHTTYPE);

char buttonState[5] = "";
static int led = 0;
static long delaytime = 1000;
unsigned long previousMillis = 0;
unsigned long  previousMillisIMU =0;
unsigned long currentMillisIMU ;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

//const int channel = 15;

#define PULSE_DURATION     (int)20       // 20 ms
const  int  FREQUENCY      = (1000/PULSE_DURATION);
#define DEFAULT_PULSE_WIDTH   1500
#define MIN_PULSE_WIDTH  102 //150 // this is the 'minimum' pulse length count (out of 4096)
#define MAX_PULSE_WIDTH  409 //600 // this is the 'maximum' pulse length count (out of 4096)


int interruptPin = 18;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount=0;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3] = {0.0,0.0,0.0};  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



class WServo{
      private : 
        int minOnTick,maxOnTick,ang1,ang2,channel_no;
      
      public :
       WServo ( int minOnTick, int maxOnTick, int ang1, int ang2 , int channel_no ){
         this->minOnTick = minOnTick;
         this->maxOnTick = maxOnTick;
         this->ang1    = ang1;
         this->ang2    = ang2; 
         this->channel_no=channel_no;
       }
       
      int Angle2Tick(int angle){
       return map(angle, ang1, ang2, minOnTick, maxOnTick);
      }
      
      void go_angle(int angle){
        int ontick = Angle2Tick(angle);
        pwm.setPWM(this->channel_no , ontick , 4096-ontick  );  
      }
      
      void go_max_angle(){
        go_angle(ang1>ang2? ang1:ang2);
      }  
      
      void go_min_angle(){
        go_angle(ang1<ang2? ang1:ang2);
      }
};

/*             //          minOnTick,  maxOnTick,   ang1,  ang2 ,  channel_no 
const int arrR[2][5] = {  { 315        , 360      ,-10   , 35    ,12  }  //Right Servo Up/Down
                    ,     {1871        ,1929      ,-20   , 38    ,13 }}; //            Fwd/Bwd
const int arrL[2][5] = {  { 369        , 321      ,-12   , 36    ,14}    //Left  Servo Up/Down
                    ,     {1934        ,1872      ,-19   , 43    ,15}};  //            Fwd/Bwd                  
*/                    
             //          minOnTick,  maxOnTick,   ang1,  ang2 ,  channel_no 
const int arrR[2][5] = {  { 320        , 370      ,-15  , 35    ,12  }  //Right Servo Up/Down 
                    ,     {1909        ,1944      ,-11  , 24    ,13 }}; //            Fwd/Bwd
const int arrL[2][5] = {  { 363        , 310      ,-18  , 35    ,14}    //Left  Servo Up/Down
                    ,     {1917        ,1878      ,-14  , 25    ,15}};  //            Fwd/Bwd     
      
/*int pos_ang[][2][2] = {
//   R1    R2    L1     L2 
 { { 35,  0 } , { 36  ,  0 } }
,{ { 30,  32} , { 30  ,  30} }
,{ {-10,  32} , {-10  ,  30} }
,{ {-10, -18} , {-10  , -15} }
,{ { 30, -18} , { 30  , -15} }
};*/

int gb_walk_state = 0;
int pos_ang[][2][2] = {
//   R1   R2      L1     L2 
//   U/D  F/W     U/D    F/W
 { { 35,  0 } , { 35  ,  0 } }
,{ { 25,  19} , { 25  ,  25} }
,{ {-15,  19} , {-18  ,  25} }
,{ {-15, -10} , {-18  , -13} }
,{ { 25, -10} , { 25  , -13} }
};              


class  Arm{
       private :
       WServo *sv1 ; //  Up/Down
       WServo *sv2 ; //  Forward/BackWord
       
       public : 
       Arm(const int array[2][5]){
            sv1 = new WServo(array[0][0],array[0][1],array[0][2],array[0][3],array[0][4]);
            sv2 = new WServo(array[1][0],array[1][1],array[1][2],array[1][3],array[1][4]);
       }
       
       void home(){
         
             sv1->go_angle(0); 
             sv2->go_angle(0);
       }
       void test_sv2(){
             test_sv2_fwd();
             delay(1000);
             test_sv2_bwd();
             delay(1000);
       }
       void test_sv2_fwd(){
            sv2->go_max_angle();
       }
        void test_sv2_bwd(){
            sv2->go_min_angle();
       }
       void test_sv1_up(){
            sv1->go_max_angle();
       }
       void test_sv1_down(){
            sv1->go_min_angle();
       } 
       void test_sv1(){
           test_sv1_up();
           delay(1000);
           test_sv1_down();
           delay(1000);
       }
       void pos_ang(int ang[]){
          sv1->go_angle(ang[0]);
          sv2->go_angle(ang[1]);
       } 
};

Arm armR = Arm( arrR );
Arm armL = Arm( arrL );


void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);

  dht.begin();   // dht11 initial sensor
  //pinMode(button, INPUT);
  //myServo.attach(5);
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MAX);
  Serial.println("Start working");
  //moving part
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);  // Set to whatever you like, we don't use it in this demo!

  Wire.setClock(400000);
  
  mpu.initialize();
     // verify connection to IMU
    Serial.println(F("Testing IMU connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 4)..."));
        pinMode(interruptPin, INPUT_PULLUP);
        // attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
  delay(3000);
  armR.home();
  armL.home();
  delay(1000);
  armR.test_sv2_fwd();
  armL.test_sv2_fwd();  delay(1000);
  armR.test_sv2_bwd();  
  armL.test_sv2_bwd();  
  delay(1000);
  armR.test_sv1_up();
  armL.test_sv1_up(); delay(1000);
  armR.test_sv1_down();
  armL.test_sv1_down();  
  
  delay(1000);
  armR.home();
  armL.home();
  delay(1000);
  
  /*
  armR.pos_ang(pos_ang[0][0]);
  armL.pos_ang(pos_ang[0][1]);   delay(1000);  
  armR.pos_ang(pos_ang[1][0]);
  armL.pos_ang(pos_ang[1][1]);   delay(1000);  
  armR.pos_ang(pos_ang[2][0]);
  armL.pos_ang(pos_ang[2][1]);   delay(1000);  
  armR.pos_ang(pos_ang[3][0]);
  armL.pos_ang(pos_ang[3][1]);   delay(1000);  
  armR.pos_ang(pos_ang[4][0]);
  armL.pos_ang(pos_ang[4][1]);   delay(1000);    
  */
  
  armR.pos_ang(pos_ang[0][0]);
  armL.pos_ang(pos_ang[0][1]);   delay(1000); 

  
}


int walk_cmd = 0 ;

void loop() {
  String dataGPS = "";    

    while (mySerial.available())
    {
      char c = mySerial.read();
      //Serial.print(c);
      if (M8_Gps.encode(c))
      {
        //gpsArray[0] = M8_Gps.altitude;
        gpsArray[0] = M8_Gps.latitude;
        gpsArray[1] = M8_Gps.longitude;
        gpsArray[2] = M8_Gps.sats_in_use;
        gpsArray[3] = M8_Gps.speed;
      }      
    }
      
    //delay(100);
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= delaytime) {
      // save the last time you your read sensor
      previousMillis = currentMillis;
      radio.stopListening();
      Serial.println("start transmitting");
      //const char text[] = "Hello World";
      //radio.write(&text, sizeof(text));
    
      float Humidity = dht.readHumidity();
      float Temp = dht.readTemperature();  // Read temperature as Celsius (the default)

     
      for (byte i = 0; i < 2; i++)
      {
        switch(i){
          case 0 : Serial.print("LAT"); Serial.print(gpsArray[i], 6);  dataGPS += ""+String(gpsArray[i],6);  break;
          case 1 : Serial.print("LON"); Serial.print(gpsArray[i], 6);  dataGPS += ","+String(gpsArray[i],6);  break;
          //case 2 : Serial.print("SAT"); Serial.print(gpsArray[i], 0);  break;
          //case 3 : Serial.print("SPD"); Serial.print(gpsArray[i], 0);  break;
        }
        if( i == 1){
          Serial.println("");
        }else
        {
          Serial.print("_");
        }   
      }
          
       + "\n\0";
      
      

      //char tmp[] ="1234567980123456789012345678901324567890\0";
      //char tmp2[] ="ABCDEFGHIJABCDEFGHIJABCDEFGHIJABCDEFGHIJ\0";
      //radio.write((void * )tmp, sizeof(tmp));
      //radio.write((void * )tmp2, sizeof(tmp2));
      Serial.println();
      for ( int i=0 ; i <=4 ; i++){
        char msg[100];
        String tmpStr;
        memset(msg,0, sizeof(msg));
        switch( i ){
         case 0:  strcpy(msg,"begin");
         break;
         case 1 : dataGPS = String("DATA1")+dataGPS;
                  dataGPS.toCharArray(msg,sizeof(msg));
         break; 
         case 2 : tmpStr  = String("DATA2")+","+String(Humidity,0) + "," + String(Temp,1);
                  tmpStr.toCharArray(msg,sizeof(msg));
         break;
         case 3:  tmpStr =  String("DATA3")+"," + String(ypr[0] * 180/M_PI,0) + "," + String(ypr[1] * 180/M_PI,0) + "," + String(ypr[2] * 180/M_PI,0) ;
                  tmpStr.toCharArray(msg,sizeof(msg));
         break;
         case 4:  tmpStr =  String("DATA4")+"," + String(aaReal.x) + "," + String(aaReal.y) + "," + String(aaReal.z) ;
                  tmpStr.toCharArray(msg,sizeof(msg));
         break;
        }       
        //Serial.println(msg);
        radio.write(msg,sizeof(msg));
        Serial.print(String(msg)+"\n");
        delay(10);
      }
      Serial.println();
      
      Serial.println("send success!!!");
      //delay(100);
    }
    radio.startListening();
    //Serial.println("start receving");
      
    //while (!radio.available() );
    if (radio.available()) {
      radio.read(&buttonState, sizeof(buttonState));
      Serial.println(buttonState);
      //drive servo here
      if (strcmp(buttonState,"W") == 0) {
        //forward
        walk_cmd = 10;
        
      } else if (strcmp(buttonState,"A") == 0) {
        //turn left
        walk_cmd = 20;
        
      } else if (strcmp(buttonState,"D") == 0) {
        //turn right
        walk_cmd = 30;
      } else if (strcmp(buttonState,"X") == 0) {
        //turn right
        walk_cmd = 40;
      } else {
        // Stop 
        walk_cmd = 1000;
      }
    }
  
    switch ( walk_cmd ){
      case 10 : forward();
      break;
      case 20 : turn_left();
      break;
      case 30 : turn_right();
      break;
      case 40 : turn_back();
      break;
      case 1000 : armR.home();
                  armL.home();
      break;
      default : armR.home();
                armL.home();
    }
  
    
  currentMillisIMU = millis();
  if (currentMillisIMU - previousMillisIMU >= delaytime) {
         previousMillisIMU = millis();
         attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);
  }
  
  
    
    if (dmpReady){
              // wait for MPU interrupt or extra packet(s) available
              while (!mpuInterrupt && fifoCount < packetSize) {
                  return;
              }
              detachInterrupt(digitalPinToInterrupt(interruptPin));
              // reset interrupt flag and get INT_STATUS byte
              mpuInterrupt = false;            
              mpuIntStatus = mpu.getIntStatus();
          
              // get current FIFO count
              fifoCount = mpu.getFIFOCount();
          
              // check for overflow (this should never happen unless our code is too inefficient)
              if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                  // reset so we can continue cleanly
                  mpu.resetFIFO();
                  Serial.println(F("FIFO overflow!"));
          
              // otherwise, check for DMP data ready interrupt (this should happen frequently)
              } else if (mpuIntStatus & 0x02) {
                  // wait for correct available data length, should be a VERY short wait
                  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
          
                  // read a packet from FIFO
                  mpu.getFIFOBytes(fifoBuffer, packetSize);
                  
                  // track FIFO count here in case there is > 1 packet available
                  // (this lets us immediately read more without waiting for an interrupt)
                  fifoCount -= packetSize;
                  // display Euler angles in degrees YawPitchRoll
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetGravity(&gravity, &q);
                  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                  
                  if( 0 ){
                    Serial.print("ypr\t");
                    Serial.print(ypr[0] * 180/M_PI);
                    Serial.print("\t");
                    Serial.print(ypr[1] * 180/M_PI);
                    Serial.print("\t");
                    Serial.println(ypr[2] * 180/M_PI);            
                  }
                  
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                 if(0){
                  Serial.print("areal\t");
                  Serial.print(aaReal.x);
                  Serial.print("\t");
                  Serial.print(aaReal.y);
                  Serial.print("\t");
                  Serial.println(aaReal.z);
                 }
            }
            
    } 

   
    

}

#define DELAY_SERVO (int)400
void forward(){
  static int walk_state = 0;
  static unsigned long currentMillis=  millis() , previousMillis =0  ,delaytime = DELAY_SERVO ;
  walk_state = gb_walk_state ;
  currentMillis = millis();
  if (currentMillis - previousMillis >= delaytime) { 
    armR.pos_ang(pos_ang[walk_state][0]);
    armL.pos_ang(pos_ang[walk_state][1]);
    walk_state++;
    previousMillis =  millis();
    if ( walk_state > 4  ){
       walk_state = 1;
    }
  } 
  gb_walk_state = walk_state;
}

 void turn_left(){
  static int walk_state = 0;
  static unsigned long currentMillis=  millis() , previousMillis =0  ,delaytime = DELAY_SERVO ;
  walk_state = gb_walk_state ;
  currentMillis = millis();
  if (currentMillis - previousMillis >= delaytime) { 
    armR.pos_ang(pos_ang[walk_state][0]);
    armL.test_sv1_down(); 
    walk_state++;
    previousMillis =  millis();
    if ( walk_state > 4  ){
       walk_state = 1;
    }
  } 
  gb_walk_state = walk_state;
}



void turn_right(){
  static int walk_state = 0;
  static unsigned long currentMillis=  millis() , previousMillis =0  ,delaytime = DELAY_SERVO ;
  walk_state = gb_walk_state ;
  currentMillis = millis();
  if (currentMillis - previousMillis >= delaytime) { 
    armR.test_sv1_down(); 
    armL.pos_ang(pos_ang[walk_state][1]);
    walk_state++;
    previousMillis =  millis();
    if ( walk_state > 4  ){
       walk_state = 1;
    }
  } 
  gb_walk_state = walk_state;
}


void turn_back(){
  static int walk_state = 4;
  static unsigned long currentMillis=  millis() , previousMillis =0  ,delaytime = DELAY_SERVO ;
  walk_state = gb_walk_state ;
  currentMillis = millis();
  if (currentMillis - previousMillis >= delaytime) { 
    armR.pos_ang(pos_ang[walk_state][0]);
    armL.pos_ang(pos_ang[walk_state][1]);
    walk_state--;
    previousMillis =  millis();
    if ( walk_state <= 0  ){
       walk_state = 4;
    }
  } 
  gb_walk_state = walk_state;
}
