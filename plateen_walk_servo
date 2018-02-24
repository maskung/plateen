/*
*  Plateen walk recieve from remote and drive motor
*  
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//#define button 4
#define DHTPIN 30
#define DHTTYPE DHT22

RF24 radio(32, 33); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
Servo myServo;
DHT dht(DHTPIN, DHTTYPE);

char buttonState[5] = "";
static int led = 0;
static long delaytime = 500;
unsigned long previousMillis = 0;

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

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
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
             //          minOnTick,  maxOnTick,   ang1,  ang2 ,  channel_no 
const int arrR[2][5] = {  { 315        , 360      ,-10  , 35    ,12  } //Fwd/Bwd
                    ,     {1871        ,1929      ,-20  , 38    ,13 }};//Up/Down
const int arrL[2][5] = {  { 321        , 369      , 36   ,-12   ,14} 
                    ,     { 1872       ,1934      , 43   ,-19   ,15}};                    
class  Arm{
 private :
 WServo *sv1 ;
 WServo *sv2 ;
 
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

 int pos_ang[][2][2] = {
//   R1    R2    L1     L2 
 { { 35,  0 } , { 36  ,  0 } }
,{ { 25,  32} , { 25  ,  30} }
,{ {-10,  32} , {-10  ,  30} }
,{ {-10, -18} , {-10  , -15} }
,{ { 25, -18} , { 25  , -15} }
};

void setup() {
  Serial.begin(115200);

  dht.begin();   // dht11 initial sensor
  //pinMode(button, INPUT);
  //myServo.attach(5);
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);
  Serial.println("Start working");
  //moving part
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);  // Set to whatever you like, we don't use it in this demo!

  Wire.setClock(400000);
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
  
  /*armR.pos_ang(pos_ang[0][0]);
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
    String data = String(Humidity) + "," + String(Temp);
    char msg[128];
    data.toCharArray(msg,data.length());
    Serial.println(msg);
    radio.write(&msg, sizeof(msg));
  
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
    } else {
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
    case 1000 : armR.home();
                armL.home();
    break;
    default : armR.home();
              armL.home();
  }
    

}


void forward(){
  static int walk_state = 0;
  static unsigned long currentMillis=  millis() , previousMillis =0  ,delaytime = 330 ;
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
}

void turn_left(){
  static int walk_state = 0;
  static unsigned long currentMillis=  millis() , previousMillis =0  ,delaytime = 330 ;
  currentMillis = millis();
  if (currentMillis - previousMillis >= delaytime) { 
    armL.test_sv1_up(); 
    armR.pos_ang(pos_ang[walk_state][1]);
    walk_state++;
    previousMillis =  millis();
    if ( walk_state > 4  ){
       walk_state = 1;
    }
  } 
}



void turn_right(){
  static int walk_state = 0;
  static unsigned long currentMillis=  millis() , previousMillis =0  ,delaytime = 330 ;
  currentMillis = millis();
  if (currentMillis - previousMillis >= delaytime) { 
    armL.pos_ang(pos_ang[walk_state][0]);
    armR.test_sv1_up(); 
    walk_state++;
    previousMillis =  millis();
    if ( walk_state > 4  ){
       walk_state = 1;
    }
  } 
}
