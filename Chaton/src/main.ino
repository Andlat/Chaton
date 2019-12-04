/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

#include <LibRobus.h>
//#include <Wire.h>
#include "Adafruit_TCS34725.h"

#include <math.h>
#include <Wire.h>

#define RESET_PIN 52

#define IR_TRIGGER 300 // ~ 15 cm

#define CAPTEUR_PROX_AVANT_INDEX 0
#define SERVO_AVANT_INDEX 0

#define PULSES_PAR_RAD 1270.05644587
#define PULSES_PAR_DEGRE 22.1666666

const float kp=1E-4;
const float ki=6E-7;

void forwardPID(float base_speed, float distance, bool (*sensors_callback)(float,float));
void tourner(int ang, bool (*sensors_callback)(float,float));

void tournerSurPlace(int ang, bool (*callback)());
void tournerSurPlaceRad(float rad, float vts, bool (*callback)());
void stop();

void Wait(unsigned seconds, bool (*callback)());

void throwObject();
bool detectObstacle();
int onObstacle();

bool sensors_callback(float base_speed, float last_speed);
bool static_callback();

void readColor(float &r, float &g, float &b);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void blt();

void purr();
void stopMusic();

void setup(){
  BoardInit();
  Serial.begin(9600);

  //Setup the reset pin
  //initResetPin();

  //Setup the servo
  SERVO_Enable(SERVO_AVANT_INDEX);
  SERVO_SetAngle(SERVO_AVANT_INDEX, 40);
  
  initVibro();

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
  }

  //Init bluetooth
  Serial2.begin(38400);

  //Init music port
  Wire.begin();
}

float speed;
bool started = false;
void loop() {
  Serial.println("Starting...");
  
  while(!started && !ROBUS_IsBumper(LEFT)){ delay(50); }
  started = true;

  if(ROBUS_IsBumper(RIGHT)){
    Serial.println("STOP");
    started=false;
    stop();
    
    return;
  }

  //Random movements
  /*
  1: Forward
  3: Turn Left
  4: Turn Right
  5: Turn on itself
  6: Stop for a few seconds
   */
 unsigned mov = 5;//random(6) + 1;//6 choices
  //distance to do  movement. Max is 2 meters or seconds
  unsigned dist = random(3) + 1;//Min is 10 cm
  speed = 0.1;//random(1,3) / 10.f;//Speed between 0.1f and 0.4f
  Serial.print("MOV: ");
  Serial.println(mov);
  
  //stop();//Needed ?
  switch(mov){
    case 1:
      forwardPID(speed, dist*10, sensors_callback);
      break;
    case 2: 
      tourner(-360/dist, sensors_callback);
      break;
    case 3:
      tourner(360/dist, sensors_callback);
      break;
    case 4:
      tournerSurPlaceRad(2*PI/dist, speed, static_callback);
      break;
    case 5:
      stop();
      Wait(random(1,6)*1000, static_callback);//Wait between 1 and 5 seconds
  }

  //stop();//Todo remove this after testing !

  //delay(50);
}

void tourner (int ang, bool (*sensors_callback)(float,float))
{
  float speed = 0.2;
  int nbtours=0;
  int roue = ang>0 ? 0:1;//>0 à droite, <0 à gauche

  ang*=PULSES_PAR_DEGRE;
Serial.println(ang);
  MOTOR_SetSpeed(roue,speed);
  MOTOR_SetSpeed(!roue,0);
  ENCODER_Reset(roue);

  while ( abs(nbtours) < abs(ang))
  {
    delay(10);
    nbtours=ENCODER_Read(roue);

    //If encountered an obstacle or the perimeter, stop turning
    if(!((*sensors_callback)(roue*speed, !roue*speed))){
      break;
    }
  } 
}  

void forwardPID(float base_speed, float distance, bool (*sensors_callback)(float,float))
{
  //reset the encoders before moving
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);

  float last_speed=base_speed;
  int total_encoder = 0, goal_encoder, total_diff=0;
  
  goal_encoder = distance * 133.67;


  //starting the robot
  MOTOR_SetSpeed(LEFT, base_speed);
  MOTOR_SetSpeed(RIGHT, base_speed);
  unsigned long last_time=millis();
  unsigned long for_delay_millis = millis();
  while (abs(total_encoder) < abs(goal_encoder))
  {
    //Delay to ajust at 10Hz
    delay(last_time+100-for_delay_millis);
    last_time = millis();

    total_encoder += ENCODER_Read(LEFT);
    //calculate difference between wheels
    int32_t diff = 0;//ENCODER_ReadReset(LEFT)-ENCODER_ReadReset(RIGHT);
    diff *= 10;
    total_diff += diff;

    //set new speed
    last_speed = min(last_speed + diff*kp + total_diff*ki, 1);
    MOTOR_SetSpeed(RIGHT, last_speed); 

    //Serial.println(total_encoder);
    
    for_delay_millis = millis();
    
    if(!((*sensors_callback)(base_speed, last_speed))){//If encountered an obstacle or is going out of the permimeter, stop the forward PID
      Serial.print("SENSOR EXITING PID");
      break;
    }
  }
}

void stop()
{
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
}

void tournerSurPlace(int ang, bool (*callback)()){

  float speed = 0.15;
  
  int sign = ang < 0 ? -1 : 1;
  ang*=PULSES_PAR_DEGRE;

  int nbtours = 0;
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,sign*speed);
  MOTOR_SetSpeed(1,-sign*speed);
  while ( abs(nbtours) < abs(ang))
    {
      delay(10);
      nbtours=(abs(ENCODER_Read(0)) + abs(ENCODER_Read(1)))*0.5;

      callback();
    }
}

void tournerSurPlaceRad(float rad, float vts, bool (*callback)()){
  int sign = rad < 0 ? -1 : 1;
  rad*=PULSES_PAR_RAD;

  int nbtours = 0;
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,sign*rad);
  MOTOR_SetSpeed(1,-sign*rad);

  while ( abs(nbtours) <= abs(rad))
    {
      //Serial.println("ENTREE");
      delay(10);
      nbtours=(abs(ENCODER_Read(0)) + abs(ENCODER_Read(1)))*0.5;

      callback();
    }
}

void Wait(unsigned seconds, bool (*callback)()){
  long time = millis();
  while(millis()-time < seconds){
    callback();
    delay(50);
  }
}

void initVibro(){
  pinMode(2,OUTPUT);
 
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
}

bool vibrate(){
  int Mes1_D=0, Mes2_D=0, RES_D=0;
  int Mes1_G=0, Mes2_G=0, RES_G=0;
  
  Mes1_D=analogRead(A0);
  Mes1_G=analogRead(A1);
  
  delay(200);
  
  Mes2_D=analogRead(A0);
  Mes2_G=analogRead(A1);

  RES_D=(Mes1_D-Mes2_D);
  RES_G=(Mes1_G-Mes2_G);
  
  if(RES_D>100 || RES_D<-100 || RES_G>100 || RES_G<-100)
  {
    purr();

    analogWrite(2,255);
    delay(1000);
    analogWrite(2,200);
    delay(500);
    analogWrite(2,150);
    delay(500);

    stopMusic();

    return false;
  }
  else
  {
    analogWrite(2,0);
  }

  return true;
}

bool detectObstacle(int capteur_id){
  Serial.println(ROBUS_ReadIR(capteur_id));
  if(ROBUS_ReadIR(capteur_id) >= IR_TRIGGER){
    return true;
  }

  return false;
}

void throwObject(int servo_id){
  SERVO_SetAngle(servo_id, 120);
  delay(2000);//Try throwing the object for 3 seconds
  SERVO_SetAngle(servo_id, 40);
}

/*
 * Tries to throw down the obstacle if it detects one
 * @return -1: no obstacles detected,
 *          false: failed to throw down obstacle,
 *          true: successfully threw down obstacle
 */
int onObstacle(int capteur_id, int servo_id){
  if(detectObstacle(capteur_id)){
    stop();
    throwObject(servo_id);
    delay(1000);
    if(detectObstacle(capteur_id)){//Arm was unable to throw the object
      return false;
    }
    return true;
  }
  return -1;
}

void readColor(float &r, float &g, float &b){
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);
  delay(60); 
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);

  uint32_t sum = clear;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
}

void blt() {
  double pos_x=0;
  double pos_y=0;
  double vts=0;
  float rad = 0;
  
  //bool isEmpty=true;
  bool isON = false;

  do{
    //Serial.println("IN LOOP");

    //isEmpty=true;
    Serial2.println("Waiting for String");
    delay(1000);
    if(Serial2.available()/* && isEmpty*/){
     Serial.println("SERIAL2 AVAILABLE");
      String r=Serial2.readStringUntil(';');
      r.trim();
      //if(r!=""){ Serial2.println("IS EMPTY"); isEmpty=false;}else{continue;}
      
      
      if(r == "ON"){//ROBOT IS BEING CONTROLLED
        isON = true;
        Serial2.println("Received ON");

        r=Serial2.readStringUntil(';');
        pos_x = r.toDouble();
        Serial2.print("pos_x: ");
        Serial2.println(pos_x);

        r=Serial2.readStringUntil('\n');
        pos_y = r.toDouble();
        Serial2.print("pos_y: ");
        Serial2.println(pos_y);


        Serial2.flush();

        Serial.println("DRIVING");
        float speed_left = pos_y,
              speed_right = pos_y;
         
        if(pos_x < 0){//Tourner a gauche
          speed_left *= (1-abs(pos_x));
        }
        else{//Tourner a droite
          speed_right *= (1-abs(pos_x));
        }

        Serial.print("Speed: LEFT: ");
        Serial.print(speed_left);
        Serial.print(" RIGHT: ");
        Serial.print(speed_right);

        MOTOR_SetSpeed(LEFT,0.2*speed_left);
        MOTOR_SetSpeed(RIGHT,0.2*speed_right);

      }else if(r == "MUSIC"){
        /************************
         * CODE POUR LA MUSIQUE
         ************************/
        Wire.beginTransmission(0x05);
        Wire.write(0x05);
        int err = Wire.endTransmission();
        Serial.print("Sent music request: ");
        Serial.println(err);

      }else{//EXIT LOOP IF NOT BEING CONTROLLED
        Serial.println("Exiting");
        isON = false;
        return;
      }   
    }else{
      isON=false;
    }   
  }while(isON);
}


bool sensors_callback(float left_speed, float right_speed){
/* #####################################################
                      ON  OBSTACLE
  ##################################################### */
  int onObstacleResult = onObstacle(CAPTEUR_PROX_AVANT_INDEX, SERVO_AVANT_INDEX);
  if(onObstacleResult != -1){//Check if the robot stopped because of an obstacle
    if(onObstacleResult == true){
      //Successfully threw down obstacle
      //Reset motors to good speed
      MOTOR_SetSpeed(LEFT, left_speed);
      MOTOR_SetSpeed(RIGHT, right_speed);
    }
    else{//Failed to throw down object. Go in the opposite direction
      tournerSurPlace(2, static_callback);
      forwardPID(left_speed, 10, sensors_callback);//Advance 10 cm before switching to another movement
      return false; //Exit the function and do another movement
    }
  }

/* #####################################################
            Check color sensor for the permimeter
  ##################################################### */
  float r,b,g;
  readColor(r, g, b);
  Serial.print("COULEUR: R:");
  Serial.print(r);
  Serial.print(" G: ");
  Serial.print(g);
  Serial.print(" B: ");
  Serial.println(b);

  if (b > 90) {
    stop();
    delay(50);
    tournerSurPlace(2, static_callback);
    stop();
    //delay(100000);
    return false;
  }

/* #####################################################
        Check for vibration and bluetooth commands
   ##################################################### */
  return static_callback();
// return true;
}

//Call back used in "static" movements. Movements that do not use the encoders. Meaning, waiting and turning on itself
bool static_callback(){
  //ResetProgramCallback();
  blt();
  return vibrate();
  //return true;
}


void initResetPin(){
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
}

void ResetProgramCallback(){
  if(ROBUS_IsBumper(RIGHT)){
    Serial.println("STOP");
    started=false;
    stop();

    //digitalWrite(RESET_PIN, HIGH);
    
    return;
  }
}

void purr(){
  Wire.beginTransmission(0x05);
  Wire.write(0x04);
  int err = Wire.endTransmission();
  Serial.print("Sent music request: ");
  Serial.println(err);
}

void stopMusic(){
  Wire.beginTransmission(0x05);
  Wire.write(0x01);
  int err = Wire.endTransmission();
  Serial.print("Sent music request: ");
  Serial.println(err);
}