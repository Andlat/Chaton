/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

#include <LibRobus.h>
#include "Adafruit_TCS34725.h"

#include <math.h>
#include <Wire.h>

#define RESET_PIN 52

#define IR_TRIGGER 300 // ~ 15 cm

#define CAPTEUR_PROX_AVANT_INDEX 0
#define SERVO_AVANT_INDEX 0

#define PULSES_PAR_RAD 1270.05644587
#define PULSES_PAR_DEGRE 22.1666666

#define MIN_SPEED 0.15
#define MAX_SPEED 0.25

const float kp=1E-4;
const float ki=6E-7;

void forwardPID(float base_speed, float distance, bool (*sensors_callback)(float,float));
void tourner(int ang, bool (*sensors_callback)(float,float));

void tournerSurPlace(int ang, bool (*callback)(float,float));
void tournerSurPlaceRad(float rad, float vts, bool (*callback)(float,float));
void stop();

void Wait(unsigned seconds, bool (*callback)(float,float));

void throwObject();
bool detectObstacle();
int onObstacle();

bool sensors_callback(float base_speed, float last_speed);
bool static_callback();

int readColor(float &r, float &g, float &b);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void blt();

void agressivePurr();
void purr();
void playMusic(int);
void stopMusic();

void setup(){
  BoardInit();
  Serial.begin(9600);

  //Setup the reset pin
  //initResetPin();

  //Setup the servo
  SERVO_Enable(SERVO_AVANT_INDEX);
  SERVO_SetAngle(SERVO_AVANT_INDEX, 0);
  
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

  startRandomPurr();//Request random purring to the raspberry pi

    Serial.println("Starting...");
}

float speed;
bool started = false;
void loop() {

  
  while(!started && !ROBUS_IsBumper(LEFT) && !Serial.available()){ delay(50); }
  while(Serial.available()) {Serial.read();}
  started = true;

   /*if(ROBUS_IsBumper(RIGHT)){
    Serial.println("STOP");
    started=false;
    stop();
    
    return;
  }
*/
  Wait(2000, sensors_callback);
//  tournerSurPlaceRad(PI,MIN_SPEED, sensors_callback);


/*
  forwardPID(0.2f, 10, sensors_callback);
  tournerSurPlaceRad(PI,MIN_SPEED, sensors_callback);
  Wait(1000, sensors_callback);
  tournerSurPlaceRad(PI,MIN_SPEED, sensors_callback);
  forwardPID(0.2f, 5, sensors_callback);
  //tournerSurPlaceRad(PI,MIN_SPEED, sensors_callback);
  Wait(1500, sensors_callback);
  forwardPID(0.2f, 10, sensors_callback);
  Wait(3000, sensors_callback);
  //tournerSurPlaceRad(PI,MIN_SPEED, sensors_callback);
  forwardPID(0.2f, 15, sensors_callback);
  //tournerSurPlaceRad(PI,MIN_SPEED, sensors_callback);
  Wait(4000, sensors_callback);
  forwardPID(0.2f, 15, sensors_callback);
*/

  //Random movements
  /*
  1: Forward
  3: Turn Left
  4: Turn Right
  5: Turn on itself
  6: Stop for a few seconds
   */
  /*unsigned mov = 1;//random(6) + 1;//5 choices
 
  unsigned dist = random(3) + 1;
  speed = 0.1;//random(1,3) / 10.f;//Speed between 0.1f and 0.4f
  Serial.print("Movement: ");
  Serial.println(mov);
  

  //stop();//Needed ?
  switch(mov){
    case 1:
      forwardPID(speed, dist*10, sensors_callback);
      break;
    case 2: 
      tourner(-360/(dist*3), sensors_callback);
      break;
    case 3:
      tourner(360/(dist*3), sensors_callback);
      break;
    case 4:
      tournerSurPlaceRad(PI/dist, speed, sensors_callback);
      break;
    case 5:
      stop();
      Wait(random(1,6)*1000, sensors_callback);//Wait between 1 and 5 seconds
  }*/

  //stop();//Todo remove this after testing !

  //delay(50);
  
}

void tourner (int ang, bool (*sensors_callback)(float,float))
{

  float speed = 0.15;
  int nbtours=0;
  int roue = ang>0 ? 0:1;//>0 à droite, <0 à gauche
  Serial.print("Starting Tourner with : ");
  Serial.print(ang);
  Serial.print(" ");
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
      Serial.println("SENSOR EXITING TOURNER");
      break;
    }
  }
  stop();
}  

void forwardPID(float base_speed, float distance, bool (*sensors_callback)(float,float))
{
  Serial.print("Starting PID with : ");
  Serial.print(base_speed);
  Serial.print(" ");
  Serial.println(distance);

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
    //MOTOR_SetSpeed(RIGHT, last_speed); 

    //Serial.println(total_encoder);
    
    for_delay_millis = millis();
    
    if (base_speed > 0) {
      if(!((*sensors_callback)(base_speed, last_speed))){//If encountered an obstacle or is going out of the permimeter, stop the forward PID
        Serial.println("SENSOR EXITING PID");
        break;
      }
    }
  }
}

void stop()
{
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
}

void tournerSurPlace(int ang, bool (*callback)()){

  float speed = 0.1;
  
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

void tournerSurPlaceRad(float rad, float vts, bool (*callback)(float,float)){
  int sign = rad < 0 ? -1 : 1;
  rad*=PULSES_PAR_RAD;

  int nbtours = 0;
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,sign*vts);
  MOTOR_SetSpeed(1,-sign*vts);

  
  while ( abs(nbtours) <= abs(rad))
    {
      Serial.print("Demi tour sur place :");
      Serial.print(nbtours);
      Serial.print(" sur ");
      Serial.println(rad);

      //Serial.println("ENTREE");
      //delay(10);
      //nbtours=(abs(ENCODER_Read(0)) + abs(ENCODER_Read(1)))*0.5;

     // callback(vts,vts);
    }
    stop(); 
}

void Wait(unsigned seconds, bool (*callback)(float,float)){
  long time = millis();
  while(millis()-time < seconds){
    callback(0.2f,0.2f);
    delay(10);
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
    
  const int limite = 20;
  if(abs(RES_D)>limite || abs(RES_G)>limite)
  {
    //Serial.println("PURRING!!");
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
  Serial.print("Distance obstacle : ");
  int dist = ROBUS_ReadIR(capteur_id);
  Serial.println(dist);
  if(dist >= IR_TRIGGER){
    return true;
  }

  return false;
}

void throwObject(int servo_id){
  agressivePurr();

  SERVO_SetAngle(servo_id, 120);
  delay(1500);//Try throwing the object for 3 seconds
  SERVO_SetAngle(servo_id, 0);
}

/*
 * Tries to throw down the obstacle if it detects one
 * @return -1: no obstacles detected,
 *          false: failed to throw down obstacle,
 *          true: successfully threw down obstacle
 */
int onObstacle(int capteur_id, int servo_id){
  if(detectObstacle(capteur_id)){
    Serial.print("objet trouvé");
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

int readColor(float &r, float &g, float &b){




  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false);
  delay(60); 

  Wire.beginTransmission(0x29);
  int err = Wire.endTransmission();

  Serial.print("c");
  if (err != 0) {
      Serial.print("I2C color senser error :");
      Serial.println(err);
      return err;
  }
  clear = tcs.read16(TCS34725_CDATAL);
  Serial.print("\b");

  Serial.print("r");
  if (err != 0) {
      Serial.print("I2C color senser error :");
      Serial.println(err);
      return err;
  }
  red = tcs.read16(TCS34725_RDATAL);
  Serial.print("\b");

  Serial.print("g");
  if (err != 0) {
      Serial.print("I2C color senser error :");
      Serial.println(err);
      return err;
  }
  green = tcs.read16(TCS34725_GDATAL);
  Serial.print("\b");

  Serial.print("b");
  Wire.beginTransmission(0x29);
  err = Wire.endTransmission();
  if (err != 0) {
      Serial.print("I2C color senser error :");
      Serial.println(err);
      return err;
  }
  blue = tcs.read16(TCS34725_BDATAL);
  Serial.print("\b");
  tcs.setInterrupt(true);

  uint32_t sum = clear;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  Serial.print("COULEUR: R ");
  Serial.print(r);
  Serial.print(" G: ");
  Serial.print(g);
  Serial.print(" B: ");
  Serial.println(b);
  return 0;
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
     Serial.println("PENDING BLUETOOTH DATA");
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

        delay(200);

      }else if(r == "MUSIC"){
        /************************
         * CODE POUR LA MUSIQUE
         ************************/
        appPurr();

      }else if(r == "FOOD"){
        
        
      }else if(r == "STOP_RAND_PURR"){
        stopRandomPurr();
        Serial.print("STOPPING RANDOM PURRING");

      }else{//EXIT LOOP IF NOT BEING CONTROLLED
        Serial.println("EXITING BLUETOOTH");
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
       //Serial.println("REUSSI TASSER LOBJET");
      //Successfully threw down obstacle
      //Reset motors to good speed
      //MOTOR_SetSpeed(LEFT, left_speed);
      //MOTOR_SetSpeed(RIGHT, right_speed);
      return false;
    }
    else{//Failed to throw down object. Go in the opposite direction
      //tournerSurPlace(2, static_callback);
      //Serial.println("PAS CAPABLE DE TASSER L'OBJET");
      //forwardPID(left_speed !=0?-left_speed:-0.1, 10, sensors_callback);//Reculer 10 cm
      return false; //Exit the function and do another movement
    }
  }

/* #####################################################
            Check color sensor for the permimeter
  ##################################################### */
  float r,b,g;
  int err = readColor(r, g, b);

  if (err == 0) {

    const int limite = 90;
    if (b >= limite || g >= limite) {
        Serial.println("FOUND LINE");
        tournerSurPlaceRad(PI,0.15,[](float, float){return true;});
      }
  }
/* #####################################################
        Check for vibration and bluetooth commands
   ##################################################### */
  //return static_callback();
  static_callback();
  return true;
}

//Call back used in "static" movements. Movements that do not use the encoders. Meaning, waiting and turning on itself
bool static_callback(){
  blt();
  i2c_scan();
  return vibrate();
  //return true;
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


void appPurr(){
   playMusic(0x05);
}
void agressivePurr(){
   playMusic(0x07);
}
void purr(){
   playMusic(0x04);
}
void startRandomPurr(){
  playMusic(0x02);
}
void stopRandomPurr(){
   playMusic(0x03);
}

void playMusic(int code){
  Wire.beginTransmission(0x05);
  Wire.write(code);
  int err = Wire.endTransmission();
  Serial.print("Sent music 0x0");
  Serial.print(code, HEX);
  Serial.print(" request: ");
  Serial.println(err);
}

void stopMusic(){
  Wire.beginTransmission(0x05);
  Wire.write(0x01);
  int err = Wire.endTransmission();
  Serial.print("Sent STOP music request: ");
  Serial.println(err);
}

void i2c_scan(){
  byte error, address;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
}