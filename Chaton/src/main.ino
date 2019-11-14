/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

#include <LibRobus.h>

#define PERIMETER 10

#define CAPTEUR_PROX_ARRIERE_INDEX 0
#define CAPTEUR_PROX_AVANT_INDEX 1
#define IR_TRIGGER 300 // ~ 15 cm

#define SERVO_ARRIERE_INDEX 0
#define SERVO_AVANT_INDEX 1

const float kp=1E-4;
const float ki=6E-7;

void forwardPID(float base_speed, float distance, void (*onObstacle)(float,float));
void tourner(int x);
void tournerSurPlace(int x);
void stop();

void throwObject();
bool detectObstacle();
int onObstacle();
void onObstacle_PID(float base_speed, float last_speed);


void setup(){
  BoardInit();
  SERVO_Enable(SERVO_ARRIERE_INDEX);
  SERVO_Enable(SERVO_AVANT_INDEX);
}

float speed;
unsigned started = false;
void loop() {
  while(!started && !ROBUS_IsBumper(LEFT)) delay(50);
  started = true;

  //Random movements
  /*
  1: Forward
  2: Backward
  3: Turn Left
  4: Turn Right
  5: Turn on itself
  6: Stop for a few seconds
   */
  unsigned mov = random(1) + 1;//6 is good value
  //distance to do  movement. Max is 5 meters/seconds
  unsigned dist = random(6) * 100;
  speed = random(1,4) / 10.f;//Speed between 0.1f and 0.4f
  Serial.print("MOV: ");
  Serial.println(mov);

  switch(mov){
    case 1:
      forwardPID(speed, dist, onObstacle_PID);
      break;
    case 2:
      forwardPID(-speed, dist, onObstacle_PID);
      break;
    case 3: 
      tourner(-dist);
      break;
    case 4:
      tourner(dist);
      break;
    case 5:
      tournerSurPlace(dist);
      break;
    case 6:
      stop();
      delay(dist*10);
  }

  stop();//Todo remove this after testing !

  delay(100000);
}

void tourner (int x)
{
  int nbtours;
  int roue = x>0?0:1;//>0 à droite, <0 à gauche
  x*=997;
  x*=x>0?1:-1;
  MOTOR_SetSpeed(roue,0.35);
  MOTOR_SetSpeed(!roue,0);
  ENCODER_Reset(roue);
  while ( nbtours < x)
  {
    delay(10);
    nbtours=ENCODER_Read(roue);
  } 
}  

void forwardPID(float base_speed, float distance, void (*onObstacle)(float,float))
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
    int32_t diff = ENCODER_ReadReset(LEFT)-ENCODER_ReadReset(RIGHT);
    diff *= 10;
    total_diff += diff;


    //set new speed
    last_speed = min(last_speed + diff*kp + total_diff*ki, 1);
    MOTOR_SetSpeed(RIGHT, last_speed); 

    //Serial.println(total_encoder);
    
    for_delay_millis = millis();
    (*onObstacle)(base_speed, last_speed);
  }

  //MOTOR_SetSpeed(LEFT,0);
  //MOTOR_SetSpeed(RIGHT,0);
}


void stop()
{
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
}

void tournerSurPlace(int x){
  int sign = x < 0 ? -1 : 1;
  x*=997;

  int nbtours = ENCODER_ReadReset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,sign*0.15);
  MOTOR_SetSpeed(1,-sign*0.15);
  while ( abs(nbtours) < abs(x))
    {
      delay(10);
      nbtours=(abs(ENCODER_Read(0)) + abs(ENCODER_Read(1)))*0.5;
    }
}

bool detectObstacle(int capteur_id){
  Serial.println(ROBUS_ReadIR(capteur_id));
  if(ROBUS_ReadIR(capteur_id) >= IR_TRIGGER){
    return true;
  }

  return false;
}

void throwObject(){
  SERVO_SetAngle(SERVO_AVANT_INDEX, 180);
  delay(3000);//Try throwing the object for 3 seconds
  SERVO_SetAngle(SERVO_AVANT_INDEX, 0);
}

/*
 * Tries to throw down the obstacle if it detects one
 * @return -1: no obstacles detected,
 *          false: failed to throw down obstacle,
 *          true: successfully threw down obstacle
 */
int onObstacle(int capteur_id){
  if(detectObstacle(capteur_id)){
    stop();
    throwObject();
    if(detectObstacle(capteur_id)){//Arm was unable to throw the object
      return false;
    }
    return true;
  }
  return -1;
}

void onObstacle_PID(float base_speed, float last_speed){
  int onObstacleResult = onObstacle(base_speed > 0 ? CAPTEUR_PROX_AVANT_INDEX : CAPTEUR_PROX_ARRIERE_INDEX);//One Sensor in front and the other in the back. Select which one to check depending on the movement of the robot (goint forwards or backwards)
  if(onObstacleResult != -1){//Check if the robot stopped because of an obstacle
    if(onObstacleResult == true){
      //Successfully threw down obstacle
      //Reset motors to good speed
      MOTOR_SetSpeed(LEFT, base_speed);
      MOTOR_SetSpeed(RIGHT, last_speed); 
    }
    else{//Failed to throw down object. Go in the opposite direction
      MOTOR_SetSpeed(LEFT, -base_speed);
      MOTOR_SetSpeed(RIGHT, -base_speed);
      delay(2000);
      stop();
      return; //Exit the function and do another movement
    }
  }
}