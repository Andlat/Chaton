/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

#include <LibRobus.h>

#define VERT 1
#define BLEU 2
#define JAUNE 3
#define ROUGE 4

#define ROBOT_A
//#define ROBOT_B

#define COULEUR VERT

const float kp=1E-4;
const float ki=6E-7;


void setup(){
  BoardInit();
}



void loop() {
 




#ifdef ROBOT_A
  robot_A();
#elif defined(ROBOT_B)
  robot_B();
#endif


  stop();

}
void tourner (int x)
{
  int nbtours;
  int roue = x>0?0:1;//>0 à droite, <0 à gauche
  x*=1995;
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

void forwardPID(float base_speed, float distance)
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

  while (total_encoder < goal_encoder)
  {
    //Delay to ajust at 10Hz
    delay(last_time+100-millis());
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
  }

  //MOTOR_SetSpeed(LEFT,0);
  //MOTOR_SetSpeed(RIGHT,0);
}
void demitour (void)
{
  int nbtours = ENCODER_ReadReset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,0.15);
  MOTOR_SetSpeed(1,-0.15);
  while ( nbtours < 3989)
    {
      delay(10);
      nbtours=(abs(ENCODER_Read(0)) + abs(ENCODER_Read(1)))*0.5;
    }
   
  //MOTOR_SetSpeed(0,0);
  //MOTOR_SetSpeed(1,0);
} 

void stop()
{
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
}

void tournerSurPlace(int x){
  x*=1995;

  int nbtours = ENCODER_ReadReset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0,0.15);
  MOTOR_SetSpeed(1,-0.15);
  while ( nbtours < x)
    {
      delay(10);
      nbtours=(abs(ENCODER_Read(0)) + abs(ENCODER_Read(1)))*0.5;
    }
   
  //MOTOR_SetSpeed(0,0);
  //MOTOR_SetSpeed(1,0);
}

int angA(){
  switch(COULEUR){
    case VERT: return 1;
    case JAUNE: return -3;
    case ROUGE: return -1;
    case BLEU: return 3;
  }

  return 0;
}
int angB(){
  switch(COULEUR){
    case VERT: return -3;
    case JAUNE: return 1;
    case ROUGE: return 3;
    case BLEU: return -1;
  }

  return 0;
}

#define VITESSE 0.3f

// Robot A
void robot_A ()
{
  forwardPID(VITESSE, 30);

  tournerSurPlace(angA());

  forwardPID(VITESSE, 115);

  activerPince(false);

  demitour();

  activerPince(true);

  forwardPID(VITESSE, 115);

  tournerSurPlace(-angA());
  stop();

  activerPince(false);

  forwardPID(-VITESSE, 30);
}

void robot_B(){
  forwardPID(VITESSE, 30);
  stop();
  
  activerPince(false);

  tournerSurPlace(angB());

  forwardPID(VITESSE,115);
  stop();

  activerPince(true);
}

void activerPince(bool ouvrir){
  float ANGLE = 90;
  ANGLE *= ouvrir ? 1 : -1;

  SERVO_Enable(0);
  SERVO_Enable(1);

  SERVO_SetAngle(0, ANGLE);
  SERVO_SetAngle(1, -ANGLE);

  SERVO_Disable(0);
  SERVO_Disable(1);
}