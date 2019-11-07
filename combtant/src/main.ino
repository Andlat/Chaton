/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

#include <LibRobus.h>

//#define UTILISER_CAPTEURS
//#define UTILISER_SUIVEUR

#define VERT 1
#define BLEU 2
#define JAUNE 3
#define ROUGE 4

//#define ROBOT_A
#define ROBOT_B

//#define COULEUR VERT
#define COULEUR BLEU
//#define COULEUR JAUNE
//#define COULEUR ROUGE

const float kp=1E-4;
const float ki=6E-7;


void setup(){
  BoardInit();
}


bool stopped = false;
void loop() {

  //Serial.print(analogRead(A0));
  //Serial.print("\n");
  //delay(500);

if(!stopped){
    SERVO_Enable(0);


#ifdef ROBOT_A
  while (!ROBUS_IsBumper(REAR)){delay(10);}
  robot_A();
#elif defined(ROBOT_B)
  robot_B();
#endif

  SERVO_Disable(0);


stop();
  stopped = true;
}

delay(1000);
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

  while (abs(total_encoder) < abs(goal_encoder))
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

void tournerSurPlace(float x){
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
  int angle_depart = (COULEUR == ROUGE ? -2 : 2);
  
  //Pour le detecteur de proximite
  float ang = 0;
  bool found = true;
  float increment = 0.1f;

  //SE RENDRE JUSQU'A LA BALLE
  if(COULEUR == ROUGE || COULEUR == VERT){
   tournerSurPlace(angle_depart);
    forwardPID(VITESSE, 60);
    tournerSurPlace(-angle_depart);
    forwardPID(VITESSE, 105);
    tournerSurPlace(angA());

  }else{//JAUNE et BLEU
    forwardPID(VITESSE+0.2, 33);
    tournerSurPlace(angA());
    forwardPID(VITESSE, 90);
  }

  forwardPID(VITESSE, 10);

#ifdef UTILISER_CAPTEURS
  Serial.print(ROBUS_ReadIR(1));
  while(ROBUS_ReadIR(1) < 150){
    Serial.print(ROBUS_ReadIR(1));
    Serial.print('\n');

    tournerSurPlace(increment);
    ang += increment;
  
    if(ang >= 0.5f){ Serial.print(ang);Serial.print("AAAAAA\n"); increment *= -1;}
    else if(ang <= -0.5f){ 
      found = false;
      stop();
      delay(250);
      break;
    }

    stop();
    delay(250);
  }
  if(!found){
    tournerSurPlace(-ang);
  }
#endif

  forwardPID(VITESSE, 20);
  stop();
  delay(1000);
  activerPince(false);
  stop();
  delay(1000);
  forwardPID(VITESSE-0.1f, 1);
  stop();
  delay(1000);
  forwardPID(-VITESSE, 30);
  
#ifdef UTILISER_CAPTEURS
   if(found){
     tournerSurPlace(-ang);
     stop();
     delay(500);
   }
#endif

 if(COULEUR == ROUGE || COULEUR == VERT){

    tournerSurPlace(COULEUR == ROUGE ? -3 : 3);
    forwardPID(VITESSE, 60);//63
    tournerSurPlace(angle_depart);
    forwardPID(VITESSE, 20);
    forwardPID(VITESSE+0.1f, 25);
 }else{
   demitour();
  forwardPID(VITESSE, 66);
 }

  stop();
  delay(1500);
  activerPince(true);
  delay(1000);
  forwardPID(-VITESSE, 35);

  if(COULEUR == JAUNE || COULEUR == BLEU){
    tournerSurPlace(1);
    stop();
  }

  forwardPID(-VITESSE, 60);
}

void robot_B(){
   while (!ROBUS_IsBumper(REAR)){delay(10);}
  delay(60000);
  activerPince(true);
  stop();
  delay(1000);

  forwardPID(VITESSE, 30);
  
  activerPince(false);
  forwardPID(VITESSE, 10);
  stop();
  delay(250);
  forwardPID(-VITESSE, 10);
  //delay(1000);

  tournerSurPlace(angB());

  forwardPID(VITESSE,120);
  stop();
  delay(1000);

  activerPince(true);
  stop();
  delay(1000);
  forwardPID(-VITESSE, 30);
}

void activerPince(bool ouvrir){
  float ANGLE = ouvrir ? 45 : 143;

  SERVO_SetAngle(0, ANGLE);
}
