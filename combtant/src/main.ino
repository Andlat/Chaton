/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */

#include <LibRobus.h> // Essentielle pour utiliser RobUS



/* ****************************************************************************
Variables globales et defines
**************************************************************************** */
// -> defines...
// L'ensemble des fonctions y ont acces
const float kp=1E-4;
const float ki=6E-7;


void setup(){
  BoardInit();
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() {
  /*while (!ROBUS_IsBumper(RIGHT)){delay(10);}
  
  forwardPID(0.3,220);
  tourner(-2);
  forwardPID(0.3,85);
  tourner(2);
  forwardPID(0.3,20);
  tourner(2);
  forwardPID(0.3,42);
  tourner(-2);
  forwardPID(0.3,78);
  tourner(2);
  forwardPID(0.3,20);
  tourner(-2);
  forwardPID(0.3,120);

  stop();
  delay(500);
  demitour();
  stop();
  delay(500);

  forwardPID(0.2,10);

  forwardPID(0.7,200);
  */

/*   stop();
  delay(500);
  demitour();
  stop();
  delay(500);

  forwardPID(0.3,115);
  tourner(-2);
  forwardPID(0.3,20);

  tourner(-2);
  forwardPID(0.3,78);
  tourner(2);
  forwardPID(0.3,42);
  tourner(-2);
  forwardPID(0.3,20);
  tourner(-2);
  forwardPID(0.3,85);
  tourner(2);
  forwardPID(0.3,220); */






robot_A();

delay(100);

  //stop();

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
  //MOTOR_SetSpeed(roue,0);


//   if (x>0)
//   //tourne à droite
//   {
    

//     while (x!=0)
//     {
//     ENCODER_Reset(0);
//     nbtours= (ENCODER_Read(0));
//     MOTOR_SetSpeed(0,0.5);
    
//     while ( nbtours < 1995)
//         {
//         delay(10);
//         nbtours=ENCODER_Read(0);
//         }
//      x--;
//     }    
//   MOTOR_SetSpeed(0,0);
//   } 
    
    
// if (x<0)
// //tourne à gauche
//   {
    

//     while (x!=0)
//     {
//     ENCODER_Reset(1);
//     nbtours= (ENCODER_Read(1));
//     MOTOR_SetSpeed(1,0.5);
    
//     while ( nbtours < 1995)
//         {
//         delay(10);
//         nbtours=ENCODER_Read(1);
//         }
//      x++;
//     }    
//   MOTOR_SetSpeed(1,0);
//   } 
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
// Robot A
void robot_A ()
{
  int sonar =  ROBUS_ReadIR(0);
  Serial.print(sonar);
  Serial.print("\n");



}