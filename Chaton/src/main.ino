/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

#include <LibRobus.h>


void setup(){
  BoardInit();
  BluetoothInit();

  BLUETOOTH_setCallback([](){
    Serial.println(BLUETOOTH_read());
  });
}


void loop() {
  BLUETOOTH_println("AT+MAC?");
  delay(1000);
}