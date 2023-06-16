// GNZDA_to_UTC, à partir du message NMEA "$GNZDA,Time,Date" envoyer le message "$UTC,Date,Timee" pour simuler ce message non standard d'Applanix "tm".
//
// Promoteur   : Raphaël Mabit 
// Auteur      : Bruno Dionne 2023-06-04
// Motivation  : Un data loggeur commercial avec 8 ports série, refuse de fonctionner tant que le message GPS non standard "$UTC,<date>,<time>" n'est pas reçu à intervalle régulier.
//               Ce Sketch Arduino insère de manière transparente le message $UTC aux autres messages NMEA d'un GPS générique ici le U-BLox ZED-F9R de Spakfun.com
//               Le message $UTC est contrut à partir du message $GNZDA pour la date et $GNRMC pour l'heure.
//               Le programme prend en compte le temps de réception, de transmission et de traitement des messages NMEA et ajoute cette correction au temps sUTC du message $UTC
//               Le temps UTC du message $UTC lorsqu'il est reçu par le data loggeur externe est donc à ± 0.567 mS du temps réel d'émission du signal PPS par le GPS.
//               Il est donc raisonnable de considérer le message $UTC comme une excellente approximation du signal PPS quand il n'est pas possible d'avoir au signal PPS original.
//
// Version    : 2023-06-15, 21h54
// Remarques  : Ajout d'une limite maximale pour DeltaUTC (Protection anti rollover de millis() et autres délais possibles dans la communication GPS/Artemis).
// 
// Suggestion d'interconnexion *****
//
//  U-Blox ZED-F9R <--uart1--> Artemis Thing Plus   ------------------> DataLogger.
//  U-Blox ZED-F9R <--QWIIC--> Artemis Thing Plus  
//  U-Blox ZED-F9R <- uart2--> Serial to Bluetooh  <------------------> Ordinateur, Tablette ou Mobile
//
//  GPS TX UART1  ----> RX Serial1 (Artemis thing plus) TX Serial1 ---> RX DataLogger.
//
//  TX/RX UART2    <---> Module Bluetooth 4.0 SPP     <---> SPP Téléphone/Tablette.
//  i2c QWIIC      <---> i2c QWIIC Artemis thing plus <---> Librairie "SparkFun_u-blox_GNSS_v3"
//
//  Artemis thing plus particularités ***** 
//
//  Serial  est le port série UART via le port USB-C.
//  Serial1 est l'autre port série UART via les broches TX et RX.
//  Module Bluetooth BLE intégré pour envoyer ou recevoir des données de surveillance, des commandes de contrôle ou autres à une tablette ou un téléphone intelligent. (Pas utilisé dans ce Sketch).
//
// Fonctionnement général :
// 
//  Horodater la détection du signal PPS en provenance du GPS
//  Lire les messages NMEA du GPS qui arrivent en bloc juste après le signal PPS.
//  Chaque caractère reçu est immédiatement renvoyé sans autres traitements vers le data loggeur. 
//  Prendre en note la date UTC à partir du message $GNZDA
//  Prendre en note l'heure UTC à partir du message $GNRMC
//  Dès la réception complète du message $GNRMC, construction et expédition du message $UTC avec la correction pour les temps de traitement.
//  Le programme est optimisé pour des messages NMEA de positionnement à 1 Hz seulement. Si vous voulez avoir plus de 1 Hz vous devez modifier le programme.
//
// Développement furtur :
//     Synchroniser l'horloge en temps réel RTC du Artemis thing plus avec le temps UTC aux fins de validation et vérification (GPS time anti spoofing).
//     Ajouter une fonction au bouton usager du Artemis Thing Plus pour des fonctionnalités supplémentaires (Ex: Start/Stop ou avec ou sans $UTC, etc. ). 
//     Ajouter une interruption pour capturer des alarmes de l'horloge en temps réel RTC du Artémis Thing Plus pour des fonctionnalités supplémentaires.
//     Ajouter des sondes externes pour supporter plus de types de messages NMEA (Magnetomètre, Temperature, Humidité, Salinité, Pression, etc.).
//     
//
// ***** Include  *****
//
//
#include <Arduino.h>          // Librairies, variables et constantes standards Arduino.
#include <SPI.h>              // used for SPI communication.
#include <Wire.h>             // Used for serial communication over USB //Needed for I2C to GNSS.
//#include "RTC.h"            // Real time clock. The RTC library included with the Arduino_Apollo3 core (Pas utilisé pour le moment).
//#include "WDT.h"            // Watchdog library (Pas utilisé pour le moment).
//#include <ArduinoBLE.h>     // Module intégré Bluetooth BLE du Artemis thing plus.(Pas utilisé pour le moment).
//
//
// ***** U-blox Sparkfun librairie en option ***** (Pas utilisé pour le moment).
//
//
//#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
//SFE_UBLOX_GNSS myGNSS;
//
//#define myWire Wire1 // Connect using the Wire1 port. Change this if required
//#define gnssAddress 0x42 // The default I2C address for u-blox modules is 0x42. Change this if required
//
//
// ***** Define *****
//
//
// ***** Constant *****
//
//
// Habituellement les SBC Arduino définissent 'LED_BUILTIN'
// Ce symbole est associé à la broche pin_size_t qui est la LED embarquée.
//
const pin_size_t ledPinNumber = LED_BUILTIN;  //LED par son numéro de broche
//
// Mbed boards use a PinName called LED1 to indicate the built-in LED
//
const PinName ledPinName = LED1;   // By name
//
const int ppsIntPin        =  5;   // Numéro de la broche pour l'Interruption  de détection du signal PPS
const int spiChipSelectPin =  9;   // Numéro de la broche pour Chip select du port SPI
const int buttonIntPin     = 10;   // Numéro de la broche pour le "user button" 
const int resetPin         =  6;   // Numéro de la broche reliée à la broche RESET du GPS U-BLox ZED-F9R (Pas utilisé pour le moment).
const int transmitReadyPin =  7;   // Numéro de broche reliée à la broche TXR du GPS U-BLox ZED-F9R (Pas utilisé pour le moment).
//
//
// ***** Variable  *****
//
//
// ***** Variable pour les interruption ISR
//
volatile bool alarmISR_         = false; // Cette variable est assignée à true si l'horloge interne du Apollo génère une interruption ex: "alarme".
volatile bool buttonISR_        = false; // Cette variable est assignée à true si on appui sur le "user button".
volatile bool ppsISR_           = false; // Cette variable est assigné à true si le signal PPS en provenance du GPS est détecté.
volatile bool TransmitReadyISR_ = false; // Cette variable est assignée à true si le signal Transmit Ready en provenance du GPS U-BLox ZED-F9R est détecté.
volatile unsigned long horodatePPS = 0;  //  Noter avec la fonction millis() l'arrivé du signal PPS. BOGUE en cas de rollover du compteur PPS.
//
volatile bool watchdogFlag = false;      // indicateur Watchdog Timer ISR flag
volatile int watchdogInterrupt = 0;      // Compteur Watchdog interrupt counter
//
//
// ***** autres variables 
//
// ***** Object  *****
//
//
///*********************************************************/
//
//
// ***** Routines *****
//
//
// Routines de gestion des interruptions
//
void ppsISR()                                    // Noter le temps d'arrivée du signal PPS.
{
  horodatePPS = millis();
  ppsISR_ = true;
}
//
void ButtonISR()                                 // Usage futur de détection du bouton. Ex: Déclencher un traitement si le bouton est poussé par un usager. (Pas utilisé pour le moment).
{
  buttonISR_ = true;
}
//
//
void TransmitReadyISR()                         // Noter le signal de la broche TXR (buffer full) du GPS (Pas utilisé pour le moment).
{
  TransmitReadyISR_ = true; 
}
//
//
//
//extern "C" void am_rtc_isr(void)              // Usage futur de l'horloge en temps réel du Artemis thing plus. Ex: déclenchement d'un traitement par une alarme de l'horloge à la seconde près ± 4 microsecondes.
//{ 
//  am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);   // Effacer RTC alarm interrupt
//  alarmISR_ = true;  
//}
//
//***************************************************************/
// ***** Appelé une fois quand le MCU est alimenté ou reset *****/
//***************************************************************/
//
void setup() {        // Au démarrare du Artemis thing plus toutes les broches sont "pulled LOW". 
  byte tempoByte = 0;
  Serial.begin( 115200 );                                             // Démarrer la communication Serie USB.
  while ( !Serial );                                                  // Attendre l'ouverture de la communication (nécessaire sur certain board seulement).
  Serial.print( "Serial USB= OK, " );                                 // Confirmation de la disponibilité du port Série USB  "Serial"  
  Serial1.begin( 115200 );                                            // Port série UART du Artemis thing plus.  "Serial1"
  while ( !Serial1 );                                                 // Wait for serial connection to open (only necessary on some boards).
  Serial.print( "Serial1 UART= OK, " );                               // Confirmation de la disponibilité du port Série UART "Serial1"
  // 
  pinMode(LED_BUILTIN, OUTPUT);                                       // Gestion de la LED bleue à OFF.  
  pinMode( ppsIntPin, INPUT );                                        //set the IRQ pin as an input pin. do not use INPUT_PULLUP - the ZED-F9R will pull the pin.
  attachInterrupt(digitalPinToInterrupt(ppsIntPin), ppsISR, RISING);  // Le ZED-F9R will pull the interrupt pin HIGH when a PPS event is triggered.
  Serial.print("PPS Interrupt= OK, " );                                // Confirmer la mise en place de l'interruption
  pinMode( transmitReadyPin, INPUT );                                 //set the IRQ pin as an input pin. do not use INPUT_PULLUP - the ZED-F9R will pull the pin.
  //
  while( Serial1.available()){ tempoByte = Serial1.read();}           // Vider le tampon des caractères reçus du GPS pour un démarrage plus propre de loop().
  Serial.println("Flush RX= OK.");
}
//
//
//******************************************************************************************
// !!!!! Après "setup()", "loop()" est une boucle infinie tant que le MCU est alimenté !!!!!
//******************************************************************************************
//
//
void loop() { // La boucle loop() est trop lente pour des SBC d'une fréquence <= 16 MHz. Mais le Artemis Thing Plus roule à 48 MHz.
  // static = La valeur de la variable est conservée entre chaque itération de loop(). Sinon c'est effacé.
  byte inByte =0;                                 // Caratère courant. 
  static byte indexMsg = 0;                       // Index du tableau des caractères pour le message courant.
  const byte numChars = 255;                      // Size du Buffer de caractères temporaire pour un message NMEA  Débutant par '$' et se terminant par une fin de ligne. 
  static char receivedChars[numChars];            // Buffer de caractères temporaire pour un message NMEA  Débutant par '$' et se terminant par une fin de ligne. 
  static char utcDate[9];                         // Extraction de la Date à partir du message $GNZDA avec la virgule    AAAAMMJJ,
  static char utcTime[7];                         // Extrsction de l'heure à partir du message $GNRMC avec le point      HHMMSS.
  static bool debutNMEA = false;                  // Détection du caractère '$'.
  static bool finNMEA = false;                    // Détection du caractère de fin de ligne \n - 10. 
  static unsigned long DeltaUTC = 0;              // Correrction du temps UTC de GNZDA pour ajouter le délai PPS-NMEA, le délai de transmission RS-232 et l'imprécision du timer millis().
  //
  //  
  if (ppsISR_ == true){                           // Un event PPS est détecté.
    //horodatePPS = millis();                     // Cette variable est assignée dans la routine de gestion de l'interruption PPS.
    ppsISR_ = false;                              // indiqué que le traitement PPS est fait.
    //Serial1.write('~');
    digitalWrite(LED_BUILTIN, HIGH);              // Allumer la LED bleue pour indiquer l'arrivé du signal PPS.
    Serial.write( "PPS -> ");
    Serial.write( "$UTC,");                // $UTC,  Nom du message     
    Serial.write( &utcDate[0], 9);         // AAAAMMJJ,
    Serial.write( &utcTime[0], 7);        // HHMMSS.
    Serial.write( " + DeltaUTC de ");
    Serial.print( DeltaUTC );
    Serial.println( " mS.");
  }//endif ppsISR
  //
  if( Serial1.available()) {                                  // Caractères disponibles en provenance du GPS.
    inByte = Serial1.read();                                  // Lire un caractère comme un byte en provenance du GPS ( Serial1 broche RX en provenance du GPS).
    Serial1.write( inByte  );                                 // Écrire ce même caractère vers le data logger         ( Serial1 broche TX à destination du data loggeur).
    //
    if( inByte == '$' ) {                                     // Est-ce un début de message '$'
      debutNMEA = true;                                       // Début de message détecté.
      finNMEA   = false;                                      // Début message alors annule fin de message.
      receivedChars[0] = inByte;                              // Enregistre la caractère courant dans le buffer. Le début de message commence toujours à l'indice zéro du tableau      
      indexMsg = 1;   
    } else if ( inByte == 10 /*|| indexMsg > 87*/ ) {         // Fin message (Max longueur NMEA = 85, mais U-BLox génère exceptionnellement des messages NMEA d'une longueur = 87 et des messages vides avec un LF à la fin.  
        receivedChars[indexMsg] = inByte;                     // Enregistre le caractère courant dans le buffer, Force à LF au cas ou le message > 82 caractères
        indexMsg++;                                           // incrémente le pointeur du tableau des caractères reçus.
        finNMEA = true;                                       // Fin de message détectée.
      }                             
    else {                                                    // Si un message est en cours de détection, accumuler les caractères dans le tableau pour traitement ultérieur
      if( debutNMEA){                                         // N'enregistre aucun caractère s'il n'y a pas un début de détecté. On évite le traitement en milieu de message, les lignes vides, les problèmes de transmission.
        receivedChars[indexMsg] = inByte;                     // Enregistre la caractère courant dans le buffer
        indexMsg++;                                           // incrémente l'index tableau  
      }
    }
    digitalWrite(LED_BUILTIN, LOW);                           // Éteindre la LED bleue pour indiquer la fin du traitement/remplissage du tableau.
    //
    // Un caractère reçu a été envoyé et traité.
    // S'il reste des caractères du bloc à recevoir, vous avez entre 2.6 mS et maximum 5.6 mS pour faires des tâches connexes. Sinon il y a risque de perdre des caractères en provenance du GPS (Tampon Serie de 32 à 64 caractères à 155200 bauds)
    // Si c'est le dernier caractère du bloc vos avez environ 900 mS pavant le prochain PPS pour faire des tâches connexes. Sinon il y a risque de perdre des caractères en provenance du GPS.
    //
    if ( debutNMEA &&  finNMEA ) {  // Le message NMEA courant est complet.
      // Message $GNRMC
      if (receivedChars[0] == '$' && receivedChars[1] == 'G' && receivedChars[2] == 'N' && receivedChars[3] == 'R' && receivedChars[4] == 'M' && receivedChars[5] == 'C' && receivedChars[6] == ',' ) { // Message "$GNRMC," détecté. Extraire l'heure UTC.
        utcTime[0] = receivedChars[7];          //H
        utcTime[1] = receivedChars[8];          //H
        utcTime[2] = receivedChars[9];          //M
        utcTime[3] = receivedChars[10];         //M
        utcTime[4] = receivedChars[11];         //S
        utcTime[5] = receivedChars[12];         //S
        utcTime[6] = receivedChars[13];         //.  on sauve des mS en utilisant directement receivedChars 
        // À partir d'ici nous avons la date et l'heure UTC, nous pouvons construire et envoyer le message $UTC.
        Serial1.write( "$UTC,");                // $UTC,  Nom du message     
        Serial1.write( &utcDate[0], 9);         // AAAAMMJJ,
        Serial1.write( &receivedChars[7], 7);   // HHMMSS.
        //Serial1.write( &utcTime[0], 7);  
        // La fonction millis() prend 0.718 mS à s'exécuter (temps réel 0.720160 mS).
        // Les 5 prochains caractères prennent 0.445 mS à transmettre à 115200 bauds (temps réel 0.446339 mS).
        // Le délais moyen PPS et $UTC est 0.258 mS (delta temps réel 0.000776 mS)
        // Total 1.167 mS à ajouter pour compenser ce temps de latence. Quand le data loggeur va recevoir le message UTC la précision par rapport au signal PPS est de ± 0.567 mS
        DeltaUTC = millis() + 1UL - horodatePPS; 
        if (DeltaUTC >= 740UL){                             // Si délais anormalement long de 740 mS. Rollover de millis() après 49 jours ? Autres ?
          Serial.write( "2591\r\n" ); }                      // Impose arbitrairement 259.1 mS. Le ".2591" est facile à chercher dans les logs, car normalement le 4e chiffre après le point est toujours zéro. 
        else {  
          if ( DeltaUTC < 100 ) { Serial1.write( '0'); }    // Format du nombre avec des zéros significatifs avant
          if ( DeltaUTC <  10 ) { Serial1.write( '0'); }    // Format du nombre avec des zéros significatifs avant
          Serial1.print( DeltaUTC );                        // Différentiel entre le temps réel UTC et le temps de transit GPS->Data Loggeur. 
          Serial1.write( "0\r\n");                          // Le 4e chiffre après le point pour les secondes.
        }  
        }//endif $GNRMC,                                  //  Duré totale de l'envoi du message UTC environ 2.4 mS :) :) :)
      // Message $GNZDA
      else if (receivedChars[0] == '$' && receivedChars[1] == 'G' && receivedChars[2] == 'N' && receivedChars[3] == 'Z' && receivedChars[4] == 'D' && receivedChars[5] == 'A' && receivedChars[6] == ',' ) { //Message "$GNZDA,"
        // Peut être entrecouper d'un délai jusqu'à 800 mS à cause de la priorité plus grande accordée au traitement du signal PPS dans le GPS quel celle pour l'envoi des messages NMEA (avant ou pendant ou  après le PPS).
        // Ce n'est pas grave si le message "$GNZDA," est entrecoupé d'un délai d'environ 750 mS car stratégiquement on ne garde que sa date. 
        // Raphaël ne fait pas de kayak à 00h00 UTC, car il y a bogue extrême ici. Le changement de date est potentiellement retardé d'une seconde à cause de la dérive du PPS dans les messages NMEA :), mais l'heure est toujours bonne.
        // Autre BOGUE potentiel si tu restes dans to Kayak plus de 49 jours tu risques d'avoir un bogue dans l'heure UTC, car le compteur millis va faire un rollover de 4294967295 à zéro. Désolé ! :)
        utcDate[0] = receivedChars[23];   //A
        utcDate[1] = receivedChars[24];   //A
        utcDate[2] = receivedChars[25];   //A        
        utcDate[3] = receivedChars[26];   //A
        utcDate[4] = receivedChars[20];   //M
        utcDate[5] = receivedChars[21];   //M
        utcDate[6] = receivedChars[17];   //J
        utcDate[7] = receivedChars[18];   //J
        utcDate[8] = receivedChars[27];   //,
        }//endif $GNZDA,
      debutNMEA = false;           // Message traité on attend un nouveau message
      finNMEA = false;                               // Message traité on attend un nouveau message laisse la fin à true et le début à false pour indiquer qu'un bloc a été entièrement traité et on est en attente du prochain PPS
    }//endif debut et fin,


    } //if find bloc NMEA depuis 10 mS
}//end loop
//
//
// ++++++++++ FIN DU PROGRAMME ++++++++++
//
//
// Note pour la section loop()  *****
//
//
// Ici faire des choses de non bloquantes et pas de niaisage.
//
// À 115200 bauds, un caractère est transmis en 86 microsecondes.
//   À 115200 bauds, entre chaque caractère reçu Arduino dispose de 86 microsecondes ou 1276 instructions Arduino pour faire d'autre chose.
// Le buffer pour les ports série est de 64 caractères, on estime 32 caractères IN et 32 caractères OUT car le flot des données est relativement symétrique.
//   32 caractères * 86 microsecondes =  2,7 millisecondes de lousse.
//   Ici tous les caractères sont envoyés à mesure de leurs réceptions, au pif 60 caractères IN et 4 caractères OUT pour le pire cas (lors de l'envoi de $UTC). Donc un lousse sécuritaire entre 5,34 et 5,69 millisecondes.  
// Quand une série complète de messages est reçue (environ 512 caractères ou 45 millisecondes) il reste environ 955 millisecondes d'attente avant la prochaine série associée au PPS suivant.
//   Le message $UTC est construit et expédié tout de suite après le message $GNRMC en moins de 2.5 mS. On est dans les marges sécuritaires pour ne pas perdre des caractères à cause d'un buffer overflow. 
//
// Attention simple guillemet pour le type char et double guillemet pour le type string
//
// Ne pas utiliser les fonctions suivantes, car elles sont bloquantes, Arduino ne peut faire rien d'autre tant que la fonction n'est pas terminée
//   Serial.parseInt()
//   Serial.parseFloat()
//   Serial.readBytes()
//   Serial.readBytesUntil()
//
// Les MCU Arduino ont généralement très peu de mémoire RAM. La string class cause possiblement des corruptions de mémoire. Voir les alternatives ici avec des fontcions des librairies "C"http://www.cplusplus.com/reference/cstring/
//
// char myChar = 'A';
// char myChar = 65; // Les deux sont équivalent
//
// Opérateurs logiques
//
// and             &&
// and_eq          &=
// bitand          &
// bitor           |
// not             !
// not_eq          !=
// or              ||
// or_eq           |=
// xor             ^
// xor_eq          ^=
//
// La fonction millis() est basée sur un compteur interne au CPU qui divise la fréquence du processeur par 1024
// Si le CPU a une fréquence 48 MHz ou 48 000 000 de cycle par secondes
// Alors 48 000 000 divisés par 1024 donnent 46 875 cycles par secondes
// On a 46 875 divisés par 1000 = 46.875 cycles par millisecondes
// Le compteur est un nombre entier, donc on arrondi à 47 cycles par milliseconde
// le ratio de 46.85 divise par 47 donne 0,9973 
// Donc un tick d'une durée supposée de 1 milliseconde est en réalité 0.9973 milliseconde.
// Donc 1 000 millisecondes multipliées par le ratio 0,9973 et arrondies donnent 977 tick de simili millisecondes pour une durée réelle de 1 000 millisecondes.
//
//
// ***** LA VRAIE FIN - Bonne campagne de mesure Raphaël *****

