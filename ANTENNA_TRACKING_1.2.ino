/*
 * Antenna Tracking SElstem to track satellite using NFW and GS232B protocol
 *
 * Autore  : David Chistoni
 * Web     :
 * Post    : 
 * Email   : chistoni@eAzsat.com
 */ 
 
//Inclusione delle librerie
#include <AccelStepper.h>
#include <Bounce2.h>

#include <LiquidCrystal.h>
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
 
//definizione delle costanti dei pin di Arduino
const int ledStatus = 13; //il led on board ci mostrerà lo stato di attivazione dei motori
const int pinHomeAz = 22; // il microswitch di homing sull'Azimuth - Attivo basso
const int pinHomeEl = 23; // il microswitch di homing sull'Elevation - Attivo basso
const int pinEnable = 24;  //i pin che comandano lo stato ENABLE dei driver A4988 sono in collegati in serie per questo basta un solo pin per gestirli entrambi

unsigned long debounceDelAz = 1; //millisecondi per il debonuce del microswitch di Azimuth
unsigned long debounceDelEl = 1; //millisecondi per il debonuce del microswitch di Elevation

long degAz = 0;
long degEl = 0;
long stepsAz = 0;
long stepsEl = 0;
  
int HomeAz_Ok = false;
int HomeEl_Ok = false;


const int stepAz= 26;  //pin digitale che invia i segnali di STEP al driver delle Azimut
const int dirAz = 27; //pin digitale che invia il segnale DIREZIONE al driver delle Azimut
long speedAz, valAz, mapAz;  //variabili di gestione movimenti motore Azimut
 
const int stepEl = 28;  //pin digitale che invia i segnali di STEP al driver delle Elevation
const int dirEl = 29; //pin digitale che invia il segnale DIREZIONE al driver delle Elevation
long speedEl, valEl, mapEl;  //variabili di gestione movimenti motore Elevation
 
//variabili utilizzate dalla libreria AccelStepper
const int maxSpeed = 1023;  //stando alla documentazione della libreria questo valore può essere impostato fino a 4000 per un Arduino UNO
const int minSpeed = 0; //velocità minima del motore
const float accelerazione = 200.0; //numero di step al secondo in accelerazione

const int treshold = 30;  //la lettura dei potenziometri non è mai affidabile al 100%, questo valore aiuta a determinare il punto da considerare come "Stai fermo" nei movimenti
long tresholdUp, tresholdDown;  //variabili di servizio per espletare il compito descritto sopra 

boolean abilitato, muoviAz, muoviEl, enable;  //variabili di gestione dei movimenti

//Bounce uSWHomeAz = Bounce();  //istanzia un bottone dalla libreria Bounce per il micruswitch di Az Homing
//Bounce uSWHomeEl = Bounce();  //istanzia un bottone dalla libreria Bounce per il microswitch di El Homing
 
//istanzia i motori
AccelStepper motoreAz(AccelStepper::DRIVER, stepAz, dirAz);
AccelStepper motoreEl(AccelStepper::DRIVER, stepEl, dirEl);

//**************************************************************************************************************************************************************** 
void setup() {
  Serial1.begin(115200); //Inizializza la porta seriale con cui comunica col PC
  lcd.begin(20, 4); // set up the LCD's number of columns and rows:
  lcd.print("Sats Tracking Sys"); // Print a message to the LCD.
  delay(1000);
  lcd.clear();  
  //inizializza valori
  speedAz = speedEl = 0;
  enable = true;    // ENABLE è attivo basso per A4988 e Alto o Basso per i Driver professionali !
 
  //definizione delle modalità dei pin
  pinMode(ledStatus, OUTPUT);
  pinMode(pinEnable, OUTPUT);
 
  pinMode(pinHomeAz, INPUT); //l'input del micro switch di Home Azimuth ha bisogno di essere settato come INPUT_PULLUP
  pinMode(pinHomeEl, INPUT); //l'input del micro switch di Home Elevation ha bisogno di essere settato come INPUT_PULLUP
  
//  digitalWrite(ledStatus, enable); //Accende il Led di Stato di Arduino per mostrare lo stato del pin di ENABLE
  digitalWrite(pinEnable, !enable); //Abilita il pin di ENABLE dei Drivers(Mettere !enable se il driver vuole livello basso, altrimenti levare il !)
 
  //configura i microswitch di Homing sia in Azimuth che in Elevation utilizzando la libreria Bounce
//  uSWHomeAz.attach(pinHomeAz);
//  uSWHomeAz.interval(debounceDelAz);
//
//  uSWHomeEl.attach(pinHomeEl);
//  uSWHomeEl.interval(debounceDelEl);
  
//  calcola il range dei valori entro i quali considerare la posizione del joystick come "stai fermo"
//  tresholdDown = (maxSpeed / 2) - treshold;
//  tresholdUp = (maxSpeed / 2) + treshold;

  //configura parametri dei motori
  motoreAz.setMaxSpeed(maxSpeed);
  motoreAz.setSpeed(minSpeed);
  motoreAz.setAcceleration(accelerazione);
 
  motoreEl.setMaxSpeed(maxSpeed);
  motoreEl.setSpeed(minSpeed);
  motoreEl.setAcceleration(accelerazione);
  
  lcd.print("End Set-Up");
  delay(1000);
  lcd.clear();
  
 
  
}


//****************************************************************************************************************************************************************
 
void loop() {
if (HomeAz_Ok == false) {  
checkHomeAzEl();    
}

//  digitalWrite(ledStatus, enable);  //mostra stato di abilitazione tramite il led su pin 13
//  digitalWrite(pinEnable, !enable); //imposta valore opposto sui pin ENABLE dei driver (Mettere !enable se il driver vuole livello basso, altrimenti levare il !)
//
//// //esegui lettura analogica dei valori provenienti dai potenziometri del joystick
////  valX = analogRead(jX);
////  valY = analogRead(jY);
// 
//  valAz = 600;
//  valEl = 256;
//  tresholdDown = 511;
//  tresholdUp = 511;
//   
//  //mappa i valori letti in funzione della velocità inima e massima
//  mapAz = map(valAz, 0, 1023, minSpeed, maxSpeed);
//  mapEl = map(valEl, 0, 1023, minSpeed, maxSpeed);
//
//  //esegui funzione di comando dei motori
//  pilotaMotori(mapAz, mapEl);
 
}
 
void pilotaMotori(long mapAz, long mapEl) {

 
  if (mapAz <= tresholdDown) {
    //Az va indietro
    speedAz = -map(mapAz, tresholdDown, minSpeed,   minSpeed, maxSpeed);
    muoviAz = true;
  } else if (mapAz >= tresholdUp) {
    //Az va avanti
    speedAz = map(mapAz,  maxSpeed, tresholdUp,  maxSpeed, minSpeed);
    muoviAz = true;
  } else {
    //Az sta fermo
    speedAz = 0;
    muoviAz = false;
  }
 
  if (mapEl <= tresholdDown) {
    //El va giù
    speedEl = -map(mapEl, tresholdDown, minSpeed,   minSpeed, maxSpeed);
    muoviEl = true;
  } else if (mapEl >= tresholdUp) {
    //El va su
    speedEl = map(mapEl,  maxSpeed, tresholdUp,  maxSpeed, minSpeed);
    muoviEl = true;
  } else {
    //El sta fermo
    speedEl = 0;
    muoviEl = false;
  }
 
  if (muoviAz) {
    motoreAz.setSpeed(speedAz);
    motoreAz.run();
  } else {
    motoreAz.stop();
  }
 
  if (muoviEl) {
    motoreEl.setSpeed(speedEl);
    motoreEl.run();
  } else {
    motoreEl.stop();
  }
}

//ciao
void checkHomeAzEl() {
    if (digitalRead(pinHomeAz)== HIGH ) { 
    motoreAz.setSpeed(200);
    motoreAz.run();
    digitalWrite(ledStatus, false); 
    }
    else {
    lcd.print("Az Home Found");
    digitalWrite(ledStatus, true); 
    motoreAz.stop();
    HomeAz_Ok = true;
    }
  if (digitalRead(pinHomeEl)== HIGH ) { 
    motoreEl.setSpeed(200);
    motoreEl.run();
    digitalWrite(ledStatus, false); 
    }
    else {
    lcd.print("El Home Found");
    digitalWrite(ledStatus, true); 
    motoreEl.stop();
    HomeEl_Ok = true;
    }
}

void checkHomeCenterAzEl() {

//  uSWHomeAz.update();
//  if (uSWHomeAz.rose()) {
//    enable = !enable;
//    stepsAz--;
//    lcd.clear();
//    lcd.print(stepsAz);
//  }
//  uSWHomeAz.update();
//  if (uSWHomeAz.fell()) {
//    enable = !enable;
//    stepsAz++;
//    lcd.setCursor(2, 1);
//    lcd.print(stepsAz);
//  }

}

