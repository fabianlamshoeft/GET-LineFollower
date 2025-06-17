/*
LINE FOLLOWER ROBOT

The Software is structured in a layer fashioned way. 
The Idea is to abstract from the complex underlying Software, so that for the Main Logic, there is a set of High Level functions,
which take care of everything hardware related.

*/

/*-------------------globale Variablen------------------------
float curSpeedR=aktuelle Geschwindigkeit des rechten Motors
float curSpeedL=aktuelle Geschwindigkeit des linken Motors
bool isFinished=gibt an, ob die Ziellinie erreicht wurde, sodass von race_state in ready_state übergegangen werden kann (ist zu Begin false)
int brightLine=wird bei Kalibrierung einmal gesetzt und gibt den Wert an, der gemessen wird, wenn der ensor auf der Linie ist
*/

//--------------------Lexikon aller Variablen------------------


void setup() {
  // Define Pin Modes

}

void loop() {
  // Controlling the States of the Automat
  //im loop muss überprüft werden, ob isFinished==true, dann ready_state, sonst wiederholt race_state
}




// ------------------------------ Layer 0 LOW LEVEL CODE - Reading states, controlling Motors -----------------------------------------------

/* 
Accessing the Motorcontroller and abstracting from it, so that the funktions take 
a value between -1023 and 1024, which corresponds to a forward Movement for positive values and a backward Movement for negative values.
*/

void setRawMotorL(int value){

}

void setRawMotorR(int value){

}

// Very simple (one line) getter Methods, which simply capsulate the Analog Read Method for the Sensors giving them a "speaking Name" in the code
int getRawSensorL(){

}

int getRawSensorR(){

}

int getRawPotiL(){

}

int getRawPotiR(){

}

void setLed(int value){
  
}

bool buttonIsPressed(){

}

// ------------------------------ Layer 1 KALIBRATION / ERROR CORRECTION ------------------------------

//Sets the difference of the Motor speeds to perform a forward movement
void calibrateMotorDiff(int delta){

}
//Sets the minimum value for the motors, so that a logical value above 0 indeet means, that the robot moves
void calibrateMotorSpeed(int delta){

}
//Translates the logical value into the actual value
void setMotorL(int value){

}
//Translates the logical value into the actual value
void setMotorR(int value){

}



//gives the Error corrected SensorValue, calculated with a Error correction Strategy
int getSensorL(){

}
int getSensorR(){

}


//Error Correction Strategies
int getMovingAverage(int newValue){

}
int getSomeSuperSmartErrorCorrectedSensorValue(){
  //Invention of a super smart Algorithm...
}




// ------------------------------Layer 2 PRIMITIVES ------------------------------

//Sets the brightnes Level of the line
void setSensorLLine(){

}
//Sets the brightness Level of the background
void setSensorLBackground(){

}
//Sets the brightnes Level of the line
void setSensorRLine(){

}
//Sets the brightness Level of the background
void setSensorRBackground(){

}
//Some function to calculate a good threshold to distinguish between line and background
void calculateThreshold(){

}

//Returns a logical state which tells, whether the left Sensor is on the line or not
bool sensorLIsOnLine(){

}
//Returns a logical state which tells, whether the right Sensor is on the line or not
bool sensorRIsOnLine(){

}

//Sets the overall Speed of the Robot
void setMotorSpeed(int error){

  //Fehlergröße normieren
  int absError=constrain(abs(error), 0, 600);

  //Bremsfaktor: je größer die Abweichung, desto stärker bremsen
  int breakFactor = map(absError, 0, 600, 40, 100) / 100.0;

  //Geschwindigkeit anpassen
  int adjustedSpeedR= constrain(curSpeedR * brakeFactor, -255, 255);
  int adjustedSpeedL= constrain(curSpeedL * brakeFactor, -255, 255);

  setMotorR(adjustedSpeedR);
  setMotorL(adjustedSpeedL);
}

//Sets how steep the Robot takes a curve. A value below 0 means a left curve, A value above 0 means a right curve and a 0 means no curve
//positive Richtung: mathematisch korrekt!!!!!!! gegen den Uhrzeigersinn
void setAngle(int error){

  float errorFactor = map(abs(error), 0, 600, 0.2 * 100, 1.0 * 100) / 100.0; //bei großm Error soll auch stark korrigiert werden (Sensoren messen einzeln Werte zwischen 0 und 1023 --> Differenz von 1023 aer unrealistisch, daher begrenzen auf 600), daher Abbildung auf Werte von 0.2 bis 1
  float speedFactorR = map(curSpeedR, 100, 200, 1.0 * 100, 0.4 * 100) / 100.0; //bei hoher Geschwindigkeit soll sanfter angepast werden, bei langsamer kann schneller gedreht werden, daher Abbildung auf einen Wertebereich von 1 bis 0.4
  float speedFactorL = map(curSpeedL, 100, 200, 1.0 * 100, 0.4 * 100) / 100.0; //gleiches für links

  float Kp = constrain(errorFactor * speedFactorR * speedFactorL, 0.2, 1.0); //Proportionalitätsfaktor, der sich nach Größe des Fehlers und aktueller Geschwindigkeit justiert

  int turn = Kp * error; //nochmal mit Error multiplizieren, wenn error negativ, dann lenken nach ...-->größerer Error = mehr Drehung

  //asymmetrische Lenkung, u Kurve zu fahren, wenn keine Kurve nötig, dann ist turn=0
  int speedL = curSpeed + turn;
  int speedR = curSpeed - turn;

  // Clipping
  speedL = constrain(speedL, -255, 255); //Speed kann maximal zwischen -255(maxSpeed rückwärts) und 255(maxSpeed vorwärts) sein
  speedR = constrain(speedR, -255, 255);

  setMotorL(speedL);
  setMotorR(speedR);
}

//Method to set different LED Modes to indicate the differend Main Logic States
void blinkLed(int speed, int pattern){

}

//Funktion, die einmalig bei Übergang in race_state() aufgerufen wird
void slowStart(){

  int speed = 100; //Anfangsgeschwindigkeit soll sofort auf 100 gesetzt werden

  while (speed < 150) {
    setMotorL(speed);
    setMotorR(speed);
    delay(50); //50ms warten
    speed += 10; //Geschwindigekeit langsam um 10 erhöhen
  }
}

//Funktion, die aufgerufen wird, wenn das Ende der Strecke erreicht ist
void finish(){
  setMotorL(0);
  setMotorR(0);

  //------------------------hier vielleicht noch blinken der LED????
}



// ------------------------------ Layer 3 MAIN LOGIC STATES ------------------------------


void calibrationState1(){

}

void calibrationStte2(){

}

void calibrationState3(){

}

void calibrationState4(){

}

//
void readyState(){

}

//Hauptmethode der Logik (Konvention: globale Variablen oben in Dokument)
void raceState(){
  //Check, ob der Wagen still steht, wenn ja dann slowStart()
  if (curSpeedL=0 && curSpeedR=0){
    slowStart();
  }

  int sensorL = getSensorL(); //hier wird angenommen, dass getSensor...() jeweils einen return hat
  int sensorR = getSensorR(); //ggf. anpassen, wenn mit zeigern gearbeitet wird

  //wenn beide Sensoren auf der Linie sind, ist die Ziellinie erreicht
  if(getSensorR()>(brightLine-50)&& getSensorL()>(brightLine-50)){ //Sensorwerte zeigen beide eine Abweichung in die gleiche Richtung (beide auf Linie) --> noch festhalten, wie groß die Abweichung sein muss (hier jetzt 50 gewählt)
    isFinished = true;
    finish();
  }

  //Differenz der Sensoren als Übergabewert für weitere Funktionen
  int diff = sensorR-sensorL;

  setMotorSpeed(diff); //erst Speed anpassen, um ggf. zu bremsen oder beschleunigen
  setAngle(diff); //danach Winkel anpassen zur Feinjustierung der Räder
}

//Überlegungen: verlorene Linie wiederfinden, PID einbauen

//wenn von readyState() zu raceState() übergegangen wird soll einmalig die Funktion slowStart() aufgerufen werden, danach wird in race_state() übergegangen
