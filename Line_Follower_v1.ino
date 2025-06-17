/*
LINE FOLLOWER ROBOT

The Software is structured in a layer fashioned way. 
The Idea is to abstract from the complex underlying Software, so that for the Main Logic, there is a set of High Level functions,
which take care of everything hardware related.

*/

//--------------------globale Variablen------------------------


//--------------------Lexikon aller Variablen------------------


void setup() {
  // Define Pin Modes

}

void loop() {
  // Controlling the States of the Automat

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
void setMotorSpeed(int value){

}

//Sets how steep the Robot takes a curve. A value below 0 means a left curve, A value above 0 means a right curve and a 0 means no curve
//positive Richtung: mathematisch korrekt!!!!!!! gegen den Uhrzeigersinn
void setAngle(int value){

}

//Method to set different LED Modes to indicate the differend Main Logic States
void blinkLed(int speed, int pattern){

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

}