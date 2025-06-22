/*
LINE FOLLOWER ROBOT

The Software is structured in a layer fashioned way. 
The Idea is to abstract from the complex underlying Software, so that for the Main Logic, there is a set of High Level functions,
which take care of everything hardware related.

*/

//Programm Mode
int PROGRAMM_MODE = 0;
//Change the Value of this constant to switch between the normal programm behaviour
//and different Test and Monitoring modes

//PROGRAMM_MODE = 0 : Normal Programm
//-----------DEBUG_MODES--------------
//PROGRAMM_MODE = 1 : Raw Sensor Monitoring --- display the raw Sensorvalues in the serial out
//PROGRAMM_MODE = 2 : Error corrected Sensor Monitoring --- display the Sensorvalues after Error correction

//PROGRAMM_MODE = 3 : LED Bilnk Modes Test --- testing all 6 LED Blink modes for 10 seconds in an infinite loop
//PROGRAMM_MODE = 4 : Sensor Kalibration Test
//PROGRAMM_MODE = 5 : Motor Kalibration Test
//------------------------------------




//-----------------------CONSTANTS-------------------------

//Pins for Motor R
const int ENABLE_MOTOR_R =  6;
const int A1_MOTOR_R = 8;
const int A2_MOTOR_R = 7;

//Pins for MotorL
const int ENABLE_MOTOR_L =  5;
const int A3_MOTOR_L = 3;
const int A4_MOTOR_L = 4;

//Pin for SensorL
const int SENSOR_L = 16;
const int SENSOR_R = 15;

const int POTI_L = 18;
const int POTI_R = 17;

const int LED_PIN = 10;

const int BUTTON_PIN = 2;//TODO 

//Statesconstants for the automat
const int START_STATE = 0;
const int CALIBRATION1 = 1;
const int CALIBRATION2 = 2;
const int CALIBRATION3 = 3;
const int CALIBRATION4 = 4;
const int READY = 5;
const int RACING_STATE = 6;

//Types to trigger the Button
const int SHORT_PRESS = 0;
const int LONG_PRESS = 1;

const int STATE_ENTERING = 0;
const int STATE_ENTERED = 1;
const int STATE_EXITED = 2;



//-------------------GLOBAL VARIABLES FOR INDIVIDUAL FUNCTIONS------------------------
float curSpeedR=0;
//aktuelle Geschwindigkeit des rechten Motors
float curSpeedL=0;
//aktuelle Geschwindigkeit des linken Motors
bool isFinished=false;
//gibt an, ob die Ziellinie erreicht wurde, sodass von race_state in ready_state übergegangen werden kann (ist zu Begin false)
int brightLine=0;
//wird bei Kalibrierung einmal gesetzt und gibt den Wert an, der gemessen wird, wenn der snsor auf der Linie ist
bool buttonDown = false;

// Variables for the LED States
int pattern = 1;  // aktuelles Muster
unsigned long previousMillis = 0;
int blinkCount = 0;
bool ledState = LOW;
int step = 0;


//-------------------GLOBAL VARIABLES FOR MULTIBLE FUNCTIONS------------------------
//State variable for the automat in main loop
int state = 0;
long buttonPressTime = 0;
bool buttonPressed = false;
int stateCon = STATE_ENTERING;


//Calibration Values Potis
float potiL_cal = 0;
float potiR_cal = 0;


//Calibrationvalues Sensors
float sensorL_mean_cal = 0;
float sensorR_mean_cal = 0;

float sensorL_var_cal = 0;
float sensorR_var_cal = 0;

float line_brightness = 0;
float background_brightness = 0;

float line_brightness_L = 0;
float line_brightness_R = 0;
float background_brightness_L = 0;
float background_brightness_R = 0;

float threshold = 0;

float motorL_max = 1023;
float motorR_max = 1023;

float motorL_min = 0;
float motorR_min = 0;


//------------------PROGRAMM---------------------------------
void setup() {
  // Define Pin Modes

  pinMode(ENABLE_MOTOR_R, OUTPUT);
  pinMode(A1_MOTOR_R, OUTPUT);
  pinMode(A2_MOTOR_R, OUTPUT);

  pinMode(ENABLE_MOTOR_L, OUTPUT);
  pinMode(A3_MOTOR_L, OUTPUT);
  pinMode(A4_MOTOR_L, OUTPUT);

  pinMode(SENSOR_L, INPUT);
  pinMode(SENSOR_R, INPUT);

  pinMode(POTI_L, INPUT);
  pinMode(POTI_R, INPUT);

  pinMode(LED_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT);
  
  Serial.begin(9600);
  
  potiL_cal = getRawPotiL();
  potiR_cal = getRawPotiR();

 
}

void loop() {
  // Controlling the States of the Automat
  /*
  
  */
  if (PROGRAMM_MODE == 0){
    //normal Programm
    switch (state){
        case START_STATE:
          startState();
          blinkLed(6);
          Serial.println("START");
          //setRawMotorL(1023);
          //setRawMotorR(1023);

        break;
        case CALIBRATION1:
          //Left sensor on Line right sensor on Background
          calibrationState1();
          blinkLed(1);
          Serial.println("CAL 1");

        break;
        case CALIBRATION2:
          //Right sensor on Line left sensor on Background
          calibrationState2();
          blinkLed(2);
          Serial.println("CAL 2");
        break;
        case CALIBRATION3:
          //Motor differende calibration
          calibrationState3();
          blinkLed(3);
          Serial.println("CAL 3");
        break;
        case CALIBRATION4:
          //Motor Speed calibration
          calibrationState4();
          blinkLed(4);
          Serial.println("CAL 4");
        break;
        case READY:
          readyState();
          blinkLed(6);
          Serial.println("READY");
          float sen_l = getSensorL();
          float sen_r = getSensorR();
          float pot_l = getRawPotiR();
          float pot_r = getRawPotiL();
          /*
          Serial.println("---------Sensor_VALUES-----------");
          Serial.print("Sensor LEFT: ");
          Serial.println(sen_l);
          Serial.print("Sensor RIGHT: ");
          Serial.println(sen_r);
          Serial.print("Potentiometer LEFT: ");
          Serial.println(pot_l);
          Serial.print("Calibrated Poti L: ");
          Serial.println(getPotiL());
          Serial.print("Potentiometer RIGHT: ");
          Serial.println(pot_r);
          Serial.print("Calibrated Poti R: ");
          Serial.println(getPotiR());
          Serial.print("L-on-line: ");
          Serial.println(sensorLIsOnLine());
          Serial.print("R-on-line: ");
          Serial.println(sensorRIsOnLine());
          Serial.println("---------------------------------");
          Serial.println();
          Serial.println();
          Serial.println();
          Serial.println();
          delay(2000);
          */

        break;
        case RACING_STATE:
          raceState();
          blinkLed(6);
          Serial.println("RACE");
        break;
      }
      stateCon = STATE_ENTERED;
      
      if (state == RACING_STATE){
        raceState();
          blinkLed(6);
          Serial.println("RACE");
      }
  }

  if (PROGRAMM_MODE == 1){
    //Sensor Monitoring
      setLed(true);
      float sen_l = getRawSensorL();
      float sen_r = getRawSensorR();
      float pot_l = getRawPotiR();
      float pot_r = getRawPotiL();
      /*
      Serial.println("---------Sensor_VALUES-----------");
      Serial.print("Sensor LEFT: ");
      Serial.println(sen_l);
      Serial.print("Sensor RIGHT: ");
      Serial.println(sen_r);
      Serial.print("Potentiometer LEFT: ");
      Serial.println(pot_l);
      Serial.print("calibrated Poti L: ");
      Serial.println(getPotiL());
      Serial.print("Potentiometer RIGHT: ");
      Serial.println(pot_r);
      Serial.print("Calibrated Poti R: ");
      Serial.println(getPotiR());
      Serial.println("---------------------------------");
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      delay(2000);
      */
    
  }
  if (PROGRAMM_MODE == 2){
    
  }
  if (PROGRAMM_MODE == 3){
   //LED Blink Test - Testing all 6 states for 10 seconds. Repeating in infinite loop,
      long unsigned stateTime = millis() % 60000;
      if (stateTime < 10000){
        blinkLed(1);
      }else if (stateTime < 20000){
        blinkLed(2);
      }else if (stateTime < 30000){
        blinkLed(3);

      }else if (stateTime < 40000){
        blinkLed(4);

      }else if (stateTime < 50000){
        blinkLed(5);

      }else if (stateTime < 60000){
        blinkLed(6);
      }
  }
  if (PROGRAMM_MODE == 4){
    
  }
  if (PROGRAMM_MODE == 5){
    
  }
  
  if (buttonPressed == false && digitalRead(BUTTON_PIN) == true){
    buttonPressed = true;
    handleButtonDown();
  }
  if (buttonPressed == true && digitalRead(BUTTON_PIN) == false){
    buttonPressed = false;
    handleButtonUp();
  }

}


void handleButtonDown(){
  //Measure Button pressing Time
  buttonPressTime = millis();
  buttonDown = true;
}

void handleButtonUp(){
  long delta = (millis() - buttonPressTime);
  if (delta > 100){
    if (delta > 700){
      switchState(LONG_PRESS);
      Serial.println("long press");
    }else{
      switchState(SHORT_PRESS);
      Serial.println("short press");
    }
    buttonPressTime = 0;
    buttonDown = false;
  }
  
  
}

void switchState(int pressingType){
  switch (state){

    case START_STATE:
      if (pressingType == SHORT_PRESS){
        state = CALIBRATION1;
      }else{
        //Nothing
      }
      
    break;
    case CALIBRATION1:
    if (pressingType == SHORT_PRESS){
        state = CALIBRATION2;
      }else{
        state = START_STATE;
      }
    break;
    case CALIBRATION2:
      if (pressingType == SHORT_PRESS){
        state = CALIBRATION3;
      }else{
        state = START_STATE;
      }
      
    break;
    case CALIBRATION3:
      if (pressingType == SHORT_PRESS){
        state = CALIBRATION4;
      }else{
        state = START_STATE;
      }
      
    break;
    case CALIBRATION4:
      if (pressingType == SHORT_PRESS){
        state = READY;
      }else{
        state = START_STATE;
      }

    break;
    case READY:
      if (pressingType == SHORT_PRESS){
        state = RACING_STATE;
        Serial.println("switch to race");
      }else{
        state = START_STATE;
      }

    break;
    case RACING_STATE:
      if (pressingType == SHORT_PRESS){
        state = READY;
      }else{
        state = START_STATE;
      }

    break;


  }

  stateCon = STATE_ENTERING;
  previousMillis = millis() + 1000000;
}

// ------------------------------ Layer 0 LOW LEVEL CODE - Reading states, controlling Motors -----------------------------------------------

/* 
Accessing the Motorcontroller and abstracting from it, so that the funktions take 
a value between -1023 and 1024, which corresponds to a forward Movement for positive values and a backward Movement for negative values.
*/

void setRawMotorL(int value){

  Serial.print("Motor L");
  Serial.println(value);
  if(value > 0){
    digitalWrite(A3_MOTOR_L, HIGH);
    digitalWrite(A4_MOTOR_L, LOW);
    analogWrite(ENABLE_MOTOR_L, value);
  }

  if (value == 0){
    digitalWrite(A3_MOTOR_L, HIGH);
    digitalWrite(A4_MOTOR_L, HIGH);
    analogWrite(ENABLE_MOTOR_L, 0);
  }

  if (value < 0){
    digitalWrite(A3_MOTOR_L, LOW);
    digitalWrite(A4_MOTOR_L, HIGH);
    analogWrite(ENABLE_MOTOR_L, value * (-1));
  }
}

void setRawMotorR(int value){
  Serial.print("Motor R");
  Serial.println(value);
  if(value > 0){
    digitalWrite(A1_MOTOR_R, HIGH);
    digitalWrite(A2_MOTOR_R, LOW);
    analogWrite(ENABLE_MOTOR_R, value);
  }

  if (value == 0){
    digitalWrite(A1_MOTOR_R, HIGH);
    digitalWrite(A2_MOTOR_R, HIGH);
    analogWrite(ENABLE_MOTOR_R, 0);
  }

  if (value < 0){
    digitalWrite(A1_MOTOR_R, LOW);
    digitalWrite(A2_MOTOR_R, HIGH);
    analogWrite(ENABLE_MOTOR_R, value * (-1));
  }
}

// Very simple (one line) getter Methods, which simply capsulate the Analog Read Method for the Sensors giving them a "speaking Name" in the code
int getRawSensorL(){
  return analogRead(SENSOR_L);
}

int getRawSensorR(){
  return analogRead(SENSOR_R);
}

float getRawPotiL(){
  float val = (float) (analogRead(POTI_L) - 850) / (float) 200;
  if (val < 0){
    val = 0;
  }
  if (val > 1){
    val = 1;
  }
  return val;
}

float getRawPotiR(){
  float val = (float) (analogRead(POTI_R) - 850)/ (float) 200;
  if (val < 0){
    val = 0;
  }
  if (val > 1){
    val = 1;
  }
  return val;
}

void setLed(bool value){
  digitalWrite(LED_PIN, value);
}


// ------------------------------ Layer 1 KALIBRATION / ERROR CORRECTION ------------------------------


//calculates the difference and the values to correct the delta
void calibrateSensorDifference(){
  line_brightness = (line_brightness_L + line_brightness_R) / 2;
  background_brightness = (background_brightness_L + background_brightness_R) / 2;

  float mean = (line_brightness + background_brightness) / 2;
  float meanL = (line_brightness_L + background_brightness_L) / 2;
  float meanR = (line_brightness_R + background_brightness_R) / 2;

  sensorL_mean_cal = mean - meanL;
  sensorR_mean_cal = mean - meanR;

  threshold = mean;

  int m_cor_line_L = line_brightness_L + sensorL_mean_cal;
  int m_cor_background_L = background_brightness_L + sensorL_mean_cal;

  int m_cor_line_R = line_brightness_R + sensorR_mean_cal;
  int m_cor_background_R = background_brightness_R + sensorR_mean_cal;
  
  sensorL_var_cal = ((line_brightness / m_cor_line_L) + (background_brightness / m_cor_background_L)) / 2;
  sensorR_var_cal = ((line_brightness / m_cor_line_R) + (background_brightness / m_cor_background_R)) / 2;

}
//Translates the logical value into the actual value
void setMotorL(int value){
  if (value > 0){
    setRawMotorL(motorL_max);
  }else{
    setRawMotorL(0);
  }
}
//Translates the logical value into the actual value
void setMotorR(int value){
  if (value > 0){
    setRawMotorR(motorR_max);
  }else{
    setRawMotorR(0);
  }
}



//gives the Error corrected SensorValue, calculated with a Error correction Strategy
int getSensorL(){
  return (getRawSensorL() + sensorL_mean_cal) * sensorL_var_cal;
}
int getSensorR(){
  return (getRawSensorR() + sensorR_mean_cal) * sensorR_var_cal;
}

float getPotiL(){
  
  float value = getRawPotiL() - potiL_cal;
  float span = 0;
  if (value < 0){
    span = potiL_cal;
  }else{
    span = (1 - potiL_cal);
  }


  return value / span;
}

float getPotiR(){
  float value = getRawPotiR() - potiR_cal;
  float span = 0;
  if (value < 0){
    span = potiR_cal;
  }else{
    span = (1 - potiR_cal);
  }


  return value / span;
}


//Error Correction Strategies
int getMovingAverage(int newValue){

}
int getSomeSuperSmartErrorCorrectedSensorValue(){
  //Invention of a super smart Algorithm...
}




// ------------------------------Layer 2 PRIMITIVES ------------------------------


//Returns a logical state which tells, whether the left Sensor is on the line or not
bool sensorLIsOnLine(){
  float sensorVal = getSensorL();
  if (line_brightness < background_brightness){
    //Black Line, White BG
    return sensorVal <= line_brightness + 20;
  }else{
    //White Line, black BG
    return sensorVal >= line_brightness - 20;
  }

}
//Returns a logical state which tells, whether the right Sensor is on the line or not
bool sensorRIsOnLine(){
  float sensorVal = getSensorR();
  if (line_brightness < background_brightness){
    //Black Line, White BG
    return sensorVal <= line_brightness + 20;
  }else{
    //White Line, black BG
    return sensorVal >= line_brightness - 20;
  }
}




/*
//Sets the overall Speed of the Robot
void setMotorSpeed(int error){ //<----------------------------------------------------------------------------------------------TODO

  //Fehlergröße normieren
  int absError=constrain(abs(error), 0, 600);

  //Bremsfaktor: je größer die Abweichung, desto stärker bremsen
  int brakeFactor = map(absError, 0, 600, 40, 100) / 100.0;

  //TODO: Angenommen, der Error ist 0, also der Roboter steht exakt auf der Line: Dann wird der absError 0, der braceFaktor 4/10 und der Roboter wird langsamer? Obwohl der auf der Linie steht???
  //Wollen wir nicht langsamer werden, wenn der Error größer wird?

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
  int speedL = curSpeedL + turn;
  int speedR = curSpeedR - turn;

  // Clipping
  speedL = constrain(speedL, -255, 255); //Speed kann maximal zwischen -255(maxSpeed rückwärts) und 255(maxSpeed vorwärts) sein
  speedR = constrain(speedR, -255, 255);

  setMotorL(speedL);
  setMotorR(speedR);
}
*/

//Method to set different LED Modes to indicate the differend Main Logic States
void blinkLed(int pattern){
    unsigned long now = millis();

  // Parameter je Muster:
  int blinks = 0;
  unsigned long onTime = 500;
  unsigned long offTime = 500;
  unsigned long pauseTime = 0;
  bool infiniteBlink = false;
  bool alwaysOn = false;
  bool alwaysOff = false;

  switch (pattern) {
    case 0: alwaysOff = true; break;
    case 1: blinks = 1; pauseTime = 4500; break;
    case 2: blinks = 2; pauseTime = 3500; break;
    case 3: blinks = 3; pauseTime = 2500; break;
    case 4: blinks = 4; pauseTime = 1500; break;
    case 5: infiniteBlink = true; break;
    case 6: alwaysOn = true; break;
  }

  if (alwaysOn) {
    setLed(true);
    return;
  }

  if (alwaysOff){
    setLed(false);
    return;
  }

  if (infiniteBlink) {
    if (ledState == LOW && (now - previousMillis >= offTime)) {
      setLed(true);
      ledState = HIGH;
      previousMillis = now;
    }
    else if (ledState == HIGH && (now - previousMillis >= onTime)) {
      setLed(false);
      ledState = LOW;
      previousMillis = now;
    }
    return;
  }

  // Standard-Blinkmuster (case 1–4)
  switch(step) {
    case 0: // LED an/aus n-mal
      if (ledState == LOW && (now - previousMillis >= offTime)) {
        setLed(true);
        ledState = HIGH;
        previousMillis = now;
      }
      else if (ledState == HIGH && (now - previousMillis >= onTime)) {
        setLed(false);
        ledState = LOW;
        previousMillis = now;
        blinkCount++;
        if (blinkCount >= blinks) {
          step = 1; // Pause starten
          blinkCount = 0;
        }
      }
      break;

    case 1: // Pause nach n Blinkzyklen
      if (now - previousMillis >= pauseTime) {
        step = 0; // Neue Blinkserie starten
        previousMillis = now;
      }
      break;
  }
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

void startState(){
  
}

void calibrationState1(){
  //Left Background Right Line
  if (stateCon == STATE_ENTERING){
      setLed(true);
      delay(20);
      line_brightness_L = getRawSensorL();
      background_brightness_R = getRawSensorR();
      setLed(false);
  }else{

  }

}

void calibrationState2(){
  if (stateCon == STATE_ENTERING){
    setLed(true);
    delay(20);
    background_brightness_L = getRawSensorL();
    line_brightness_R = getRawSensorR();
    setLed(false);
      
  }else{
    
  }
}

void calibrationState3(){
  if (stateCon == STATE_ENTERING){
    calibrateSensorDifference();
    
  }else{
    float delta = getPotiL();

    float motorL = 1023;
    float motorR = 1023;

    if (delta > 0){
      motorR = (1023 * (1 - delta));
    }else{
      motorL = (1023 * (1 + delta));
    }
    setRawMotorL(motorL);
    setRawMotorR(motorR);

    motorL_max = motorL;
    motorR_max = motorR;
  }
}

void calibrationState4(){
  if (stateCon == STATE_ENTERING){
    



  }else{
    float speed = getPotiR();

    float motorL = motorL_max * speed;
    float motorR = motorR_max * speed;

    setRawMotorL(motorL);
    setRawMotorR(motorR);


    motorL_min = motorL;
    motorR_min = motorR;
  }
}

//
void readyState(){
    setRawMotorL(0);
    setRawMotorR(0);
}

//Hauptmethode der Logik (Konvention: globale Variablen oben in Dokument)
/*
void raceState(){
  //Check, ob der Wagen still steht, wenn ja dann slowStart()
  if (curSpeedL==0 && curSpeedR==0){
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

  //setMotorSpeed(diff); //erst Speed anpassen, um ggf. zu bremsen oder beschleunigen
  //setAngle(diff); //danach Winkel anpassen zur Feinjustierung der Räder
}
*/

void raceState(){
  bool sensorL = sensorLIsOnLine();
  bool sensorR = sensorRIsOnLine();

  if (!sensorL && !sensorR){
    setRawMotorL(1023);
    setRawMotorR(1023);
  }

  if (sensorL && !sensorR){
    setRawMotorL(1023);
    setRawMotorR(0);
  }

  if (!sensorL && sensorR){
    setRawMotorL(0);
    setRawMotorR(1023);
  }
  if (sensorL && sensorR){
    setRawMotorL(1023);
    setRawMotorR(1023);
    

  }
}
//Überlegungen: verlorene Linie wiederfinden, PID einbauen

//wenn von readyState() zu raceState() übergegangen wird soll einmalig die Funktion slowStart() aufgerufen werden, danach wird in race_state() übergegangen
