// **************** L298N H-Bridge driving DC motor on Arduino *****************

// ----------------------------------- Motors ----------------------------------
// Left-sided motors:
int ENB = 4;
//Backward
int IN1 = 11;
//Forward
int IN2 = 10;

// Right-side motors:
int ENA = 9;
//Backward
int IN3 = 6;
//Forward
int IN4 = 5;


// ----------------------------------- Front IR Sensor -------------------------
const int analogInpinFrontSS = A0;
float sensorVoltageFrontSS = 0.0;
int sensorValueFrontSS = 0;

// ----------------------------------- Rear IR Sensor --------------------------
const int analogInpinRearSS = A1;
float sensorVoltageRearSS = 0.0;
int sensorValueRearSS = 0;


// ---------------------------- SpeedOfTheWheels -------------------------------
// int speedOfRightWheel = 255;  // The speed of the right wheel
// int speedOfLeftWheel = 247;   // The speed of the left wheel



// ###############################  setup()  ###################################
void setup()
{
  Serial.begin(9600);

  // Get the signal from IRsenosor:
  pinMode(analogInpinFrontSS, INPUT);


  //Set motor control pins to be outputs from the Arduino
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //set both motors speed to 0% duty cycle (off)
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

}



// ################################  loop()  ###################################
void loop()
{
  forward();
  delay(1000);
  backward();
  delay(1000);
  sharpRightTurn();
  delay(1000);
  sharpLeftTurn();
  delay(1000);
  rightTurn();
  delay(1000);
  leftTurn();





}




// #####################  Function Declare& Implementation  ####################
// Front IR Sensor:
void frontIRSensor()
{
    // Get the input reading:
    Serial.println("========== Front SS ==========");
    sensorValueFrontSS = analogRead(analogInpinFrontSS);
    sensorVoltageFrontSS = (sensorValueFrontSS * (5.0 / 1023.0));
    Serial.print("sensorValue : ");
    Serial.print(sensorValueFrontSS);

    Serial.print("\t\tsensorVoltage : ");
    Serial.print(sensorVoltageFrontSS);
    Serial.println("\n");
}

// Rear IR Sensor:
void rearIRSensor()
{
    // Get the input reading:
    Serial.println("========== Rear SS ==========");
    sensorValueRearSS = analogRead(analogInpinRearSS);
    sensorVoltageRearSS = (sensorValueRearSS * (5.0 / 1023.0));
    Serial.print("sensorValue : ");
    Serial.print(sensorValueRearSS);

    Serial.print("\t\tsensorVoltage : ");
    Serial.print(sensorVoltageRearSS);
    Serial.println("\n");
}

//--------------------- returnVoltageFromFrontIRsensor() function --------------
float returnVoltageFromFrontIRsensor()
{

  // Covert into Volts:
  sensorValueFrontSS = analogRead(analogInpinFrontSS);
  sensorVoltageFrontSS = (sensorValueFrontSS * (5.0 / 1023.0));
  Serial.print("\t\tsensorVoltage = ");
  Serial.println(sensorVoltageFrontSS);

  return sensorVoltageFrontSS;
}

//--------------------- returnVoltageFromRearIRsensor() function ---------------
float returnVoltageFromRearIRsensor()
{

  // Covert into Volts:
  sensorValueRearSS = analogRead(analogInpinRearSS);
  sensorVoltageRearSS = (sensorValueRearSS * (5.0 / 1023.0));
  Serial.print("\t\tsensorVoltage = ");
  Serial.println(sensorVoltageRearSS);

  return sensorVoltageRearSS;
}

// ################################  Movement(s)  ##############################
//---------------------------- forward() function ------------------------------
void forward()
{
  Serial.println("\n===== Forward =====");
  // Turn on motors on the Left-sided:(A)
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1,LOW);
  // Turn on motors on the Right-sided:(B)
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  // TO set the turning speed to 200 out of possible range 0 to 255
  analogWrite(ENB, 130);
  analogWrite(ENA, 180);
  delay(2000);

}

//---------------------------- backward() function -----------------------------
void backward()
{
  Serial.println("\n+++++ Backward +++++");

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // TO set the turning speed to 200 out of possible range 0 to 255
  analogWrite(ENB, 130);
  analogWrite(ENA, 180);
  delay(2000);

}

//---------------------------- rightTurn() function ----------------------------
void rightTurn()
{
  Serial.println("==== Righh turn >>>>");
  // Turn on motors on the left-sided:(A)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Turn on motors on the right-sided:(B)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);


  // TO set the turning speed to 200 out of possible range 0 to 255
  analogWrite(ENA, 75);
  analogWrite(ENB, 150);
  delay(1000);

}

//---------------------------- sharpRightTurn() function ---------------------
void sharpRightTurn()
{
  Serial.println("==== Sharp Righh turn >>>>");

  // Turn on motors on the right-sided:(B)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Turn on motors on the left-sided:(B)
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);


  // TO set the turning speed to 200 out of possible range 0 to 255
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(1000);

}

//---------------------------- leftTurn() function -----------------------------
void leftTurn()
{
  Serial.println("<<<< Left turn ====");
  // Turn on motors on the left-sided:(A)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW);

  // Turn on motors on the right-sided:(B)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);


  // TO set the turning speed to 200 out of possible range 0 to 255
  analogWrite(ENA, 150);
  analogWrite(ENB, 75);
  delay(1000);

}

//---------------------------- sharpLeftTurn() function ----------------------
void sharpLeftTurn()
{
  Serial.println("<<<< Sharp Left turn ====");

  // Turn on motors on the right-sided:(B)
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);

  // Turn on motors on the left-sided:(B)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // TO set the turning speed to 200 out of possible range 0 to 255
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(1000);

}
