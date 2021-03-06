/*

 Title:  shadow_chassis_withIR

 Description:
 Using the Shadow Chassis, with Arduino Moto driver shied, IR sensor.  Detect obstacles and back away.

 Version 1.0  Nov 2015
 */


//Defining Arduino Pin Assignments

//A is the Right Wheel and B is the Left Wheel
int pwm_a = 3;   // Right Wheel Speed
int pwm_b = 11;  // Left Wheel Speed
int dir_a = 12;  // Right Wheel Direction
int dir_b = 13;  // Left Wheel Direction


const int IRsensorPin = A0;    // Define the input pin for the IR sensor
int IRsensorValue = 0;   // Variable to store the Integer value coming from the IR sensor

int LEDForward = 4 ;         // Status LED for moving forward
int LEDBackward = 5 ;        // Status LED for moving backward
int LEDRight = 6 ;           // Status LED for moving right
int LEDLeft = 7 ;           // Status LED for moving left

int movingPin = 8;       //Enable movement of the robot.  Low for sensor/Serial work, High moving around (Movement Enabled)
int MoveSize = 100;
int obstacleSeen = 500;  // value when the robot IR Sensor will trip and it should back up

int speedOfRightWheel = 255;  // The speed of the right wheel
int speedOfLeftWheel = 247;   // The speed of the left wheel

void setup()
{
  Serial.begin(9600);


  pinMode(pwm_a, OUTPUT);      //Set motor control pins to be outputs from the Arduino
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  //LED setups
  pinMode(LEDForward, OUTPUT);    //Moving forward
  pinMode(LEDBackward, OUTPUT);   //Moving backwards
  pinMode(LEDRight, OUTPUT);      //turning right
  pinMode(LEDLeft, OUTPUT);       //turning left

  //Input pins
  pinMode(IRsensorPin, INPUT);   // Reading from the IR sensor
  pinMode(movingPin, INPUT);     // Enables movement but still collects compass and IR readings and reports to serial

  analogWrite(pwm_a, 0);          //set both motors speed to 0% duty cycle (off)
  analogWrite(pwm_b, 0);


}

void loop()
{

  //turn all leds off at beginning of loop
  digitalWrite(LEDForward, LOW);    //forwards
  digitalWrite(LEDBackward, LOW);   //backwards
  digitalWrite(LEDRight, LOW);      //right
  digitalWrite(LEDLeft, LOW);       //left


  IRsensorValue = analogRead(IRsensorPin);    //Find out if there is anything in the way
  Serial.print("IR Reading: ");
  Serial.print(IRsensorValue);
  Serial.print("\t");


  //Determine if the robot should move ("1") or if it should stay still ("0")
  // First case, moving pin == 0. Robot does not move.  movingPin is set to 0V.

  if (digitalRead(movingPin) == 0)
  {
    // turn the motors off
    analogWrite(pwm_a, 0);  //set both motors to run at 0% duty cycle (off)
    analogWrite(pwm_b, 0);
    //wait
    Serial.println("motors off");
    delay(1000);
  }

  // This else case is where the robot should move.  "movingPin ==1"
  else
  {
    // Check IR Sensor. If it is too close to something take evasive maneuvers
    if (IRsensorValue > obstacleSeen)
    {

      //evasive maneuvers
      Serial.println("");
      Serial.println("Danger!!  Object detected!");

      // Put it in reverse
      backward();
      // Turn to the Left
      left();
      left();
      left();
      // Move forward
      forward();

      //end of the evasive maneuvers
    }


    //The robot input is set to "move" and nothing is in the way
    else {
      forward();
      forward();


      
    }
  }
}







void forward()
{
  digitalWrite(LEDForward, HIGH);
  Serial.println("Forward");
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, speedOfRightWheel);
  analogWrite(pwm_b, speedOfLeftWheel);
  delay(MoveSize * 4);
  digitalWrite(LEDForward, LOW);
}




void backward()
{
  digitalWrite(LEDBackward, HIGH);
  Serial.println("Backward");
  digitalWrite(dir_a, LOW);
  digitalWrite(dir_b, LOW);
  analogWrite(pwm_a, speedOfRightWheel);
  analogWrite(pwm_b, speedOfLeftWheel);
  delay(MoveSize * 7);
  digitalWrite(LEDBackward, LOW);
}


void left()
{
  digitalWrite(LEDLeft, HIGH);
  Serial.println("Left Turn");
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, speedOfRightWheel);
  analogWrite(pwm_b, speedOfLeftWheel - 50);  //spin the left wheel slower so the robot turns left
  delay(MoveSize);
  digitalWrite(LEDLeft, LOW);
}

void right()
{
  digitalWrite(LEDRight, HIGH);
  Serial.println("Right Turn");
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, speedOfRightWheel - 50); //spin the right wheel slower so the robot turns right
  analogWrite(pwm_b, speedOfLeftWheel);
  delay(MoveSize);
  digitalWrite(LEDRight, LOW);
}
