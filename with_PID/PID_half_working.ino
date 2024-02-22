const int Kp = 0.7;   // Kp value that you have to change(Not sure yet)
const int Kd = 0.3;   // Kd value that you have to change(Not sure yet)
const int setPoint = 35;    // Middle point of sensor array
const int lowSpeed = 35;    // Lowest analog value where the motor starts (Bottom threshold)
const int baseSpeed = 100;    // Base speed for your motors
const int maxSpeed = 225;   // Maximum speed for your motors

const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
const byte serialEn = 9;    // Connect UART output enable of LSA08 to pin 2
const byte junctionPulse = 10;   // Connect JPULSE of LSA08 to pin 10
const byte dir1 = 4;   // Connect DIR1 of motor driver to pin 4
const byte dir2 = 5;   // Connect DIR2 of motor driver to pin 5
const byte pwm1 = 3;   // Connect PWM1 of motor driver to pin 3
const byte pwm2 = 6;   // Connect PWM2 of motor driver to pin 6

void setup() {
  pinMode(serialEn,OUTPUT);   // Setting serialEn as digital output pin
  pinMode(junctionPulse,INPUT);   // Setting junctionPulse as digital input pin

  // Setting pin 3-6 as digital output pin
  for(byte i=3;i<=6;i++) {
    pinMode(i,OUTPUT);
  }

  // Setting initial condition of serialEn pin to HIGH
  digitalWrite(serialEn,HIGH);

  // Setting the initial condition of motors
  // make sure both PWM pins are LOW
  digitalWrite(pwm1,LOW);
  digitalWrite(pwm2,LOW);

  // State of DIR pins are depending on your physical connection
  // if your robot behaves strangely, try changing thses two values
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,LOW);

  // Begin serial communication with baudrate 9600
  Serial.begin(9600);

}

int lastError = 0;    // Declare a variable to store previous error

void loop() {
  digitalWrite(serialEn,LOW);   // Set serialEN to LOW to request UART data
  while(Serial.available() <= 0);   // Wait for data to be available
  int positionVal = Serial.read();    // Read incoming data and store in variable positionVal
  digitalWrite(serialEn,HIGH);    // Stop requesting for UART data
  
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);

  // If no line is detected, stay at the position
  if(positionVal == 255) {
    analogWrite(pwm1,0);
    analogWrite(pwm2,0);
  }

  // Else if line detected, calculate the motor speed and apply
  else {
    int error = positionVal - setPoint;   // Calculate the deviation from position to the set point
    int motorSpeed = Kp * error + Kd * (error - lastError);   // Applying formula of PID
    lastError = error;    // Store current error as previous error for next iteration use

    // Adjust the motor speed based on calculated value
    // You might need to interchange the + and - sign if your robot move in opposite direction
    int rightMotorSpeed = baseSpeed - motorSpeed;
    int leftMotorSpeed = baseSpeed + motorSpeed;

    // If the speed of motor exceed max speed, set the speed to max speed
    if(rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if(leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;

    // If the speed of motor is negative, set it to 0
    if(rightMotorSpeed < 0) rightMotorSpeed = 0;
    else if (rightMotorSpeed < lowSpeed) rightMotorSpeed = lowSpeed;
    if(leftMotorSpeed < 0) leftMotorSpeed = 0;
    else if (leftMotorSpeed < lowSpeed) leftMotorSpeed = lowSpeed;

    // 

    // Writing the motor speed value as output to hardware motor
    analogWrite(pwm1,rightMotorSpeed);
    analogWrite(pwm2,leftMotorSpeed);
  }

}
