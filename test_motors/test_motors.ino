
const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
const byte serialEn = 9;    // Connect UART output enable of LSA08 to pin 2
const byte junctionPulse = 10;   // Connect JPULSE of LSA08 to pin 4 

const byte dir1 = 4;   // Connect DIR1 of motor driver to pin 13
const byte dir2 = 5;   // Connect DIR2 of motor driver to pin 12
const byte pwm1 = 3;   // Connect PWM1 of motor driver to pin 11
const byte pwm2 = 6;   // Connect PWM2 of motor driver to pin 10

void setup() {
  pinMode(serialEn,OUTPUT);   // Setting serialEn as digital output pin
  pinMode(junctionPulse,INPUT);   // Setting  junctionPulse as digital input pin

  // Setting pin 10 - 13 as digital output pin
  for(byte i=3;i<=6;i++) {
    pinMode(i,OUTPUT);
  }

  // Setting initial condition of serialEn pin to HIGH
  digitalWrite(serialEn,HIGH);

  // Setting the initial condition of motors
  // make sure both PWM pins are LOW
  analogWrite(pwm1,0);
  analogWrite(pwm2,0);

  // State of DIR pins are depending on your physical connection
  // if your robot behaves strangely, try changing thses two values
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,LOW);

  // Begin serial communication with baudrate 9600
  Serial.begin(9600);

  // Clear internal junction counter of LSA08
  clearJunction();

}

void loop() {

  
  moveForward();
  delay(1000);
  
  /*moveRight();
  delay(6000);
  moveLeft();
  delay(6000);*/
}

// Function to clear internal junction counter of LSA08
void clearJunction() {
  char address = 0x01;
  char command = 'X';
  char data = 0x00;
  char checksum = address + command + data;

  Serial.write(address);
  Serial.write(command);
  Serial.write(data);
  Serial.write(checksum);
}

// Function to retrieve junction count from LSA08
int getJunction() {
  char address = 0x01;
  char command = 'X';
  char data = 0x01;
  char checksum = address + command + data;

  Serial.write(address);
  Serial.write(command);
  Serial.write(data);
  Serial.write(checksum);

  while(Serial.available() <= 0);
  return (int(Serial.read()));
}

// The values work good in my case, you could use other values set
// to archieve a performance that satisfy you
void moveLeft() {
  // For robot to move left, right motor has to be faster than left motor
  Serial.println("MOVING LEFT");
  analogWrite(pwm1,90);
  analogWrite(pwm2,10);
}

void moveRight() {
  // For robot to move right, left motor has to be faster than right motor
  Serial.println("MOVING RIGHT");
  analogWrite(pwm1,10);
  analogWrite(pwm2,90);
}

void moveForward() {
  // For robot to move forward, both motors have to be same speed
  Serial.println("MOVING FORWARD");
    digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
  analogWrite(pwm1,150);
  analogWrite(pwm2,150);  //was 70
}

void wait() {
  // Function to makes the robot stay
  Serial.println("WAITING");
  analogWrite(pwm1,0);
  analogWrite(pwm2,0);
}
