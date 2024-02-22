
const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
const byte serialEn = 9;    // Connect UART output enable of LSA08 to pin 2
const byte junctionPulse = 10;   // Connect JPULSE of LSA08 to pin 4 

const byte dir1 = 4;   // Connect DIR1 of motor driver to pin 13
const byte dir2 = 5;   // Connect DIR2 of motor driver to pin 12
const byte pwm1 = 3;   // Connect PWM1 of motor driver to pin 11
const byte pwm2 = 6;   // Connect PWM2 of motor driver to pin 10

int count =0;
unsigned int junctionCount = 0;   // Variable to store junction count value

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

void loop() 
{
  byte dummy = 0;   
  
  // if juction detected, keep moving forward
  Serial.print("Junction detected? : ");
  int a=digitalRead(junctionPulse);
  Serial.println(a);
  delay(1000);
  
  while(digitalRead(junctionPulse)) 
    moveForward();

  digitalWrite(serialEn,LOW);   // Set serialEN to LOW to request UART data
  Serial.println("starting data collection "); 
  while(!(Serial.available()));   // Wait for data to be available
  dummy = Serial.read();    // Read incoming data and store in dummy
  Serial.print("uart data: ");
  Serial.println(dummy);
  digitalWrite(serialEn,HIGH);    // Stop requesting for UART data

  if(a==0)
  {
      // Checking for sensor number 0 and 1, if line detected, move left
      if(bitRead(dummy,0) && bitRead(dummy,1))
      {
        moveLeft();
        Serial.print("Pin 0 and 1 detected line....Must move left now!!!\n");
      }

      // Checking for sensor number 4 and 5, if line detected, move right
      else if( bitRead(dummy,4) && bitRead(dummy,5) )
      {
        moveRight();
        Serial.print("Pin 4 and 5 detected line.... moving right!!!\n");
      }

      // Checking for sensor number 4 and 5, if line detected, move right
      else if( bitRead(dummy,6) && bitRead(dummy,7) )
      {
        moveRight();
        Serial.print("Pin 6 and 7 detected line.... moving right!!!\n");
      }

      // if line is detected by either 3 or 4 of these sensor, move forward
      else if(bitRead(dummy,3)||bitRead(dummy,4))
      {
        moveForward();
        Serial.print("Pin 3 and 4 detected line.... moving forward!\n");
      }

      // If no line is detected, stay at the position
      else
        {
          wait();
          Serial.print("ERROR!!!!NO LINE DETECTED\n");
        }
      // Put some delay to avoid the robot jig while making a turn  
      delay(1000);
  }
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
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
  analogWrite(pwm1,150);
  analogWrite(pwm2,70);
}

void moveRight() {
  // For robot to move right, left motor has to be faster than right motor
  Serial.println("MOVING RIGHT");
   digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
  analogWrite(pwm1,70);
  analogWrite(pwm2,150);
}

void moveForward() {
  // For robot to move forward, both motors have to be same speed
  Serial.println("MOVING FORWARD");
   digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
  analogWrite(pwm1,150);
  analogWrite(pwm2,150);
}

void wait() {
  // Function to makes the robot stay
  Serial.println("WAITING");
   digitalWrite(dir1,LOW);
  digitalWrite(dir2,LOW);
  analogWrite(pwm1,0);
  analogWrite(pwm2,0);
}
