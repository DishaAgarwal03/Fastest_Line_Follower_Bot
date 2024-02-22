#include <SoftwareSerial.h>

SoftwareSerial lsa08Serial(1, 0); // RX, TX

void setup() {
  Serial.begin(9600); // configure serial communication to PC
  lsa08Serial.begin(9600); // configure serial communication to LSA08
}

void loop() {
  Serial.println("printing");
  lsa08Serial.write("U\r\n"); // send the command to request sensor data
  delay(10); // wait for the response to be available
  String response = lsa08Serial.readStringUntil('\r'); // read the response from the LSA08
  if (response.length() > 0) {
    int distance = response.toInt(); // parse the sensor data from the response
    Serial.println(distance); // use the sensor data as needed in your program
  }
}
